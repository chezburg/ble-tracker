/*
 * ble_scanner.cpp
 * Continuously scans for BLEBase advertisements, extracts
 * RSSI + TX-power-at-1m from manufacturer data, applies a
 * per-base Kalman filter, and converts to distance via the
 * log-distance path-loss model.
 */

#include "ble_scanner.h"
#include <math.h>

BLEScanner bleScanner;

// ────────────────────────────────────────────────────────────────
void BLEScanner::begin() {
  NimBLEDevice::init("");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);   // max RX sensitivity

  _scan = NimBLEDevice::getScan();
  _scan->setScanCallbacks(this, false);
  _scan->setInterval(BLE_SCAN_INTERVAL);
  _scan->setWindow(BLE_SCAN_WINDOW);
  _scan->setActiveScan(false);   // passive — bases use adv only
  _scan->start(0, false);        // continuous, non-blocking

  // Initialise Kalman state
  for (int i = 0; i < NUM_BASES; i++) {
    _bases[i].id          = i;
    _bases[i].valid       = false;
    _bases[i].kEstimate   = -70.0f;
    _bases[i].kError      = 2.0f;
    _bases[i].lastSeenMs  = 0;
  }

  Serial.println("[BLE] Scanner started.");
}

// ────────────────────────────────────────────────────────────────
void BLEScanner::update() {
  uint32_t now = millis();
  for (int i = 0; i < NUM_BASES; i++) {
    if (_bases[i].valid && (now - _bases[i].lastSeenMs) > RSSI_STALE_MS) {
      _bases[i].valid = false;
      Serial.printf("[BLE] Base %d went stale\n", i);
    }
  }
}

// ────────────────────────────────────────────────────────────────
//  NimBLE advertised-device callback (called from BLE task)
// ────────────────────────────────────────────────────────────────
void BLEScanner::onResult(const NimBLEAdvertisedDevice* dev) {
  // Filter by manufacturer data
  if (!dev->haveManufacturerData()) return;

  std::string mfg = dev->getManufacturerData();
  // Manufacturer data format from base firmware:
  //   Bytes 0-1 : Company ID (little-endian) — 0x59, 0x00 for Nordic
  //   Byte  2   : Base ID
  //   Byte  3   : TX power at 1m (int8)
  if (mfg.length() < 4) return;

  uint16_t cid = (uint8_t)mfg[0] | ((uint8_t)mfg[1] << 8);
  if (cid != BASE_MFG_CID) return;

  uint8_t baseId    = (uint8_t)mfg[2];
  int8_t  txPower   = (int8_t) mfg[3];

  if (baseId >= NUM_BASES) return;

  BaseReading& b = _bases[baseId];
  b.id         = baseId;
  b.txPower1m  = txPower;
  b.rssiRaw    = dev->getRSSI();
  b.lastSeenMs = millis();
  b.valid      = true;

  kalmanUpdate(b, b.rssiRaw);
  b.distanceM = estimateDistance(b.txPower1m, b.rssiFiltered, baseId);
}

// ────────────────────────────────────────────────────────────────
//  Kalman filter for RSSI smoothing
// ────────────────────────────────────────────────────────────────
void BLEScanner::kalmanUpdate(BaseReading& b, int newRssi) {
  // Predict
  float predError = b.kError + KALMAN_Q;

  // Update
  float K         = predError / (predError + KALMAN_R);
  b.kEstimate     = b.kEstimate + K * ((float)newRssi - b.kEstimate);
  b.kError        = (1.0f - K) * predError;
  b.rssiFiltered  = b.kEstimate;
}

// ────────────────────────────────────────────────────────────────
//  Log-distance path-loss model
//    d = 10 ^ ((TxPower_1m - RSSI_filtered) / (10 * n)) + offset
// ────────────────────────────────────────────────────────────────
float BLEScanner::estimateDistance(int8_t txPower, float rssiFiltered, uint8_t baseId) const {
  if (rssiFiltered >= 0) return 0.01f;

  float n     = _baseN[baseId];
  float txRef = _baseTxRef[baseId];

  float ratio = (txRef - rssiFiltered) / (10.0f * n);
  float d = powf(10.0f, ratio);

  if (baseId < NUM_BASES) {
    d += _offsets[baseId];
  }

  return (d < 0.01f) ? 0.01f : d;
}

// ────────────────────────────────────────────────────────────────
bool BLEScanner::addCalPoint(uint8_t id, float dist, float& outN, float& outTxRef) {
  if (id >= NUM_BASES || !_bases[id].valid) return false;

  uint8_t idx = _calCount[id] % 5;
  _calPoints[id][idx].log10D = log10f(dist);
  _calPoints[id][idx].rssi   = _bases[id].rssiFiltered;
  _calPoints[id][idx].set    = true;
  _calCount[id]++;

  int nPoints = min((int)_calCount[id], 5);
  if (nPoints < 2) {
    // Single point: Adjust TX power reference only, keep default N
    outN     = _baseN[id];
    outTxRef = _calPoints[id][idx].rssi + 10.0f * outN * _calPoints[id][idx].log10D;
    _baseN[id]     = outN;
    _baseTxRef[id] = outTxRef;
    return true;
  }

  // Linear Regression (Least Squares)
  // RSSI = (-10n) * log10(d) + TxRef
  // y = mx + c
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  for (int i = 0; i < 5; i++) {
    if (!_calPoints[id][i].set) continue;
    float x = _calPoints[id][i].log10D;
    float y = _calPoints[id][i].rssi;
    sumX  += x;
    sumY  += y;
    sumXY += x * y;
    sumX2 += x * x;
  }

  float denominator = (nPoints * sumX2 - sumX * sumX);
  if (fabsf(denominator) < 1e-5f) return false;

  float m = (nPoints * sumXY - sumX * sumY) / denominator;
  float c = (sumY - m * sumX) / nPoints;

  outN     = -m / 10.0f;
  outTxRef = c;

  // Sanity check N
  if (outN < 1.0f) outN = 1.0f;
  if (outN > 5.0f) outN = 5.0f;

  _baseN[id]     = outN;
  _baseTxRef[id] = outTxRef;
  return true;
}

// ────────────────────────────────────────────────────────────────
bool BLEScanner::hasValidFix() const {
  for (int i = 0; i < NUM_BASES; i++)
    if (!_bases[i].valid) return false;
  return true;
}

uint8_t BLEScanner::validCount() const {
  uint8_t c = 0;
  for (int i = 0; i < NUM_BASES; i++)
    if (_bases[i].valid) c++;
  return c;
}

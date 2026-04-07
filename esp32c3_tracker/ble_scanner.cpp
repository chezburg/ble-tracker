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

  // Initialise Kalman state and history
  for (int i = 0; i < NUM_BASES; i++) {
    _bases[i].id          = i;
    _bases[i].valid       = false;
    _bases[i].kEstimate   = -70.0f;
    _bases[i].kError      = 2.0f;
    _bases[i].lastSeenMs  = 0;
    _bases[i].distHistIdx = 0;
    _bases[i].distHistCount = 0;
    for (int j = 0; j < RSSI_HISTORY_LEN; j++) {
      _bases[i].distHistory[j] = 0.0f;
    }
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
  float rssiDistanceRaw = estimateDistance(b.txPower1m, b.rssiFiltered, baseId);

  // Add new distance to history buffer
  b.distHistory[b.distHistIdx] = rssiDistanceRaw;
  b.distHistIdx = (b.distHistIdx + 1) % RSSI_HISTORY_LEN;
  if (b.distHistCount < RSSI_HISTORY_LEN) {
    b.distHistCount++;
  }

  // Calculate moving-window average of distances
  float distTotal = 0.0f;
  for (int i = 0; i < b.distHistCount; i++) {
    distTotal += b.distHistory[i];
  }
  
  float movavg = (b.distHistCount > 0) ? (distTotal / (float)b.distHistCount) : rssiDistanceRaw;
  
  // Standard smoothing: average of windowed readings
  b.distanceM = movavg;
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
//    d = 10 ^ ((TxPower_1m - RSSI_filtered) / (10 * n))
// ────────────────────────────────────────────────────────────────
float BLEScanner::estimateDistance(int8_t txPower, float rssiFiltered, uint8_t baseId) const {
  if (rssiFiltered >= 0) return 0.01f;

  float ratio = ((float)txPower - rssiFiltered) / (10.0f * _pathLossN);
  float d = powf(10.0f, ratio);

  return (d < 0.01f) ? 0.01f : d;
}

// ────────────────────────────────────────────────────────────────
bool BLEScanner::hasValidFix() const {
  return validCount() > 0;
}

uint8_t BLEScanner::validCount() const {
  uint8_t c = 0;
  for (int i = 0; i < NUM_BASES; i++)
    if (_bases[i].valid) c++;
  return c;
}

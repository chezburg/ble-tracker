#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>
#include "config.h"

// ── Per-base ranging data ────────────────────────────────────────
struct BaseReading {
  uint8_t  id;                          // 0, 1, 2
  int8_t   txPower1m;                   // from advertisement payload
  int      rssiRaw;                     // latest raw RSSI
  float    rssiFiltered;               // Kalman-filtered RSSI
  float    distanceM;                  // estimated distance (metres)
  uint32_t lastSeenMs;                 // millis() of last packet
  bool     valid;                       // false if stale

  // Kalman state
  float    kEstimate;
  float    kError;
};

// ── Per-base calibration data ───────────────────────────────────
struct CalPoint {
  float log10D;
  float rssi;
  bool  set;
};

// ── BLEScanner ──────────────────────────────────────────────────
class BLEScanner : public NimBLEScanCallbacks {
public:
  void     begin();
  void     update();                    // call frequently from loop()

  bool     hasValidFix() const;         // true if all NUM_BASES are fresh
  uint8_t  validCount()  const;

  const BaseReading& base(uint8_t id) const { return _bases[id]; }

  void     setPathLossN(float n) { _pathLossN = n; }
  float    getPathLossN()  const { return _pathLossN; }

  void     setBaseModel(uint8_t id, float n, float txRef) {
    if (id < NUM_BASES) { _baseN[id] = n; _baseTxRef[id] = txRef; }
  }
  float    getBaseN(uint8_t id)     const { return (id < NUM_BASES) ? _baseN[id] : _pathLossN; }
  float    getBaseTxRef(uint8_t id) const { return (id < NUM_BASES) ? _baseTxRef[id] : -59.0f; }

  void     setBaseOffset(uint8_t id, float offset) { if (id < NUM_BASES) _offsets[id] = offset; }
  float    getBaseOffset(uint8_t id) const { return (id < NUM_BASES) ? _offsets[id] : 0.0f; }

  bool     addCalPoint(uint8_t id, float actualDist, float& outN, float& outTxRef);

private:
  // NimBLE callback — signature changed in NimBLE-Arduino 2.x
  void onResult(const NimBLEAdvertisedDevice* dev) override;

  float   estimateDistance(int8_t txPower, float rssiFiltered, uint8_t baseId) const;
  void    kalmanUpdate(BaseReading& b, int newRssi);

  BaseReading _bases[NUM_BASES];
  float       _pathLossN = DEFAULT_PATH_LOSS_N;
  float       _baseN[NUM_BASES];
  float       _baseTxRef[NUM_BASES];
  float       _offsets[NUM_BASES] = {0.0f, 0.0f, 0.0f};

  CalPoint    _calPoints[NUM_BASES][5];
  uint8_t     _calCount[NUM_BASES] = {0, 0, 0};

  NimBLEScan* _scan      = nullptr;
};

extern BLEScanner bleScanner;

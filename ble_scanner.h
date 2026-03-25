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

private:
  // NimBLE callback — signature changed in NimBLE-Arduino 2.x
  void onResult(const NimBLEAdvertisedDevice* dev) override;

  float   estimateDistance(int8_t txPower, float rssiFiltered) const;
  void    kalmanUpdate(BaseReading& b, int newRssi);

  BaseReading _bases[NUM_BASES];
  float       _pathLossN = DEFAULT_PATH_LOSS_N;
  NimBLEScan* _scan      = nullptr;
};

extern BLEScanner bleScanner;

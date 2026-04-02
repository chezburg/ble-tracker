#pragma once
#include <Arduino.h>
#include "config.h"
#include "positioning.h"

struct TrackerConfig {
  char    wifiSsid[64];
  char    wifiPass[64];
  BaseCoord baseCoords[NUM_BASES];
  float   floorHeights[MAX_FLOORS];
  float   baseOffsets[NUM_BASES];
  float   baseN[NUM_BASES];
  float   baseTxRef[NUM_BASES];
  uint8_t floorCount;
  float   pathLossN;
};

// Log entry stored in NVS circular buffer
struct LogEntry {
  uint32_t ts;      // millis
  float    x, y;
  float    altM;
  int      floor;
  float    accuracy;
};

class Storage {
public:
  void  begin();
  void  loadConfig(TrackerConfig& cfg);
  void  saveConfig(const TrackerConfig& cfg);

  // Flash log
  void  appendLog(const LogEntry& entry);
  int   logCount() const;
  LogEntry getLog(int index) const;
  void  clearLog();

private:
  int _logHead  = 0;
  int _logCount = 0;
};

extern Storage storage;

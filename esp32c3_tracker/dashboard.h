#pragma once
#include <Arduino.h>
#include "config.h"

#include "positioning.h"

class Dashboard {
public:
  void begin(const char* ssid, const char* pass);
  void update();   // call from loop()
  void pushUpdate(const PositionFix& fix);
  
  // UI state helpers
  void setNextPingTime(uint32_t ms) { _nextPingMs = ms; }
  void setPingStatus(bool active) { _isPinging = active; }

  uint32_t _nextPingMs = 0;
  bool     _isPinging = false;

private:
  void setupRoutes();
};

extern Dashboard dashboard;

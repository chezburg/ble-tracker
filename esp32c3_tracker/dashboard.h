#pragma once
#include <Arduino.h>
#include "config.h"

class Dashboard {
public:
  void begin(const char* ssid, const char* pass);
  void update();   // call from loop()

private:
  void setupRoutes();
  void pushUpdate();

  uint32_t _lastPush = 0;
};

extern Dashboard dashboard;

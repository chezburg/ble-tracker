#pragma once
#include <Arduino.h>
#include "config.h"

class WebServer {
public:
  void begin(const char* ssid, const char* pass);
  void update();   // call from loop() — pushes WS updates

private:
  void setupRoutes();
  void pushUpdate();

  uint32_t _lastPush = 0;
};

extern WebServer webServer;

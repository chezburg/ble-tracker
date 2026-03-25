#pragma once
#include <Arduino.h>
#include "config.h"

class Altimeter {
public:
  bool  begin();                      // returns false if sensor not found
  void  update();                     // call at BARO_UPDATE_HZ

  void  calibrateGround();           // set current pressure as floor-0 reference
  void  registerFloor(uint8_t floorIndex);  // record current altitude as floor boundary

  float getAltitudeM()    const { return _altM; }
  float getPressurePa()   const { return _pressurePa; }
  float getTemperatureC() const { return _tempC; }

  // Detected floor: set externally by Positioning after floor-height table is built
  void  setFloor(int f)   { _floor = f; }
  int   getFloor()  const { return _floor; }

private:
  float   _altM       = 0.0f;
  float   _refPa      = 101325.0f;   // sea-level reference (Pa)
  float   _pressurePa = 101325.0f;
  float   _tempC      = 20.0f;
  int     _floor      = 0;
};

extern Altimeter altimeter;

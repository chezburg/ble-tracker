#pragma once
#include <Arduino.h>
#include "config.h"

// ── Base station world-coordinate ───────────────────────────────
struct BaseCoord {
  float x;   // metres
  float y;   // metres
};

// ── 2-D position fix ────────────────────────────────────────────
struct PositionFix {
  float    x;           // metres
  float    y;           // metres
  int      floor;       // detected floor number (0-based)
  float    altitudeM;   // relative altitude (metres above boot point)
  float    accuracy;    // estimated error radius (metres), -1 if unknown
  uint32_t timestampMs;
  bool     valid;
};

// ── Positioning engine ──────────────────────────────────────────
class Positioning {
public:
  void  begin();
  void  update();   // call at POSITION_UPDATE_HZ

  void  setBaseCoord(uint8_t id, float x, float y);
  const BaseCoord& getBaseCoord(uint8_t id) const { return _coords[id]; }

  void  setFloorHeight(uint8_t floor, float heightM);
  float getFloorHeight(uint8_t floor) const;
  void  setFloorCount(uint8_t n) { _floorCount = n; }
  uint8_t getFloorCount() const  { return _floorCount; }

  const PositionFix& latest() const { return _fix; }
  PositionFix getAveragedFix() const;

private:
  /*
   * Weighted Least-Squares Trilateration
   * Iterative Gauss-Newton solver — more robust than closed-form
   * when distances are noisy.
   */
  bool  trilaterate(const float* dist,
                    const BaseCoord* coords,
                    const float* weights,
                    float& outX, float& outY);

  int   detectFloor(float altM) const;

  BaseCoord   _coords[NUM_BASES];
  float       _floorHeights[MAX_FLOORS]; // height of each floor in metres
  uint8_t     _floorCount = 1;

  PositionFix _fix;
  PositionFix _history[3];
  uint8_t     _historyIdx = 0;
  bool        _historyFull = false;
  float       _smoothX = 0, _smoothY = 0;
  bool        _firstFix = true;
};

extern Positioning positioning;

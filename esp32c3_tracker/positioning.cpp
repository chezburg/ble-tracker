/*
 * positioning.cpp
 *
 * Weighted Least-Squares trilateration using an iterative
 * Gauss-Newton solver.  Weights are derived from signal quality
 * (inverse of distance variance estimate).
 *
 * Floor detection: binary-searches the floor-height table using
 * the BMP390 relative altitude, with hysteresis to avoid
 * flickering at boundaries.
 */

#include "positioning.h"
#include "ble_scanner.h"
#include <math.h>

Positioning positioning;

// ────────────────────────────────────────────────────────────────
void Positioning::begin() {
  // Default: bases in an equilateral triangle, 3 m sides
  _coords[0] = {  0.0f,  0.0f };
  _coords[1] = {  3.0f,  0.0f };
  _coords[2] = {  1.5f,  2.6f };

  for (int i = 0; i < MAX_FLOORS; i++)
    _floorHeights[i] = (float)i * 3.0f;   // default 3 m per floor

  _fix.valid = false;
  Serial.println("[POS] Positioning engine ready.");
}

// ────────────────────────────────────────────────────────────────
void Positioning::update() {
  if (!bleScanner.hasValidFix()) {
    _fix.valid = false;
    return;
  }

  uint8_t count = bleScanner.validCount();
  float nx, ny;
  bool trilatSuccess = false;

  // Attempt trilateration only if we have at least 3 valid bases
  if (count >= 3) {
    float dist[NUM_BASES], weights[NUM_BASES];
    for (int i = 0; i < NUM_BASES; i++) {
      const BaseReading& b = bleScanner.base(i);
      dist[i] = b.distanceM;
      if (b.valid) {
        weights[i] = 1.0f / max(dist[i] * dist[i], 0.01f);
      } else {
        weights[i] = 0.0f; // Ignore invalid bases in weighted least squares
      }
    }

    trilatSuccess = trilaterate(dist, _coords, weights, nx, ny);
  }

  if (!trilatSuccess) {
    // Fallback: Use the coordinate of the base with minimum estimated distance
    float minDist = 1000000.0f;
    int closestBase = -1;
    for (int i = 0; i < NUM_BASES; i++) {
      const BaseReading& b = bleScanner.base(i);
      if (b.valid && b.distanceM < minDist) {
        minDist = b.distanceM;
        closestBase = i;
      }
    }
    
    if (closestBase != -1) {
      nx = _coords[closestBase].x;
      ny = _coords[closestBase].y;
    } else {
      _fix.valid = false;
      return;
    }
  }

  // EMA smoothing on output
  if (_firstFix) {
    _smoothX  = nx;
    _smoothY  = ny;
    _firstFix = false;
  } else {
    _smoothX = POSITION_SMOOTH_ALPHA * nx + (1.0f - POSITION_SMOOTH_ALPHA) * _smoothX;
    _smoothY = POSITION_SMOOTH_ALPHA * ny + (1.0f - POSITION_SMOOTH_ALPHA) * _smoothY;
  }

  _fix.x           = _smoothX;
  _fix.y           = _smoothY;
  _fix.timestampMs = millis();
  _fix.valid       = true;

  // Accuracy estimate: RMS of residuals
  float rms = 0;
  for (int i = 0; i < NUM_BASES; i++) {
    const BaseReading& b = bleScanner.base(i);
    float dx = _fix.x - _coords[i].x;
    float dy = _fix.y - _coords[i].y;
    float calc = sqrtf(dx*dx + dy*dy);
    float res  = calc - b.distanceM;
    rms += res * res;
  }
  _fix.accuracy    = sqrtf(rms / NUM_BASES);

  // Update averaging history
  _history[_historyIdx] = _fix;
  _historyIdx++;
  if (_historyIdx >= 3) {
    _historyIdx = 0;
    _historyFull = true;
  }
}

// ────────────────────────────────────────────────────────────────
PositionFix Positioning::getAveragedFix() const {
  if (!_historyFull) {
    // Before buffer is full, just return the newest single fix
    uint8_t lastIdx = (_historyIdx > 0) ? (_historyIdx - 1) : 2;
    return _history[lastIdx];
  }

  PositionFix avg = _history[0]; // Copy metadata (floor, altitude, time, validity)
  avg.x = 0;
  avg.y = 0;
  avg.accuracy = 0;

  for (int i = 0; i < 3; i++) {
    avg.x += _history[i].x;
    avg.y += _history[i].y;
    avg.accuracy += _history[i].accuracy;
  }

  avg.x /= 3.0f;
  avg.y /= 3.0f;
  avg.accuracy /= 3.0f;

  return avg;
}

// ────────────────────────────────────────────────────────────────
//  Gauss-Newton Weighted Least-Squares Trilateration
//  Minimises:  sum_i  w_i * (||p - b_i|| - d_i)^2
// ────────────────────────────────────────────────────────────────
bool Positioning::trilaterate(const float* dist,
                              const BaseCoord* coords,
                              const float* weights,
                              float& outX, float& outY) {
  // Initial guess: centroid of base stations
  float x = 0, y = 0;
  for (int i = 0; i < NUM_BASES; i++) { x += coords[i].x; y += coords[i].y; }
  x /= NUM_BASES;  y /= NUM_BASES;

  const int   MAX_ITER = 50;
  const float TOL      = 1e-5f;

  for (int iter = 0; iter < MAX_ITER; iter++) {
    float JtWJ_00=0, JtWJ_01=0, JtWJ_11=0;
    float JtWr_0=0,  JtWr_1=0;

    for (int i = 0; i < NUM_BASES; i++) {
      float dx   = x - coords[i].x;
      float dy   = y - coords[i].y;
      float calc = sqrtf(dx*dx + dy*dy);
      if (calc < 1e-6f) calc = 1e-6f;

      float res  = calc - dist[i];
      float Jx   = dx / calc;
      float Jy   = dy / calc;
      float w    = weights[i];

      JtWJ_00 += w * Jx * Jx;
      JtWJ_01 += w * Jx * Jy;
      JtWJ_11 += w * Jy * Jy;
      JtWr_0  += w * Jx * res;
      JtWr_1  += w * Jy * res;
    }

    // Solve 2×2 system: JtWJ * delta = -JtWr
    float det = JtWJ_00 * JtWJ_11 - JtWJ_01 * JtWJ_01;
    if (fabsf(det) < 1e-10f) return false;

    float dx = -(JtWJ_11 * JtWr_0 - JtWJ_01 * JtWr_1) / det;
    float dy = -(JtWJ_00 * JtWr_1 - JtWJ_01 * JtWr_0) / det;

    x += dx;
    y += dy;

    if (fabsf(dx) < TOL && fabsf(dy) < TOL) break;
  }

  outX = x;
  outY = y;
  return true;
}

// ────────────────────────────────────────────────────────────────
//  Floor detection with hysteresis
// ────────────────────────────────────────────────────────────────
int Positioning::detectFloor(float altM) const {
  static int lastFloor = 0;

  for (int f = _floorCount - 1; f >= 0; f--) {
    float boundary = _floorHeights[f];
    // Apply hysteresis: require extra clearance to move UP
    float threshold = (f > lastFloor)
                      ? boundary + FLOOR_HYSTERESIS_M
                      : boundary - FLOOR_HYSTERESIS_M;
    if (altM >= threshold) {
      lastFloor = f;
      return f;
    }
  }
  return 0;
}

// ────────────────────────────────────────────────────────────────
void Positioning::setBaseCoord(uint8_t id, float x, float y) {
  if (id < NUM_BASES) {
    _coords[id] = {x, y};
    Serial.printf("[POS] Base %d coord set to (%.2f, %.2f)\n", id, x, y);
  }
}

void Positioning::setFloorHeight(uint8_t floor, float heightM) {
  if (floor < MAX_FLOORS) {
    _floorHeights[floor] = heightM;
    Serial.printf("[POS] Floor %d height set to %.2f m\n", floor, heightM);
  }
}

float Positioning::getFloorHeight(uint8_t floor) const {
  if (floor < MAX_FLOORS) return _floorHeights[floor];
  return 0.0f;
}

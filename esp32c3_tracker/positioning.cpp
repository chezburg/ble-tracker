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
  _stableFix = false;
  _stabilityCounter = 0;
  Serial.println("[POS] Positioning engine ready.");
}

void Positioning::resetFix() {
  _fix.valid = false;
  _stableFix = false;
  _stabilityCounter = 0;
  _firstFix = true;
  _historyIdx = 0;
  _historyFull = false;
  Serial.println("[POS] Fix reset. Dialing in startup sequence...");
}

// ────────────────────────────────────────────────────────────────
void Positioning::update() {
  uint32_t now = millis();
  uint8_t count = bleScanner.validCount();

  // Coasting Logic: If we recently had a stable fix, hold it for up to 3s 
  // even if some bases go stale (common in noisy environments)
  bool coasting = false;
  if (_stableFix && _fix.valid && count < MIN_BASES_FOR_FIX) {
    if (now - _fix.timestampMs < 3000) {
      coasting = true;
    }
  }

  if (count < MIN_BASES_FOR_FIX && !coasting) {
    _fix.valid = false;
    _stableFix = false;
    _stabilityCounter = 0;
    return;
  }

  float nx, ny;
  bool trilatSuccess = false;

  if (coasting) {
    nx = _fix.x;
    ny = _fix.y;
    trilatSuccess = true;
  } else if (count >= 3) {
    float dist[NUM_BASES], weights[NUM_BASES];
    for (int i = 0; i < NUM_BASES; i++) {
      const BaseReading& b = bleScanner.base(i);
      dist[i] = b.distanceM;
      if (b.valid) {
        // Bermudan improvement: scale weights by inverse distance variance
        // and cap distance to prevent distant bases from skewing solution
        weights[i] = 1.0f / max(dist[i] * dist[i], 0.01f);
      } else {
        weights[i] = 0.0f; // Ignore invalid bases in weighted least squares
      }
    }

    trilatSuccess = trilaterate(dist, _coords, weights, nx, ny);
    
    // Outlier Rejection: Check if point is significantly outside the bounds of the bases
    if (trilatSuccess) {
      float minX = _coords[0].x, maxX = _coords[0].x;
      float minY = _coords[0].y, maxY = _coords[0].y;
      for (int i = 1; i < NUM_BASES; i++) {
        minX = min(minX, _coords[i].x); maxX = max(maxX, _coords[i].x);
        minY = min(minY, _coords[i].y); maxY = max(maxY, _coords[i].y);
      }
      
      if (nx < minX - OUTLIER_REJECTION_MARGIN || nx > maxX + OUTLIER_REJECTION_MARGIN ||
          ny < minY - OUTLIER_REJECTION_MARGIN || ny > maxY + OUTLIER_REJECTION_MARGIN) {
        trilatSuccess = false;
        // Serial.println("[POS] Outlier rejected: Outside base bounds.");
      }
    }

    // Velocity Rejection: Check if distance from last fix implies impossible speed
    if (trilatSuccess && _stableFix) {
      float dx = nx - _fix.x;
      float dy = ny - _fix.y;
      float travelDist = sqrtf(dx*dx + dy*dy);
      uint32_t dt = millis() - _fix.timestampMs;
      
      if (dt < MAX_DELTA_T_MS) {
        float velocity = travelDist / (dt / 1000.0f);
        if (velocity > MAX_VELOCITY_MPS) {
          trilatSuccess = false;
          // Serial.printf("[POS] Outlier rejected: Velocity too high (%.2f m/s)\n", velocity);
        }
      }
    }
  }

  if (!trilatSuccess) {
    // Improved Fallback: If we have a stable fix, hold position. 
    // Otherwise use a weighted centroid of all valid bases.
    if (_stableFix) {
      nx = _fix.x;
      ny = _fix.y;
      trilatSuccess = true;
    } else {
      float totalW = 0;
      nx = 0; ny = 0;
      for (int i = 0; i < NUM_BASES; i++) {
        const BaseReading& b = bleScanner.base(i);
        if (b.valid) {
          float w = 1.0f / max(b.distanceM, 0.1f);
          nx += _coords[i].x * w;
          ny += _coords[i].y * w;
          totalW += w;
        }
      }
      if (totalW > 0) {
        nx /= totalW;
        ny /= totalW;
        trilatSuccess = true;
      }
    }
    
    if (!trilatSuccess) {
      _fix.valid = false;
      _stableFix = false;
      _stabilityCounter = 0;
      return;
    }
  }

  // Dial-in Startup sequence: wait for multiple fixes to stabilise filters
  if (!_stableFix) {
    _stabilityCounter++;
    if (_stabilityCounter >= 5) { // Require 5 samples to "dial in"
      _stableFix = true;
      Serial.println("[POS] Position stabilized.");
    }
    // During dial-in, we update _smoothX/Y but don't mark fix as valid for output
    // This primes the EMA filters.
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
  _fix.valid       = _stableFix;

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

  // Accumulate for high-quality ping (up to 300 samples)
  if (_fix.valid && _accumCount < 300) {
    _accumBuffer[_accumCount].x = _fix.x;
    _accumBuffer[_accumCount].y = _fix.y;
    _accumBuffer[_accumCount].accuracy = _fix.accuracy;
    _accumCount++;
  }

  // Update averaging history (legacy 3-sample buffer)
  if (_fix.valid) {
    _history[_historyIdx] = _fix;
    _historyIdx++;
    if (_historyIdx >= 3) {
      _historyIdx = 0;
      _historyFull = true;
    }
  }
}

// ────────────────────────────────────────────────────────────────
void Positioning::clearAccumulator() {
  _accumCount = 0;
}

PositionFix Positioning::getHighQualityFix() {
  if (_accumCount == 0) return _fix;

  // Bermuda high-quality ping: pick the top 20% most accurate samples
  // For simplicity, we'll use a selection sort/partition on the best accuracy
  // but for 300 samples, let's just find the best Accuracy and average samples 
  // that are close to it.
  
  float bestAcc = 1000.0f;
  for (int i = 0; i < _accumCount; i++) {
    if (_accumBuffer[i].accuracy < bestAcc) bestAcc = _accumBuffer[i].accuracy;
  }

  // Average all samples within 50% of the best accuracy (the "clean" signals)
  float avgX = 0, avgY = 0, avgAcc = 0;
  int count = 0;
  float threshold = bestAcc * 1.5f; // include samples with up to 50% more error than the best

  for (int i = 0; i < _accumCount; i++) {
    if (_accumBuffer[i].accuracy <= threshold) {
      avgX += _accumBuffer[i].x;
      avgY += _accumBuffer[i].y;
      avgAcc += _accumBuffer[i].accuracy;
      count++;
    }
  }

  PositionFix highQuality = _fix;
  if (count > 0) {
    highQuality.x = avgX / (float)count;
    highQuality.y = avgY / (float)count;
    highQuality.accuracy = avgAcc / (float)count;
    highQuality.valid = true;
  }
  
  Serial.printf("[POS] High quality ping derived from %d/%d samples (threshold=%.2fm)\n", 
                count, _accumCount, threshold);
  
  return highQuality;
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

    // Damping: Limit the maximum step size per iteration to prevent explosions
    const float MAX_STEP = 2.0f; 
    float stepLen = sqrtf(dx*dx + dy*dy);
    if (stepLen > MAX_STEP) {
      dx = (dx / stepLen) * MAX_STEP;
      dy = (dy / stepLen) * MAX_STEP;
    }

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

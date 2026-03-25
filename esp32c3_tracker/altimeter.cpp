/*
 * altimeter.cpp
 * BMP390 via Adafruit library.  Provides relative altitude (m)
 * from a boot-time pressure reference and a floor-registration
 * helper for manual floor-height calibration.
 */

#include "altimeter.h"
#include <Adafruit_BMP3XX.h>
#include <Wire.h>
#include <math.h>

Altimeter altimeter;
static Adafruit_BMP3XX bmp;

// ────────────────────────────────────────────────────────────────
bool Altimeter::begin() {
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bmp.begin_I2C(BMP390_I2C_ADDR)) {
    Serial.println("[ALT] BMP390 not found! Check wiring.");
    return false;
  }

  // Oversampling & filter settings for best accuracy
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
  bmp.setOutputDataRate(BMP3_ODR_12_5_HZ);

  // Boot-time ground reference
  calibrateGround();

  Serial.printf("[ALT] BMP390 ready. Ref pressure: %.2f Pa\n", _refPa);
  return true;
}

// ────────────────────────────────────────────────────────────────
void Altimeter::calibrateGround() {
  // Average BARO_SAMPLES readings for a stable reference
  double sum = 0;
  int    count = 0;
  for (int i = 0; i < BARO_SAMPLES; i++) {
    if (bmp.performReading()) {
      sum += bmp.pressure;
      count++;
    }
    delay(20);
  }
  if (count > 0) {
    _refPa = (float)(sum / count);
    Serial.printf("[ALT] Ground calibrated: %.2f Pa\n", _refPa);
  }
}

// ────────────────────────────────────────────────────────────────
void Altimeter::update() {
  if (!bmp.performReading()) return;

  _pressurePa = bmp.pressure;
  _tempC      = bmp.temperature;

  /*
   * Barometric altitude formula (hypsometric):
   *   h = (T0/L) * [1 - (P/P0)^(R*L/g*M)]
   * Simplified for small altitude differences:
   *   h ≈ (P0 - P) / (ρ * g)  where ρ ≈ 1.225 kg/m³
   *
   * Using the international barometric formula for better accuracy:
   *   h = (T_K / 0.0065) * (1 - (P/P0)^(1/5.257))
   */
  float T_K  = _tempC + 273.15f;
  float ratio = _pressurePa / _refPa;
  _altM = (T_K / 0.0065f) * (1.0f - powf(ratio, 1.0f / 5.2558f));
}

// ────────────────────────────────────────────────────────────────
void Altimeter::registerFloor(uint8_t floorIndex) {
  // Called via serial command "REGFLOOR <n>" to set the height
  // of floor n to the current altitude reading.
  Serial.printf("[ALT] Registering floor %d at %.3f m\n", floorIndex, _altM);
  // Actual storage is handled by positioning.setFloorHeight()
  // and storage.saveConfig() — altimeter just reports current altitude.
}

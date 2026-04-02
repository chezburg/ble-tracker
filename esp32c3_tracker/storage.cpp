/*
 * storage.cpp
 * NVS-backed config + circular-buffer flash log using
 * the Arduino ESP32 Preferences library.
 */

#include "storage.h"
#include <Preferences.h>

Storage storage;
static Preferences prefs;

// Simple in-RAM circular buffer backed to NVS periodically
static LogEntry logBuffer[LOG_MAX_ENTRIES];

// ────────────────────────────────────────────────────────────────
void Storage::begin() {
  prefs.begin(NVS_NS_CONFIG, false);

  _logHead  = 0;  // RAM-only log
  _logCount = 0;  //RAM-only log

  Serial.printf("[NVS] Storage ready. (RAM-only log buffer initialized)\n");
}

// ────────────────────────────────────────────────────────────────
void Storage::loadConfig(TrackerConfig& cfg) {
  // WiFi
  String ssid = prefs.getString("wifiSsid", DEFAULT_WIFI_SSID);
  String pass = prefs.getString("wifiPass",  DEFAULT_WIFI_PASS);
  strlcpy(cfg.wifiSsid, ssid.c_str(), sizeof(cfg.wifiSsid));
  strlcpy(cfg.wifiPass, pass.c_str(), sizeof(cfg.wifiPass));

  // Base coords
  cfg.baseCoords[0].x = prefs.getFloat("b0x", 0.0f);
  cfg.baseCoords[0].y = prefs.getFloat("b0y", 0.0f);
  cfg.baseCoords[1].x = prefs.getFloat("b1x", 3.0f);
  cfg.baseCoords[1].y = prefs.getFloat("b1y", 0.0f);
  cfg.baseCoords[2].x = prefs.getFloat("b2x", 1.5f);
  cfg.baseCoords[2].y = prefs.getFloat("b2y", 2.6f);

  // Floor heights
  cfg.floorCount = prefs.getUChar("floorCount", 1);
  for (int i = 0; i < MAX_FLOORS; i++) {
    char key[12];
    snprintf(key, sizeof(key), "fh%d", i);
    cfg.floorHeights[i] = prefs.getFloat(key, (float)i * 3.0f);
  }

  // Path loss
  cfg.pathLossN = prefs.getFloat("pathLossN", DEFAULT_PATH_LOSS_N);

  // Per-base Path loss model
  for (int i = 0; i < NUM_BASES; i++) {
    char keyN[12], keyTX[12];
    snprintf(keyN, sizeof(keyN), "bn%d", i);
    snprintf(keyTX, sizeof(keyTX), "btx%d", i);
    cfg.baseN[i]     = prefs.getFloat(keyN,  DEFAULT_PATH_LOSS_N);
    cfg.baseTxRef[i] = prefs.getFloat(keyTX, (float)DEFAULT_TX_POWER_1M);
  }

  // Base offsets
  for (int i = 0; i < NUM_BASES; i++) {
    char key[12];
    snprintf(key, sizeof(key), "bo%d", i);
    cfg.baseOffsets[i] = prefs.getFloat(key, 0.0f);
  }

  Serial.println("[NVS] Config loaded.");
}

// ────────────────────────────────────────────────────────────────
void Storage::saveConfig(const TrackerConfig& cfg) {
  prefs.putString("wifiSsid",   cfg.wifiSsid);
  prefs.putString("wifiPass",   cfg.wifiPass);

  prefs.putFloat("b0x", cfg.baseCoords[0].x);
  prefs.putFloat("b0y", cfg.baseCoords[0].y);
  prefs.putFloat("b1x", cfg.baseCoords[1].x);
  prefs.putFloat("b1y", cfg.baseCoords[1].y);
  prefs.putFloat("b2x", cfg.baseCoords[2].x);
  prefs.putFloat("b2y", cfg.baseCoords[2].y);

  prefs.putUChar("floorCount", cfg.floorCount);
  for (int i = 0; i < MAX_FLOORS; i++) {
    char key[12];
    snprintf(key, sizeof(key), "fh%d", i);
    prefs.putFloat(key, cfg.floorHeights[i]);
  }

  prefs.putFloat("pathLossN", cfg.pathLossN);

  for (int i = 0; i < NUM_BASES; i++) {
    char keyN[12], keyTX[12];
    snprintf(keyN, sizeof(keyN), "bn%d", i);
    snprintf(keyTX, sizeof(keyTX), "btx%d", i);
    prefs.putFloat(keyN,  cfg.baseN[i]);
    prefs.putFloat(keyTX, cfg.baseTxRef[i]);
  }

  for (int i = 0; i < NUM_BASES; i++) {
    char key[12];
    snprintf(key, sizeof(key), "bo%d", i);
    prefs.putFloat(key, cfg.baseOffsets[i]);
  }

  Serial.println("[NVS] Config saved.");
}

// ────────────────────────────────────────────────────────────────
//  Circular log buffer (RAM-only)
// ────────────────────────────────────────────────────────────────
void Storage::appendLog(const LogEntry& entry) {
  logBuffer[_logHead] = entry;
  _logHead = (_logHead + 1) % LOG_MAX_ENTRIES;
  if (_logCount < LOG_MAX_ENTRIES) _logCount++;
}

int Storage::logCount() const {
  return _logCount;
}

LogEntry Storage::getLog(int index) const {
  // index 0 = oldest
  int realIdx = (_logHead - _logCount + index + LOG_MAX_ENTRIES) % LOG_MAX_ENTRIES;
  return logBuffer[realIdx];
}

void Storage::clearLog() {
  _logHead  = 0;
  _logCount = 0;
  Serial.println("[RAM] Log cleared.");
}

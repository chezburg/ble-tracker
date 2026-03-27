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

  // Restore log metadata
  _logHead  = prefs.getInt("logHead",  0);
  _logCount = prefs.getInt("logCount", 0);

  // Restore log entries from NVS blob
  size_t sz = sizeof(logBuffer);
  if (prefs.isKey("logBuf")) {
    prefs.getBytes("logBuf", logBuffer, sz);
  }

  Serial.printf("[NVS] Storage ready. %d log entries.\n", _logCount);
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

  Serial.println("[NVS] Config saved.");
}

// ────────────────────────────────────────────────────────────────
//  Circular log buffer
// ────────────────────────────────────────────────────────────────
void Storage::appendLog(const LogEntry& entry) {
  logBuffer[_logHead] = entry;
  _logHead = (_logHead + 1) % LOG_MAX_ENTRIES;
  if (_logCount < LOG_MAX_ENTRIES) _logCount++;

  // Persist metadata and buffer to NVS
  prefs.putInt("logHead",  _logHead);
  prefs.putInt("logCount", _logCount);
  prefs.putBytes("logBuf", logBuffer, sizeof(logBuffer));
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
  prefs.putInt("logHead",  0);
  prefs.putInt("logCount", 0);
  Serial.println("[NVS] Log cleared.");
}

/*
 * ================================================================
 *  ESP32-C3 BLE Tracker — esp32c3_tracker.ino
 *  Hardware : Seeed XIAO ESP32-C3
 *             Adafruit BMP390 (I2C: SDA=6, SCL=7)
 *             800 mAh LiPo
 *
 *  IDE      : Arduino + Espressif ESP32 BSP ≥ 2.0.14
 *  Libraries:
 *    - NimBLE-Arduino       (BLE scanner)
 *    - ESPAsyncWebServer    (HTTP + WebSocket server)
 *    - Adafruit BMP3XX      (barometric altimeter)
 *    - ArduinoJson          (JSON serialisation)
 *    - AsyncTCP             (dependency of ESPAsyncWebServer)
 *
 *  Serial commands (115200 baud):
 *    WIFI <ssid> <pass>        — set WiFi credentials & reboot
 *    BASE <id> <x> <y>         — set base coordinate
 *    REGFLOOR <n>              — register current altitude as floor n
 *    FLOORS <n>                — set total floor count
 *    PATHLOSS <n>              — set path-loss exponent
 *    OFFSET <id> <m>           — set distance offset (metres) for base id
 *    KNOWNDIST <id> <m>        — auto-calibrate offset from known distance
 *    CALPOINT <id> <m>         — calibrate multi-point path-loss model
 *    CALGROUND                 — re-calibrate barometer ground reference
 *    CLEARLOG                  — erase flash position log
 *    STATUS                    — print current state
 *    REBOOT                    — restart
 * ================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "ble_scanner.h"
#include "positioning.h"
#include "altimeter.h"
#include "storage.h"
#include "webserver.h"

// ── Task timing ─────────────────────────────────────────────────
static uint32_t lastPositionUpdate = 0;
static uint32_t lastBaroUpdate     = 0;
static uint32_t lastLogWrite       = 0;
static uint32_t lastDebugPrint     = 0;

static const uint32_t POSITION_INTERVAL_MS = 1000 / POSITION_UPDATE_HZ;
static const uint32_t BARO_INTERVAL_MS     = 1000 / BARO_UPDATE_HZ;
static const uint32_t LOG_INTERVAL_MS      = 5000;   // log every 5 s
static const uint32_t DEBUG_INTERVAL_MS    = 1000;

// ── Loaded runtime config ────────────────────────────────────────
static TrackerConfig activeCfg;

// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n\n[MAIN] ═══ ESP32-C3 BLE Tracker booting ═══");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // ── Flash / NVS ──────────────────────────────────────────────
  storage.begin();
  storage.loadConfig(activeCfg);

  // ── Altimeter ────────────────────────────────────────────────
  if (!altimeter.begin()) {
    Serial.println("[MAIN] WARNING: BMP390 not responding. Altitude disabled.");
  }

  // ── Positioning engine ───────────────────────────────────────
  positioning.begin();
  for (int i = 0; i < NUM_BASES; i++) {
    positioning.setBaseCoord(i, activeCfg.baseCoords[i].x, activeCfg.baseCoords[i].y);
  }
  for (int i = 0; i < MAX_FLOORS; i++) {
    positioning.setFloorHeight(i, activeCfg.floorHeights[i]);
  }
  positioning.setFloorCount(activeCfg.floorCount);

  // ── BLE scanner ──────────────────────────────────────────────
  bleScanner.setPathLossN(activeCfg.pathLossN);
  for (int i = 0; i < NUM_BASES; i++) {
    bleScanner.setBaseModel(i, activeCfg.baseN[i], activeCfg.baseTxRef[i]);
    bleScanner.setBaseOffset(i, activeCfg.baseOffsets[i]);
  }
  bleScanner.begin();

  // ── WiFi + Web server ────────────────────────────────────────
  webServer.begin(activeCfg.wifiSsid, activeCfg.wifiPass);

  Serial.println("[MAIN] Boot complete. System running.");
  Serial.println("[MAIN] Type 'STATUS' for current state.");
}

// ════════════════════════════════════════════════════════════════
void loop() {
  uint32_t now = millis();

  // ── BLE scanner upkeep ───────────────────────────────────────
  bleScanner.update();

  // ── Barometer ────────────────────────────────────────────────
  if (now - lastBaroUpdate >= (1000 / BARO_UPDATE_HZ)) {
    lastBaroUpdate = now;
    altimeter.update();

    // Determine floor from altitude
    // const PositionFix& fix = positioning.latest();
    // Re-use last fix's floor; detectFloor is internal to positioning
    // We call a small helper to propagate floor to altimeter
    float alt = altimeter.getAltitudeM();
    int floor = 0;
    for (int f = activeCfg.floorCount - 1; f >= 0; f--) {
      if (alt >= activeCfg.floorHeights[f] - FLOOR_HYSTERESIS_M) {
        floor = f;
        break;
      }
    }
    altimeter.setFloor(floor);
  }

  // ── Positioning ──────────────────────────────────────────────
  if (now - lastPositionUpdate >= POSITION_INTERVAL_MS) {
    lastPositionUpdate = now;
    positioning.update();

    // Copy floor from altimeter into position fix display
    // (altimeter is the authoritative Z source)
  }

  // ── Flash log ────────────────────────────────────────────────
  if (now - lastLogWrite >= LOG_INTERVAL_MS) {
    lastLogWrite = now;
    const PositionFix& fix = positioning.latest();
    if (fix.valid) {
      LogEntry entry;
      entry.ts       = now;
      entry.x        = fix.x;
      entry.y        = fix.y;
      entry.altM     = altimeter.getAltitudeM();
      entry.floor    = altimeter.getFloor();
      entry.accuracy = fix.accuracy;
      storage.appendLog(entry);

      // LED blink on log write
      digitalWrite(LED_PIN, LOW);
      delay(20);
      digitalWrite(LED_PIN, HIGH);
    }
  }

  // ── WebSocket push ───────────────────────────────────────────
  webServer.update();

  // ── USB serial debug ─────────────────────────────────────────
  if (now - lastDebugPrint >= DEBUG_INTERVAL_MS) {
    lastDebugPrint = now;
    printDebug();
  }

  // ── Serial command handler ───────────────────────────────────
  handleSerialCommands();
}

// ════════════════════════════════════════════════════════════════
//  Serial debug output
// ════════════════════════════════════════════════════════════════
void printDebug() {
  const PositionFix& fix = positioning.getAveragedFix();

  Serial.println("─────────────────────────────────────");
  Serial.printf("BLE bases: %d/%d valid\n", bleScanner.validCount(), NUM_BASES);
  for (int i = 0; i < NUM_BASES; i++) {
    const BaseReading& b = bleScanner.base(i);
    Serial.printf("  Base %d: RSSI=%.1f  Dist=%.2fm  %s\n",
                  i, b.rssiFiltered, b.distanceM,
                  b.valid ? "OK" : "STALE");
  }
  Serial.printf("Position: ");
  if (fix.valid) {
    Serial.printf("X=%.2f Y=%.2f  Acc=±%.2fm\n", fix.x, fix.y, fix.accuracy);
  } else {
    Serial.println("NO FIX");
  }
  Serial.printf("Altitude: %.3fm  Floor: %d  Pressure: %.2fPa  Temp: %.1fC\n",
                altimeter.getAltitudeM(), altimeter.getFloor(),
                altimeter.getPressurePa(), altimeter.getTemperatureC());
  Serial.printf("Log entries: %d/%d\n", storage.logCount(), LOG_MAX_ENTRIES);
}

// ════════════════════════════════════════════════════════════════
//  Serial command parser
// ════════════════════════════════════════════════════════════════
void handleSerialCommands() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  Serial.print("[CMD] ");
  Serial.println(line);

  if (line.startsWith("WIFI ")) {
    // WIFI <ssid>|<pass>  — use pipe separator for SSIDs with spaces
    int sp = line.indexOf('|', 5);
    if (sp < 0) {
      // Fallback to legacy space separator if no pipe found
      sp = line.indexOf(' ', 5);
    }

    if (sp < 0) {
      Serial.println("Usage: WIFI <ssid>|<pass>  (Use | if SSID has spaces)");
      return;
    }

    String ssid = line.substring(5, sp);
    String pass = line.substring(sp + 1);
    ssid.trim();
    pass.trim();

    strlcpy(activeCfg.wifiSsid, ssid.c_str(), sizeof(activeCfg.wifiSsid));
    strlcpy(activeCfg.wifiPass, pass.c_str(), sizeof(activeCfg.wifiPass));
    Serial.printf("Setting WiFi: SSID='%s' PASS='%s'\n", ssid.c_str(), pass.c_str());
    storage.saveConfig(activeCfg);
    Serial.println("WiFi credentials saved. Rebooting...");
    delay(500); ESP.restart();
  }
  else if (line.startsWith("BASE ")) {
    // BASE <id> <x> <y>
    int id; float x, y;
    if (sscanf(line.c_str() + 5, "%d %f %f", &id, &x, &y) == 3 && id < NUM_BASES) {
      activeCfg.baseCoords[id] = {x, y};
      positioning.setBaseCoord(id, x, y);
      storage.saveConfig(activeCfg);
      Serial.printf("Base %d set to (%.2f, %.2f)\n", id, x, y);
    } else {
      Serial.println("Usage: BASE <id 0-2> <x> <y>");
    }
  }
  else if (line.startsWith("REGFLOOR ")) {
    int f = line.substring(9).toInt();
    if (f >= 0 && f < MAX_FLOORS) {
      float h = altimeter.getAltitudeM();
      activeCfg.floorHeights[f] = h;
      activeCfg.floorCount = max((int)activeCfg.floorCount, f + 1);
      positioning.setFloorHeight(f, h);
      positioning.setFloorCount(activeCfg.floorCount);
      storage.saveConfig(activeCfg);
      Serial.printf("Floor %d registered at %.3f m\n", f, h);
    }
  }
  else if (line.startsWith("FLOORS ")) {
    int n = line.substring(7).toInt();
    if (n > 0 && n <= MAX_FLOORS) {
      activeCfg.floorCount = n;
      positioning.setFloorCount(n);
      storage.saveConfig(activeCfg);
      Serial.printf("Floor count set to %d\n", n);
    }
  }
  else if (line.startsWith("PATHLOSS ")) {
    float n = line.substring(9).toFloat();
    if (n >= 1.5f && n <= 5.0f) {
      activeCfg.pathLossN = n;
      bleScanner.setPathLossN(n);
      storage.saveConfig(activeCfg);
      Serial.printf("Path-loss exponent set to %.2f\n", n);
    }
  }
  else if (line.startsWith("OFFSET ")) {
    // OFFSET <id> <meters>
    int id; float m;
    if (sscanf(line.c_str() + 7, "%d %f", &id, &m) == 2 && id < NUM_BASES) {
      activeCfg.baseOffsets[id] = m;
      bleScanner.setBaseOffset(id, m);
      storage.saveConfig(activeCfg);
      Serial.printf("Base %d distance offset set to %.2f m\n", id, m);
    } else {
      Serial.println("Usage: OFFSET <id 0-2> <meters>");
    }
  }
  else if (line.startsWith("KNOWNDIST ")) {
    // KNOWNDIST <id> <actual_distance>
    int id; float actual;
    if (sscanf(line.c_str() + 10, "%d %f", &id, &actual) == 2 && id < NUM_BASES) {
      const BaseReading& b = bleScanner.base(id);
      if (!b.valid) {
        Serial.printf("Base %d is not visible (stale). Can't calibrate.\n", id);
      } else {
        float currentDist = b.distanceM;
        float currentOffset = bleScanner.getBaseOffset(id);
        float rawDist = currentDist - currentOffset; // Distance WITHOUT the current offset

        float newOffset = actual - rawDist;
        activeCfg.baseOffsets[id] = newOffset;
        bleScanner.setBaseOffset(id, newOffset);
        storage.saveConfig(activeCfg);
        Serial.printf("Base %d recalibrated: raw=%.2fm, new offset=%.2fm -> reported=%.2fm\n",
                      id, rawDist, newOffset, actual);
      }
    } else {
      Serial.println("Usage: KNOWNDIST <id 0-2> <actual_meters>");
    }
  }
  else if (line.startsWith("CALPOINT ")) {
    // CALPOINT <id> <actual_distance>
    int id; float dist;
    if (sscanf(line.c_str() + 9, "%d %f", &id, &dist) == 2 && id < NUM_BASES) {
      float n, txRef;
      if (bleScanner.addCalPoint(id, dist, n, txRef)) {
        activeCfg.baseN[id] = n;
        activeCfg.baseTxRef[id] = txRef;
        storage.saveConfig(activeCfg);
        Serial.printf("Base %d model updated: N=%.2f, TXRef=%.1f\n", id, n, txRef);
      } else {
        Serial.println("Error: Base not visible or regression failure.");
      }
    } else {
      Serial.println("Usage: CALPOINT <id 0-2> <actual_meters>");
    }
  }
  else if (line == "CALGROUND") {
    altimeter.calibrateGround();
    Serial.println("Barometer ground reference recalibrated.");
  }
  else if (line == "CLEARLOG") {
    storage.clearLog();
  }
  else if (line == "STATUS") {
    printDebug();
    Serial.printf("WiFi SSID : %s\n", activeCfg.wifiSsid);
    Serial.printf("IP        : %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Path loss n: %.2f\n", activeCfg.pathLossN);
    Serial.printf("Floors    : %d\n", activeCfg.floorCount);
    for (int i = 0; i < activeCfg.floorCount; i++)
      Serial.printf("  Floor %d: %.2f m\n", i, activeCfg.floorHeights[i]);
    for (int i = 0; i < NUM_BASES; i++) {
      Serial.printf("  Base %d: (%.2f, %.2f)  Offset: %.2f m  N: %.2f  TXRef: %.1f\n", i,
                    activeCfg.baseCoords[i].x, activeCfg.baseCoords[i].y,
                    activeCfg.baseOffsets[i], activeCfg.baseN[i], activeCfg.baseTxRef[i]);
    }
  }
  else if (line == "REBOOT") {
    Serial.println("Rebooting...");
    delay(300); ESP.restart();
  }
  else {
    Serial.println("Unknown command. Commands: WIFI, BASE, REGFLOOR, FLOORS, PATHLOSS, OFFSET, KNOWNDIST, CALGROUND, CLEARLOG, STATUS, REBOOT");
  }
}

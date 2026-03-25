/*
 * ================================================================
 *  BLE Base Station Firmware — nrf52840_base.ino
 *  Hardware : Seeed XIAO nRF52840
 *  IDE      : Arduino + Adafruit nRF52 BSP
 *  Libraries: ArduinoBLE, LittleFS (via InternalFileSystem), ArduinoJson
 *
 *  First-boot provisioning
 *  ───────────────────────
 *  If no ID has ever been saved to flash the device enters
 *  PROVISIONING MODE on every power-on:
 *    • LED pulses slowly (breathing effect) instead of blinking
 *    • Serial prints a prominent prompt
 *    • BLE does NOT advertise yet — tracker won't see this base
 *    • Waiting for: SETID <0|1|2>  over USB serial (115200 baud)
 *  Once an ID is saved the device reboots straight into normal
 *  advertising mode on every subsequent power-on — no serial
 *  connection needed ever again.
 *
 *  Serial commands (always available, not just first boot):
 *    SETID <0|1|2>        — assign / reassign Base ID, saves & reboots
 *    SETTXPOWER <dBm>     — e.g.  SETTXPOWER -65
 *    SETINTERVAL <ms>     — advertisement interval, e.g. SETINTERVAL 200
 *    STATUS               — print current config
 *    RESET                — erase config and re-enter provisioning mode
 *
 *  Flash layout (LittleFS):
 *    /base_config.json
 *      { "id":0, "idSet":true, "txPower":-59, "advIntervalMs":100 }
 * ================================================================
 */

#include <ArduinoBLE.h>
#include <InternalFileSystem.h>
#include <ArduinoJson.h>
#include "config.h"

// ── Runtime config ───────────────────────────────────────────────
struct BaseConfig {
  uint8_t  baseId        = 0;
  bool     idSet         = false;      // ← key flag: has ID ever been saved?
  int8_t   txPower1m     = DEFAULT_TX_POWER_1M;
  uint16_t advIntervalMs = ADV_INTERVAL_MS;
};

static BaseConfig cfg;

// ── BLE objects ──────────────────────────────────────────────────
BLEService             configService  (CONFIG_SERVICE_UUID);
BLEByteCharacteristic  charBaseId     (CHAR_BASE_ID_UUID,    BLERead | BLEWrite);
BLEByteCharacteristic  charTxPower    (CHAR_TX_POWER_UUID,   BLERead | BLEWrite);
BLEShortCharacteristic charAdvInterval(CHAR_ADV_INTERVAL_UUID, BLERead | BLEWrite);

// ── Forward declarations ─────────────────────────────────────────
void loadConfig();
void saveConfig();
void buildAdvertisement();
void startAdvertising();
void runProvisioningMode();
void handleSerialCommands();
void printStatus();
void ledBreath(uint32_t periodMs);   // provisioning LED animation
void onBaseIdWritten     (BLEDevice, BLECharacteristic);
void onTxPowerWritten    (BLEDevice, BLECharacteristic);
void onAdvIntervalWritten(BLEDevice, BLECharacteristic);

// ════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(600);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);   // active-low on XIAO: HIGH = off

  InternalFS.begin();
  loadConfig();

  if (!cfg.idSet) {
    // ── PROVISIONING MODE ─────────────────────────────────────
    runProvisioningMode();   // blocks until SETID received, then reboots
    // execution never reaches here
  }

  // ── NORMAL BOOT ───────────────────────────────────────────────
  Serial.printf("\n[BASE] ID=%d  TxPwr=%ddBm  Interval=%dms\n",
                cfg.baseId, cfg.txPower1m, cfg.advIntervalMs);

  if (!BLE.begin()) {
    Serial.println("[BASE] BLE init failed — halting");
    while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }

  String devName = String(BLE_DEVICE_NAME_PREFIX) + "_" + String(cfg.baseId);
  BLE.setLocalName(devName.c_str());
  BLE.setDeviceName(devName.c_str());

  // GATT config service (allows runtime updates even after provisioning)
  charBaseId.setValue(cfg.baseId);
  charBaseId.setEventHandler(BLEWritten, onBaseIdWritten);
  charTxPower.setValue((uint8_t)cfg.txPower1m);
  charTxPower.setEventHandler(BLEWritten, onTxPowerWritten);
  charAdvInterval.setValue(cfg.advIntervalMs);
  charAdvInterval.setEventHandler(BLEWritten, onAdvIntervalWritten);

  configService.addCharacteristic(charBaseId);
  configService.addCharacteristic(charTxPower);
  configService.addCharacteristic(charAdvInterval);
  BLE.addService(configService);

  buildAdvertisement();
  BLE.advertise();

  Serial.printf("[BASE] Advertising as \"%s\"\n", devName.c_str());
}

// ════════════════════════════════════════════════════════════════
void loop() {
  // Handle any incoming BLE central connection (for GATT config)
  BLEDevice central = BLE.central();
  if (central) {
    Serial.printf("[BASE] Central connected: %s\n", central.address().c_str());
    while (central.connected()) {
      BLE.poll();
      handleSerialCommands();
      delay(10);
    }
    Serial.println("[BASE] Central disconnected — re-advertising.");
    BLE.advertise();
  }

  // Blink LED each advertisement cycle to show liveness
  static uint32_t lastBlink = 0;
  if (millis() - lastBlink >= cfg.advIntervalMs) {
    lastBlink = millis();
    digitalWrite(LED_PIN, LOW);
    delay(LED_BLINK_MS);
    digitalWrite(LED_PIN, HIGH);
  }

  handleSerialCommands();
}

// ════════════════════════════════════════════════════════════════
//  PROVISIONING MODE — blocks until valid SETID received
// ════════════════════════════════════════════════════════════════
void runProvisioningMode() {
  Serial.println("\n");
  Serial.println("╔══════════════════════════════════════════╗");
  Serial.println("║   BLE BASE — FIRST BOOT SETUP REQUIRED   ║");
  Serial.println("╠══════════════════════════════════════════╣");
  Serial.println("║  This base has no ID assigned yet.       ║");
  Serial.println("║  BLE advertising is PAUSED.              ║");
  Serial.println("║                                          ║");
  Serial.println("║  Send:  SETID 0   (or 1, or 2)           ║");
  Serial.println("║  via this serial port (115200 baud)      ║");
  Serial.println("║                                          ║");
  Serial.println("║  The ID is saved permanently to flash.   ║");
  Serial.println("║  No setup needed on future power cycles. ║");
  Serial.println("╚══════════════════════════════════════════╝\n");

  uint32_t lastReminder = millis();

  while (true) {
    // Breathing LED — slow pulse to distinguish from normal blink
    ledBreath(2000);

    // Periodic reminder on serial
    if (millis() - lastReminder >= 10000) {
      lastReminder = millis();
      Serial.println("[BASE] Waiting for: SETID 0  (or 1 or 2)");
    }

    // Check for serial input
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.startsWith("SETID ") || line.startsWith("setid ")) {
        int id = line.substring(6).toInt();
        if (id >= 0 && id <= 2) {
          cfg.baseId = (uint8_t)id;
          cfg.idSet  = true;
          saveConfig();

          Serial.println();
          Serial.printf("  ✓ Base ID set to %d — saved to flash.\n", id);
          Serial.println("  Rebooting into advertising mode...\n");

          // Fast blink to confirm
          for (int i = 0; i < 6; i++) {
            digitalWrite(LED_PIN, LOW);  delay(80);
            digitalWrite(LED_PIN, HIGH); delay(80);
          }
          delay(300);

          // Reboot — next power-on skips provisioning entirely
          NVIC_SystemReset();
        } else {
          Serial.println("  ✗ Invalid ID. Use 0, 1, or 2.");
        }
      } else if (line == "STATUS" || line == "status") {
        Serial.println("[BASE] Status: UNPROVISIONED — no ID set yet.");
      } else if (line.length() > 0) {
        Serial.printf("  Unknown command \"%s\". Send: SETID 0\n", line.c_str());
      }
    }
  }
}

// ════════════════════════════════════════════════════════════════
//  Serial command handler (normal operating mode)
// ════════════════════════════════════════════════════════════════
void handleSerialCommands() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("SETID ")) {
    int id = line.substring(6).toInt();
    if (id >= 0 && id <= 2) {
      cfg.baseId = (uint8_t)id;
      cfg.idSet  = true;
      saveConfig();
      Serial.printf("[BASE] ID changed to %d — rebooting.\n", id);
      delay(300);
      NVIC_SystemReset();
    } else {
      Serial.println("[BASE] Invalid ID. Use 0, 1, or 2.");
    }
  }
  else if (line.startsWith("SETTXPOWER ")) {
    int v = line.substring(11).toInt();
    if (v >= -100 && v <= 0) {
      cfg.txPower1m = (int8_t)v;
      saveConfig();
      buildAdvertisement();
      Serial.printf("[BASE] TX power set to %d dBm\n", v);
    } else {
      Serial.println("[BASE] Expected negative dBm value, e.g. SETTXPOWER -65");
    }
  }
  else if (line.startsWith("SETINTERVAL ")) {
    int v = line.substring(12).toInt();
    if (v >= 20 && v <= 10000) {
      cfg.advIntervalMs = (uint16_t)v;
      saveConfig();
      BLE.stopAdvertise();
      buildAdvertisement();
      BLE.advertise();
      Serial.printf("[BASE] Adv interval set to %d ms\n", v);
    } else {
      Serial.println("[BASE] Interval must be 20–10000 ms");
    }
  }
  else if (line == "STATUS") {
    printStatus();
  }
  else if (line == "RESET") {
    Serial.println("[BASE] Erasing config — will re-enter provisioning on next boot.");
    InternalFS.remove(SETTINGS_FILE);
    delay(300);
    NVIC_SystemReset();
  }
  else {
    Serial.println("[BASE] Commands: SETID, SETTXPOWER, SETINTERVAL, STATUS, RESET");
  }
}

// ════════════════════════════════════════════════════════════════
//  Advertisement builder
// ════════════════════════════════════════════════════════════════
void buildAdvertisement() {
  /*
   * Manufacturer-specific data (2 bytes after 2-byte company ID):
   *   Byte 0 : Base ID      (uint8)
   *   Byte 1 : TX power 1m  (int8 cast to uint8)
   */
  BLEAdvertisingData adData;
  uint8_t mfgData[2] = { cfg.baseId, (uint8_t)cfg.txPower1m };
  adData.setManufacturerData(0x0059, mfgData, sizeof(mfgData));
  adData.setFlags(BLEFlagsBREDRNotSupported | BLEFlagsGeneralDiscoverable);
  BLE.setAdvertisingData(adData);

  BLEAdvertisingData srData;
  String devName = String(BLE_DEVICE_NAME_PREFIX) + "_" + String(cfg.baseId);
  srData.setLocalName(devName.c_str());
  BLE.setScanResponseData(srData);

  BLE.setAdvertisingInterval(cfg.advIntervalMs * 8 / 5);
}

// ════════════════════════════════════════════════════════════════
//  LED breathing animation for provisioning mode
//  (Uses analogWrite — XIAO nRF52840 supports PWM on LED pin)
// ════════════════════════════════════════════════════════════════
void ledBreath(uint32_t periodMs) {
  // Call repeatedly from a while loop — each call advances one frame
  static uint32_t startMs = 0;
  if (startMs == 0) startMs = millis();

  uint32_t elapsed  = millis() - startMs;
  float    phase    = (float)(elapsed % periodMs) / (float)periodMs;
  // Sine wave 0→1→0, gamma corrected for perceived brightness
  float    bright   = sinf(phase * PI);
  bright = bright * bright;                        // gamma ~2
  uint8_t  pwm      = (uint8_t)(bright * 255);
  analogWrite(LED_PIN, 255 - pwm);                 // active-low
  delay(16);                                        // ~60 fps
}

// ════════════════════════════════════════════════════════════════
//  GATT write handlers
// ════════════════════════════════════════════════════════════════
void onBaseIdWritten(BLEDevice central, BLECharacteristic c) {
  uint8_t v = charBaseId.value();
  if (v <= 2) {
    cfg.baseId = v;
    cfg.idSet  = true;
    saveConfig();
    buildAdvertisement();
    Serial.printf("[BASE] ID updated via BLE to %d\n", cfg.baseId);
  }
}

void onTxPowerWritten(BLEDevice central, BLECharacteristic c) {
  cfg.txPower1m = (int8_t)charTxPower.value();
  saveConfig();
  buildAdvertisement();
  Serial.printf("[BASE] TX power updated via BLE to %d dBm\n", cfg.txPower1m);
}

void onAdvIntervalWritten(BLEDevice central, BLECharacteristic c) {
  uint16_t v = charAdvInterval.value();
  if (v >= 20 && v <= 10000) {
    cfg.advIntervalMs = v;
    saveConfig();
    BLE.stopAdvertise();
    buildAdvertisement();
    BLE.advertise();
    Serial.printf("[BASE] Adv interval updated via BLE to %d ms\n", v);
  }
}

// ════════════════════════════════════════════════════════════════
//  Flash persistence
// ════════════════════════════════════════════════════════════════
void loadConfig() {
  File f = InternalFS.open(SETTINGS_FILE, FILE_O_READ);
  if (!f) {
    Serial.println("[BASE] No config file — entering provisioning.");
    cfg.idSet = false;
    return;
  }

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();

  if (err) {
    Serial.println("[BASE] Config corrupt — entering provisioning.");
    cfg.idSet = false;
    return;
  }

  cfg.idSet         = doc["idSet"]         | false;
  cfg.baseId        = doc["id"]            | (uint8_t)0;
  cfg.txPower1m     = doc["txPower"]       | (int8_t)DEFAULT_TX_POWER_1M;
  cfg.advIntervalMs = doc["advIntervalMs"] | (uint16_t)ADV_INTERVAL_MS;

  if (!cfg.idSet) {
    Serial.println("[BASE] ID not yet provisioned — entering provisioning.");
  } else {
    Serial.printf("[BASE] Config loaded: id=%d txPower=%d interval=%d\n",
                  cfg.baseId, cfg.txPower1m, cfg.advIntervalMs);
  }
}

void saveConfig() {
  InternalFS.remove(SETTINGS_FILE);
  File f = InternalFS.open(SETTINGS_FILE, FILE_O_WRITE);
  if (!f) { Serial.println("[BASE] ERROR: could not write config!"); return; }

  StaticJsonDocument<128> doc;
  doc["idSet"]        = cfg.idSet;
  doc["id"]           = cfg.baseId;
  doc["txPower"]      = cfg.txPower1m;
  doc["advIntervalMs"]= cfg.advIntervalMs;

  serializeJson(doc, f);
  f.close();
  Serial.println("[BASE] Config saved.");
}

void printStatus() {
  Serial.println("\n── Base Station Status ──────────────");
  Serial.printf("  ID          : %s\n", cfg.idSet ? String(cfg.baseId).c_str() : "NOT SET");
  Serial.printf("  TX Power    : %d dBm\n", cfg.txPower1m);
  Serial.printf("  Adv Interval: %d ms\n", cfg.advIntervalMs);
  Serial.println("─────────────────────────────────────\n");
}

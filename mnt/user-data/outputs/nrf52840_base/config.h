#pragma once

// =============================================================
//  BLE Base Station — config.h
//  Flash-persisted values are loaded at boot from LittleFS.
//  These constants are compile-time defaults only.
// =============================================================

// --- Identity ---
// Each base must have a unique ID: 0, 1, or 2.
// Override at runtime via the Config BLE characteristic.
#define DEFAULT_BASE_ID          0

// --- BLE ---
#define BLE_DEVICE_NAME_PREFIX   "BLEBase"          // final name e.g. "BLEBase_0"
#define ADV_INTERVAL_MS          100                // advertisement interval (ms)
#define ADV_INTERVAL_BLE         (ADV_INTERVAL_MS * 8 / 5)  // units of 0.625 ms

// TX Power at 1 metre (dBm) — calibrate per unit with a ruler.
// Stored in flash and included in every advertisement payload.
#define DEFAULT_TX_POWER_1M      (-59)              // typical value; calibrate!

// --- GATT UUIDs (128-bit, little-endian) ---
// Config Service
#define CONFIG_SERVICE_UUID      "12345678-1234-1234-1234-123456789abc"
// Characteristics
#define CHAR_BASE_ID_UUID        "12345678-1234-1234-1234-123456789ab0"
#define CHAR_TX_POWER_UUID       "12345678-1234-1234-1234-123456789ab1"
#define CHAR_ADV_INTERVAL_UUID   "12345678-1234-1234-1234-123456789ab2"

// --- Storage ---
#define SETTINGS_FILE            "/base_config.json"

// --- Hardware ---
#define LED_PIN                  LED_BUILTIN        // blinks on each advertisement
#define LED_BLINK_MS             20

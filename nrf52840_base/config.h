#pragma once

// =================================================================
//  BLE Base Station — config.h
// =================================================================

// ── BLE identity ────────────────────────────────────────────────
#define BLE_DEVICE_NAME_PREFIX   "BLEBase"
#define DEFAULT_TX_POWER_1M      -59         // rssi @ 1m calibration
#define ADV_INTERVAL_MS          100         // normally 100ms for tracking
#define ADV_INTERVAL_BLE         160         // 100ms in 0.625ms units

// ── Flash persistence ──────────────────────────────────────────
#define SETTINGS_FILE            "/base_config.json"

// ── Hardware ────────────────────────────────────────────────────
#define LED_PIN                  LED_RED     // XIAO nRF52840 Red LED
#define LED_BLINK_MS             20

// ── GATT Service UUIDs — Match those in esp32c3_tracker ────────
#define CONFIG_SERVICE_UUID      "ABCDEF01-1234-1234-1234-123456789abc"
#define CHAR_BASE_ID_UUID        "ABCDEF01-1234-1234-1234-123456789ab2"
#define CHAR_TX_POWER_UUID       "ABCDEF01-1234-1234-1234-123456789ab4"
#define CHAR_ADV_INTERVAL_UUID   "ABCDEF01-1234-1234-1234-123456789ab5"

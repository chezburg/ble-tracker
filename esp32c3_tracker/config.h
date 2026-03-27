#pragma once

// =============================================================
//  ESP32-C3 Tracker — config.h
//  Compile-time defaults. Runtime values are stored in NVS.
// =============================================================

// ── WiFi (overridden via serial or BLE config characteristic) ──
#define DEFAULT_WIFI_SSID        "YourSSID"
#define DEFAULT_WIFI_PASS        "YourPassword"

// ── BLE scanner ────────────────────────────────────────────────
#define NUM_BASES                3
#define BASE_MFG_CID             0x0059      // Nordic Semiconductor CID in adv payload
#define BASE_NAME_PREFIX         "BLEBase"
#define BLE_SCAN_INTERVAL        100         // ms — must be >= adv interval on bases
#define BLE_SCAN_WINDOW          80          // ms — active listen window per interval
#define RSSI_HISTORY_LEN         10          // samples in rolling average
#define RSSI_STALE_MS            2000        // drop base if no packet for this long
#define MIN_BASES_FOR_FIX        3           // need all 3 for 2D trilateration

// ── Path-loss model  d = 10^((TxPwr - RSSI) / (10*n)) ─────────
#define DEFAULT_PATH_LOSS_N      2.5f        // 2.0 = free space, 3-4 = indoors
                                             // calibrate by walking known distances

// ── Kalman filter (per-base RSSI smoothing) ────────────────────
#define KALMAN_Q                 0.05f       // process noise  (lower = smoother)
#define KALMAN_R                 3.0f        // measurement noise (higher = smoother)

// ── Positioning ────────────────────────────────────────────────
#define POSITION_UPDATE_HZ       2           // trilateration runs at this rate
#define POSITION_SMOOTH_ALPHA    0.3f        // EMA alpha for output XY (0=still,1=raw)

// ── Altimeter / floor detection ────────────────────────────────
#define BMP390_I2C_ADDR          0x77        // SDO high = 0x77, low = 0x76
#define BARO_SAMPLES             16          // oversampling count for averaging
#define BARO_UPDATE_HZ           4
#define FLOOR_HYSTERESIS_M       0.4f        // dead-band to prevent floor flickering
#define MAX_FLOORS               20

// ── Flash log ──────────────────────────────────────────────────
#define LOG_MAX_ENTRIES          1000
#define LOG_NVS_NAMESPACE        "tracker_log"

// ── NVS namespaces ─────────────────────────────────────────────
#define NVS_NS_CONFIG            "tracker_cfg"

// ── Web server ─────────────────────────────────────────────────
#define WEB_SERVER_PORT          80
#define WS_PORT                  81
#define WS_PUSH_INTERVAL_MS      500         // 2 Hz

// ── Hardware ───────────────────────────────────────────────────
#define LED_PIN                  10          // XIAO C3 LED
#define SDA_PIN                  6
#define SCL_PIN                  7

// ── GATT Config Service (same UUIDs as bases, separate chars) ──
#define TRACKER_SERVICE_UUID     "ABCDEF01-1234-1234-1234-123456789abc"
#define CHAR_WIFI_SSID_UUID      "ABCDEF01-1234-1234-1234-123456789ab0"
#define CHAR_WIFI_PASS_UUID      "ABCDEF01-1234-1234-1234-123456789ab1"
#define CHAR_BASE_COORDS_UUID    "ABCDEF01-1234-1234-1234-123456789ab2"  // JSON string
#define CHAR_FLOOR_HEIGHTS_UUID  "ABCDEF01-1234-1234-1234-123456789ab3"  // JSON string
#define CHAR_PATH_LOSS_N_UUID    "ABCDEF01-1234-1234-1234-123456789ab4"

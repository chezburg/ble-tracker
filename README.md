# BLE Indoor Tracking System

**Hardware:** 3× Seeed XIAO nRF52840 (bases) + Seeed XIAO ESP32-C3 + Adafruit BMP390 + 800 mAh LiPo

---

## Architecture

```
[NRF Base 0]   [NRF Base 1]   [NRF Base 2]
     │               │               │
     └───────────────┴───────────────┘
       BLE advertisements every 100 ms
       Payload: [ BaseID | TxPower@1m ]
                        │
                 [ESP32-C3 Tracker]
                 ├─ NimBLE scanner → RSSI per base
                 ├─ Kalman filter  → smoothed RSSI
                 ├─ Path-loss model→ distance (m)
                 ├─ WLS trilateration → XY position
                 ├─ BMP390 → relative altitude → floor
                 ├─ NVS flash → config + position log
                 └─ WiFi → WebSocket → Browser dashboard
```

---

## Repository Layout

```
nrf52840_base/
  nrf52840_base.ino   ← flash to all 3 bases
  config.h

esp32c3_tracker/
  esp32c3_tracker.ino ← main sketch
  config.h
  ble_scanner.h/.cpp
  positioning.h/.cpp
  altimeter.h/.cpp
  storage.h/.cpp
  webserver.h/.cpp
```

---

## 1. Base Station Setup (nRF52840)

### Arduino IDE Board Setup
1. Add board URL: `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`
2. Board: **Seeed nRF52 Boards → Seeed XIAO nRF52840**

### Libraries Required
| Library | Install via |
|---|---|
| ArduinoBLE | Arduino Library Manager |
| ArduinoJson | Arduino Library Manager |
| InternalFileSystem | Included with Seeed nRF52 BSP |

### Flashing All 3 Bases
Each base runs **identical firmware**. The Base ID (0, 1, 2) is set at runtime via BLE.

1. Flash `nrf52840_base.ino` to all 3 units
2. Use a BLE scanner app (e.g. **nRF Connect**) to connect to each base
3. Find the **Config Service** (`12345678-...`)
4. Write `0x00`, `0x01`, `0x02` to the **Base ID characteristic** on each unit respectively
5. Write the **TX Power characteristic** for your calibrated RSSI-at-1m value (default: `-59` = `0xC5` as signed byte)

### TX Power Calibration (Important for accuracy)
1. Hold your phone 1 metre from the base
2. Read RSSI in nRF Connect
3. Write that value to the TX Power characteristic
4. Repeat for each base — values may differ slightly per unit

---

## 2. Tracker Setup (ESP32-C3)

### Arduino IDE Board Setup
1. Add board URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Board: **ESP32 Arduino → XIAO_ESP32C3**

### Libraries Required
| Library | Install via |
|---|---|
| NimBLE-Arduino | Arduino Library Manager |
| ESPAsyncWebServer | GitHub: me-no-dev/ESPAsyncWebServer |
| AsyncTCP | GitHub: me-no-dev/AsyncTCP |
| Adafruit BMP3XX | Arduino Library Manager |
| ArduinoJson | Arduino Library Manager |

### Wiring (BMP390 → ESP32-C3)
```
BMP390  →  XIAO ESP32-C3
VIN     →  3.3V
GND     →  GND
SDA     →  D4 (GPIO6)
SCL     →  D5 (GPIO7)
SDO     →  3.3V (sets I2C address to 0x77)
```

### First-Time Configuration
Set WiFi credentials via Serial (115200 baud) **before** first boot, or edit `config.h`:

```
WIFI MyNetworkName MyPassword
```

The device will reboot and connect. The dashboard IP is printed to Serial.

---

## 3. Coordinate System Setup

The tracker uses a simple 2D Cartesian coordinate system in **metres**.

```
         Y
         │
     B2  │
    ╱    │
   ╱     │
B0───────────── X
         B1
```

### Setting Base Coordinates

#### Via Serial
```
BASE 0 0.0 0.0
BASE 1 5.0 0.0
BASE 2 2.5 4.3
```

#### Via Web Dashboard
In the **Configuration** panel: type `id,x,y` (e.g. `1,5.0,0.0`) and click **Set Base Coord**.

> **Tip:** Measure physical distances between base mounting points with a tape measure. Place bases in a triangle — wider spacing = better accuracy. Minimum recommended separation: 2 m. Optimal: 3–8 m.

---

## 4. Floor Height Calibration

This must be done once per environment. The BMP390 measures **relative pressure change from boot**.

### Procedure
1. Start the tracker on Floor 0 (ground floor / reference floor)
2. Wait 30 seconds for the barometer to stabilise
3. Send: `REGFLOOR 0` — this registers floor 0 at 0.0 m (boot reference)
4. Walk to Floor 1
5. Wait 10 seconds
6. Send: `REGFLOOR 1`
7. Repeat for each floor
8. Set total floor count: `FLOORS 3`

Alternatively, use the **Register Current Alt as Floor** button in the dashboard.

Floor heights are persisted to NVS and survive reboots.

---

## 5. Path-Loss Calibration (Improves XY accuracy)

The default path-loss exponent `n = 2.5` works for typical office environments. To calibrate:

1. Place the tracker at known positions (measured with tape measure)
2. Observe the reported position on the dashboard
3. Adjust `n` via: `PATHLOSS 2.8` (increase `n` if distances read too short; decrease if too long)
4. Typical ranges: `n = 2.0` (open space) → `n = 3.5` (heavy obstructions)

---

## 6. Web Dashboard

After connecting to WiFi, open a browser on the same network and navigate to the IP shown in Serial output (e.g. `http://192.168.1.42`).

### Dashboard Features
| Panel | Description |
|---|---|
| Position Map | Live XY canvas with base markers, distance circles, position dot, trail |
| Position | X, Y coordinates in metres, altitude, accuracy estimate, floor badge |
| Base Stations | Per-base RSSI, estimated distance, signal quality bar |
| Configuration | Set path-loss `n`, base coordinates, register floor heights |
| Display | Toggle trail and auto-scale |

### Position Log Download
Navigate to `http://<ip>/log` for a JSON array of all logged fixes.

---

## 7. Serial Command Reference

| Command | Description |
|---|---|
| `WIFI <ssid> <pass>` | Set WiFi credentials (saves + reboots) |
| `BASE <id> <x> <y>` | Set base station coordinate |
| `REGFLOOR <n>` | Register current altitude as floor n boundary |
| `FLOORS <n>` | Set total floor count |
| `PATHLOSS <n>` | Set path-loss exponent (1.5–5.0) |
| `CALGROUND` | Re-calibrate barometer to current pressure |
| `CLEARLOG` | Erase NVS position log |
| `STATUS` | Print full system state |
| `REBOOT` | Restart ESP32-C3 |

---

## 8. Accuracy Expectations

| Condition | Expected XY Accuracy |
|---|---|
| 3 bases, good geometry, open room | 0.5 – 1.5 m |
| 3 bases, walls/obstacles between | 1.5 – 3.0 m |
| Only 2 bases visible | No fix (requires 3) |

**Altitude / floor accuracy:** ±0.25 m typical → reliable floor detection in buildings with ≥2.5 m floor height.

### Tips for Better Accuracy
- Mount bases at consistent heights (same Z level as tracker, or all elevated equally)
- Maximise triangle area — avoid collinear placement
- Keep bases away from metal objects and WiFi access points
- Calibrate TX power per base unit
- Use `n = 2.0` to start, then tune upward if distances over-read

---

## 9. BLE GATT Service Reference

### Base Station Config Service
UUID: `12345678-1234-1234-1234-123456789abc`

| Characteristic | UUID suffix | Type | Description |
|---|---|---|---|
| Base ID | `...9ab0` | uint8 R/W | 0, 1, or 2 |
| TX Power | `...9ab1` | int8 R/W | RSSI at 1 m (dBm) |
| Adv Interval | `...9ab2` | uint16 R/W | milliseconds |

---

## 10. Troubleshooting

**No bases detected:**
- Verify all 3 bases are powered and LED is blinking
- Check that Base IDs 0/1/2 have been assigned correctly
- Ensure bases are within ~10 m of tracker

**Position jumping:**
- Increase `PATHLOSS` value (multipath in dense environment)
- Check for metal shelving or thick walls between bases and tracker
- Ensure bases are not co-located (need geometric spread)

**Wrong floor detected:**
- Re-run floor calibration (make sure to wait for pressure to stabilise after walking between floors)
- Run `CALGROUND` while on floor 0

**WebSocket not connecting:**
- Confirm tracker and browser are on the same WiFi network
- Try `http://<ip>/` directly (not HTTPS)
- Check Serial output for WiFi connection status

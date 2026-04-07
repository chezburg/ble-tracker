# Indoor BLE Tracker & Asset Management System

A comprehensive indoor positioning solution utilizing ESP32-C3 trackers, nRF52840 beacons, and a centralized Node.js web application for real-time monitoring and asset management.

## System Architecture

The system consists of three main components:

1.  **Beacons (nRF52840 Base):** Stationary units that broadcast BLE advertisements every 100ms containing their ID and calibrated TX power.
2.  **Tracker (ESP32-C3):** A mobile device that scans for beacon signals, calculates distances using a path-loss model, and performs WLS trilateration for XY positioning. It also uses a BMP390 barometer for Z-axis (floor) detection.
3.  **Web App (Node.js/Express):** A central dashboard for user authentication (JWT), asset categorization, and long-term location history tracking.

### Data Flow
1. **Beacons** broadcast Manufacturer Data (Company ID 0x0059 + BaseID + TxPower).
2. **Tracker** scans BLE, calculates distances, and determines coordinates (XY/Z).
3. **Local Dashboard (Port 80):** Hosted on the ESP32-C3 for real-time tuning and local Map UI.
4. **Central Web App (Port 3000):** Aggregates data for multi-user management and history.

---

## Hardware Requirements
- **Tracker:** Seeed Studio XIAO ESP32-C3 + Adafruit BMP390 Barometer.
- **Beacons:** 3× Seeed Studio XIAO nRF52840.
- **Standard Wiring (I2C):** SDA=6, SCL=7. BMP390 address is typically `0x77`.

---

## Developer Commands (Arduino CLI)

**Arduino CLI Path:** `arduino-cli`

### ESP32-C3 Tracker
- **Compile:**
  `arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C3 ./esp32c3_tracker`
- **Upload:**
  `arduino-cli upload -p <PORT> --fqbn esp32:esp32:XIAO_ESP32C3 ./esp32c3_tracker`

### nRF52840 Base (Beacon)
- **Compile:**
  `arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840 ./nrf52840_base`
- **Upload:**
  `arduino-cli upload -p <PORT> --fqbn Seeeduino:nrf52:xiaonRF52840 ./nrf52840_base`

---

## Critical Serial Commands (115200 baud)

### Tracker (XIAO C3)
- `WIFI <ssid> <pass>`: Connects to WiFi. Supports spaces in SSID.
- `KNOWNDIST <id> <m>`: Calibrates base offset given a known distance.
- `REGFLOOR <n>`: Registers current altitude as floor level `n`.
- `STATUS`: Dumps current coordinates, base visibility, and WiFi state.
- `BASE <id> <x> <y>`: Set base station coordinate in meters.
- `PATHLOSS <n>`: Set path-loss exponent (typically 2.0 to 3.5).

### Base (XIAO nRF52)
- `SETID <0|1|2>`: Required for first boot or after `RESET`.
- `RESET`: Clears ID and enters breathing-LED provisioning mode.

---

## Web Application Setup

The central web app provides user authentication and asset management.

### Installation
1. Navigate to `web-app/`
2. Install dependencies: `npm install`
3. Start the server: `npm start`
4. Access via: `http://localhost:3000`

### Key Dependencies
- `express`, `bcryptjs`, `jsonwebtoken`, `pug`.

---

## Positioning Logic
- **XY Positioning:** Uses Weighted Least Squares (WLS) trilateration based on Kalman-filtered RSSI values.
- **Z-Axis:** Uses relative barometer delta from the BMP390 sensor to determine the current floor level.
- **Local Tuning:** The ESP32-C3 hosts a web server on Port 80 (`dashboard.cpp`) for real-time calibration and visual feedback.

---

## Accuracy Expectations
- **Open Space:** 0.5m – 1.5m accuracy.
- **Obstructed:** 1.5m – 3.0m accuracy.
- **Floor Detection:** Reliable within building environments (±0.25m typical precision).

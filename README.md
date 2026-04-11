<p align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=0:00C6FF,100:0072FF&height=200&section=header&text=SensorLib&fontSize=64&fontColor=FFFFFF&animation=fadeIn&desc=Arduino%20%7C%20PlatformIO%20%7C%20ESP-IDF&descAlignY=72&descSize=18" />
</p>

<h1 align="center">SensorLib</h1>

<p align="center">
  A multi-platform sensor driver library for Arduino / PlatformIO / ESP-IDF.
</p>

<p align="center">
  <a href="https://github.com/lewisxhe/SensorLib/actions/workflows/esp-idf.yml"><img src="https://github.com/lewisxhe/SensorLib/actions/workflows/esp-idf.yml/badge.svg" /></a>
  <a href="https://github.com/lewisxhe/SensorLib/actions/workflows/arduino_ci.yml"><img src="https://github.com/lewisxhe/SensorLib/actions/workflows/arduino_ci.yml/badge.svg" /></a>
  <a href="https://github.com/lewisxhe/SensorLib/actions/workflows/platformio.yml"><img src="https://github.com/lewisxhe/SensorLib/actions/workflows/platformio.yml/badge.svg" /></a>
  <a href="https://www.ardu-badge.com/SensorLib"><img src="https://www.ardu-badge.com/badge/SensorLib.svg?" /></a>
  <a href="https://registry.platformio.org/libraries/lewisxhe/SensorLib"><img src="https://badges.registry.platformio.org/packages/lewisxhe/library/SensorLib.svg" /></a>
  <a href="https://components.espressif.com/components/lewisxhe/sensorlib">
  <img src="https://img.shields.io/badge/ESP--IDF%20Registry-v0.4.0-success?logo=espressif&logoColor=white&color=009688&style=flat-square" alt="ESP-IDF Component Registry"></a>
</p>

<p align="center">
  <a href="https://github.com/lewisxhe/SensorLib/blob/master/LICENSE"><img src="https://img.shields.io/github/license/lewisxhe/SensorLib" /></a>
  <a href="https://github.com/lewisxhe/SensorLib/issues"><img src="https://img.shields.io/github/issues/lewisxhe/SensorLib" /></a>
  <a href="https://github.com/lewisxhe/SensorLib/graphs/contributors"><img src="https://img.shields.io/github/forks/lewisxhe/SensorLib" /></a>
  <a href="https://github.com/lewisxhe/SensorLib/stargazers"><img src="https://img.shields.io/github/stars/lewisxhe/SensorLib" /></a>
  <a href="https://github.com/lewisxhe/SensorLib/releases"><img src="https://img.shields.io/github/release/lewisxhe/SensorLib" /></a>
</p>

---

### Highlights
- One library for **Arduino / PlatformIO / ESP-IDF**
- Supports **I2C** and **SPI** devices
- A collection of ready-to-run examples under `examples/`

---

## Contents
- [Contents](#contents)
- [Installation](#installation)
  - [Arduino IDE](#arduino-ide)
  - [PlatformIO](#platformio)
  - [ESP-IDF](#esp-idf)
- [Examples](#examples)
- [Support list](#support-list)
- [Notes](#notes)
- [License](#license)
  - [Third-party licenses](#third-party-licenses)

## Installation

### Arduino IDE

**Option A: Install from Library Manager (recommended)**
1. Open Arduino IDE
2. Go to `Tools` → `Manage Libraries...`
3. Search for **SensorLib**
4. Click **Install**
5. Restart Arduino IDE if needed

**Option B: Install from ZIP**
1. GitHub page → `Code` → `Download ZIP`
2. Arduino IDE → `Sketch` → `Include Library` → `Add .ZIP Library...`
3. Select the downloaded ZIP file
4. Restart Arduino IDE if the library does not appear immediately

**Option C: Install with Git (manual)**
1. Clone this repository into your Arduino libraries folder:
   - Windows: `Documents/Arduino/libraries/`
   - macOS: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
2. Folder name should be `SensorLib`
3. Restart Arduino IDE

### PlatformIO

**Option A: Install from PlatformIO Registry (recommended)**

Add to your `platformio.ini`:

```ini
[env:your_env]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
  lewisxhe/SensorLib@^0.4.0
```

**Option B: Install from GitHub (latest)**

```ini
[env:your_env]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
  https://github.com/lewisxhe/SensorLib.git
```

**Option C: Put it into `lib/` (local library)**
1. Copy/clone this repository into your PlatformIO project folder:
   - `<your_project>/lib/SensorLib/`
2. PlatformIO will auto-detect it as a local library

### ESP-IDF

SensorLib is now published in the official **ESP-IDF Component Registry**, and supports ESP-IDF v4.4+ (recommended v5.1+).

**1. Add Dependency (in project root `idf_component.yml`)**

Create or edit `idf_component.yml` in your ESP-IDF project root:

```yaml
dependencies:
  lewisxhe/sensorlib:
    version: "^0.4.0"          # or use "*" to get the latest version
```

## Examples

- PlatformIO examples: see `platformio.ini` and uncomment the example `src_dir` you want to build.
- More examples are located in the `examples/` folder.

## Support list

| Sensor          | Description              | I2C | SPI |
| --------------- | ------------------------ | --- | --- |
| PCF8563/HYM8563 | Real-time clock          | ✔️   | ❌   |
| PCF85063        | Real-time clock          | ✔️   | ❌   |
| QMI8658         | IMU                      | ✔️   | ✔️   |
| BHI260AP        | IMU                      | ✔️   | ✔️   |
| BHI360          | IMU                      | ✔️   | ✔️   |
| QMC6309         | Magnetic Sensor          | ✔️   | ❌   |
| QMC6310U/N      | Magnetic Sensor          | ✔️   | ❌   |
| QMC5883P        | Magnetic Sensor          | ✔️   | ❌   |
| QMC5883L        | Magnetic Sensor          | ✔️   | ❌   |
| BMM150          | Magnetic Sensor          | ✔️   | ❌   |
| XL9555          | I/O expander             | ✔️   | ❌   |
| PCA9570         | I/O expander             | ✔️   | ❌   |
| BMA422          | Accelerometer            | ✔️   | ❌   |
| BMA423          | Accelerometer            | ✔️   | ❌   |
| BMA456          | Accelerometer            | ✔️   | ❌   |
| DRV2605         | Haptic Driver            | ✔️   | ❌   |
| AW86224         | Haptic Driver            | ✔️   | ❌   |
| CM32181         | Ambient Light Sensor     | ✔️   | ❌   |
| LTR553          | Light & Proximity Sensor | ✔️   | ❌   |
| FT3267          | Capacitive touch         | ✔️   | ❌   |
| FT5206          | Capacitive touch         | ✔️   | ❌   |
| FT6206          | Capacitive touch         | ✔️   | ❌   |
| FT6236          | Capacitive touch         | ✔️   | ❌   |
| CST820          | Capacitive touch         | ✔️   | ❌   |
| CST816S/T/D     | Capacitive touch         | ✔️   | ❌   |
| CST226SE        | Capacitive touch         | ✔️   | ❌   |
| CHSC5816        | Capacitive touch         | ✔️   | ❌   |
| GT911           | Capacitive touch         | ✔️   | ❌   |
| CST9217         | Capacitive touch         | ✔️   | ❌   |
| CST9220         | Capacitive touch         | ✔️   | ❌   |
| GT9895          | Capacitive touch         | ✔️   | ❌   |
| HI8561          | Capacitive touch         | ✔️   | ❌   |
| AW9364          | Led Driver (GPIO)        | ❌   | ❌   |
| BQ27220         | Gauge Meter              | ✔️   | ❌   |
| AXP2602         | Gauge Meter              | ✔️   | ❌   |

## Notes

- I2C devices typically require proper pull-up resistors on SDA/SCL.
- If you cannot detect devices, check wiring, address selection, and I2C speed (100kHz / 400kHz).

## License

SensorLib is licensed under the **MIT License**. See `LICENSE`.

### Third-party licenses

This repository includes third-party code under `src/bosch/` from **Bosch Sensortec**, licensed under the **BSD 3-Clause License (BSD-3-Clause)**.

See `THIRD_PARTY_NOTICES.md` for details.
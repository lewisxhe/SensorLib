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
  <img src="https://img.shields.io/badge/ESP--IDF%20Registry-v0.4.1-success?logo=espressif&logoColor=white&color=009688&style=flat-square" alt="ESP-IDF Component Registry"></a>
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

- **44+ devices** across 11 categories — Touch, PMIC, IMU, Magnetometer, Accelerometer, RTC, Gauge, Haptic, Light Sensor, I/O Expander, LED
- **158 ready-to-run examples** covering every supported device
- **Full PMIC subsystem** — charger, ADC, GPIO, IRQ, LED, power channels, coulomb counter
- One library for **Arduino / PlatformIO / ESP-IDF**
- Supports both **I2C** and **SPI** buses

---

## Contents
- [Contents](#contents)
- [Installation](#installation)
  - [Arduino IDE](#arduino-ide)
  - [PlatformIO](#platformio)
  - [ESP-IDF](#esp-idf)
- [Quick Start](#quick-start)
  - [Including Headers](#including-headers)
    - [Option A: Per-driver Include (Recommended)](#option-a-per-driver-include-recommended)
    - [Option B: Aggregate Include (Quick Prototyping)](#option-b-aggregate-include-quick-prototyping)
    - [Which should I use?](#which-should-i-use)
  - [Minimal Example: Touch](#minimal-example-touch)
  - [Minimal Example: PMIC](#minimal-example-pmic)
- [Examples](#examples)
  - [Using Examples](#using-examples)
- [Supported Devices](#supported-devices)
- [Platform Compatibility](#platform-compatibility)
- [Notes](#notes)
- [License](#license)
  - [Third-party licenses](#third-party-licenses)

## Installation

### Arduino IDE

Install from **Library Manager** (recommended):
1. Open Arduino IDE
2. Go to `Tools` → `Manage Libraries...`
3. Search for **SensorLib**
4. Click **Install**

<details>
<summary>Alternative install methods</summary>

**From ZIP**
1. GitHub page → `Code` → `Download ZIP`
2. Arduino IDE → `Sketch` → `Include Library` → `Add .ZIP Library...`
3. Select the downloaded ZIP file

**With Git**
1. Clone this repository into your Arduino libraries folder:
   - Windows: `Documents/Arduino/libraries/`
   - macOS: `~/Documents/Arduino/libraries/`
   - Linux: `~/Arduino/libraries/`
2. Folder name should be `SensorLib`
3. Restart Arduino IDE

</details>

### PlatformIO

Add to your `platformio.ini`:

```ini
[env:your_env]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
  lewisxhe/SensorLib@^0.4.0
```

<details>
<summary>Alternative: install from GitHub or local lib</summary>

**From GitHub (latest)**
```ini
lib_deps =
  https://github.com/lewisxhe/SensorLib.git
```

**Local library**
Copy/clone this repository into `<your_project>/lib/SensorLib/`. PlatformIO will auto-detect it.

</details>

### ESP-IDF

SensorLib is published in the [ESP-IDF Component Registry](https://components.espressif.com/components/lewisxhe/sensorlib). Supports ESP-IDF v4.4+ (recommended v5.1+).

**1. Add dependency** in project root `idf_component.yml`:

```yaml
dependencies:
  lewisxhe/sensorlib:
    version: "^0.4.0"
```

**2. Use in your code**:

```c
#include "touch/TouchDrvGT911.hpp"

// or for PMIC:
// #include "pmic/xpowers/axp2101/PmicAXP2101.hpp"
```

## Quick Start

### Including Headers

SensorLib provides two ways to include drivers in your sketch:

#### Option A: Per-driver Include (Recommended)

Include only the specific driver(s) you need. This minimizes compile-time
namespace pollution and avoids pulling in unnecessary macros.

```cpp
// Touch — pick the exact chip you use
#include "touch/TouchDrvGT911.hpp"

// PMIC — pick the exact chip you use
#include "pmic/xpowers/axp2101/PmicAXP2101.hpp"

// Magnetometer
#include "sensor/magnetometer/qmc/SensorQMC5883L.hpp"

// IMU
#include "sensor/imu/qmi8658/SensorQMI8658.hpp"

// RTC
#include "time/pcf8563/SensorPCF8563.hpp"
```

#### Option B: Aggregate Include (Quick Prototyping)

One header pulls in **all** drivers for a category. Convenient for prototyping,
but brings in extra macros and classes you may not need.

```cpp
#include <TouchDrv.hpp>        // All touch drivers
#include <PmicDrv.hpp>         // All PMIC drivers
#include <MagnetometerDrv.hpp> // All magnetometer drivers
#include <ImuDrv.hpp>          // All IMU drivers
#include <RtcDrv.hpp>          // All RTC drivers
#include <GaugeDrv.hpp>        // All gauge drivers
#include <IoExpanderDrv.hpp>   // All I/O expanders
#include <HapticDrivers.hpp>   // All haptic drivers
#include <LightSensorDrv.hpp>  // All light sensors
#include <AccelerometerDrv.hpp> // All accelerometers
```

#### Which should I use?

| | Per-driver (Option A) | Aggregate (Option B) |
|---|---|---|
| **Best for** | Production, libraries, multi-library projects | Prototyping, learning |
| **Compile hygiene** | Minimal footprint | Pulls in all drivers & macros |
| **Namespace** | Only your chip's classes visible | All chip classes in scope |
| **Include path** | `"category/subcategory/ChipName.hpp"` | `<CategoryDrv.hpp>` |

> **Recommendation**: Use **Option A** (per-driver) for anything beyond quick experiments.
> It keeps your build clean and avoids macro conflicts with other libraries.

### Minimal Example: Touch

**GT911 on ESP32 (Arduino)**

```cpp
#include <Wire.h>
#include "touch/TouchDrvGT911.hpp"

TouchDrvGT911 touch;

void setup() {
    Serial.begin(115200);
    touch.setPins(15, 18);   // INT pin = 15, IRQ pin = 18
    touch.begin(Wire, GT911_SLAVE_ADDRESS_L, 21,22); // SDA=21,SCL=22
    Serial.println("GT911 ready");
}

void loop() {
    TouchPoints touch_points = touch.getTouchPoints();
    if (touch_points.hasPoints()) {
        for (int i = 0; i < touch_points.getPointCount(); ++i) {
            const TouchPoint &point = touch_points.getPoint(i);
            Serial.print("X[");
            Serial.print(i);
            Serial.print("]:");
            Serial.print(point.x);
            Serial.print(" ");
            Serial.print(" Y[");
            Serial.print(i);
            Serial.print("]:");
            Serial.print(point.y);
            Serial.print(" ");
        }
        Serial.println();
    }
    delay(10);
}
```

### Minimal Example: PMIC

**AXP2101 on ESP32 (Arduino)**

```cpp
#include <Wire.h>
#include "pmic/xpowers/axp2101/PmicAXP2101.hpp"

PmicAXP2101 pmic;

void setup() {
    Serial.begin(115200);

    if (!pmic.begin(Wire, AXP2101_SLAVE_ADDRESS, 3, 2)) { // SDA=3, SCL=2
        Serial.println("AXP2101 not found!");
        while (1) delay(1000);
    }

    Serial.print("Chip ID: 0x");
    Serial.println(pmic.getChipID(), HEX);

    // Set DCDC1 to 3.3V
    pmic->getChannel()->setVoltage(AXP2101Channel::CH_DCDC1, 3300);
    pmic->getChannel()->enable(AXP2101Channel::CH_DCDC1, true);

    // Read battery voltage
    pmic.enableModule(PmicAXP2101::Module::GENERAL_ADC, true);
    Serial.print("VBUS: ");
    Serial.print(pmic->getAdc()->getVBUSVoltage());
    Serial.println(" mV");
}

void loop() {
    Serial.print("Battery: ");
    Serial.print(pmic->getAdc()->getBattVoltage());
    Serial.println(" mV");
    delay(2000);
}
```

---

## Examples

158 examples are organized by category in the `examples/` directory:

```
examples/
├── Actuators/
│   ├── Haptic/               # DRV2605, AW86224 haptic motors (3 examples)
│   └── LED/                  # AW9364 LED driver
├── IO/
│   └── Expander/             # XL9555 (5 examples), PCA9570
├── Platform/
│   └── idf-examples/         # ESP-IDF framework examples
├── PowerManagement/
│   ├── Gauge/                # AXP2602, BQ27220 battery gauges
│   └── Pmic/
│       ├── AXP192/           # TODO
│       ├── AXP202/           # TODO
│       ├── AXP2101/          # TODO
│       ├── AXP517/           # Basic, BC1.2, Charger, GPIO, Gauge, LED, Power
│       ├── BQ25896_Charger/  # TI charger
│       ├── SY6970_Charger/   # Silergy charger
│       └── ...               # PMIC Probe, xxxx WebPanel, etc.
├── Sensors/
│   ├── Accelerometer/        # BMA422 (3), BMA423 (8), BMA456H (7), Universal
│   ├── FingerNavigation/     # PAW-A350
│   ├── IMU/
│   │   ├── BHI260AP/         # 6DoF, Euler, StepCounter, Klio, GPIO... (18 examples)
│   │   ├── BHI360/           # 6DoF, Euler, MultiTap, aux BMM350... (10 examples)
│   │   ├── QMI8658/          # BasicRead, FIFO, Pedometer, Tap, SyncMode... (10 examples)
│   │   └── QMI8658_Deprecated/  # Legacy API examples
│   ├── LightSensor/          # CM32181, LTR553
│   ├── Magnetometer/         # BMM150, QMC5883L/P, QMC6309, QMC6310
│   └── Touch/                # GT911, GT9895, FT6232, CST series, CHSC5816, HI8561 (10 examples)
├── Time/
│   └── RTC/                  # PCF85063, PCF8563 (10 examples)
└── Utilities/
    ├── CustomCallback/       # Custom callback patterns
    └── SensorWireHelper/     # I2C scanning & debugging
```

### Using Examples

**PlatformIO**: Edit `platformio.ini` and set `src_dir` to the example you want:

```ini
[platformio]
src_dir = examples/Sensors/IMU/QMI8658/BasicRead
```

**Arduino IDE**: `File` → `Open` → navigate to the `.ino` file in `examples/`.

**ESP-IDF**: See `examples/Platform/idf-examples/` for ESP-IDF specific projects.

## Supported Devices

<details>
<summary>44 supported devices (click to expand)</summary>

| Device | Description | I2C | SPI | Header |
|--------|-------------|:---:|:---:|--------|
| **RTC** |||||
| PCF8563 / HYM8563 | Real-time clock | ✔️ | ❌ | `time/pcf8563/SensorPCF8563.hpp` |
| PCF85063 | Real-time clock | ✔️ | ❌ | `time/pcf85063/SensorPCF85063.hpp` |
| **IMU** |||||
| QMI8658 | 6-axis IMU | ✔️ | ✔️ | `sensor/imu/qmi8658/SensorQMI8658.hpp` |
| BHI260AP | Smart IMU (Bosch) | ✔️ | ✔️ | `sensor/imu/bhi260/SensorBHI260AP.hpp` |
| BHI360 | Smart IMU (Bosch) | ✔️ | ✔️ | `sensor/imu/bhi360/SensorBHI360.hpp` |
| **Magnetometer** |||||
| QMC6309 | Magnetic Sensor | ✔️ | ❌ | `sensor/magnetometer/qmc/SensorQMC6309.hpp` |
| QMC6310U/N | Magnetic Sensor | ✔️ | ❌ | `sensor/magnetometer/qmc/SensorQMC6310.hpp` |
| QMC5883P | Magnetic Sensor | ✔️ | ❌ | `sensor/magnetometer/qmc/SensorQMC5883P.hpp` |
| QMC5883L | Magnetic Sensor | ✔️ | ❌ | `sensor/magnetometer/qmc/SensorQMC5883L.hpp` |
| BMM150 | Magnetic Sensor | ✔️ | ❌ | `sensor/magnetometer/bmm150/SensorBMM150.hpp` |
| **Accelerometer** |||||
| BMA422 | Accelerometer | ✔️ | ❌ | `sensor/accelerometer/bma/SensorBMA422.hpp` |
| BMA423 | Accelerometer | ✔️ | ❌ | `sensor/accelerometer/bma/SensorBMA423.hpp` |
| BMA456H | Accelerometer | ✔️ | ❌ | `sensor/accelerometer/bma/SensorBMA456H.hpp` |
| **I/O Expander** |||||
| XL9555 | 16-bit I/O Expander | ✔️ | ❌ | `expander/IoExpanderXL9555.hpp` |
| PCA9570 | 4-bit I/O Expander | ✔️ | ❌ | `expander/IoExpanderPCA9570.hpp` |
| **Haptic** |||||
| DRV2605 | Haptic Driver (TI) | ✔️ | ❌ | `haptic_drivers/HapticDriver_DRV2605.hpp` |
| AW86224 | Haptic Driver (Awinic) | ✔️ | ❌ | `haptic_drivers/HapticDriver_AW86224.hpp` |
| **Light Sensor** |||||
| CM32181 | Ambient Light Sensor | ✔️ | ❌ | `sensor/light_sensor/SensorCM32181.hpp` |
| LTR553 | Light & Proximity | ✔️ | ❌ | `sensor/light_sensor/SensorLTR553.hpp` |
| **Touch** |||||
| GT911 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvGT911.hpp` |
| GT9895 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvGT9895.hpp` |
| FT3267 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvFT6X36.hpp` |
| FT5206 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvFT6X36.hpp` |
| FT6206 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvFT6X36.hpp` |
| FT6236 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvFT6X36.hpp` |
| CST226SE | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvCST226.hpp` |
| CST820 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvCST816.hpp` |
| CST816S/T/D | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvCST816.hpp` |
| CST9217 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvCST92xx.hpp` |
| CST9220 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvCST92xx.hpp` |
| CST3240 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvCST92xx.hpp` |
| CST3530 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvCST3530.hpp` |
| CHSC5816 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvCHSC5816.hpp` |
| HI8561 | Capacitive Touch | ✔️ | ❌ | `touch/TouchDrvHI8561.hpp` |
| **LED** |||||
| AW9364 | LED Driver (GPIO) | ❌ | ❌ | `actuator/AW9364LedDriver.hpp` |
| **PMIC** |||||
| AXP192 | PMIC (XPowers) | ✔️ | ❌ | `pmic/xpowers/axp192/PmicAXP192.hpp` |
| AXP202 | PMIC (XPowers) | ✔️ | ❌ | `pmic/xpowers/axp202/PmicAXP202.hpp` |
| AXP2101 | PMIC (XPowers) | ✔️ | ❌ | `pmic/xpowers/axp2101/PmicAXP2101.hpp` |
| AXP517 | PMIC (XPowers) | ✔️ | ❌ | `pmic/xpowers/axp517/PmicAXP517.hpp` |
| BQ25896 | Charger (TI) | ✔️ | ❌ | `pmic/ti/bq25896/PmicBQ25896.hpp` |
| SY6970 | Charger (Silergy) | ✔️ | ❌ | `pmic/silergy/sy6970/PmicSY6970.hpp` |
| **Gauge** |||||
| BQ27220 | Battery Gauge (TI) | ✔️ | ❌ | `gauge/ti/GaugeBQ27220.hpp` |
| AXP2602 | Battery Gauge (XPowers) | ✔️ | ❌ | `gauge/xpowers/GaugeAXP2602.hpp` |
| **Other** |||||
| PAW-A350 | Finger Navigation (PixArt) | ✔️ | ❌ | `sensor/pixart/pawA350/SensorPawA350.hpp` |

</details>

## Platform Compatibility

| Platform | Status | Notes |
|----------|--------|-------|
| ESP32 | ✔️ | Primary target, full support |
| ESP32-S2 | ✔️ | Single-core, USB OTG |
| ESP32-S3 | ✔️ | Dual-core, USB OTG |
| ESP32-C3 | ✔️ | RISC-V single-core |
| ESP32-C6 | ✔️ | RISC-V, Wi-Fi 6 |

> Other Arduino-compatible boards (RP2040, nRF52, etc.) may work for I2C devices
> but are not actively tested.

## Notes

- **I2C pull-ups**: Most I2C devices require proper pull-up resistors (4.7kΩ typical) on SDA/SCL lines.
- **I2C speed**: Default is 100kHz. For faster transfers, use `Wire.setClock(400000)` before `begin()`.
- **SPI devices**: QMI8658, BHI260AP, BHI360 support SPI. Touch and PMIC devices are I2C only.
- **I2C addresses**: Some devices have configurable addresses (e.g., GT911 has `GT911_SLAVE_ADDRESS_L` / `GT911_SLAVE_ADDRESS_H`). Check the header file for available constants.
- **Troubleshooting**: If a device is not detected, verify wiring, address selection, and I2C speed. Use `examples/Utilities/SensorWireHelper/` to scan the I2C bus.

## License

SensorLib is licensed under the **MIT License**. See `LICENSE`.

### Third-party licenses

This repository includes third-party code under `src/bosch/` from **Bosch Sensortec**, licensed under the **BSD 3-Clause License (BSD-3-Clause)**.

See `THIRD_PARTY_NOTICES.md` for details.

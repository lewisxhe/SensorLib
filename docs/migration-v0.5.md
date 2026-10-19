# Migrating from SensorLib v0.4.x to v0.5.0

SensorLib v0.5.0 reorganizes public headers, expands the common IMU API, and
stops exporting several global compatibility macros. Most applications only
need to update their includes. Applications that derive from `ImuBase`, use
internal driver paths, or rely on SensorLib's helper macros require additional
changes.

## Recommended upgrade process

1. Update SensorLib and perform a clean build.
2. Replace deprecated or removed includes using the table below.
3. Update custom `ImuBase` implementations.
4. Replace renamed macros and SensorLib-provided bit helpers.
5. Check custom Bosch firmware and example paths.
6. Run device-level tests, especially interrupt, motion, and power-management
   workflows.

## Public header layout

Prefer the flat category or vendor headers in `src/`. They work with Arduino,
PlatformIO, and ESP-IDF without adding internal directories to the include
path.

| v0.4.x include | v0.5.0 include |
| --- | --- |
| `SensorDrv.hpp` | `MagnetometerDrv.hpp`, `AccelerometerDrv.hpp`, or `ImuDrv.hpp` |
| `SensorBMM150.hpp` | `MagnetometerDrv.hpp` |
| `SensorBMA422.hpp`, `SensorBMA423.hpp`, `SensorBMA456H.hpp` | `AccelerometerDrv.hpp` |
| `SensorBHI260AP.hpp`, `SensorBHI360.hpp` | `ImuDrv.hpp` |
| Legacy `SensorQMI8658.hpp` | `ImuDrv.hpp` |
| `SensorPCF85063.hpp`, `SensorPCF8563.hpp` | `RtcDrv.hpp` |
| `GaugeAXP2602.hpp`, `GaugeBQ27220.hpp` | `GaugeDrv.hpp` |
| `ExtensionIOXL9555.hpp`, `IoExpanderXL9555.hpp` | `IoExpanderDrv.hpp` |
| `TouchDrvGT911.hpp`, `TouchDrvGT9895.hpp` | `TouchDrvGoodix.hpp` |
| `TouchDrvCSTXXX.hpp` and CST device headers | `TouchDrvCST.hpp` |
| `TouchDrvFT6X36.hpp` | `TouchDrvFocalTech.hpp` |
| `TouchDrvHI8561.hpp` | `TouchDrvJadard.hpp` |
| `TouchDrvCHSC5816.hpp` | `TouchDrvChipshine.hpp` |
| Direct `haptic_drivers/...` includes | `HapticDrivers.hpp` |

Most old top-level headers remain as compatibility wrappers and emit a
deprecation message. `SensorDrv.hpp` and the top-level `SensorBMM150.hpp` were
removed and must be replaced. Direct paths under old internal directories are
not compatibility APIs.

## QMI8658

The legacy `SensorQMI8658.hpp` API is deprecated. New code should include the
unified driver:

```cpp
#include "ImuDrv.hpp"

SensorQMI8658 imu;
```

The legacy and unified headers define similarly named types and should not be
included in the same translation unit. Use the examples under
`examples/sensor/qmi8658_*` when converting feature-specific code.

## ImuBase implementations

`ImuBase` now models the complete lifecycle and data path of an IMU. The old
data-ready methods were replaced as follows:

```cpp
// v0.4.x
imu.accelIsDataReady();
imu.gyroIsDataReady();

// v0.5.0
imu.isDataReady(static_cast<uint8_t>(ImuBase::DataReadyMask::ACCEL));
imu.isDataReady(static_cast<uint8_t>(ImuBase::DataReadyMask::GYRO));
```

Custom classes derived from `ImuBase` must also implement the new device,
power, raw-data, synchronous sampling, and FIFO virtual methods declared in
`src/sensor/ImuBase.hpp`. A missing implementation is reported as an abstract
class compile error.

## SensorLib helper macros

SensorLib no longer defines or overrides global Arduino-style bit and byte
macros. Code that relied on the fallback definitions from `SensorLib.h` should
use the type-safe replacements:

| v0.4.x helper | v0.5.0 helper |
| --- | --- |
| `_BV(bit)` | `sensorlib::_bv(bit)` |
| `lowByte(value)` | `sensorlib::_lowByte(value)` |
| `highByte(value)` | `sensorlib::_highByte(value)` |
| `bitRead(value, bit)` | `sensorlib::_bitRead(value, bit)` |
| `bitSet(value, bit)` | `sensorlib::_bitSet(value, bit)` |
| `bitClear(value, bit)` | `sensorlib::_bitClear(value, bit)` |
| `bitToggle(value, bit)` | `sensorlib::_bitToggle(value, bit)` |
| `bitWrite(value, bit, state)` | `sensorlib::_bitWrite(value, bit, state)` |
| `isBitSet(value, bit)` | `sensorlib::_isBitSet(value, bit)` |

Arduino cores may still provide some of the old macros independently. The new
helpers are portable across all SensorLib targets.

## ESP-IDF legacy I2C selection

The misspelled legacy API macro was renamed:

```cpp
// v0.4.x
#define USEING_I2C_LEGACY

// v0.5.0
#define SENSORLIB_USE_I2C_LEGACY
```

ESP-IDF projects should normally select the appropriate I2C API through the
SensorLib Kconfig option instead of defining this macro directly.

## Device pin definitions

`SensorLib.h` no longer includes `DevicesPins.h` unconditionally. Include it
directly when an application uses SensorLib's board pin definitions:

```cpp
#include "DevicesPins.h"
```

Defining `INCLUDE_DEVICES_PINS` before including `SensorLib.h` preserves the
old implicit behavior:

```cpp
#define INCLUDE_DEVICES_PINS
#include "SensorLib.h"
```

## Driver exclusion

v0.5.0 can exclude complete driver families or individual devices. ESP-IDF
projects configure exclusions under:

```text
Component config -> SensorLib Configuration -> Driver exclusion
```

Arduino and PlatformIO projects can define the corresponding
`SENSORLIB_EXCLUDE_*` macros in `SensorBuildOptUser.h` or through compiler build
flags. Exclusions default to disabled, so existing projects retain all drivers
unless they opt in.

## Bosch firmware files

Raw Bosch `*.fw` files are no longer bundled. The generated firmware headers
used by SensorLib examples remain in `src/bosch/firmware/`. For custom firmware,
convert the binary to a C/C++ byte array and pass it to the applicable driver's
`setFirmware()` API.

## Example paths

Examples now use category directories and lowercase `snake_case` names. Build
scripts that reference old example paths must be updated. The main categories
are `examples/actuator`, `examples/io`, `examples/platform`, `examples/power`,
`examples/rtc`, `examples/sensor`, `examples/touch`, and `examples/utility`.

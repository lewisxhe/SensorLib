# Changelog

All notable changes to SensorLib are documented in this file.

The project is still below version 1.0. Minor releases may contain API changes;
these changes are called out explicitly in each release and in the associated
migration guide.

## [Unreleased]

## [0.5.0] - 2026-09-22

v0.5.0 is a major pre-1.0 update. It adds a complete PMIC subsystem, introduces
more consistent device interfaces, and reorganizes drivers and examples by
category. Projects upgrading from v0.4.x should read the
[v0.5 migration guide](docs/migration-v0.5.md).

### Added

- Added a common PMIC framework covering charger, power, ADC, GPIO, IRQ, LED,
  power button, regulator channel, timer, watchdog, BC1.2, and Type-C features.
- Added PMIC and charger support for AXP192, AXP202, AXP2101, AXP517, BQ25896,
  and SY6970.
- Added AXP517 Type-C/TCPC support and interrupt-driven USB-PD sink voltage
  negotiation, including fixed PDO selection examples.
- Added the PAW-A350 finger navigation driver and example.
- Added CST3240 touch support and QMC6309H compatibility.
- Added `GaugeBase` and common battery status APIs for BQ27220 and AXP2602.
- Added category and vendor aggregate headers, including `ImuDrv.hpp`,
  `MagnetometerDrv.hpp`, `AccelerometerDrv.hpp`, `RtcDrv.hpp`, `GaugeDrv.hpp`,
  `IoExpanderDrv.hpp`, `LightSensorDrv.hpp`, `PmicXPowers.hpp`, `PmicTI.hpp`,
  `PmicSilergy.hpp`, and vendor-specific touch headers.
- Added build-time driver exclusion through ESP-IDF Kconfig/CMake and
  `SensorBuildOpt.h` for Arduino and PlatformIO builds.
- Added Doxygen API documentation deployment and automated formatting and
  workflow validation.
- Added basic, IRQ, charger, web monitor, probe, and USB-PD examples for the new
  power devices.

### Changed

- Expanded `ImuBase` into a complete common IMU interface and added a unified
  QMI8658 implementation.
- Reworked BMM150 to use the common magnetometer interface.
- Moved BMA4XX motion capabilities into the common BMA4XX base implementation.
- Reorganized drivers into category and vendor directories while retaining
  deprecated top-level compatibility headers where practical.
- Reorganized examples into category directories with lowercase `snake_case`
  names.
- Replaced SensorLib-provided global bit and byte macros with type-safe helpers
  in the `sensorlib` namespace.
- Renamed `USEING_I2C_LEGACY` to `SENSORLIB_USE_I2C_LEGACY`.
- Unified device initialization so failed initialization releases communication
  state consistently.
- Changed the common PMIC ADC channel enum to a combinable bitmask.
- Expanded PMIC IRQ masks from 32 bits to 64 bits.

### Deprecated

- Deprecated the legacy QMI8658 API in `SensorQMI8658.hpp` in favor of the
  implementation exposed by `ImuDrv.hpp`.
- Deprecated individual top-level driver headers in favor of category or vendor
  aggregate headers.
- Deprecated touch `getPoint()` in favor of `getTouchPoints()`.

### Removed

- Removed the top-level `SensorBMM150.hpp`; include `MagnetometerDrv.hpp`
  instead.
- Removed the internal `SensorCommDebug` implementation.
- Removed bundled vendor datasheet PDFs.
- Removed raw Bosch `*.fw` images. Generated firmware headers used by the
  examples remain available.

### Fixed

- Fixed swapped BMA423 any-motion and no-motion configuration.
- Fixed CST3240 point reads and invalid CST92xx touch reporting.
- Fixed QMC6309 self-test and initialization behavior.
- Fixed the ESP-IDF I2C initialization early-return condition.
- Fixed Arduino STM32 typed GPIO compatibility and related compile failures.
- Fixed XL9555 expander probing.
- Fixed AXP2101 charge-current steps and interrupt clearing on older revisions.
- Fixed AXP517 return values, TCPC initialization, and PD alert handling.
- Fixed BHI260 error propagation.
- Fixed touch byte helpers for ESP-IDF and non-Arduino builds.

[Unreleased]: https://github.com/lewisxhe/SensorLib/compare/v0.5.0...HEAD
[0.5.0]: https://github.com/lewisxhe/SensorLib/compare/v0.4.1...v0.5.0

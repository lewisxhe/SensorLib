/**
 *
 * @license MIT License
 *
 * Copyright (c) 2026 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      AdvancedFeatures.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-15
 *
 * @brief     Advanced features example using SensorQMI8658.
 *
 * @details   This example demonstrates how to:
 *            - Use axis conversion for different mounting orientations
 *            - Perform static calibration
 *            - Enable dynamic gyroscope calibration
 *            - Run hardware self-test
 *
 * @note      Place the sensor in a stable position for static calibration.
 *            The sensor should remain motionless during calibration.
 */
#include <stdarg.h>
#include "ImuDrv.hpp"
#include "DevicesPins.h"

// Select one interface.
// #define USE_I2C_INTERFACE
// #define USE_SPI_INTERFACE

#if !defined(USE_I2C_INTERFACE) && !defined(USE_SPI_INTERFACE)
#define USE_I2C_INTERFACE
#endif

#if defined(USE_SPI_INTERFACE)

#ifndef SPI_MOSI
#define SPI_MOSI 35
#endif

#ifndef SPI_MISO
#define SPI_MISO 37
#endif

#ifndef SPI_SCK
#define SPI_SCK 36
#endif

#ifndef IMU_CS
#define IMU_CS 34
#endif

#else

#ifndef IMU_SDA
#define IMU_SDA 17
#endif

#ifndef IMU_SCL
#define IMU_SCL 18
#endif

#endif

#ifndef IMU_IRQ
#define IMU_IRQ 33
#endif

#ifdef ARDUINO_T_BEAM_S3_SUPREME
#include <XPowersAXP2101.tpp>
#endif

static void serialPrintFmt(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
}

void setupPower()
{
#if defined(ARDUINO_T_BEAM_S3_SUPREME)
    XPowersAXP2101 power;
    power.begin(Wire1, AXP2101_SLAVE_ADDRESS, 42, 41);
    power.disableALDO1();
    power.disableALDO2();
    delay(250);
    power.setALDO1Voltage(3300);
    power.enableALDO1();
    power.setALDO2Voltage(3300);
    power.enableALDO2();
#endif
}

SensorQMI8658 imu;



void setup()
{
    Serial.begin(115200);
    while (!Serial);

    setupPower();

    Serial.println("QMI8658 Advanced Features Example");

#ifdef USE_I2C_INTERFACE
    if (!imu.begin(Wire, QMI8658_H_SLAVE_ADDRESS, IMU_SDA, IMU_SCL)) {
        Serial.println("Failed to initialize QMI8658!");
        while (1) {
            delay(1000);
        }
    }
#endif

#ifdef USE_SPI_INTERFACE
    // Using SPI interface
    if (!imu.begin(SPI, IMU_CS, SPI_MOSI, SPI_MISO, SPI_SCK)) {
        Serial.println("Failed to initialize QMI8658!");
        while (1) {
            delay(1000);
        }
    }
#endif

    Serial.println("\n=== Hardware Self-Test ===");
    Serial.println("Running hardware self-test...");
    if (imu.hardwareSelfTest(true, true)) {
        Serial.println("Hardware self-test: PASSED");
        float accel_result[3], gyro_result[3];
        imu.getHardwareSelfTestResults(accel_result, gyro_result);
        serialPrintFmt("Accel ST results: X=%.1f Y=%.1f Z=%.1f mg\n",
                       accel_result[0], accel_result[1], accel_result[2]);
        serialPrintFmt("Gyro ST results: X=%.1f Y=%.1f Z=%.1f dps\n",
                       gyro_result[0], gyro_result[1], gyro_result[2]);
    } else {
        Serial.println("Hardware self-test: FAILED");
    }

    Serial.println("\n=== Helper Configuration Functions ===");
    Serial.println("configMotionDetectDefault() - Auto-configure motion detection");
    imu.configMotionDetectDefault(SensorQMI8658::MotionType::ANY_MOTION);
    Serial.println("configTapDefault() - Auto-configure tap detection");
    imu.configTapDefault();
    Serial.println("configPedometerDefault(125) - Auto-configure pedometer at 125Hz");
    imu.configPedometerDefault(125.0f);
    Serial.println("Helper functions configured successfully!\n");

    Serial.println("\n=== Axis Layout Configuration ===");
    Serial.println("Current layout: DEFAULT");
    imu.setAxisLayout(SensorQMI8658::Layout::LAYOUT_DEFAULT);
    serialPrintFmt("Current layout value: %d\n", (int)imu.getAxisLayout());

    Serial.println("\nSupported layouts:");
    Serial.println("  0 = DEFAULT (X=right, Y=forward, Z=up)");
    Serial.println("  1 = ROTATE_90 (X=forward, Y=left, Z=up)");
    Serial.println("  2 = ROTATE_180 (X=left, Y=backward, Z=up)");
    Serial.println("  3 = ROTATE_270 (X=backward, Y=right, Z=up)");
    Serial.println("  4 = FLIP_Z (X=right, Y=backward, Z=down)");
    Serial.println("  5 = FLIP_Z_90 (X=backward, Y=left, Z=down)");
    Serial.println("  6 = FLIP_Z_180 (X=left, Y=forward, Z=down)");
    Serial.println("  7 = FLIP_Z_270 (X=forward, Y=right, Z=down)");

    Serial.println("\n=== Static Calibration ===");
    Serial.println("Place sensor in stable position and keep it still!");
    Serial.println("Static calibration will run for ~100 samples (blocking)...");

    // Accelerometer: FS_2G(000), FS_4G(001), FS_8G(010), FS_16G(011)
    // ODR: 1000, 500, 250, 125, 62.5, 31.25, 128, 21, 11, 3 Hz (6DOF: 448/224/112/56/28 Hz)
    imu.configAccel(AccelFullScaleRange::FS_8G, 100.0f);
    // Gyroscope: FS_125_DPS, FS_250_DPS, FS_500_DPS, FS_1000_DPS, FS_2000_DPS, FS_4000_DPS
    // ODR: 7174/3587/1793/896/448/224/112/56/28 Hz
    imu.configGyro(GyroFullScaleRange::FS_1000_DPS, 100.0f);
    // When ACC and GYR are both enabled, effective ODR base
    // is synchronized to gyroscope natural frequency.
    imu.enableAccel();
    imu.enableGyro();

    imu.enableDynamicGyroCalibration(false);
    imu.enableStaticCalibration(true);

    uint32_t cali_start = millis();
    uint32_t last_progress = 0;
    while (!imu.isStaticCalibrationComplete()) {
        if (imu.isDataReady()) {
            AccelerometerData accel;
            GyroscopeData gyro;
            imu.readAccel(accel);
            imu.readGyro(gyro);
        }

        uint32_t now = millis();
        if (now - last_progress >= 250) {
            Serial.print(".");
            last_progress = now;
        }

        if (now - cali_start > 10000) {
            Serial.println("\nStatic calibration timeout, continue with current offsets.");
            break;
        }
        delay(2);
    }

    int16_t accel_offset[3], gyro_offset[3];
    imu.getStaticCalibrationOffsets(accel_offset, gyro_offset);
    Serial.println("\nStatic calibration done.");
    serialPrintFmt("Static Cal Offsets - Accel: %d %d %d  Gyro: %d %d %d\n",
                   accel_offset[0], accel_offset[1], accel_offset[2],
                   gyro_offset[0], gyro_offset[1], gyro_offset[2]);

    Serial.println("\n=== Dynamic Gyroscope Calibration ===");
    Serial.println("Dynamic gyro calibration enabled.");
    Serial.println("Offsets will be updated when sensor is stationary.");
    imu.enableDynamicGyroCalibration(true);

    Serial.println("\n=== Data Output ===");
    Serial.println("Accel(m/s2)              | Gyro(dps)                | Calibrated");
    Serial.println("X        Y        Z      | X        Y        Z     | Accel  Gyro");
}

void loop()
{
    if (imu.isDataReady()) {
        AccelerometerData accel;
        GyroscopeData gyro;
        imu.readAccel(accel);
        imu.readGyro(gyro);

        serialPrintFmt("%7.3f %7.3f %7.3f | %7.3f %7.3f %7.3f | %s  %s\n",
                       accel.mps2.x, accel.mps2.y, accel.mps2.z,
                       gyro.dps.x, gyro.dps.y, gyro.dps.z,
                       imu.isStaticCalibrationComplete() ? "Yes " : "No  ",
                       imu.isDynamicGyroCalibrationEnabled() ? "Yes" : "No");

        if (millis() % 5000 < 100 && imu.isStaticCalibrationComplete()) {
            int16_t accel_offset[3], gyro_offset[3];
            imu.getStaticCalibrationOffsets(accel_offset, gyro_offset);
            serialPrintFmt("\nStatic Cal Offsets - Accel: %d %d %d  Gyro: %d %d %d\n",
                           accel_offset[0], accel_offset[1], accel_offset[2],
                           gyro_offset[0], gyro_offset[1], gyro_offset[2]);

            int16_t dyn_gyro_offset[3];
            imu.getDynamicGyroCalibrationOffsets(dyn_gyro_offset);
            serialPrintFmt("Dynamic Gyro Offsets: %d %d %d\n",
                           dyn_gyro_offset[0], dyn_gyro_offset[1], dyn_gyro_offset[2]);
        }
    }
    delay(10);
}

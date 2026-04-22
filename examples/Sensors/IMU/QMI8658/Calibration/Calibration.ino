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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      Calibration.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-15
 *
 * @brief     Gyroscope calibration example using SensorQMI8658.
 *
 * @details   This example demonstrates how to:
 *            - Perform on-demand gyroscope calibration
 *            - Read and save calibration gains
 *            - Apply saved calibration gains
 *            - Set accelerometer and gyroscope offsets
 *
 * @note      Place the sensor flat and motionless during calibration.
 *            Calibration corrects for sensor bias and gain errors.
 */
#include <stdarg.h>
#include "SensorDrv.hpp"
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
bool staticCalDonePrinted = false;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    setupPower();

    Serial.println("QMI8658 Calibration Example");

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

    // Accelerometer: FS_2G(000), FS_4G(001), FS_8G(010), FS_16G(011)
    // ODR: 1000, 500, 250, 125, 62.5, 31.25, 128, 21, 11, 3 Hz (6DOF: 448/224/112/56/28 Hz)
    imu.configAccel(AccelFullScaleRange::FS_8G,
                   1000.0f);

    // Gyroscope: FS_125_DPS, FS_250_DPS, FS_500_DPS, FS_1000_DPS, FS_2000_DPS, FS_4000_DPS
    // ODR: 7174/3587/1793/896/448/224/112/56/28 Hz
    imu.configGyro(GyroFullScaleRange::FS_1000_DPS,
                  1000.0f);

    // In 6DOF mode (ACC+GYR both enabled), ODR synchronization
    // is derived from gyroscope natural frequency.
    imu.enableAccel();
    imu.enableGyro();

    Serial.println("\n=== On-Demand Calibration ===");
    Serial.println("\n--- Before Calibration ---");
    for (int i = 0; i < 10; ++i) {
        GyroscopeData gyro;
        imu.readGyro(gyro);
        serialPrintFmt("Gyro: %8.3f %8.3f %8.3f\n", gyro.dps.x, gyro.dps.y, gyro.dps.z);
        delay(100);
    }

    Serial.println("\nStarting calibration... Please keep sensor motionless!");
    delay(1000);

    uint16_t gyro_x, gyro_y, gyro_z;
    if (imu.calibrate(&gyro_x, &gyro_y, &gyro_z)) {
        Serial.println("Calibration successful!");
        serialPrintFmt("Gyro gains - X: 0x%04X, Y: 0x%04X, Z: 0x%04X\n",
                     gyro_x, gyro_y, gyro_z);

        delay(500);

        if (imu.writeCalibration(gyro_x, gyro_y, gyro_z)) {
            Serial.println("Calibration gains applied!");
        }

        // On-demand calibration path disables sensors; re-enable runtime streaming.
        bool enA = imu.enableAccel();
        bool enG = imu.enableGyro();
        serialPrintFmt("Runtime re-enable: accel=%s gyro=%s\n", enA ? "OK" : "FAIL", enG ? "OK" : "FAIL");
    } else {
        Serial.println("Calibration failed!");
    }

    Serial.println("\n--- After Calibration ---");
    for (int i = 0; i < 10; ++i) {
        GyroscopeData gyro;
        imu.readGyro(gyro);
        serialPrintFmt("Gyro: %8.3f %8.3f %8.3f\n", gyro.dps.x, gyro.dps.y, gyro.dps.z);
        delay(100);
    }

    Serial.println("\n=== Static Calibration (Software) ===");
    Serial.println("Enable software-based static calibration:");
    Serial.println("- Collects 100 samples when sensor is stationary");
    Serial.println("- Calculates accelerometer and gyroscope offsets");
    imu.enableStaticCalibration(true);

    Serial.println("\n=== Dynamic Gyro Calibration ===");
    Serial.println("Enable dynamic gyroscope calibration:");
    Serial.println("- Continuously updates offsets when stationary detected");
    Serial.println("- Uses accelerometer variance to detect stillness");
    imu.enableDynamicGyroCalibration(true);

    Serial.println("\nWaiting for static calibration to complete (about 10s @100ms loop)...");

    Serial.println("\nCalibration complete. Data is now being processed.");
}

void loop()
{
    static uint32_t failCount = 0;

    GyroscopeData gyro;
    if (imu.readGyro(gyro)) {
        failCount = 0;
        if (!staticCalDonePrinted && imu.isStaticCalibrationComplete()) {
            int16_t accelOff[3], gyroOff[3];
            imu.getStaticCalibrationOffsets(accelOff, gyroOff);
            serialPrintFmt("[CAL] Static calibration complete. Gyro offsets: %d %d %d\n",
                          gyroOff[0], gyroOff[1], gyroOff[2]);
            staticCalDonePrinted = true;
        }
        serialPrintFmt("Gyro: %8.3f %8.3f %8.3f\n", gyro.dps.x, gyro.dps.y, gyro.dps.z);
    } else {
        failCount++;
        if ((failCount % 20) == 1) {
            bool enG = imu.enableGyro();
            serialPrintFmt("[WARN] readGyro failed (%lu), trying enableGyro: %s\n",
                          static_cast<unsigned long>(failCount), enG ? "OK" : "FAIL");
        }
    }
    delay(100);
}

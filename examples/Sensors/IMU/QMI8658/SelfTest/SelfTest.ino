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
 * @file      SelfTest.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-15
 *
 * @brief     Self-test example using SensorQMI8658.
 *
 * @details   This example demonstrates how to:
 *            - Perform accelerometer self-test
 *            - Perform gyroscope self-test
 *            - Verify sensor functionality
 *
 * @note      Self-test activates internal test structures in the sensors
 *            and verifies the output is within expected ranges.
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
volatile bool isInterruptTriggered = false;

static void printAccelSample(const char *tag)
{
    AccelerometerData accel;
    if (imu.readAccel(accel)) {
        serialPrintFmt("[%s] Accel: %7.3f %7.3f %7.3f\n",
                       tag,
                       accel.mps2.x, accel.mps2.y, accel.mps2.z);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    setupPower();

    Serial.println("QMI8658 Self-Test Example");

#ifdef USE_I2C_INTERFACE
    if (!imu.begin(Wire, QMI8658_DEFAULT_I2C_ADDR_HIGH, IMU_SDA, IMU_SCL)) {
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

    imu.setPins(IMU_IRQ);

    Serial.println("\n=== Software Self-Test ===");
    Serial.println("\n--- Accelerometer Self-Test ---");
    if (imu.selfTestAccel()) {
        Serial.println("Accelerometer self-test: PASSED");
    } else {
        Serial.println("Accelerometer self-test: FAILED");
    }

    Serial.println("\n--- Gyroscope Self-Test ---");
    if (imu.selfTestGyro()) {
        Serial.println("Gyroscope self-test: PASSED");
    } else {
        Serial.println("Gyroscope self-test: FAILED");
    }

    Serial.println("\n--- Combined Self-Test ---");
    if (imu.selfTest()) {
        Serial.println("Combined self-test: PASSED");
    } else {
        Serial.println("Combined self-test: FAILED");
    }

    Serial.println("\n=== Hardware Self-Test ===");
    Serial.println("Hardware self-test uses different methodology:");
    Serial.println("- Uses internal test structures in the sensors");
    Serial.println("- Returns raw output values for verification");
    Serial.println("- Can test accel and gyro independently");

    Serial.println("\nRunning hardware self-test (accel only)...");
    if (imu.hardwareSelfTest(true, false)) {
        Serial.println("Hardware self-test accel: PASSED");
        float accel_result[3], gyro_result[3];
        imu.getHardwareSelfTestResults(accel_result, gyro_result);
        serialPrintFmt("Accel ST results: X=%.1f Y=%.1f Z=%.1f mg\n",
                       accel_result[0], accel_result[1], accel_result[2]);
    } else {
        Serial.println("Hardware self-test accel: FAILED");
    }

    Serial.println("\nRunning hardware self-test (gyro only)...");
    if (imu.hardwareSelfTest(false, true)) {
        Serial.println("Hardware self-test gyro: PASSED");
        float accel_result[3], gyro_result[3];
        imu.getHardwareSelfTestResults(accel_result, gyro_result);
        serialPrintFmt("Gyro ST results: X=%.1f Y=%.1f Z=%.1f dps\n",
                       gyro_result[0], gyro_result[1], gyro_result[2]);
    } else {
        Serial.println("Hardware self-test gyro: FAILED");
    }

    Serial.println("\nSelf-test notes from QMI8658X:");
    Serial.println("- Self-test result is read from dVX/dVY/dVZ (0x51~0x56), not normal data registers.");
    Serial.println("- Accel self-test threshold: |dV*| > 200mg on all three axes.");
    Serial.println("- Gyro self-test threshold: |dV*| > 300dps on all three axes.");
    Serial.println("- During self-test, device internally controls FS/ODR and restores CTRL2/CTRL3 afterward.");

    // Reinitialize runtime data path after self-test sequence.
    // REG-QMI8658A notes CTRL1.ADDR_AI defaults to 0; burst reads require ADDR_AI=1.
    // Driver reset path enables ADDR_AI, so do a clean reset before normal streaming.
    if (!imu.reset()) {
        Serial.println("WARNING: reset after self-test failed");
    }

    // Accelerometer: FS_2G(000), FS_4G(001), FS_8G(010), FS_16G(011)
    // ODR: 1000, 500, 250, 125, 62.5, 31.25, 128, 21, 11, 3 Hz (6DOF: 448/224/112/56/28 Hz)
    bool cfgA = imu.configAccel(AccelFullScaleRange::FS_8G, 1000.0f);
    // Gyroscope: FS_125_DPS, FS_250_DPS, FS_500_DPS, FS_1000_DPS, FS_2000_DPS, FS_4000_DPS
    // ODR: 7174/3587/1793/896/448/224/112/56/28 Hz
    bool cfgG = imu.configGyro(GyroFullScaleRange::FS_1000_DPS, 1000.0f);
    // With ACC+GYR enabled together, synchronized ODR base
    // is derived from gyroscope natural frequency.
    bool enA = imu.enableAccel();
    bool enG = imu.enableGyro();
    serialPrintFmt("Runtime config: accel=%s gyro=%s enableA=%s enableG=%s\n",
                   cfgA ? "OK" : "FAIL",
                   cfgG ? "OK" : "FAIL",
                   enA ? "OK" : "FAIL",
                   enG ? "OK" : "FAIL");

    // Self-test flow may change control registers; enable DRDY interrupt at the end.
    // NOTE: QMI8658 DRDY can only be mapped to INT2.
    bool drdy_ok = imu.enableDataReadyInterrupt(SensorQMI8658::IntPin::PIN2);

    int irq = digitalPinToInterrupt(IMU_IRQ);
    if (irq >= 0) {
        attachInterrupt(irq, []() {
            isInterruptTriggered = true;
        }, CHANGE);
        serialPrintFmt("IRQ attached on pin %d (CHANGE trigger)\n", IMU_IRQ);
    } else {
        serialPrintFmt("IRQ pin %d has no interrupt mapping, polling only\n", IMU_IRQ);
    }

    Serial.println("\nData-ready interrupt enabled on INT2 (PIN2). Waiting for IRQ...");
    Serial.println("Note: DRDY requires FIFO bypass mode; enabling FIFO will disable DRDY.");
    Serial.println("Note: If your board only wires INT1, DRDY IRQ will not arrive; polling output is used.");
    serialPrintFmt("DRDY enable result: %s\n", drdy_ok ? "OK" : "FAIL");
    Serial.println("If no IRQ is seen, example will fallback to polling output.");
}

void loop()
{
    static uint32_t lastPollMs = 0;

    if (isInterruptTriggered) {
        isInterruptTriggered = false;

        // Clear/consume status bits and read data after hardware IRQ.
        imu.update();

        if (imu.isDataReady(static_cast<uint8_t>(ImuBase::DataReadyMask::ACCEL))) {
            printAccelSample("IRQ");
        }
    } else if (millis() - lastPollMs > 100) {
        // Fallback path for boards that do not wire INT2 DRDY.
        lastPollMs = millis();
        printAccelSample("POLL");
    }

    delay(5);
}

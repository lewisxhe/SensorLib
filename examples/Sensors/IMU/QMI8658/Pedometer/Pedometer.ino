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
 * @file      Pedometer.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-15
 *
 * @brief     Pedometer (step counter) example using SensorQMI8658.
 *
 * @details   This example demonstrates how to:
 *            - Configure pedometer parameters
 *            - Enable and disable pedometer
 *            - Read step count
 *            - Reset step count
 *
 * @note      The pedometer counts steps based on accelerometer patterns.
 *            Calibration may be needed for accurate step detection.
 */
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <Wire.h>
#include "SensorDrv.hpp"
#include "DevicesPins.h"


// #define USE_I2C_INTERFACE        true
// #define USE_SPI_INTERFACE        true

#if !defined(USE_I2C_INTERFACE) && !defined(USE_SPI_INTERFACE)
#define USE_I2C_INTERFACE
#warning "No interface type is selected, use I2C interface"
#endif

#if defined(USE_SPI_INTERFACE)

#ifndef SPI_MOSI
#define SPI_MOSI    35
#endif

#ifndef SPI_MISO
#define SPI_MISO    37
#endif

#ifndef SPI_SCK
#define SPI_SCK     36
#endif

#ifndef IMU_CS
#define IMU_CS      34
#endif

#ifndef IMU_IRQ
#define IMU_IRQ      33
#endif

#else /* USE_I2C_INTERFACE */

#ifndef IMU_SDA
#define IMU_SDA      17
#endif

#ifndef IMU_SCL
#define IMU_SCL      18
#endif

#ifndef IMU_IRQ
#define IMU_IRQ       33
#endif

#endif /* USE_SPI_INTERFACE */


SensorQMI8658 imu;
uint32_t lastSteps = 0;
volatile bool isInterruptTriggered = false;

#ifdef ARDUINO_T_BEAM_S3_SUPREME
#include <XPowersAXP2101.tpp>   //PMU Library https://github.com/lewisxhe/XPowersLib.git
#endif
__attribute__ ((weak))  void setupPower()
{
    // T_BEAM_S3_SUPREME The PMU voltage needs to be turned on to use the sensor
#if defined(ARDUINO_T_BEAM_S3_SUPREME)
    XPowersAXP2101 power;
    power.begin(Wire1, AXP2101_SLAVE_ADDRESS, 42, 41);
    power.disableALDO1();
    power.disableALDO2();
    delay(250);
    power.setALDO1Voltage(3300); power.enableALDO1();
    power.setALDO2Voltage(3300); power.enableALDO2();
#endif

}

void pedometerCallback()
{
    Serial.println("[PEDOMETER] Step detected!");
    uint32_t steps = imu.getStepCount();
    if (steps != lastSteps) {
        serialPrintFmt("[PEDOMETER]  Steps: %u\n", steps);
        lastSteps = steps;
    }
}

static void serialPrintFmt(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    setupPower();

    Serial.println("QMI8658 Pedometer Example");

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

    // Keep ACC+GYR both enabled, matching vendor pedometer runtime path.
    // In 6DOF mode, synchronized ODR base follows gyro natural frequency.
    // Accelerometer: FS_2G(000), FS_4G(001), FS_8G(010), FS_16G(011)
    // ODR: 1000, 500, 250, 125, 62.5, 31.25, 128, 21, 11, 3 Hz (6DOF: 448/224/112/56/28 Hz)
    imu.configAccel(AccelFullScaleRange::FS_4G,
                    112.0f);
    // Gyroscope: FS_125_DPS, FS_250_DPS, FS_500_DPS, FS_1000_DPS, FS_2000_DPS, FS_4000_DPS
    // ODR: 7174/3587/1793/896/448/224/112/56/28 Hz
    imu.configGyro(GyroFullScaleRange::FS_1000_DPS,
                   112.0f);

    imu.enableAccel();
    imu.enableGyro();

    Serial.println("\nUsing pedometer debug profile (easier trigger than strict walking profile)");
    // Datasheet profile is conservative and designed to reject non-step vibration.
    // For handheld bring-up, use a lower entry count and lower thresholds.
    // sample_cnt=50, peak2peak=80mg, peak=60mg, time_up=400, time_low=8,
    // entry=1, precision=0, sig_count=1
    bool cfg_ok = imu.configPedometer(50, 80, 60, 400, 8, 1, 0, 1);
    serialPrintFmt("configPedometer: %s\n", cfg_ok ? "OK" : "FAIL");

    imu.resetStepCount();
    bool ped_ok = imu.enablePedometer(SensorQMI8658::IntPin::PIN1);
    serialPrintFmt("enablePedometer: %s\n", ped_ok ? "OK" : "FAIL");

    Serial.println("\nRegister snapshot after pedometer setup:");
    imu.dumpRegisters();

    imu.setPedometerCallback(pedometerCallback);

    Serial.println("\nPedometer configured. Start walking!");
    Serial.println("Tip: step engine is for periodic gait, not random shaking.");
    Serial.println("For handheld debug: simulate walking cadence (1-2Hz up/down motion).\n");
    Serial.println("Note: Pedometer callback is triggered by step-count change, not only by status bit.");

    int irq = digitalPinToInterrupt(IMU_IRQ);
    if (irq >= 0) {
        attachInterrupt(irq, []() {
            isInterruptTriggered = true;
        }, CHANGE);
        serialPrintFmt("IRQ attached on pin %d (CHANGE trigger)\n", IMU_IRQ);
    } else {
        serialPrintFmt("IRQ pin %d has no interrupt mapping, polling only\n", IMU_IRQ);
    }

}

void loop()
{
    // Fallback poll period when no IRQ arrives (ms).
    constexpr uint32_t kPollIntervalMs = 50;
    static uint32_t lastPollMs = 0;

    bool doUpdate = false;
    if (isInterruptTriggered) {
        isInterruptTriggered = false;
        doUpdate = true;
    }

    uint32_t now = millis();
    if (!doUpdate && (now - lastPollMs >= kPollIntervalMs)) {
        doUpdate = true;
    }

    if (doUpdate) {
        lastPollMs = now;
        imu.update();
    }
    delay(10);
}

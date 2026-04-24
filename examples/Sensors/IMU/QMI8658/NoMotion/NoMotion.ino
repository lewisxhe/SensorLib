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
* @file      NoMotion.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-15
 *
 * @brief     NoMotion example using SensorQMI8658.
 *
 * @details   This example demonstrates how to:
 *            - Configure no-motion detection
 *            - Set motion detection thresholds and time window
 *            - Handle no-motion detection callbacks
 *
 * @important NoMotion Notes:
 *            - No-motion triggers when acceleration stays below threshold for set time.
 *            - The detection threshold and time window can be tuned for sensitivity.
 *            - Uses hardware interrupt callback for low-power operation.
 *
 * @usage
 *            Connect the QMI8658 sensor via I2C or SPI and upload this sketch.
 *            Open the Serial Monitor to see no-motion detection events.
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
volatile bool irqTriggered = false;

constexpr float kAccelOdrHz = 224.0f;
// Any-motion companion parameters (used to leave no-motion state when movement starts):
// - thresholdX/Y/Z (mg): larger value means less sensitive to movement.
// - window (samples): consecutive above-threshold samples required.
constexpr float kAnyThresholdMg = 62.5f;
constexpr uint8_t kAnyWindow = 3;
// No-motion parameters:
// - thresholdX/Y/Z (mg): acceleration must stay below this value.
// - window (samples): consecutive below-threshold samples required to assert no-motion.
constexpr float kNoThresholdMg = 15.625f;
constexpr uint8_t kNoWindow = 30;
// Fallback poll period when no IRQ arrives (ms).
constexpr uint32_t kPollIntervalMs = 50;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    setupPower();
    Serial.println("QMI8658 No-Motion Example");

#ifdef USE_I2C_INTERFACE
    if (!imu.begin(Wire, QMI8658_H_SLAVE_ADDRESS, IMU_SDA, IMU_SCL)) {
        Serial.println("Failed to initialize QMI8658!");
        while (1) delay(1000);
    }
#endif

#ifdef USE_SPI_INTERFACE
    if (!imu.begin(SPI, IMU_CS, SPI_MOSI, SPI_MISO, SPI_SCK)) {
        Serial.println("Failed to initialize QMI8658!");
        while (1) delay(1000);
    }
#endif

    imu.setPins(IMU_IRQ);
    // Accelerometer: FS_2G(000), FS_4G(001), FS_8G(010), FS_16G(011)
    // ODR: 1000, 500, 250, 125, 62.5, 31.25, 128, 21, 11, 3 Hz (6DOF: 448/224/112/56/28 Hz)
    imu.configAccel(AccelFullScaleRange::FS_8G, kAccelOdrHz);
    imu.enableAccel();

    // Keep Any-Motion engine configured so movement can drive state away from No-Motion.
    // We only report No-Motion in this example.
    bool cfgAny = imu.configMotionDetect(SensorQMI8658::MotionType::ANY_MOTION,
                                         kAnyThresholdMg, kAnyThresholdMg, kAnyThresholdMg,
                                         kAnyWindow);
    bool cfg = imu.configMotionDetect(SensorQMI8658::MotionType::NO_MOTION,
                                      kNoThresholdMg, kNoThresholdMg, kNoThresholdMg,
                                      kNoWindow);
    bool en = imu.enableMotionDetect(SensorQMI8658::IntPin::PIN1);

    int irq = digitalPinToInterrupt(IMU_IRQ);
    if (irq >= 0) {
        attachInterrupt(irq, []() {
            irqTriggered = true;
        }, CHANGE);
    }

    serialPrintFmt("No-Motion config any=%s no=%s enable=%s\n",
                  cfgAny ? "OK" : "FAIL",
                  cfg ? "OK" : "FAIL",
                  en ? "OK" : "FAIL");
    Serial.println("Keep sensor still to trigger No-Motion.");


    imu.setNoMotionCallback([]() {
        Serial.println("[EVENT] No motion detected!");
    });
}

void loop()
{
    static uint32_t lastPollMs = 0;

    bool doUpdate = false;
    if (irqTriggered) {
        irqTriggered = false;
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

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
 * @file      WakeOnMotion.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-15
 *
 * @brief     Wake-on-motion example using SensorQMI8658.
 *
 * @details   This example demonstrates how to:
 *            - Configure wake-on-motion (WoM) detection
 *            - Set up callbacks for WoM events
 *            - Use low-power accelerometer mode for power efficiency
 *
 * @important WoM Notes:
 *            - In Non-SyncSample mode, DRDY routing and WoM routing are different paths.
 *            - This example uses hardware interrupt callback on a dedicated interrupt pin.
 *            - `configWakeOnMotionAdvanced(...)` exposes vendor-style parameters:
 *              threshold, ODR, interrupt route, default pin level, blanking time, accel range.
 *            - Runtime serial tuning commands are provided in this example for quick debug.
 *
 * @note      Wake-on-motion is ideal for battery-powered applications
 *            where the system should wake up only when motion is detected.
 *            The accelerometer runs in low-power mode when not triggered.
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


uint32_t lastPollMs = 0;
volatile bool isInterruptTriggered = false;
constexpr uint32_t kPollIntervalMs = 50;
uint8_t womThreshold = 150;
float womOdrHz = 11.0f;
uint8_t womDefaultPinValue = 1;
uint8_t womBlankingTime = 0x20;
AccelFullScaleRange womRange = AccelFullScaleRange::FS_8G;
SensorQMI8658::IntPin womIntPin = SensorQMI8658::IntPin::PIN2;

void womCallback()
{
    Serial.print("["); Serial.print(millis() / 1000);
    Serial.println("][WOM] Wake-on-motion detected!");
}

void applyWomConfig()
{
    // womRange: FS_2G, FS_4G, FS_8G, FS_16G
    // womOdrHz: 3, 11, 21, 128 Hz
    bool ok = imu.configWakeOnMotionAdvanced(womThreshold,
              womOdrHz,
              womIntPin,
              womDefaultPinValue,
              womBlankingTime,
              womRange);
    serialPrintFmt("[WOM CFG] thr=%u odr=%.1f pinDefault=%u blank=0x%02X range=%s -> %s\n",
                   womThreshold,
                   womOdrHz,
                   womDefaultPinValue,
                   womBlankingTime,
                   (womRange == AccelFullScaleRange::FS_2G) ? "2G" :
                   (womRange == AccelFullScaleRange::FS_4G) ? "4G" :
                   (womRange == AccelFullScaleRange::FS_8G) ? "8G" : "16G",
                   ok ? "OK" : "FAIL");
}

void printWomHelp()
{
    Serial.println("\nWoM debug commands:");
    Serial.println("  t+ / t-   : threshold +/- 10");
    Serial.println("  b+ / b-   : blanking +/- 1");
    Serial.println("  p0 / p1   : default pin level 0/1");
    Serial.println("  o3/o11/o21/o128 : ODR 3/11/21/128Hz");
    Serial.println("  r2/r4/r8/r16    : accel range 2/4/8/16G");
    Serial.println("  s         : show/apply current config");
    Serial.println("  h         : show this help");
}

void handleWomCommands()
{
    if (!Serial.available()) {
        return;
    }

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) {
        return;
    }

    bool changed = false;

    if (cmd == "t+") {
        womThreshold = (womThreshold <= 245) ? womThreshold + 10 : 255;
        changed = true;
    } else if (cmd == "t-") {
        womThreshold = (womThreshold >= 10) ? womThreshold - 10 : 0;
        changed = true;
    } else if (cmd == "b+") {
        womBlankingTime = (womBlankingTime < 0x3F) ? (womBlankingTime + 1) : 0x3F;
        changed = true;
    } else if (cmd == "b-") {
        womBlankingTime = (womBlankingTime > 0) ? (womBlankingTime - 1) : 0;
        changed = true;
    } else if (cmd == "p0") {
        womDefaultPinValue = 0;
        changed = true;
    } else if (cmd == "p1") {
        womDefaultPinValue = 1;
        changed = true;
    } else if (cmd == "o3") {
        womOdrHz = 3.0f;
        changed = true;
    } else if (cmd == "o11") {
        womOdrHz = 11.0f;
        changed = true;
    } else if (cmd == "o21") {
        womOdrHz = 21.0f;
        changed = true;
    } else if (cmd == "o128") {
        womOdrHz = 128.0f;
        changed = true;
    } else if (cmd == "r2") {
        womRange = AccelFullScaleRange::FS_2G;
        changed = true;
    } else if (cmd == "r4") {
        womRange = AccelFullScaleRange::FS_4G;
        changed = true;
    } else if (cmd == "r8") {
        womRange = AccelFullScaleRange::FS_8G;
        changed = true;
    } else if (cmd == "r16") {
        womRange = AccelFullScaleRange::FS_16G;
        changed = true;
    } else if (cmd == "s") {
        applyWomConfig();
    } else if (cmd == "h") {
        printWomHelp();
    } else {
        Serial.println("Unknown command, input 'h' for help.");
    }

    if (changed) {
        applyWomConfig();
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    setupPower();

    Serial.println("QMI8658 Wake-on-Motion Example");

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

    imu.setPins(IMU_IRQ);

    imu.setWakeOnMotionCallback(womCallback);

    Serial.println("\n=== Advanced Usage ===");
    Serial.println("Custom: threshold=150, ODR=11Hz, PIN2, pinDefault=1, blanking=0x20, range=8G");
    applyWomConfig();

    Serial.println("\nWake-on-motion configured. Move the sensor to trigger!");
    printWomHelp();

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
    handleWomCommands();

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

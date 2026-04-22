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
 * @file      TapDetection.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-15
 *
 * @brief     Tap detection example using SensorQMI8658.
 *
 * @details   This example demonstrates how to:
 *            - Configure single and double tap detection
 *            - Set up tap detection thresholds
 *            - Handle tap detection callbacks
 *
 * @important Tap Tuning Notes:
 *            - Tap sensitivity strongly depends on board mechanics and mounting.
 *            - Use high enough ODR (typically >= 500Hz) for reliable tap recognition.
 *            - Prefer tuning parameters first (peak/quiet thresholds, tap windows)
 *              before adding software-side tap reclassification.
 *            - Use serial debug commands in this example for runtime tuning:
 *              p+/p-, q+/q-, w+/w-, d+/d-, s, h.
 *
 * @note      Tap detection is useful for gesture recognition and
 *            user interface control through tap gestures.
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

float peakThreshold = 0.3f;
float quietThreshold = 0.18f;
uint16_t tapWindow = 55;
uint16_t doubleTapWindow = 140;
uint8_t peakWindow = 20;



void tapCallback(SensorQMI8658::TapEvent event)
{
    switch (event) {
        case SensorQMI8658::TapEvent::SINGLE:
            Serial.println("[TAP] Single tap detected!");
            break;
        case SensorQMI8658::TapEvent::DOUBLE:
            Serial.println("[TAP] Double tap detected!");
            break;
        default:
            Serial.println("[TAP] Invalid tap");
            break;
    }
}

void applyTapConfig()
{
    imu.disableTap();
    delay(20);

    imu.configTap(SensorQMI8658::TapPriority::Z_GT_Y_GT_X,
                  peakWindow,
                  tapWindow,
                  doubleTapWindow,
                  peakThreshold,
                  quietThreshold);

    imu.enableTap(SensorQMI8658::IntPin::PIN1);

    serialPrintFmt("[TAP CFG] peak=%.3f quiet=%.3f peakWin=%u tapWin=%u dblWin=%u\n",
                  peakThreshold, quietThreshold, peakWindow, tapWindow, doubleTapWindow);
}

void printCommandHelp()
{
    Serial.println("\nTap debug commands:");
    Serial.println("  p+ / p-   : peak threshold +/- 0.05 g^2");
    Serial.println("  q+ / q-   : quiet threshold +/- 0.02 g^2");
    Serial.println("  w+ / w-   : tap window +/- 5 samples");
    Serial.println("  d+ / d-   : double tap window +/- 10 samples");
    Serial.println("  s         : show current tap config");
    Serial.println("  h         : show this help");
    Serial.println("Single-key shortcuts (no Enter needed):");
    Serial.println("  1:p- 2:p+ 3:q- 4:q+ 5:w- 6:w+ 7:d- 8:d+");
}

bool applyCommand(const String &cmd)
{
    bool changed = false;

    if (cmd == "p+") {
        peakThreshold += 0.05f;
        changed = true;
    } else if (cmd == "p-") {
        peakThreshold -= 0.05f;
        if (peakThreshold < 0.1f) peakThreshold = 0.1f;
        changed = true;
    } else if (cmd == "q+") {
        quietThreshold += 0.02f;
        changed = true;
    } else if (cmd == "q-") {
        quietThreshold -= 0.02f;
        if (quietThreshold < 0.05f) quietThreshold = 0.05f;
        changed = true;
    } else if (cmd == "w+") {
        tapWindow += 5;
        changed = true;
    } else if (cmd == "w-") {
        tapWindow = (tapWindow > 10) ? (tapWindow - 5) : 5;
        changed = true;
    } else if (cmd == "d+") {
        doubleTapWindow += 10;
        changed = true;
    } else if (cmd == "d-") {
        doubleTapWindow = (doubleTapWindow > 20) ? (doubleTapWindow - 10) : 10;
        changed = true;
    } else if (cmd == "s") {
        applyTapConfig();
    } else if (cmd == "h") {
        printCommandHelp();
    } else {
        return false;
    }

    if (changed) {
        applyTapConfig();
    }

    return true;
}

void handleSerialCommands()
{
    static String lineBuf;
    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\r' || c == '\n') {
            lineBuf.trim();
            if (lineBuf.length() > 0) {
                if (!applyCommand(lineBuf)) {
                    Serial.println("Unknown command, input 'h' for help.");
                }
            }
            lineBuf = "";
            continue;
        }

        // Single-key quick adjust (works without line ending)
        if (c == 'h' || c == 's') {
            String cmd = String(c);
            applyCommand(cmd);
            continue;
        }
        if (c >= '1' && c <= '8') {
            switch (c) {
                case '1': applyCommand("p-"); break;
                case '2': applyCommand("p+"); break;
                case '3': applyCommand("q-"); break;
                case '4': applyCommand("q+"); break;
                case '5': applyCommand("w-"); break;
                case '6': applyCommand("w+"); break;
                case '7': applyCommand("d-"); break;
                case '8': applyCommand("d+"); break;
            }
            continue;
        }

        if (isPrintable(c)) {
            lineBuf += c;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    setupPower();

    Serial.println("QMI8658 Tap Detection Example");

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

    // Accelerometer: FS_2G(000), FS_4G(001), FS_8G(010), FS_16G(011)
    // ODR: 1000, 500, 250, 125, 62.5, 31.25, 128, 21, 11, 3 Hz (6DOF: 448/224/112/56/28 Hz)
    imu.configAccel(AccelFullScaleRange::FS_2G,
                   500.0f,
                   SensorQMI8658::LpfMode::OFF);

    imu.enableAccel();

    Serial.println("\n=== Simple Usage ===");
    Serial.println("Using configTapDefault() with sensible defaults:");
    Serial.println("- peak_window: 30 samples");
    Serial.println("- tap_window: 100 samples");
    Serial.println("- double_tap_window: 500 samples");
    Serial.println("- peak_mag_threshold: 1.5 g^2");
    Serial.println("- quiet_threshold: 0.5 g^2");
    imu.configTapDefault();

    Serial.println("\n=== Advanced Usage ===");
    Serial.println("Using configTap() with runtime tuning support");
    Serial.println("(Tuned to reduce single-tap reported as double)");
    applyTapConfig();

    imu.setTapCallback(tapCallback);

    Serial.println("\nTap detection configured. Tap the sensor!");
    Serial.println("Expected: Single tap = 1 event, Double tap = 2 events");
    printCommandHelp();
}

void loop()
{
    handleSerialCommands();
    imu.update();
    delay(10);
}

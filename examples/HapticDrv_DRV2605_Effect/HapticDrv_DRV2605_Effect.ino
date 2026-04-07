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
 * @file      HapticDrv_DRV2605_Effect.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-07
 */
#include <HapticDrivers.hpp>

#ifndef SENSOR_SDA
#define SENSOR_SDA  3
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  2
#endif

HapticDriver_DRV2605 haptic;

static const char *effectList[] = {
    "1 - Strong Click 100%", "2 - Strong Click 60%", "3 - Strong Click 30%",
    "4 - Sharp Click 100%", "5 - Sharp Click 60%", "6 - Sharp Click 30%",
    "7 - Soft Bump 100%", "8 - Soft Bump 60%", "9 - Soft Bump 30%",
    "10 - Double Click 100%", "11 - Double Click 60%", "12 - Triple Click 100%",
    "13 - Soft Fuzz 60%", "14 - Strong Buzz 100%", "15 - Alert 750ms 100%",
    "16 - Alert 1000ms 100%"
};

void printBanner()
{
    Serial.println("");
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║     DRV2605 Haptic Driver Test           ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println("");
}

void printHelp()
{
    Serial.println("\n========== Commands ==========");
    Serial.println("1-16  : Play effect (ERM Library 1)");
    Serial.println("e     : List all effects (1-117)");
    Serial.println("l 1-6 : Select ROM library");
    Serial.println("a     : Toggle ERM/LRA mode");
    Serial.println("s     : Stop playback");
    Serial.println("r     : RTP test (0-255)");
    Serial.println("c     : Calibrate");
    Serial.println("h     : Help");
    Serial.println("==============================");
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    printBanner();

    Serial.println("\n=== Driver Initialization ===");

    if (!haptic.begin(Wire, DRV2605_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("[FATAL] DRV2605 init failed!");
        while (1) delay(1000);
    }

    Serial.println("[OK] Driver initialized");
    Serial.print("  Chip Name:    "); Serial.println(haptic.getChipName());
    Serial.print("  Chip ID:      0x"); Serial.println(haptic.getChipID(), HEX);
    Serial.print("  Actuator:     "); Serial.println(haptic.getActuatorType() == HapticActuatorType::LRA ? "LRA" : "ERM");
    Serial.print("  VBAT:         "); Serial.print(haptic.getVbat()); Serial.println(" mV");
    Serial.print("  Mode:         "); Serial.println(static_cast<uint8_t>(haptic.getMode()));
    printHelp();
}

void loop()
{
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd >= '1' && cmd <= '9') {
            uint8_t idx = cmd - '1';
            Serial.print("Playing: "); Serial.println(effectList[idx]);
            haptic.playEffect(idx + 1);
        } else if (cmd == 'e' || cmd == 'E') {
            Serial.println("\n--- Effect List (1-117) ---");
            Serial.println("ERM Library 1: Strong/Sharp/Soft Click, Double/Triple Click, Buzz");
            Serial.println("ERM Library 2-5: Various click patterns");
            Serial.println("ERM Library 6: LRA Library (use 'l 6')");
            Serial.println("Send number 1-117 to play effect");
        } else if (cmd == 'l' || cmd == 'L') {
            Serial.read(); // skip space
            while (Serial.available() == 0) delay(10);
            char lib = Serial.read();
            uint8_t libNum = lib - '0';
            if (libNum >= 1 && libNum <= 6) {
                haptic.selectLibrary(libNum);
                Serial.print("Library set to: "); Serial.println(libNum);
            }
        } else if (cmd == 'a' || cmd == 'A') {
            auto type = haptic.getActuatorType();
            haptic.setActuatorType(type == HapticActuatorType::ERM ? HapticActuatorType::LRA : HapticActuatorType::ERM);
            Serial.print("Actuator: "); Serial.println(haptic.getActuatorType() == HapticActuatorType::ERM ? "ERM" : "LRA");
        } else if (cmd == 's' || cmd == 'S') {
            haptic.stop();
            Serial.println("Stopped");
        } else if (cmd == 'r' || cmd == 'R') {
            haptic.setMode(HapticMode::REAL_TIME_PLAYBACK);
            Serial.println("RTP Mode: Send 0-255 for intensity");
            while (Serial.available() == 0) delay(10);
            int val = Serial.parseInt();
            if (val >= 0 && val <= 255) {
                haptic.setRealtimeValue(static_cast<uint8_t>(val));
                Serial.print("RTP Value: "); Serial.println(val);
            }
            haptic.setMode(HapticMode::INTERNAL_TRIGGER);
        } else if (cmd == 'c' || cmd == 'C') {
            Serial.println("Calibrating...");
            if (haptic.calibrate()) {
                Serial.println("[OK] Calibration done");
            } else {
                Serial.println("[FAIL] Calibration failed");
            }
        } else if (cmd == 'h' || cmd == 'H' || cmd == '?') {
            printHelp();
        } else if (cmd >= '0' && cmd <= '9') {
            // Multi-digit effect number
            String numStr = String(cmd);
            while (Serial.available()) {
                char c = Serial.peek();
                if (c >= '0' && c <= '9') {
                    numStr += Serial.read();
                } else {
                    Serial.read(); // consume non-digit
                    break;
                }
            }
            uint8_t effectNum = numStr.toInt();
            if (effectNum >= 1 && effectNum <= 117) {
                Serial.print("Playing effect: "); Serial.println(effectNum);
                haptic.playEffect(effectNum);
            } else {
                Serial.print("Invalid: "); Serial.print(effectNum); Serial.println(" (range 1-117)");
            }
        } else if (cmd != '\n' && cmd != '\r' && cmd != ' ') {
            Serial.print("[?] Unknown: '"); Serial.println(cmd);
        }
    }

    delay(10);
}

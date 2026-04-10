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
 * @file      HapticDrv_AW86224_Full.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-07
 */

#include <HapticDrivers.hpp>
#include <SensorWireHelper.h>

#ifndef SENSOR_SDA
#define SENSOR_SDA  20
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  21
#endif

HapticDriver_AW86224 haptic;

// LilyGo T-Display-P4 peripherals power control pins
#ifdef ARDUINO_ESP32P4_DEV
#include <IoExpanderXL9555.hpp>
#define IO_EXPANDER_3V3_POWER_EN 0
#define IO_EXPANDER_5V0_POWER_EN 6
#define IO_EXPANDER_P4_VCCA_POWER_EN 8
#define P4_TOUCH_SDA  7
#define P4_TOUCH_SCL  8
IoExpanderXL9555 expander;
#endif


void printBanner()
{
    Serial.println("");
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║     AW86224 Haptic Driver Test           ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println("");
}

void testCapabilities()
{
    Serial.println("\n[TEST] Device Capabilities");
    auto cap = haptic.getCapabilities();

    Serial.print("  F0 Calibration:    "); Serial.println(cap.hasF0Calibration ? "Yes" : "No");
    Serial.print("  VBAT Compensation: "); Serial.println(cap.hasVbatCompensation ? "Yes" : "No");
    Serial.print("  RTP Mode:          "); Serial.println(cap.hasRTP ? "Yes" : "No");
    Serial.print("  Sequence:          "); Serial.println(cap.hasSequence ? "Yes" : "No");
    Serial.print("  Break Effect:      "); Serial.println(cap.hasBreakEffect ? "Yes" : "No");
    Serial.print("  Continuous Mode:   "); Serial.println(cap.hasContinuousMode ? "Yes" : "No");
    Serial.print("  Max Effects:       "); Serial.println(cap.maxEffectCount);
    Serial.print("  Max Sequence:      "); Serial.println(cap.maxSequenceLength);
    Serial.print("  Max Duration:      "); Serial.println(cap.maxDurationMs == 0 ? "Unlimited" : String(cap.maxDurationMs));
    Serial.println("[OK] Done");
}

void testInit()
{
    Serial.println("\n=== Driver Initialization ===");
    Serial.print("Wire: SDA="); Serial.print(SENSOR_SDA); Serial.print(", SCL="); Serial.println(SENSOR_SCL);

#ifdef ARDUINO_ESP32P4_DEV
    // Enable power supply for T-Display-P4 peripherals
    Wire1.begin(P4_TOUCH_SDA, P4_TOUCH_SCL);
    SensorWireHelper::dumpDevices(Wire1);

    if (!expander.begin(Wire1, XL9555_SLAVE_ADDRESS0)) {
        while (1) {
            Serial.println("Failed to find XL9555 - check your wiring!");
            delay(1000);
        }
    }
    Serial.println("Sensor Expander Initialized");

    expander.pinMode(IO_EXPANDER_P4_VCCA_POWER_EN, OUTPUT);
    expander.digitalWrite(IO_EXPANDER_P4_VCCA_POWER_EN, LOW);

    expander.pinMode(IO_EXPANDER_5V0_POWER_EN, OUTPUT);
    expander.digitalWrite(IO_EXPANDER_5V0_POWER_EN, HIGH);

    expander.pinMode(IO_EXPANDER_3V3_POWER_EN, OUTPUT);
    expander.digitalWrite(IO_EXPANDER_3V3_POWER_EN, LOW);

    Wire.begin(SENSOR_SDA, SENSOR_SCL);
    SensorWireHelper::dumpDevices(Wire);
#endif

    if (!haptic.begin(Wire, AW8624_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("[FATAL] AW86224 init failed!");
        while (1) delay(1000);
    }

    Serial.println("[OK] Driver initialized");
    Serial.print("  Chip Name:    "); Serial.println(haptic.getChipName());
    Serial.print("  Chip ID:      0x"); Serial.println(haptic.getChipID(), HEX);
    Serial.print("  Actuator:     "); Serial.println(haptic.getActuatorType() == HapticActuatorType::LRA ? "LRA" : "ERM");
    Serial.print("  F0:           "); Serial.print(haptic.getF0()); Serial.println(" Hz");
    Serial.print("  VBAT:         "); Serial.print(haptic.getVbat()); Serial.println(" mV");
    Serial.print("  RAM Waveforms: "); Serial.println(haptic.getRamNum());
    Serial.print("  F0 Cali Data: 0x"); Serial.println(haptic.getF0CaliData(), HEX);
}

void testPlayEffect()
{
    Serial.println("\n[TEST] playEffect() - HapticBase Interface");
    Serial.println("  Playing effect 1...");
    haptic.playEffect(1);
    delay(300);
    Serial.println("[OK] Done");
}

void testPlayEffectAll()
{
    Serial.println("\n[TEST] playEffect() - All Waveforms");
    for (uint8_t i = 1; i <= haptic.getRamNum(); i++) {
        Serial.print("  Effect "); Serial.print(i); Serial.println("...");
        haptic.playEffect(i);
        delay(400);
    }
    Serial.println("[OK] Done");
}

void testShortVibration()
{
    Serial.println("\n[TEST] shortVibration()");
    Serial.println("  index=1, gain=0x80, loop=1");
    haptic.shortVibration(1, 0x80, 1);
    Serial.println("[OK] Done");
}

void testShortVibrationGain()
{
    Serial.println("\n[TEST] shortVibration() - Different Gains");

    uint8_t gains[] = {0x20, 0x40, 0x60, 0x80, 0xA0, 0xC0, 0xFF};
    for (size_t i = 0; i < sizeof(gains); i++) {
        Serial.print("  Gain 0x"); Serial.print(gains[i], HEX); Serial.println("...");
        haptic.shortVibration(1, gains[i], 1);
        delay(300);
    }
    Serial.println("[OK] Done");
}

void testShortVibrationLoop()
{
    Serial.println("\n[TEST] shortVibration() - Loop Count");
    for (uint8_t loop = 1; loop <= 5; loop++) {
        Serial.print("  Loop "); Serial.print(loop); Serial.println("...");
        haptic.shortVibration(1, 0x80, loop);
        delay(500);
    }
    Serial.println("[OK] Done");
}

void testContinuousVibration()
{
    Serial.println("\n[TEST] continuousVibration() - HapticBase Interface");
    Serial.println("  Using unified interface: 2000ms blocking");
    haptic.setGain(0x80);
    haptic.continuousVibration(2000, true);
    Serial.println("[OK] Done");
}

void testLongVibrationBlocking()
{
    Serial.println("\n[TEST] longVibration() - Blocking Mode");
    Serial.println("  index=2, gain=0x80, duration=2000ms, blocking=true");
    uint32_t start = millis();
    haptic.longVibration(2, 0x80, 2000, true);
    uint32_t elapsed = millis() - start;
    Serial.print("[OK] Done (elapsed: "); Serial.print(elapsed); Serial.println(" ms)");
}

void testLongVibrationNonBlocking()
{
    Serial.println("\n[TEST] longVibration() - Non-Blocking Mode");
    Serial.println("  index=2, gain=0x80, duration=5000ms, blocking=false");
    haptic.longVibration(2, 0x80, 5000, false);
    Serial.println("  Started! Use 's' to stop or wait...");
}

void testWaveformSwitching()
{
    Serial.println("\n[TEST] Waveform Switching (Non-Blocking)");
    Serial.println("  Rapid switching between waveforms during vibration");

    haptic.longVibration(1, 0x80, 5000, false);
    delay(500);

    for (int i = 0; i < 3; i++) {
        for (uint8_t w = 1; w <= haptic.getRamNum(); w++) {
            if (!haptic.isPlaying()) break;
            Serial.print("  Switch to waveform "); Serial.print(w); Serial.println("...");
            haptic.longVibration(w, 0x80, 1000, false);
            delay(100);
        }
    }

    if (haptic.isPlaying()) {
        Serial.println("  Stopping...");
        haptic.stop();
    }
    Serial.println("[OK] Done");
}

void testSetSequence()
{
    Serial.println("\n[TEST] setSequence() - HapticBase Interface");
    haptic.clearSequence();
    haptic.setSequence(0, 1);
    haptic.setSequence(1, 2);
    haptic.setSequence(2, 0);
    Serial.println("  Sequence set: [1, 2, 0]");
    Serial.println("  Playing sequence...");
    haptic.playSequence();
    delay(1000);
    Serial.println("[OK] Done");
}

void testStop()
{
    Serial.println("\n[TEST] stop() - HapticBase Interface");
    haptic.continuousVibration(5000, false);
    Serial.println("  Started 5s vibration...");
    delay(1000);
    Serial.println("  Calling stop()...");
    haptic.stop();
    Serial.print("  isPlaying: "); Serial.println(haptic.isPlaying() ? "Yes" : "No");
    Serial.println("[OK] Done");
}

void testStatus()
{
    Serial.println("\n[TEST] getStatus() - HapticBase Interface");
    HapticStatus status = haptic.getStatus();
    const char *statusStr = "UNKNOWN";
    switch (status) {
    case HapticStatus::IDLE: statusStr = "IDLE"; break;
    case HapticStatus::PLAYING: statusStr = "PLAYING"; break;
    case HapticStatus::PAUSED: statusStr = "PAUSED"; break;
    case HapticStatus::CALIBRATING: statusStr = "CALIBRATING"; break;
    case HapticStatus::ERROR: statusStr = "ERROR"; break;
    case HapticStatus::STANDBY: statusStr = "STANDBY"; break;
    }
    Serial.print("  Status: "); Serial.println(statusStr);
    Serial.println("[OK] Done");
}

void testIntensitySweep()
{
    Serial.println("\n[TEST] Intensity Sweep");
    Serial.println("  Gradually increasing intensity");

    for (uint16_t gain = 0x20; gain <= 0xFF; gain += 0x20) {
        Serial.print("  Gain: 0x"); Serial.println(gain, HEX);
        haptic.shortVibration(1, gain, 3);
        delay(400);
    }
    Serial.println("[OK] Done");
}

void testAutoDemo()
{
    Serial.println("\n=== Auto Demo Sequence ===");
    delay(1000);

    Serial.println("\n[1/12] Basic playEffect()...");
    haptic.playEffect(1);
    delay(800);

    Serial.println("\n[2/12] Basic short vibration...");
    haptic.shortVibration(1, 0x80, 1);
    delay(800);

    Serial.println("\n[3/12] All waveforms (short)...");
    for (uint8_t i = 1; i <= haptic.getRamNum(); i++) {
        Serial.print("  Wave "); Serial.print(i); Serial.println("...");
        haptic.shortVibration(i, 0x80, 1);
        delay(400);
    }

    Serial.println("\n[4/12] Continuous vibration (HapticBase)...");
    haptic.continuousVibration(1500, true);

    Serial.println("\n[5/12] Long vibration (2s)...");
    haptic.longVibration(2, 0x80, 2000, true);

    Serial.println("\n[6/12] Intensity sweep...");
    testIntensitySweep();

    Serial.println("\n[7/12] Non-blocking long vibration (3s)...");
    haptic.longVibration(2, 0xA0, 3000, false);
    uint32_t start = millis();
    while (haptic.isPlaying() && (millis() - start < 3500)) {
        delay(100);
    }
    if (haptic.isPlaying()) {
        haptic.stop();
        Serial.println("  Stopped by demo");
    }

    Serial.println("\n[8/12] Sequence test...");
    testSetSequence();
    delay(500);

    Serial.println("\n[9/12] Status check...");
    testStatus();

    Serial.println("\n[10/12] Rapid waveform switching...");
    testWaveformSwitching();

    Serial.println("\n[11/12] Gain test...");
    testShortVibrationGain();

    Serial.println("\n[12/12] Final short vibration...");
    haptic.shortVibration(1, 0xFF, 2);

    Serial.println("\n=== Demo Complete ===");
}

void printHelp()
{
    Serial.println("\n========== Commands ==========");
    Serial.println("Initialization:");
    Serial.println("  i   - Re-initialize driver");
    Serial.println("");
    Serial.println("HapticBase Interface:");
    Serial.println("  e   - playEffect(1)");
    Serial.println("  E   - playEffect() all waveforms");
    Serial.println("  c   - continuousVibration()");
    Serial.println("  S   - setSequence() test");
    Serial.println("  p   - stop() test");
    Serial.println("  t   - getStatus()");
    Serial.println("");
    Serial.println("AW86224 Extended:");
    Serial.println("  1   - Short vibration");
    Serial.println("  a   - All waveforms (short)");
    Serial.println("  g   - Different gains test");
    Serial.println("  l   - Loop count test (1-5)");
    Serial.println("  2   - Long vibration (blocking, 2s)");
    Serial.println("  L   - Long vibration all waveforms");
    Serial.println("  n   - Non-blocking long vibration (5s)");
    Serial.println("  w   - Waveform switching");
    Serial.println("");
    Serial.println("Special Tests:");
    Serial.println("  x   - Sweep: gradual intensity increase");
    Serial.println("  d   - Auto demo sequence");
    Serial.println("");
    Serial.println("Info:");
    Serial.println("  k   - Show capabilities");
    Serial.println("  v   - Read voltage");
    Serial.println("  f   - Read F0");
    Serial.println("  h   - Help (this message)");
    Serial.println("==============================");
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    printBanner();
    testInit();
    testCapabilities();

    Serial.println("\n========== Quick Test ==========");
    Serial.println("Running auto demo sequence...");

    testAutoDemo();

    printHelp();
}

void loop()
{
    if (haptic.isPlaying()) {
        haptic.vibrationUpdate();
    }

    if (Serial.available()) {
        char cmd = Serial.read();

        switch (cmd) {
        case 'i':
            testInit();
            break;
        case 'k':
            testCapabilities();
            break;
        case 'e':
            testPlayEffect();
            break;
        case 'E':
            testPlayEffectAll();
            break;
        case 'c':
            testContinuousVibration();
            break;
        case 'S':
            testSetSequence();
            break;
        case 'p':
            testStop();
            break;
        case 't':
            testStatus();
            break;
        case '1':
            testShortVibration();
            break;
        case 'a':
            testPlayEffectAll();
            break;
        case 'g':
            testShortVibrationGain();
            break;
        case 'l':
            testShortVibrationLoop();
            break;
        case '2':
            testLongVibrationBlocking();
            break;
        case 'L':
            for (uint8_t i = 1; i <= haptic.getRamNum(); i++) {
                Serial.print("  Waveform "); Serial.print(i); Serial.println(", 1000ms...");
                haptic.longVibration(i, 0x80, 1000, true);
                delay(200);
            }
            Serial.println("[OK] Done");
            break;
        case 'n':
            testLongVibrationNonBlocking();
            break;
        case 'w':
            testWaveformSwitching();
            break;
        case 'x':
            testIntensitySweep();
            break;
        case 'd':
            testAutoDemo();
            break;
        case 'v':
            Serial.print("[VBAT] "); Serial.print(haptic.getVbat()); Serial.println(" mV");
            break;
        case 'f':
            Serial.print("[F0] "); Serial.print(haptic.getF0()); Serial.println(" Hz");
            break;
        case 'h':
        case 'H':
        case '?':
            printHelp();
            break;
        default:
            if (cmd != '\n' && cmd != '\r' && cmd != ' ') {
                Serial.print("[?] Unknown: '"); Serial.print(cmd); Serial.print(" (0x"); Serial.print(cmd, HEX); Serial.println(")");
            }
            break;
        }
    }

    delay(10);
}

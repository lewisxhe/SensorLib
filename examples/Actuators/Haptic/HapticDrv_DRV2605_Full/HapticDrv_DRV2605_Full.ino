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
 * @file      HapticDrv_DRV2605_Full.ino
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

void printBanner()
{
    Serial.println("");
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║     DRV2605 Haptic Driver Test           ║");
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
    Serial.print("  Auto Calibration:  "); Serial.println(cap.hasAutoCalibration ? "Yes" : "No");
    Serial.print("  Break Effect:      "); Serial.println(cap.hasBreakEffect ? "Yes" : "No");
    Serial.print("  Overdrive:         "); Serial.println(cap.hasOverdrive ? "Yes" : "No");
    Serial.print("  Continuous Mode:   "); Serial.println(cap.hasContinuousMode ? "Yes" : "No");
    Serial.print("  Max Effects:       "); Serial.println(cap.maxEffectCount);
    Serial.print("  Max Sequence:      "); Serial.println(cap.maxSequenceLength);
    Serial.println("[OK] Done");
}

void testInit()
{
    Serial.println("\n=== Driver Initialization ===");
    Serial.print("Wire: SDA="); Serial.print(SENSOR_SDA); Serial.print(", SCL="); Serial.println(SENSOR_SCL);

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
}

void testActuatorType()
{
    Serial.println("\n[TEST] Actuator Type");

    Serial.println("  Current: ");
    haptic.getActuatorType() == HapticActuatorType::ERM ? Serial.println("ERM") : Serial.println("LRA");

    Serial.println("  Switching to LRA...");
    haptic.setActuatorType(HapticActuatorType::LRA);
    Serial.print("  Now: "); Serial.println(haptic.getActuatorType() == HapticActuatorType::LRA ? "LRA" : "ERM");

    Serial.println("  Switching to ERM...");
    haptic.setActuatorType(HapticActuatorType::ERM);
    Serial.print("  Now: "); Serial.println(haptic.getActuatorType() == HapticActuatorType::ERM ? "ERM" : "LRA");

    Serial.println("[OK] Done");
}

void testLibrarySelection()
{
    Serial.println("\n[TEST] Library Selection");

    Serial.println("  Selecting ERM Library 1 (Strong Click)...");
    haptic.selectLibrary(1);
    haptic.playEffect(1);
    delay(500);

    Serial.println("  Selecting ERM Library 2 (Sharp Click)...");
    haptic.selectLibrary(2);
    haptic.playEffect(1);
    delay(500);

    Serial.println("  Selecting ERM Library 5 (Buzz)...");
    haptic.selectLibrary(5);
    haptic.playEffect(1);
    delay(500);

    Serial.println("  Reset to Library 1...");
    haptic.selectLibrary(1);
    Serial.println("[OK] Done");
}

void testPlayEffect()
{
    Serial.println("\n[TEST] playEffect() - HapticBase Interface");
    Serial.println("  Playing effect 1...");
    bool result = haptic.playEffect(1);
    if (result) {
        delay(300);
        Serial.println("[OK] Done");
    } else {
        Serial.println("[FAIL] playEffect() returned false");
    }
}

void testPlayEffectRange()
{
    Serial.println("\n[TEST] playEffect() - Effect Range (1-10)");
    for (uint8_t i = 1; i <= 10; i++) {
        Serial.print("  Effect "); Serial.print(i); Serial.println("...");
        haptic.playEffect(i);
        delay(400);
    }
    Serial.println("[OK] Done");
}

void testSetSequence()
{
    Serial.println("\n[TEST] setSequence() - HapticBase Interface");
    haptic.clearSequence();
    haptic.setSequence(0, 1);
    haptic.setSequence(1, 2);
    haptic.setSequence(2, 3);
    haptic.setSequence(3, 0);
    Serial.println("  Sequence set: [1, 2, 3, 0]");
    Serial.println("  Playing sequence...");
    haptic.playSequence();
    delay(2000);
    Serial.println("[OK] Done");
}

void testRTP()
{
    Serial.println("\n[TEST] setRealtimeValue() - HapticBase Interface");

    haptic.setMode(HapticMode::REAL_TIME_PLAYBACK);
    Serial.println("  Mode set to RTP");

    Serial.println("  Playing RTP values...");
    uint8_t values[] = {0x00, 0x40, 0x80, 0xC0, 0xFF, 0xC0, 0x80, 0x40};
    for (size_t i = 0; i < sizeof(values); i++) {
        Serial.print("  Value: 0x"); Serial.println(values[i], HEX);
        haptic.setRealtimeValue(values[i]);
        delay(2000);
    }

    haptic.stop();
    haptic.setMode(HapticMode::INTERNAL_TRIGGER);
    Serial.println("[OK] Done");
}

void testStop()
{
    Serial.println("\n[TEST] stop() - HapticBase Interface");
    haptic.playEffect(1);
    Serial.println("  Started playback, waiting...");
    delay(500);
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

void testModes()
{
    Serial.println("\n[TEST] Mode Switching");

    Serial.println("  Mode: INTERNAL_TRIGGER");
    haptic.setMode(HapticMode::INTERNAL_TRIGGER);
    Serial.print("  Current: "); Serial.println(static_cast<uint8_t>(haptic.getMode()));

    Serial.println("  Mode: REAL_TIME_PLAYBACK");
    haptic.setMode(HapticMode::REAL_TIME_PLAYBACK);
    Serial.print("  Current: "); Serial.println(static_cast<uint8_t>(haptic.getMode()));

    Serial.println("  Mode: DIAGNOSTICS");
    haptic.setMode(HapticMode::DIAGNOSTICS);
    Serial.print("  Current: "); Serial.println(static_cast<uint8_t>(haptic.getMode()));

    Serial.println("  Mode: STANDBY");
    haptic.setMode(HapticMode::STANDBY);
    Serial.print("  Current: "); Serial.println(static_cast<uint8_t>(haptic.getMode()));

    haptic.setMode(HapticMode::INTERNAL_TRIGGER);
    Serial.println("[OK] Done");
}

void testCalibration()
{
    Serial.println("\n[TEST] calibrate() - HapticBase Interface");
    Serial.println("  WARNING: May take a few seconds...");

    if (haptic.calibrate()) {
        Serial.println("  Calibration successful!");
    } else {
        Serial.println("  Calibration failed!");
    }
    Serial.println("[OK] Done");
}

void testF0()
{
    Serial.println("\n[TEST] getF0() - HapticBase Interface");
    uint32_t f0 = haptic.getF0();
    if (f0 > 0) {
        Serial.print("  LRA F0: "); Serial.print(f0); Serial.println(" Hz");
    } else {
        Serial.println("  F0 not available or LRA not detected");
    }
    Serial.println("[OK] Done");
}

void testStandby()
{
    Serial.println("\n[TEST] setMode(STANDBY) - HapticBase Interface");
    haptic.setMode(HapticMode::STANDBY);
    Serial.println("[OK] Done");
}

void testNormalMode()
{
    Serial.println("\n[TEST] setMode(NORMAL) - HapticBase Interface");
    haptic.setMode(HapticMode::INTERNAL_TRIGGER);
    Serial.println("[OK] Done");
}

void testAutoDemo()
{
    Serial.println("\n=== Auto Demo Sequence ===");
    delay(1000);

    Serial.println("\n[1/11] Basic playEffect()...");
    haptic.playEffect(1);
    delay(800);

    Serial.println("\n[2/11] Library selection...");
    testLibrarySelection();

    Serial.println("\n[3/11] Effect range (1-10)...");
    testPlayEffectRange();

    Serial.println("\n[4/11] Sequence test...");
    testSetSequence();

    Serial.println("\n[5/11] RTP test...");
    testRTP();

    Serial.println("\n[6/11] Status check...");
    testStatus();

    Serial.println("\n[7/11] Mode switching...");
    testModes();

    Serial.println("\n[8/11] Actuator type switching...");
    testActuatorType();

    Serial.println("\n[9/11] Stop test...");
    haptic.playEffect(1);
    delay(200);
    haptic.stop();
    delay(300);

    Serial.println("\n[10/11] Calibration...");
    haptic.calibrate();
    delay(500);

    Serial.println("\n[11/11] F0 reading...");
    testF0();

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
    Serial.println("  E   - playEffect() range (1-10)");
    Serial.println("  S   - setSequence() test");
    Serial.println("  r   - setRealtimeValue() (RTP)");
    Serial.println("  p   - stop() test");
    Serial.println("  t   - getStatus()");
    Serial.println("  c   - calibrate()");
    Serial.println("  f   - getF0()");
    Serial.println("  k   - Show capabilities");
    Serial.println("  s   - Enter standby mode");
    Serial.println("  n   - Return to normal mode");
    Serial.println("");
    Serial.println("DRV2605 Extended:");
    Serial.println("  l   - Library selection");
    Serial.println("  a   - Actuator type switch");
    Serial.println("  m   - Mode switching");
    Serial.println("");
    Serial.println("Special Tests:");
    Serial.println("  d   - Auto demo sequence");
    Serial.println("");
    Serial.println("Info:");
    Serial.println("  v   - Read voltage");
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
            testPlayEffectRange();
            break;
        case 'S':
            testSetSequence();
            break;
        case 'r':
            testRTP();
            break;
        case 'p':
            testStop();
            break;
        case 't':
            testStatus();
            break;
        case 'c':
            testCalibration();
            break;
        case 'f':
            testF0();
            break;
        case 'l':
            testLibrarySelection();
            break;
        case 'a':
            testActuatorType();
            break;
        case 'm':
            testModes();
            break;
        case 'd':
            testAutoDemo();
            break;
        case 'v':
            Serial.print("[VBAT] "); Serial.print(haptic.getVbat()); Serial.println(" mV");
            break;
        case 's':
            testStandby();
            break;
        case 'n':
            testNormalMode();
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

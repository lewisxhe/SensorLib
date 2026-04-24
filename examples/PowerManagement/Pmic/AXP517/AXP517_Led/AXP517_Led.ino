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
 * @file      AXP517_Led.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-15
 */

#include <PmicDrv.hpp>

#ifndef PMIC_SDA
#define PMIC_SDA  3
#endif

#ifndef PMIC_SCL
#define PMIC_SCL  2
#endif

#ifndef PMIC_IRQ
#define PMIC_IRQ  21
#endif

PmicAXP517 pmic;

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== AXP517 LED Example ===\n");

    if (!pmic.begin(Wire, AXP517_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        Serial.println("Failed to init AXP517!");
        while (1) delay(1000);
    }

    pmic.enableModule(PmicAXP517::Module::CHGLED, true);

    auto &led = pmic.led();

    Serial.println("\n--- LED Mode Demo ---");

    // 1. AUTO - PMIC controls LED automatically (charge indicator)
    led.setMode(PmicLedBase::Mode::AUTO);
    Serial.println("Mode: AUTO (Charge Indicator)");
    delay(1000);

    // 2. MANUAL - Manual control
    led.setMode(PmicLedBase::Mode::MANUAL);

    // Set LED off
    led.setManualState(PmicLedBase::ManualState::HiZ);
    Serial.println("Manual: Hi-Z (Off)");
    delay(1000);

    // Set LED on (low)
    led.setManualState(PmicLedBase::ManualState::LEVEL_LOW);
    Serial.println("Manual: LEVEL_LOW (On)");
    delay(1000);

    // 3. BREATH - Breathing mode
    led.setMode(PmicLedBase::Mode::BREATH);
    Serial.println("Mode: BREATH");

    Serial.println("\n=== Setup Complete ===\n");
}

void loop()
{
    static uint8_t state = 0;
    static uint32_t lastChange = 0;

    if (millis() - lastChange > 3000) {
        lastChange = millis();

        auto &led = pmic.led();

        switch (state % 5) {
        case 0:
            led.setMode(PmicLedBase::Mode::AUTO);
            Serial.println("LED: AUTO");
            break;
        case 1:
            led.setMode(PmicLedBase::Mode::MANUAL);
            led.setManualState(PmicLedBase::ManualState::HiZ);
            Serial.println("LED: Off");
            break;
        case 2:
            led.setMode(PmicLedBase::Mode::MANUAL);
            led.setManualState(PmicLedBase::ManualState::LEVEL_LOW);
            Serial.println("LED: On");
            break;
        case 3:
            led.setMode(PmicLedBase::Mode::MANUAL);
            led.setManualState(PmicLedBase::ManualState::BLINK_1HZ);
            Serial.println("LED: BLINK 1HZ");
            break;
        case 4:
            led.setMode(PmicLedBase::Mode::MANUAL);
            led.setManualState(PmicLedBase::ManualState::BLINK_4HZ);
            Serial.println("LED: BLINK 4HZ");
            break;
        }
        state++;
    }

    delay(100);
}
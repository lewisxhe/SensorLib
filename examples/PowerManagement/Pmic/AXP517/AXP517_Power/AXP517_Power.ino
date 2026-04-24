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
 * @file      AXP517_Power.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-15
 */

#include <Arduino.h>
#include <Wire.h>
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

    Serial.println("\n=== AXP517 Power Example ===\n");

    if (!pmic.begin(Wire, AXP517_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        Serial.println("Failed to init AXP517!");
        while (1) delay(1000);
    }

    auto &power = pmic.power();

    Serial.println("\n--- Configure Power ---");

    // Minimum system voltage: 3.3V
    power.setMinimumSystemVoltage(3300);
    Serial.print("Min Sys: ");
    Serial.print(power.getMinimumSystemVoltage());
    Serial.println(" mV");

    // Input voltage limit: 5V
    power.setInputVoltageLimit(5000);
    Serial.print("VIN Limit: ");
    Serial.print(power.getInputVoltageLimit());
    Serial.println(" mV");

    // Input current limit: 2A
    power.setInputCurrentLimit(2000);
    Serial.print("IIN Limit: ");
    Serial.print(power.getInputCurrentLimit());
    Serial.println(" mA");

    // Boost voltage: 5V
    power.setBoostVoltage(5000);
    Serial.print("Boost: ");
    Serial.print(power.getBoostVoltage());
    Serial.println(" mV");

    Serial.println("\n=== Setup Complete ===\n");
}

void loop()
{
    static uint32_t lastPrint = 0;
    static bool boostState = false;

    if (millis() - lastPrint > 5000) {
        lastPrint = millis();

        auto &power = pmic.power();

        // Toggle boost
        boostState = !boostState;
        power.enableBoost(boostState);

        Serial.println("\n=== Power Settings ===");
        Serial.print("Min Sys: ");
        Serial.print(power.getMinimumSystemVoltage());
        Serial.println(" mV");
        Serial.print("VIN Limit: ");
        Serial.print(power.getInputVoltageLimit());
        Serial.println(" mV");
        Serial.print("IIN Limit: ");
        Serial.print(power.getInputCurrentLimit());
        Serial.println(" mA");
        Serial.print("Boost: ");
        Serial.print(power.getBoostVoltage());
        Serial.print(" mV (");
        Serial.print(power.isBoostEnabled() ? "ON" : "OFF");
        Serial.println(")");
    }

    delay(100);
}
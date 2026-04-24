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
 * @file      AXP517_Charger.ino
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

    Serial.println("\n=== AXP517 Charger Example ===\n");

    if (!pmic.begin(Wire, AXP517_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        Serial.println("Failed to init AXP517!");
        while (1) delay(1000);
    }

    pmic.enableModule(PmicAXP517::Module::CHARGE, true);

    auto *charger = pmic.getCharger();
    if (!charger) {
        Serial.println("No charger!");
        while (1) delay(1000);
    }

    Serial.println("\n--- Configure Charger ---");

    charger->setChargeVoltage(4200);
    Serial.print("CV: ");
    Serial.print(charger->getChargeVoltage());
    Serial.println(" mV");

    charger->setFastChargeCurrent(1000);
    Serial.print("ICC: ");
    Serial.print(charger->getFastChargeCurrent());
    Serial.println(" mA");

    charger->setPreChargeCurrent(128);
    Serial.print("IPRECHG: ");
    Serial.print(charger->getPreChargeCurrent());
    Serial.println(" mA");

    charger->setTerminationCurrent(256);
    Serial.print("ITERM: ");
    Serial.print(charger->getTerminationCurrent());
    Serial.println(" mA");

    charger->enableCharging(true);
    Serial.println("Charging enabled");

    Serial.println("\n=== Setup Complete ===\n");
}

void loop()
{
    static uint32_t lastPrint = 0;

    if (millis() - lastPrint > 2000) {
        lastPrint = millis();

        auto *charger = pmic.getCharger();
        auto status = charger->getStatus();

        Serial.println("\n--- Charger Status ---");

        Serial.print("State: ");
        Serial.println(status.charging ? "Charging" : "Idle");
        Serial.print("Vbus: ");
        Serial.println(status.vbusPresent ? "Present" : "Absent");
        Serial.print("Done: ");
        Serial.println(status.chargeDone ? "Yes" : "No");

        // Current settings
        Serial.print("CV: ");
        Serial.print(charger->getChargeVoltage());
        Serial.println(" mV");
        Serial.print("ICC: ");
        Serial.print(charger->getFastChargeCurrent());
        Serial.println(" mA");
    }

    delay(100);
}
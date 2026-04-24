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
 * @file      AXP517_Gauge.ino
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

    Serial.println("\n=== AXP517 Gauge Example ===\n");

    if (!pmic.begin(Wire, AXP517_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        Serial.println("Failed to init AXP517!");
        while (1) delay(1000);
    }

    pmic.enableModule(PmicAXP517::Module::GAUGE, true);

    // Enable ADC channels
    pmic.adc().enableChannels(0xFF);

    Serial.println("ADC channels enabled");
    Serial.println("\n=== Setup Complete ===\n");
}

void loop()
{
    static uint32_t lastPrint = 0;

    if (millis() - lastPrint > 2000) {
        lastPrint = millis();

        auto &adc = pmic.adc();

        Serial.println("\n=== Fuel Gauge ===");

        float val = 0;

        // Battery voltage
        if (adc.read(PmicAdcBase::Channel::BAT_VOLTAGE, val)) {
            Serial.print("VBAT: ");
            Serial.print(val);
            Serial.println(" mV");
        }

        // Battery current
        if (adc.read(PmicAdcBase::Channel::BAT_CURRENT, val)) {
            Serial.print("IBAT: ");
            Serial.print(val);
            Serial.println(" mA");
        }

        // VBUS voltage
        if (adc.read(PmicAdcBase::Channel::VBUS_VOLTAGE, val)) {
            Serial.print("VBUS: ");
            Serial.print(val);
            Serial.println(" mV");
        }

        // VBUS current
        if (adc.read(PmicAdcBase::Channel::VBUS_CURRENT, val)) {
            Serial.print("IBUS: ");
            Serial.print(val);
            Serial.println(" mA");
        }

        // System voltage
        if (adc.read(PmicAdcBase::Channel::VSYS_VOLTAGE, val)) {
            Serial.print("VSYS: ");
            Serial.print(val);
            Serial.println(" mV");
        }

        // Die temperature
        if (adc.read(PmicAdcBase::Channel::DIE_TEMPERATURE, val)) {
            Serial.print("TDIE: ");
            Serial.print(val);
            Serial.println(" C");
        }

        // Battery temperature
        if (adc.read(PmicAdcBase::Channel::BAT_TEMPERATURE, val)) {
            Serial.print("TBAT: ");
            Serial.print(val);
            Serial.println(" C");
        }

        // Battery percentage
        if (adc.read(PmicAdcBase::Channel::BAT_PERCENTAGE, val)) {
            Serial.print("SOC: ");
            Serial.print(val);
            Serial.println(" %");
        }
    }

    delay(100);
}
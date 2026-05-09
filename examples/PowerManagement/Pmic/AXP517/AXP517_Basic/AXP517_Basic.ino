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
 * @file      AXP517_Basic.ino
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

    Serial.println("\n=== AXP517 Basic Example ===\n");

    // Initialize PMIC using PmicDrv facade
    if (!pmic.begin(Wire, AXP517_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        Serial.println("Failed to init AXP517!");
        while (1) delay(1000);
    }
    Serial.println("AXP517 initialized");

    // Enable modules
    pmic.enableModule(PmicAXP517::Module::BUCK, true);
    pmic.enableModule(PmicAXP517::Module::BOOST, true);
    pmic.enableModule(PmicAXP517::Module::CHARGE, true);
    pmic.enableModule(PmicAXP517::Module::TYPEC, true);
    pmic.enableModule(PmicAXP517::Module::BC12, true);
    Serial.println("Modules enabled");

    Serial.println("\n--- Charger Test ---");

    // Configure charger via facade
    auto *charger = pmic.getCharger();
    if (charger) {
        charger->setChargeVoltage(4200);
        charger->setFastChargeCurrent(1000);
        charger->setPreChargeCurrent(128);
        charger->enableCharging(true);
        Serial.println("Charger configured");
        Serial.print("CV: ");
        Serial.print(charger->getChargeVoltage());
        Serial.println(" mV");
        Serial.print("ICC: ");
        Serial.print(charger->getFastChargeCurrent());
        Serial.println(" mA");
    }

    Serial.println("\n--- ADC Test ---");


    // Enable ADC channels
    auto &adc = pmic.adc();
    adc.enableChannels(
        PmicAdcBase::Channel::VBUS_VOLTAGE |
        PmicAdcBase::Channel::VBUS_CURRENT |
        PmicAdcBase::Channel::VSYS_VOLTAGE |
        PmicAdcBase::Channel::BAT_VOLTAGE |
        PmicAdcBase::Channel::BAT_CURRENT |
        PmicAdcBase::Channel::BAT_TEMPERATURE
    );

    // Enable LED set blinking
    pmic.led().setMode(PmicLedBase::Mode::MANUAL);
    pmic.led().setManualState(PmicLedBase::ManualState::BLINK_1HZ);

    Serial.println("\n=== Setup Complete ===\n");
}

void loop()
{
    static uint32_t lastPrint = 0;

    if (millis() - lastPrint > 2000) {
        lastPrint = millis();

        Serial.println("\n--- Status ---");

        // Charger status
        auto *charger = pmic.getCharger();
        if (charger) {
            auto status = charger->getStatus();
            Serial.print("Charger: ");
            Serial.println(status.charging ? "Charging" : "Idle");
            Serial.print("Vbus: ");
            Serial.println(status.vbusPresent ? "Present" : "Absent");
        }

        // ADC readings
        auto &adc = pmic.adc();
        float val = 0;
        if (adc.read(PmicAdcBase::Channel::VBUS_VOLTAGE, val)) {
            Serial.print("VBUS: ");
            Serial.print(val);
            Serial.println(" mV");
        }
        if (adc.read(PmicAdcBase::Channel::BAT_VOLTAGE, val)) {
            Serial.print("VBAT: ");
            Serial.print(val);
            Serial.println(" mV");
        }
        if (adc.read(PmicAdcBase::Channel::BAT_CURRENT, val)) {
            Serial.print("IBAT: ");
            Serial.print(val);
            Serial.println(" mA");
        }
        if (adc.read(PmicAdcBase::Channel::DIE_TEMPERATURE, val)) {
            Serial.print("TEMP: ");
            Serial.print(val);
            Serial.println(" C");
        }
        if (adc.read(PmicAdcBase::Channel::VSYS_VOLTAGE, val)) {
            Serial.print("VSYS: ");
            Serial.print(val);
            Serial.println(" mV");
        }
    }

    delay(100);
}
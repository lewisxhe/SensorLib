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
 * @file      AXP2101_Basic.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-05-08
 *
 */
#include <pmic/xpowers/axp2101/PmicAXP2101.hpp>

#ifndef PMIC_SDA
#define PMIC_SDA  3
#endif

#ifndef PMIC_SCL
#define PMIC_SCL  2
#endif

#ifndef PMIC_IRQ
#define PMIC_IRQ  21
#endif

PmicAXP2101 pmic;

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== AXP2101 Basic Example ===\n");

    if (!pmic.begin(Wire, AXP2101_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        Serial.println("Failed to init AXP2101!");
        while (1) delay(1000);
    }
    Serial.println("AXP2101 initialized");

    Serial.print("Chip ID: 0x");
    Serial.println(pmic.getChipID(), HEX);

    // Enable core modules
    pmic.enableModule(PmicAXP2101::Module::GENERAL_ADC, true);
    pmic.enableModule(PmicAXP2101::Module::CELL_CHARGE, true);
    pmic.enableModule(PmicAXP2101::Module::BAT_DETECT, true);

    Serial.println("\n--- Channel Test (DCDC1 & ALDO1) ---");

    // Unified channel API: enable DCDC1 at 3.3V
    pmic.getChannel()->enable(AXP2101Channel::CH_DCDC1, true);
    pmic.getChannel()->setVoltage(AXP2101Channel::CH_DCDC1, 3300);
    Serial.print("DCDC1: ");
    Serial.print(pmic.getChannel()->getVoltage(AXP2101Channel::CH_DCDC1));
    Serial.println(" mV");

    // Enable ALDO1 at 1.8V
    pmic.getChannel()->enable(AXP2101Channel::CH_ALDO1, true);
    pmic.getChannel()->setVoltage(AXP2101Channel::CH_ALDO1, 1800);
    Serial.print("ALDO1: ");
    Serial.print(pmic.getChannel()->getVoltage(AXP2101Channel::CH_ALDO1));
    Serial.println(" mV");

    Serial.print("Total channels: ");
    Serial.println(pmic.getChannel()->count());

    // List all channels
    for (uint8_t i = 0; i < pmic.getChannel()->count(); i++) {
        auto info = pmic.getChannel()->getInfo(i);
        Serial.print("  Ch");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(info.name);
        Serial.print(" ");
        Serial.print(info.minMillivolt);
        Serial.print("-");
        Serial.print(info.maxMillivolt);
        Serial.print(" mV");
        Serial.print("    Step: ");
        Serial.print(info.stepMillivolt);
        Serial.println(" mV");
    }

    Serial.println("\n--- Charger Test ---");

    auto *charger = pmic.getCharger();
    if (charger) {
        // The Charge input parameter is fuzzed and set to the value closest to the input.
        charger->setChargeVoltage(4200);
        charger->setFastChargeCurrent(500);
        charger->enableCharging(true);
        Serial.print("CV: ");
        Serial.print(charger->getChargeVoltage());
        Serial.println(" mV");
        Serial.print("ICC: ");
        Serial.print(charger->getFastChargeCurrent());
        Serial.println(" mA");
    }

    Serial.println("\n--- Power Button Test ---");
    auto &button = pmic.pwron();
    uint16_t onLevel[] = {512, 1000, 2000};
    for (int i = 0; i < 3; i++) {
        if (button.setOnDurationMs(onLevel[i])) {
            delay(100);
            uint16_t readLevel = 0;
            button.getOnDurationMs(readLevel);
            Serial.print("On Level ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(readLevel);
            if (readLevel == onLevel[i]) {
                Serial.println("On Level set successfully");
            } else {
                Serial.print("On Level set failed");
                Serial.print("Expected: ");
                Serial.print(onLevel[i]);
                Serial.print(", Got: ");
                Serial.println(readLevel);
            }
        }
    }

    uint16_t offLevel[] = {6000, 8000, 10000};
    for (int i = 0; i < 3; i++) {
        if (button.setOffDurationMs(offLevel[i])) {
            delay(100);
            uint16_t readLevel = 0;
            button.getOffDurationMs(readLevel);
            Serial.print("Get Off Level ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(readLevel);
            if (readLevel == offLevel[i]) {
                Serial.println("Off Level set successfully");
            } else {
                Serial.print("Off Level set failed");
                Serial.print("Expected: ");
                Serial.print(offLevel[i]);
                Serial.print(", Got: ");
                Serial.println(readLevel);
            }
        }
    }

    Serial.println("\n--- ADC Test ---");

    // Enable ADC channels
    auto &adc = pmic.adc();
    adc.enableChannels(
        PmicAdcBase::Channel::VBUS_VOLTAGE |
        PmicAdcBase::Channel::VSYS_VOLTAGE |
        PmicAdcBase::Channel::BAT_VOLTAGE |
        PmicAdcBase::Channel::BAT_TEMPERATURE
    );

    Serial.println("\n=== Setup Complete ===\n");
}

void loop()
{
    static uint32_t lastPrint = 0;

    if (millis() - lastPrint > 2000) {
        lastPrint = millis();

        Serial.println("\n--- Status ---");

        Serial.print("VBUS good: ");
        Serial.println(pmic.isVbusGood() ? "Yes" : "No");
        Serial.print("Battery: ");
        Serial.println(pmic.isBatteryConnect() ? "Present" : "Absent");

        auto *charger = pmic.getCharger();
        if (charger) {
            auto status = charger->getStatus();
            Serial.print("Charging: ");
            Serial.println(status.charging ? "Yes" : "No");
            Serial.print("Vbus: ");
            Serial.println(status.vbusPresent ? "Present" : "Absent");
            Serial.print("CHG Status: ");
            switch (status.chargingStatus) {
            case PmicChargerBase::ChargingStatus::FAST_CHARGE:
                Serial.println("Fast Charging");
                break;
            case PmicChargerBase::ChargingStatus::PRE_CHARGE:
                Serial.println("Pre Charging");
                break;
            case PmicChargerBase::ChargingStatus::NO_CHARGING:
                Serial.println("No Charging");
                break;
            case PmicChargerBase::ChargingStatus::TERMINATION:
                Serial.println("Termination Charging");
                break;
            default:
                Serial.println("Unknown Charging Status");
                break;
            }
        }

        auto &adc = pmic.adc();
        float val = 0;
        if (adc.read(PmicAdcBase::Channel::BAT_PERCENTAGE, val)) {
            Serial.print("BAT%: ");
            Serial.print(val);
            Serial.println("%");
        }
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

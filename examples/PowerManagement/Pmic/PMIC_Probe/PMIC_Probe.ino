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
 * @file      PMIC_Probe.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 *
 * @brief PMIC Auto-Probe via Base Class Interface
 *
 * This example demonstrates automatic PMIC detection by probing multiple
 * drivers on the I2C bus and accessing the detected device through the
 * abstract PmicBase interface.
 *
 * Architecture:
 *   PmicBase (abstract)
 *     ├── PmicSY6970  (Silergy,  addr 0x6A)
 *     ├── PmicBQ25896 (TI,       addr 0x6B)
 *     └── PmicAXP517  (X-Powers, addr 0x34)
 *
 * Each driver's begin() validates the chip revision. The first successful
 * probe selects the active PMIC. All subsequent access goes through the
 * PmicBase pointer, demonstrating polymorphic PMIC control.
 */
#include <Arduino.h>
#include <Wire.h>
#include <PmicDrv.hpp>

#ifndef PMIC_SDA
#define PMIC_SDA  5
#endif

#ifndef PMIC_SCL
#define PMIC_SCL  6
#endif

// Instantiate all supported PMIC drivers
PmicSY6970  sy6970;
PmicBQ25896 bq25896;
PmicAXP517  axp517;

// Base-class pointer to the detected PMIC
PmicBase *pmic = nullptr;

static void printCapabilities(PmicCapability::Capability caps)
{
    Serial.print("Capabilities:");
    if (caps & PmicCapability::Capability::PmicSupportCharger)  Serial.print(" CHARGER");
    if (caps & PmicCapability::Capability::PmicSupportPower)    Serial.print(" POWER");
    if (caps & PmicCapability::Capability::PmicSupportAdc)      Serial.print(" ADC");
    if (caps & PmicCapability::Capability::PmicSupportLed)      Serial.print(" LED");
    if (caps & PmicCapability::Capability::PmicSupportGpio)     Serial.print(" GPIO");
    if (caps & PmicCapability::Capability::PmicSupportIrq)      Serial.print(" IRQ");
    if (caps & PmicCapability::Capability::PmicSupportBc12)     Serial.print(" BC12");
    if (caps & PmicCapability::Capability::PmicSupportPowerBtn) Serial.print(" PWR_BTN");
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("\n=== PMIC Auto-Probe Example ===");

    // Probe each PMIC at its known address.
    // begin() internally reads the device-revision register; only a
    // matching chip revision will return true.
    if (sy6970.begin(Wire, SY6970_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        pmic = &sy6970;
        Serial.println(" -> SY6970 detected at 0x6A");
    } else if (bq25896.begin(Wire, BQ25896_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        pmic = &bq25896;
        Serial.println(" -> BQ25896 detected at 0x6B");
    } else if (axp517.begin(Wire, AXP517_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        pmic = &axp517;
        Serial.println(" -> AXP517 detected at 0x34");
    }

    if (!pmic) {
        Serial.println("ERROR: No PMIC detected! Check wiring.");
        while (1) delay(1000);
    }

    Serial.print("\nChip: ");
    Serial.println(pmic->getChipName());

    PmicCapability::Capability caps = pmic->getCapabilities();
    printCapabilities(caps);

    // --- Charger configuration (base-class pointer) ---
    PmicChargerBase *chg = pmic->getCharger();
    if (chg) {
        chg->setPreChargeCurrent(256);
        chg->setFastChargeCurrent(1024);
        chg->setTerminationCurrent(64);
        chg->setChargeVoltage(4288);

        Serial.print("Pre-Charge Current: ");
        Serial.print(chg->getPreChargeCurrent());
        Serial.println(" mA");
        Serial.print("Fast-Charge Current: ");
        Serial.print(chg->getFastChargeCurrent());
        Serial.println(" mA");
        Serial.print("Termination Current: ");
        Serial.print(chg->getTerminationCurrent());
        Serial.println(" mA");
        Serial.print("Charge Voltage: ");
        Serial.print(chg->getChargeVoltage());
        Serial.println(" mV");
    }

    // --- Power configuration (base-class reference) ---
    pmic->power().setInputCurrentLimit(1000);
    pmic->power().setInputVoltageLimit(5000);
    pmic->power().setMinimumSystemVoltage(3300);

    Serial.print("Input Current Limit: ");
    Serial.print(pmic->power().getInputCurrentLimit());
    Serial.println(" mA");
    Serial.print("Input Voltage Limit: ");
    Serial.print(pmic->power().getInputVoltageLimit());
    Serial.println(" mV");
    Serial.print("Min System Voltage: ");
    Serial.print(pmic->power().getMinimumSystemVoltage());
    Serial.println(" mV");

    // --- LED configuration (base-class reference) ---
    if (caps & PmicCapability::Capability::PmicSupportLed) {
        pmic->led().setMode(PmicLedBase::Mode::DISABLE);
    }

    Serial.println("\nSetup complete! Printing status every 3s...\n");
}

void loop()
{
    static uint32_t interval = 0;

    if (millis() > interval) {
        interval = millis() + 3000;

        Serial.print("========== ");
        Serial.print(pmic->getChipName());
        Serial.println(" Status ==========");

        // --- Charger status ---
        PmicChargerBase *chg = pmic->getCharger();
        if (chg) {
            PmicChargerBase::Status s = chg->getStatus();
            Serial.print("Online: ");
            Serial.println(s.online ? "Yes" : "No");
            Serial.print("VBUS: ");
            Serial.println(s.vbusPresent ? "Present" : "Absent");
            Serial.print("Battery: ");
            Serial.println(s.batteryPresent ? "Present" : "Absent");
            Serial.print("Charging: ");
            Serial.println(s.charging ? "Yes" : "No");
            Serial.print("Charge Done: ");
            Serial.println(s.chargeDone ? "Yes" : "No");
            Serial.print("Fault: ");
            Serial.println(s.fault ? "Yes" : "No");
        }

        // --- ADC readings ---
        float val;
        if (pmic->adc().read(PmicAdcBase::Channel::VBUS_VOLTAGE, val)) {
            Serial.print("VBUS Voltage: ");
            Serial.print(val);
            Serial.println(" mV");
        }
        if (pmic->adc().read(PmicAdcBase::Channel::BAT_VOLTAGE, val)) {
            Serial.print("BAT Voltage: ");
            Serial.print(val);
            Serial.println(" mV");
        }
        if (pmic->adc().read(PmicAdcBase::Channel::VSYS_VOLTAGE, val)) {
            Serial.print("VSYS Voltage: ");
            Serial.print(val);
            Serial.println(" mV");
        }
        if (pmic->adc().read(PmicAdcBase::Channel::BAT_CURRENT, val)) {
            Serial.print("BAT Current: ");
            Serial.print(val);
            Serial.println(" mA");
        }

        // --- Power status ---
        Serial.print("Boost: ");
        Serial.println(pmic->power().isBoostEnabled() ? "Enabled" : "Disabled");
        Serial.print("Boost Voltage: ");
        Serial.print(pmic->power().getBoostVoltage());
        Serial.println(" mV");

        Serial.println("===================================\n");
    }
}

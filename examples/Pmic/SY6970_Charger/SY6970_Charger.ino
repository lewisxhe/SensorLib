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
 * @file      SY6970_Charger.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-10
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

#ifndef PMIC_IRQ
#define PMIC_IRQ  21
#endif

volatile bool isFaultTrigger = false;
uint32_t interval = 0;

PmicSY6970 bmu;

#define CHECK_FOR_ERRORS(condition) if (condition) {Serial.println("SY6970 error"); while(1)delay(1000);}

void printBanner()
{
    Serial.println("");
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║     SY6970 Charger Test                  ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println("");
}

void setup()
{
    bool rlst;

    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    printBanner();

    rlst = bmu.begin(Wire, SY6970_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL);
    if (!rlst) {
        Serial.println("SY6970 begin() failed. Check wiring");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("SY6970 initialized successfully");

    // bmu.led().setMode(PmicLedBase::Mode::AUTO); // STAT pin indicates charging status
    bmu.led().setMode(PmicLedBase::Mode::DISABLE); // DISABLE LED to save power (if STAT pin is not used for indication)

    // Charge current: 64-1024mA, 64mA steps (fuzzy: auto-adjusts to nearest valid value)
    rlst = bmu.charger().setPreChargeCurrent(256);
    CHECK_FOR_ERRORS(!rlst);
    // Fast charge current: 0-3008mA, 64mA steps (fuzzy: auto-adjusts to nearest valid value)
    rlst = bmu.charger().setFastChargeCurrent(1024);
    CHECK_FOR_ERRORS(!rlst);
    // Termination current: 64-1024mA, 64mA steps (fuzzy: auto-adjusts to nearest valid value)
    rlst = bmu.charger().setTerminationCurrent(64);
    CHECK_FOR_ERRORS(!rlst);
    // Charge voltage: 3840-4608mV, 16mV steps (fuzzy: auto-adjusts to nearest valid value)
    rlst = bmu.charger().setChargeVoltage(4288);
    CHECK_FOR_ERRORS(!rlst);

    Serial.print("Pre Charge Current: ");
    Serial.println(bmu.charger().getPreChargeCurrent());
    Serial.print("Fast Charge Current: ");
    Serial.println(bmu.charger().getFastChargeCurrent());
    Serial.print("Termination Current: ");
    Serial.println(bmu.charger().getTerminationCurrent());
    Serial.print("Charge Voltage: ");
    Serial.println(bmu.charger().getChargeVoltage());

    // Input current limit: 100-3008mA, 100mA steps (fuzzy: auto-adjusts to nearest valid value)
    bmu.power().setInputCurrentLimit(1000);
    // Input voltage limit: 3900-14000mV, 100mV steps (fuzzy: auto-adjusts to nearest valid value)
    bmu.power().setInputVoltageLimit(5000);
    // System minimum voltage: 3000-3700mV, 100mV steps (fuzzy: auto-adjusts to nearest valid value)
    bmu.power().setMinimumSystemVoltage(3300);

    Serial.print("Input Current Limit: ");
    Serial.println(bmu.power().getInputCurrentLimit());
    Serial.print("Input Voltage Limit: ");
    Serial.println(bmu.power().getInputVoltageLimit());
    Serial.print("Minimum System Voltage: ");
    Serial.println(bmu.power().getMinimumSystemVoltage());

    // Enable ADC channels (VBUS voltage, Battery voltage, VSYS voltage, Charging current, NTC temperature)
    bmu.adc().enableChannels(SY6970Adc::ADC_CONV_START);

    // Disabling ADC can reduce quiescent current.
    // If call `read` directly after disabling `adc`, it will trigger a single read operation.
    // bmu.adc().disableChannels(0); // No specific channels to disable, but call for compatibility

    Serial.println("Setup complete!");

#if PMIC_IRQ != -1
    pinMode(PMIC_IRQ, INPUT_PULLUP);
    attachInterrupt(PMIC_IRQ, []() {
        isFaultTrigger = true;
    }, FALLING);
#endif
}


void loop()
{
#if PMIC_IRQ != -1
    if (isFaultTrigger) {
        isFaultTrigger = false;
        PmicChargerBase::Status status = bmu.charger().getStatus();
        if (status.fault) {
            Serial.print("Fault Code: ");
            Serial.println(status.faultCode);

            using namespace SY6970Faults;
            if (isWatchdogTimeout(status.faultCode)) {
                Serial.println("\t Watchdog Fault");
            }
            if (isBoostFault(status.faultCode)) {
                Serial.println("\t Boost Mode Fault");
            }
            if (isChargeFault(status.faultCode)) {
                Serial.println("\t Charge Mode Fault");
                if (isChargeInputFault(status.faultCode)) {
                    Serial.println("\t   - Input Fault (BUS OVP or VBAT<BUS<3.8V)");
                }
                if (isChargeThermalFault(status.faultCode)) {
                    Serial.println("\t   - Thermal Shutdown");
                }
                if (isChargeTimerFault(status.faultCode)) {
                    Serial.println("\t   - Safety Timer Expiration");
                }
            }
            if (isBatteryFault(status.faultCode)) {
                Serial.println("\t Battery Fault (BATOVP)");
            }
            uint8_t ntcFault = getNtcFault(status.faultCode);
            if (ntcFault) {
                Serial.println("\t NTC Fault");
            }
        }
    }
#endif


    if (millis() > interval) {
        interval = millis() + 3000;

        Serial.println("\n========== SY6970 Status ==========");

        float val;
        bool ret;

        ret = bmu.adc().read(PmicAdcBase::Channel::VBUS_VOLTAGE, val);
        Serial.print("VBUS Voltage: ");
        Serial.print(ret ? val : -1);
        Serial.println(" mV");

        ret = bmu.adc().read(PmicAdcBase::Channel::BAT_VOLTAGE, val);
        Serial.print("Battery Voltage: ");
        Serial.print(ret ? val : -1);
        Serial.println(" mV");

        ret = bmu.adc().read(PmicAdcBase::Channel::VSYS_VOLTAGE, val);
        Serial.print("VSYS Voltage: ");
        Serial.print(ret ? val : -1);
        Serial.println(" mV");

        ret = bmu.adc().read(PmicAdcBase::Channel::BAT_CURRENT, val);
        Serial.print("Battery Current: ");
        Serial.print(ret ? val : -1);
        Serial.println(" mA");

        ret = bmu.adc().read(PmicAdcBase::Channel::BAT_TEMPERATURE, val);
        Serial.print("NTC Temperature: ");
        Serial.print(ret ? val : -1);
        Serial.println(" %");

        Serial.print("Charging: ");
        Serial.println(bmu.charger().isCharging() ? "Yes" : "No");

        PmicChargerBase::Status status = bmu.charger().getStatus();
        Serial.print("Online: ");
        Serial.println(status.online ? "Yes" : "No");
        Serial.print("VBUS Present: ");
        Serial.println(status.vbusPresent ? "Yes" : "No");
        Serial.print("Charge Done: ");
        Serial.println(status.chargeDone ? "Yes" : "No");
        Serial.print("Fault: ");
        Serial.println(status.fault ? "Yes" : "No");

        Serial.print("Boost Enabled: ");
        Serial.println(bmu.power().isBoostEnabled() ? "Yes" : "No");

        Serial.print("Boost Voltage: ");
        Serial.println(bmu.power().getBoostVoltage());

        Serial.println("===================================\n");
    }
}

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
 * @file      AXP2101_Irq.ino
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

void printU64(uint64_t value)
{
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "0x%016llX", value);
    Serial.print(buffer);
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== AXP2101 IRQ Example ===\n");

    if (!pmic.begin(Wire, AXP2101_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        Serial.println("Failed to init AXP2101!");
        while (1) delay(1000);
    }
    Serial.println("AXP2101 initialized");

    // Enable ADC and charger modules
    pmic.enableModule(PmicAXP2101::Module::GENERAL_ADC, true);
    pmic.enableModule(PmicAXP2101::Module::CELL_CHARGE, true);
    pmic.enableModule(PmicAXP2101::Module::BAT_DETECT, true);

    // Enable selected interrupt sources
    pmic.irq().enable(
            AXP2101Irq::IRQ_VBUS_INSERT |
            AXP2101Irq::IRQ_VBUS_REMOVE |
            AXP2101Irq::IRQ_BAT_INSERT |
            AXP2101Irq::IRQ_BAT_REMOVE |
            AXP2101Irq::IRQ_BAT_CHG_START |
            AXP2101Irq::IRQ_BAT_CHG_DONE |
            AXP2101Irq::IRQ_PKEY_SHORT_PRESS |
            AXP2101Irq::IRQ_PKEY_LONG_PRESS |
            AXP2101Irq::IRQ_WDT_EXPIRE |
            AXP2101Irq::IRQ_DIE_OVER_TEMP
        );
    Serial.println("IRQ sources enabled");

    // Clear any stale interrupts
    pmic.irq().clearStatus();

    Serial.println("\n=== Setup Complete ===\n");
}

void loop()
{
    uint64_t irqStatus = pmic.irq().readStatus(true);
    if (irqStatus) {

        Serial.print("\n--- Interrupt : 0x");
        printU64(irqStatus);
        Serial.print(" --- ");

        if (pmic.irq().isVbusInsert(irqStatus))
            Serial.println("VBUS inserted");
        if (pmic.irq().isVbusRemove(irqStatus))
            Serial.println("VBUS removed");
        if (pmic.irq().isBatInsert(irqStatus))
            Serial.println("Battery inserted");
        if (pmic.irq().isBatRemove(irqStatus))
            Serial.println("Battery removed");
        if (pmic.irq().isBatChgStart(irqStatus))
            Serial.println("Charge started");
        if (pmic.irq().isBatChgDone(irqStatus))
            Serial.println("Charge done");
        if (pmic.irq().isPkeyShortPress(irqStatus)) {
            Serial.println("PEKEY short press");
            pmic.led().setMode(PmicLedBase::Mode::MANUAL);

            AXP2101Led::ManualState state = pmic.led().getManualState();
            switch (state) {
            case PmicLedBase::ManualState::LEVEL_HIGH:
                Serial.println("LED is ON");
                break;
            case PmicLedBase::ManualState::LEVEL_LOW:
                Serial.println("LED is OFF");
                break;
            case PmicLedBase::ManualState::HiZ:
                Serial.println("LED is HiZ");
                break;
            case PmicLedBase::ManualState::BLINK_1HZ:
                Serial.println("LED is blinking at 1Hz");
                break;
            case PmicLedBase::ManualState::BLINK_4HZ:
                Serial.println("LED is blinking at 4Hz");
                break;
            default:
                break;
            }
            if (state == PmicLedBase::ManualState::LEVEL_HIGH) {
                Serial.println("ON LED");
                pmic.led().setManualState(PmicLedBase::ManualState::LEVEL_LOW);
            } else {
                Serial.println("OFF LED");
                pmic.led().setManualState(PmicLedBase::ManualState::LEVEL_HIGH);
            }
        }
        if (pmic.irq().isPkeyLongPress(irqStatus))
            Serial.println("PEKEY long press");
        if (pmic.irq().isWdtExpire(irqStatus))
            Serial.println("Watchdog expired");
        if (pmic.irq().isDieOverTemp(irqStatus))
            Serial.println("Die over-temperature");
        if (pmic.irq().isLdoOverCurr(irqStatus))
            Serial.println("LDO over-current");
        if (pmic.irq().isBatfetOverCurr(irqStatus))
            Serial.println("BATFET over-current");
        if (pmic.irq().isBatOverVolt(irqStatus))
            Serial.println("Battery over-voltage");
        if (pmic.irq().isChgTimeout(irqStatus))
            Serial.println("Charge timeout");
        if (pmic.irq().isGaugeNewSoc(irqStatus))
            Serial.println("New SOC reading");
        if (pmic.irq().isBatOverTempWork(irqStatus))
            Serial.println("Battery over-temp (work)");
        if (pmic.irq().isBatUnderTempWork(irqStatus))
            Serial.println("Battery under-temp (work)");
        if (pmic.irq().isBatOverTempChg(irqStatus))
            Serial.println("Battery charge over-temp");
        if (pmic.irq().isBatUnderTempChg(irqStatus))
            Serial.println("Battery charge under-temp");
        if (pmic.irq().isPkeyNegativeEdge(irqStatus))
            Serial.println("PEKEY negative edge");
        if (pmic.irq().isPkeyPositiveEdge(irqStatus))
            Serial.println("PEKEY positive edge");
    }
    delay(100);
}

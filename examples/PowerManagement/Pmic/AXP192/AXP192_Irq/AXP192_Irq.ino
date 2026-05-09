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
 * @file      AXP192_Irq.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-05-08
 *
 */

#include <pmic/xpowers/axp192/PmicAXP192.hpp>

#ifndef PMIC_SDA
#define PMIC_SDA  3
#endif

#ifndef PMIC_SCL
#define PMIC_SCL  2
#endif

#ifndef PMIC_IRQ
#define PMIC_IRQ  21
#endif

PmicAXP192 pmic;

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

    Serial.println("\n=== AXP192 IRQ Example ===\n");

    if (!pmic.begin(Wire, AXP192_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        Serial.println("Failed to init AXP192!");
        while (1) delay(1000);
    }
    Serial.println("AXP192 initialized");

    // Set timer 1 minute timeout trigger interrupt ,auto-restart every minute
    pmic.timer().setTimer(1);

    // Enable selected interrupt sources via base class
    pmic.irq().enable(
            AXP192Irq::IRQ_VBUS_INSERT |
            AXP192Irq::IRQ_VBUS_REMOVE |
            AXP192Irq::IRQ_BAT_INSERT |
            AXP192Irq::IRQ_BAT_REMOVE |
            AXP192Irq::IRQ_BAT_CHG_START |
            AXP192Irq::IRQ_BAT_CHG_DONE |
            AXP192Irq::IRQ_PEKEY_SHORT_PRESS |
            AXP192Irq::IRQ_PEKEY_LONG_PRESS |
            AXP192Irq::IRQ_TIMER_TIMEOUT
        );
    Serial.println("IRQ sources enabled");

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
        if (AXP192Irq::isVbusInsert(irqStatus))
            Serial.println("VBUS inserted");
        if (AXP192Irq::isVbusRemove(irqStatus))
            Serial.println("VBUS removed");
        if (AXP192Irq::isBatInsert(irqStatus))
            Serial.println("Battery inserted");
        if (AXP192Irq::isBatRemove(irqStatus))
            Serial.println("Battery removed");
        if (AXP192Irq::isBatChgStart(irqStatus))
            Serial.println("Charge started");
        if (AXP192Irq::isBatChgDone(irqStatus))
            Serial.println("Charge done");
        if (AXP192Irq::isPekeyShortPress(irqStatus))
            Serial.println("PEKEY short press");
        if (AXP192Irq::isPekeyLongPress(irqStatus))
            Serial.println("PEKEY long press");
        if (AXP192Irq::isAcinOv(irqStatus))
            Serial.println("ACIN over-voltage");
        if (AXP192Irq::isAcinInsert(irqStatus))
            Serial.println("ACIN inserted");
        if (AXP192Irq::isAcinRemove(irqStatus))
            Serial.println("ACIN removed");
        if (AXP192Irq::isVbusOv(irqStatus))
            Serial.println("VBUS over-voltage");
        if (AXP192Irq::isChipOvTemp(irqStatus))
            Serial.println("Chip over-temperature");
        if (AXP192Irq::isChgCurrLess(irqStatus))
            Serial.println("Charge current low");
        if (AXP192Irq::isBatTempHigh(irqStatus))
            Serial.println("Battery temp high");
        if (AXP192Irq::isBatTempLow(irqStatus))
            Serial.println("Battery temp low");
        if (AXP192Irq::isDc1VoltLess(irqStatus))
            Serial.println("DCDC1 voltage low");
        if (AXP192Irq::isDc2VoltLess(irqStatus))
            Serial.println("DCDC2 voltage low");
        if (AXP192Irq::isDc3VoltLess(irqStatus))
            Serial.println("DCDC3 voltage low");
        if (AXP192Irq::isTimerIrq(irqStatus)) {
            static uint32_t lastTimerIrq = 0;
            uint32_t now = millis() / 1000;
            Serial.print("[");
            Serial.print(now);
            Serial.println("] Timer interrupt");
            if (pmic.timer().isTimedOut()) {
                Serial.print("Timer timed out");
                Serial.print(" [");
                Serial.print((now - lastTimerIrq) / 60);
                Serial.print("] minute have passed");
                pmic.timer().clearTimeout();
            }
            lastTimerIrq = now;
        }
    }

    delay(100);
}

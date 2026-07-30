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
 * @file      axp517_pd_auto_request.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-07-25
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
#define PMIC_IRQ  44
#endif

#ifndef PD_TARGET_MV
#define PD_TARGET_MV  9000
#endif

#ifndef PD_TIMEOUT_MS
#define PD_TIMEOUT_MS  6000
#endif

#if PMIC_IRQ < 0
#error "AXP517 PD negotiation requires a valid PMIC_IRQ pin."
#endif

PmicAXP517 pmic;
AXP517PdNegotiator::SourceCaps sourceCaps;

volatile bool irqTriggered = false;
bool pdReady = false;

static constexpr uint16_t VBUS_PRESENT_MV = 4000;
static constexpr uint32_t VBUS_SETTLE_MS = 300;
static constexpr uint32_t IRQ_PENDING_PRINT_MS = 1000;

void onPmicIrq()
{
    irqTriggered = true;
}

void printSourceCaps(const AXP517PdNegotiator::SourceCaps &caps)
{
    Serial.println("\n--- Source Capabilities ---");
    Serial.print("Total PDO count: ");
    Serial.println(caps.totalPdoCount);

    if (caps.fixedCount == 0) {
        Serial.println("No fixed PDO found.");
        return;
    }

    for (uint8_t i = 0; i < caps.fixedCount; ++i) {
        const auto &pdo = caps.fixed[i];

        Serial.print("PDO");
        Serial.print(pdo.pdoIndex1Based);
        Serial.print(": ");
        Serial.print(pdo.mv);
        Serial.print(" mV / ");
        Serial.print(pdo.maMax);
        Serial.print(" mA, raw=0x");
        Serial.println(pdo.rawPdo, HEX);
    }
}

void printIrqStatus(uint64_t status)
{
    Serial.print("IRQ Status: high=0x");
    Serial.print(static_cast<uint32_t>(status >> 32), HEX);
    Serial.print(" low=0x");
    Serial.println(static_cast<uint32_t>(status), HEX);
}

void printVbus()
{
    uint16_t vbusMv = 0;

    if (pmic.tcpc().readVbusMv(vbusMv)) {
        Serial.print("TCPC VBUS: ");
        Serial.print(vbusMv);
        Serial.println(" mV");
    }
}

bool isVbusPresentNow()
{
    if (pmic.charger().getStatus().vbusPresent) {
        return true;
    }

    if (pmic.tcpc().isVbusPresent()) {
        return true;
    }

    uint16_t vbusMv = 0;
    return pmic.tcpc().readVbusMv(vbusMv) && vbusMv >= VBUS_PRESENT_MV;
}

bool waitForVbusPresent(uint32_t timeoutMs)
{
    uint32_t start = millis();

    do {
        if (isVbusPresentNow()) {
            return true;
        }
        delay(10);
    } while (millis() - start < timeoutMs);

    return false;
}

bool isTcpcDetachAlert(uint64_t irqStatus, bool vbusPresent)
{
    uint16_t pdAlert = static_cast<uint16_t>(irqStatus >> 32);

    if (vbusPresent) {
        return false;
    }

    if (pdAlert & AXP517Tcpc::AlertVbusDisconnect) {
        return true;
    }

    if ((pdAlert & AXP517Tcpc::AlertCcStatus) && !pmic.tcpc().isAttached()) {
        return true;
    }

    if (pdAlert & AXP517Tcpc::AlertPowerStatus) {
        return true;
    }

    return false;
}

bool isTcpcSourceActivityAlert(uint64_t irqStatus)
{
    uint16_t pdAlert = static_cast<uint16_t>(irqStatus >> 32);
    return (pdAlert & (AXP517Tcpc::AlertPowerStatus |
                       AXP517Tcpc::AlertRxStatus)) != 0;
}

bool isTcpcFaultAlert(uint64_t irqStatus)
{
    uint16_t pdAlert = static_cast<uint16_t>(irqStatus >> 32);
    return (pdAlert & AXP517Tcpc::AlertFault) != 0;
}

bool isStickyTcpcAlert(uint64_t irqStatus)
{
    uint16_t pdAlert = static_cast<uint16_t>(irqStatus >> 32);
    uint32_t pmicIrq = static_cast<uint32_t>(irqStatus);
    constexpr uint16_t actionable =
        AXP517Tcpc::AlertCcStatus |
        AXP517Tcpc::AlertPowerStatus |
        AXP517Tcpc::AlertRxStatus |
        AXP517Tcpc::AlertRxHardReset |
        AXP517Tcpc::AlertTxFailed |
        AXP517Tcpc::AlertTxDiscarded |
        AXP517Tcpc::AlertTxSuccess |
        AXP517Tcpc::AlertVbusDisconnect;

    return pmicIrq == 0 && pdAlert != 0 && (pdAlert & actionable) == 0;
}

void resetPdState(const char *reason)
{
    Serial.println(reason);
    pdReady = false;
    sourceCaps = AXP517PdNegotiator::SourceCaps{};
    pmic.pd().reset();
    pmic.tcpc().clearTx();

    if (!pmic.irq().clearStatus()) {
        Serial.println("Failed to clear IRQ status after PD reset.");
    }
}

void requestPdVoltage()
{
    sourceCaps = AXP517PdNegotiator::SourceCaps{};

    Serial.print("Requesting PD voltage: ");
    Serial.print(PD_TARGET_MV);
    Serial.println(" mV");

    pdReady = pmic.requestPd(PD_TARGET_MV, PD_TIMEOUT_MS, &sourceCaps);
    printSourceCaps(sourceCaps);

    Serial.println("\n--- PD Result ---");
    Serial.println(pdReady ? "Negotiation success." : "Negotiation failed.");
    printVbus();

    // Keep GPIO0 mux untouched in this mixed IRQ example. The PD negotiator
    // only needs the IRQ pin level; PWRON/BAT/VBUS events still use PMIC_IRQ.
    if (!pmic.irq().clearStatus()) {
        Serial.println("Failed to clear IRQ status after PD request.");
    }
}

bool handlePmicIrq()
{
    uint64_t irqStatus = pmic.irq().readStatus(true);
    if (irqStatus == 0) {
        return false;
    }

    bool stickyTcpc = isStickyTcpcAlert(irqStatus);
    if (!stickyTcpc) {
        printIrqStatus(irqStatus);
    }

    if (pmic.irq().isPwrOnShortPress(irqStatus)) {
        Serial.println("Power On Short Press Detected");
        // requestPdVoltage();
    }

    if (pmic.irq().isBatteryInsert(irqStatus)) {
        Serial.println("Battery Inserted");
    }

    if (pmic.irq().isBatteryRemove(irqStatus)) {
        Serial.println("Battery Removed");
    }

    bool vbusInserted = pmic.irq().isVbusInsert(irqStatus);
    bool vbusRemoved = pmic.irq().isVbusRemove(irqStatus);
    bool tcpcSourceActivity = isTcpcSourceActivityAlert(irqStatus);
    bool vbusPresent = isVbusPresentNow();
    if (!pdReady && !vbusPresent && (vbusInserted || tcpcSourceActivity)) {
        vbusPresent = waitForVbusPresent(VBUS_SETTLE_MS);
    }

    bool tcpcDetached = isTcpcDetachAlert(irqStatus, vbusPresent);
    bool tcpcFault = isTcpcFaultAlert(irqStatus);
    if (pdReady && (vbusRemoved || tcpcDetached || (!vbusPresent && (tcpcFault || stickyTcpc)))) {
        if (vbusRemoved) {
            Serial.println("VBUS Removed");
        }
        if (tcpcDetached) {
            Serial.println("TCPC detached");
        }
        resetPdState("PD state reset.");
        return true;
    }

    if (!pdReady && (vbusInserted || tcpcSourceActivity)) {
        if (vbusInserted) {
            Serial.println("VBUS Inserted");
        } else {
            Serial.println("TCPC VBUS activity detected");
        }

        if (vbusPresent) {
            requestPdVoltage();
        } else {
            Serial.println("VBUS is not present, skip PD request.");
        }
        return true;
    }

    if (stickyTcpc) {
        static uint32_t lastFaultPrint = 0;
        if (millis() - lastFaultPrint > 1000) {
            lastFaultPrint = millis();
            printIrqStatus(irqStatus);
            Serial.println(tcpcFault ? "TCPC fault pending" : "TCPC alert pending");
        }
        return false;
    }

    return true;
}

void drainPmicIrq()
{
    uint32_t deadline = millis() + 250;

    while (millis() < deadline) {
        bool handled = handlePmicIrq();
        delay(2);

        if (digitalRead(PMIC_IRQ) != LOW) {
            return;
        }

        if (!handled) {
            return;
        }
    }

    if (digitalRead(PMIC_IRQ) == LOW) {
        Serial.println("PMIC_IRQ is still LOW after drain.");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== AXP517 PD Auto Request Example ===\n");
    Serial.print("Target voltage: ");
    Serial.print(PD_TARGET_MV);
    Serial.println(" mV");

    if (!pmic.begin(Wire, AXP517_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL, PMIC_IRQ)) {
        Serial.println("Failed to init AXP517 or PMIC_IRQ pin!");
        while (1) delay(1000);
    }

    pmic.irq().enable(AXP517Irq::IRQ_VBUS_INSERT |
                      AXP517Irq::IRQ_VBUS_REMOVE |
                      AXP517Irq::IRQ_PWR_ON_SHORT_PRESS |
                      AXP517Irq::IRQ_BATTERY_INSERT |
                      AXP517Irq::IRQ_BATTERY_REMOVE);
    pmic.irq().clearStatus();

    pinMode(PMIC_IRQ, INPUT_PULLUP);
    attachInterrupt(PMIC_IRQ, onPmicIrq, FALLING);

    Serial.print("PMIC_IRQ pin: ");
    Serial.println(PMIC_IRQ);
    Serial.println("Waiting for VBUS insert interrupt...");

    if (pmic.charger().getStatus().vbusPresent) {
        Serial.println("VBUS already present");
        requestPdVoltage();
    }
}

void loop()
{
    static uint32_t lastPrint = 0;
    static uint32_t lastPendingPrint = 0;

    if (irqTriggered || digitalRead(PMIC_IRQ) == LOW) {
        if (irqTriggered) {
            Serial.println("IRQ Triggered");
        } else if (millis() - lastPendingPrint > IRQ_PENDING_PRINT_MS) {
            lastPendingPrint = millis();
            Serial.println("IRQ Pending");
        }
        irqTriggered = false;
        drainPmicIrq();
    }

    if (millis() - lastPrint > 2000) {
        lastPrint = millis();

        Serial.println("\n--- Status ---");
        Serial.print("PD: ");
        Serial.println(pdReady ? "Ready" : "Waiting");
        printVbus();
    }

    delay(100);
}

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
 * @file      AXP517_Interrupt.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-07-29
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

#if PMIC_IRQ < 0
#error "AXP517 interrupt example requires a valid PMIC_IRQ pin."
#endif

static constexpr uint32_t IRQ_DRAIN_MS = 250;
static constexpr uint32_t IRQ_PENDING_PRINT_MS = 1000;

PmicAXP517 pmic;
volatile bool irqTriggered = false;

struct StatusSnapshot {
    bool valid = false;
    bool vbusPresent = false;
    bool batteryPresent = false;
    bool charging = false;
    bool chargeDone = false;
    bool chargerFault = false;
    uint32_t chargerFaultCode = 0;
    bool tcpcAttached = false;
    bool tcpcVbusValid = false;
    uint16_t tcpcVbusMv = 0;
    PmicBc12Base::PortType bc12Type = PmicBc12Base::PortType::Unknown;
    uint8_t bc12Raw = 0;
    bool bc12Detecting = false;
};

StatusSnapshot lastSnapshot;
uint32_t irqCount = 0;

void onPmicIrq()
{
    irqTriggered = true;
}

const char *bc12TypeName(PmicBc12Base::PortType type)
{
    switch (type) {
    case PmicBc12Base::PortType::None:      return "None";
    case PmicBc12Base::PortType::SDP:       return "SDP";
    case PmicBc12Base::PortType::CDP:       return "CDP";
    case PmicBc12Base::PortType::DCP:       return "DCP";
    case PmicBc12Base::PortType::Apple_1A:  return "Apple 1A";
    case PmicBc12Base::PortType::Apple_2A:  return "Apple 2A";
    case PmicBc12Base::PortType::Apple_2_4A:return "Apple 2.4A";
    default:                                return "Unknown";
    }
}

const char *presentName(bool present)
{
    return present ? "Present" : "Absent";
}

const char *attachedName(bool attached)
{
    return attached ? "Attached" : "Detached";
}

const char *yesNoName(bool value)
{
    return value ? "yes" : "no";
}

const char *chargeName(const StatusSnapshot &snapshot)
{
    if (snapshot.charging) return "Charging";
    if (snapshot.chargeDone) return "Done";
    return "Idle";
}

void printHex16(uint16_t value)
{
    char buffer[7];
    snprintf(buffer, sizeof(buffer), "0x%04X", value);
    Serial.print(buffer);
}

void printHex32(uint32_t value)
{
    char buffer[11];
    snprintf(buffer, sizeof(buffer), "0x%08lX", static_cast<unsigned long>(value));
    Serial.print(buffer);
}

StatusSnapshot readStatusSnapshot()
{
    StatusSnapshot snapshot;
    auto charger = pmic.charger().getStatus();
    auto bc12 = pmic.bc12().readResult();

    snapshot.valid = true;
    snapshot.vbusPresent = charger.vbusPresent;
    snapshot.batteryPresent = charger.batteryPresent;
    snapshot.charging = charger.charging;
    snapshot.chargeDone = charger.chargeDone;
    snapshot.chargerFault = charger.fault;
    snapshot.chargerFaultCode = charger.faultCode;
    snapshot.tcpcAttached = pmic.tcpc().isAttached();
    snapshot.tcpcVbusValid = pmic.tcpc().readVbusMv(snapshot.tcpcVbusMv);
    snapshot.bc12Type = bc12.type;
    snapshot.bc12Raw = bc12.raw;
    snapshot.bc12Detecting = bc12.detecting;
    return snapshot;
}

void printEventItem(bool &first, const char *text)
{
    if (!first) {
        Serial.print(", ");
    }
    Serial.print(text);
    first = false;
}

void printPdAlertLine(uint16_t pdAlert)
{
    Serial.print("  TCPC  ");
    bool first = true;
    if (pdAlert & AXP517Tcpc::AlertCcStatus) {
        printEventItem(first, "CC status");
    }
    if (pdAlert & AXP517Tcpc::AlertPowerStatus) {
        printEventItem(first, "Power status");
    }
    if (pdAlert & AXP517Tcpc::AlertRxStatus) {
        printEventItem(first, "RX SOP message");
    }
    if (pdAlert & AXP517Tcpc::AlertRxHardReset) {
        printEventItem(first, "RX hard reset");
    }
    if (pdAlert & AXP517Tcpc::AlertTxFailed) {
        printEventItem(first, "TX failed");
    }
    if (pdAlert & AXP517Tcpc::AlertTxDiscarded) {
        printEventItem(first, "TX discarded");
    }
    if (pdAlert & AXP517Tcpc::AlertTxSuccess) {
        printEventItem(first, "TX success");
    }
    if (pdAlert & AXP517Tcpc::AlertVbusAlarmHi) {
        printEventItem(first, "VBUS alarm high");
    }
    if (pdAlert & AXP517Tcpc::AlertVbusAlarmLo) {
        printEventItem(first, "VBUS alarm low");
    }
    if (pdAlert & AXP517Tcpc::AlertFault) {
        printEventItem(first, "Fault");
    }
    if (pdAlert & AXP517Tcpc::AlertRxBufOverflow) {
        printEventItem(first, "RX buffer overflow");
    }
    if (pdAlert & AXP517Tcpc::AlertVbusDisconnect) {
        printEventItem(first, "VBUS sink disconnect");
    }
    if (pdAlert & AXP517Tcpc::AlertExtendedStatus) {
        printEventItem(first, "Extended status");
    }
    if (pdAlert & AXP517Tcpc::AlertExtended) {
        printEventItem(first, "Extended alert");
    }
    if (first) {
        Serial.print("none");
    }
    Serial.println();
}

void printPmicIrqLine(uint64_t status)
{
    Serial.print("  PMIC  ");
    bool first = true;
    if (pmic.irq().isSocWarningLevel(status)) {
        printEventItem(first, "SOC warning");
    }
    if (pmic.irq().isSocShutdownLevel(status)) {
        printEventItem(first, "SOC shutdown");
    }
    if (pmic.irq().isGaugeNewSoc(status)) {
        printEventItem(first, "Gauge new SOC");
    }
    if (pmic.irq().isChargeToNormal(status)) {
        printEventItem(first, "Charge normal");
    }
    if (pmic.irq().isBoostOverVoltage(status)) {
        printEventItem(first, "Boost over voltage");
    }
    if (pmic.irq().isVbusOverVoltage(status)) {
        printEventItem(first, "VBUS over voltage");
    }
    if (pmic.irq().isVbusFault(status)) {
        printEventItem(first, "VBUS fault");
    }
    if (pmic.irq().isVbusInsert(status)) {
        printEventItem(first, "VBUS inserted");
    }
    if (pmic.irq().isVbusRemove(status)) {
        printEventItem(first, "VBUS removed");
    }
    if (pmic.irq().isBatteryInsert(status)) {
        printEventItem(first, "Battery inserted");
    }
    if (pmic.irq().isBatteryRemove(status)) {
        printEventItem(first, "Battery removed");
    }
    if (pmic.irq().isPwrOnShortPress(status)) {
        printEventItem(first, "PWRON short press");
    }
    if (pmic.irq().isPwrOnLongPress(status)) {
        printEventItem(first, "PWRON long press");
    }
    if (pmic.irq().isPwrOnNegativeEdge(status)) {
        printEventItem(first, "PWRON negative edge");
    }
    if (pmic.irq().isPwrOnPositiveEdge(status)) {
        printEventItem(first, "PWRON positive edge");
    }
    if (pmic.irq().isWatchdogExpire(status)) {
        printEventItem(first, "Watchdog expired");
    }
    if (pmic.irq().isBatFetOcp(status)) {
        printEventItem(first, "Battery FET OCP");
    }
    if (pmic.irq().isChargeDone(status)) {
        printEventItem(first, "Charge done");
    }
    if (pmic.irq().isChargeStart(status)) {
        printEventItem(first, "Charge started");
    }
    if (pmic.irq().isDieOverTempLevel1(status)) {
        printEventItem(first, "Die over temperature");
    }
    if (pmic.irq().isChgSafetyTimer(status)) {
        printEventItem(first, "Charge safety timer");
    }
    if (pmic.irq().isBatOverVoltage(status)) {
        printEventItem(first, "Battery over voltage");
    }
    if (pmic.irq().isBc12DetectFinished(status)) {
        printEventItem(first, "BC1.2 detect finished");
    }
    if (pmic.irq().isBc12DetectChange(status)) {
        printEventItem(first, "BC1.2 result changed");
    }
    if (pmic.irq().isBatOverTempQuitChg(status)) {
        printEventItem(first, "Battery over temp quit charge");
    }
    if (pmic.irq().isBatOverTempChg(status)) {
        printEventItem(first, "Battery over temp charge");
    }
    if (pmic.irq().isBatUnderTempChg(status)) {
        printEventItem(first, "Battery under temp charge");
    }
    if (pmic.irq().isBatOverTempWork(status)) {
        printEventItem(first, "Battery over temp work");
    }
    if (pmic.irq().isBatUnderTempWork(status)) {
        printEventItem(first, "Battery under temp work");
    }
    if (first) {
        Serial.print("none");
    }
    Serial.println();
}

bool printBoolChange(const char *label, bool before, bool after, const char *(*name)(bool), bool &first)
{
    if (before == after) {
        return false;
    }

    if (!first) {
        Serial.print(" | ");
    }
    Serial.print(label);
    Serial.print(": ");
    Serial.print(name(before));
    Serial.print(" -> ");
    Serial.print(name(after));
    first = false;
    return true;
}

bool printChargeChange(const StatusSnapshot &before, const StatusSnapshot &after, bool &first)
{
    const char *beforeName = chargeName(before);
    const char *afterName = chargeName(after);
    if (strcmp(beforeName, afterName) == 0) {
        return false;
    }

    if (!first) {
        Serial.print(" | ");
    }
    Serial.print("Charge: ");
    Serial.print(beforeName);
    Serial.print(" -> ");
    Serial.print(afterName);
    first = false;
    return true;
}

void printStatusLine(const char *label, bool anyChange)
{
    if (!anyChange) {
        Serial.print("  ");
        Serial.print(label);
        Serial.println("  no change");
    } else {
        Serial.println();
    }
}

void printSnapshot(const StatusSnapshot &snapshot)
{
    Serial.print("  CHG   VBUS=");
    Serial.print(presentName(snapshot.vbusPresent));
    Serial.print(" Battery=");
    Serial.print(presentName(snapshot.batteryPresent));
    Serial.print(" Charge=");
    Serial.print(chargeName(snapshot));
    Serial.print(" Fault=");
    Serial.print(yesNoName(snapshot.chargerFault));
    if (snapshot.chargerFault) {
        Serial.print(" code=");
        printHex32(snapshot.chargerFaultCode);
    }
    Serial.println();

    Serial.print("  TCPC  CC=");
    Serial.print(attachedName(snapshot.tcpcAttached));
    Serial.print(" VBUS=");
    if (snapshot.tcpcVbusValid) {
        Serial.print(snapshot.tcpcVbusMv);
        Serial.print(" mV");
    } else {
        Serial.print("n/a");
    }
    Serial.println();

    Serial.print("  BC1.2 Type=");
    Serial.print(bc12TypeName(snapshot.bc12Type));
    Serial.print(" raw=");
    printHex16(snapshot.bc12Raw);
    Serial.print(" detecting=");
    Serial.println(yesNoName(snapshot.bc12Detecting));
}

void printSnapshotChanges(const StatusSnapshot &before, const StatusSnapshot &after)
{
    if (!before.valid) {
        printSnapshot(after);
        return;
    }

    Serial.print("  CHG   ");
    bool first = true;
    printBoolChange("VBUS", before.vbusPresent, after.vbusPresent, presentName, first);
    printBoolChange("Battery", before.batteryPresent, after.batteryPresent, presentName, first);
    printChargeChange(before, after, first);
    printBoolChange("Fault", before.chargerFault, after.chargerFault, yesNoName, first);
    if (before.chargerFaultCode != after.chargerFaultCode) {
        if (!first) Serial.print(" | ");
        Serial.print("FaultCode: ");
        printHex32(before.chargerFaultCode);
        Serial.print(" -> ");
        printHex32(after.chargerFaultCode);
        first = false;
    }
    printStatusLine("CHG ", !first);

    Serial.print("  TYPEC ");
    first = true;
    printBoolChange("CC", before.tcpcAttached, after.tcpcAttached, attachedName, first);
    if (before.tcpcVbusValid != after.tcpcVbusValid ||
            before.tcpcVbusMv != after.tcpcVbusMv) {
        if (!first) Serial.print(" | ");
        Serial.print("VBUS: ");
        if (before.tcpcVbusValid) {
            Serial.print(before.tcpcVbusMv);
            Serial.print(" mV");
        } else {
            Serial.print("n/a");
        }
        Serial.print(" -> ");
        if (after.tcpcVbusValid) {
            Serial.print(after.tcpcVbusMv);
            Serial.print(" mV");
        } else {
            Serial.print("n/a");
        }
        first = false;
    }
    printStatusLine("TYPEC", !first);

    Serial.print("  BC1.2 ");
    first = true;
    if (before.bc12Type != after.bc12Type) {
        Serial.print("Type: ");
        Serial.print(bc12TypeName(before.bc12Type));
        Serial.print(" -> ");
        Serial.print(bc12TypeName(after.bc12Type));
        first = false;
    }
    if (before.bc12Raw != after.bc12Raw) {
        if (!first) Serial.print(" | ");
        Serial.print("Raw: ");
        printHex16(before.bc12Raw);
        Serial.print(" -> ");
        printHex16(after.bc12Raw);
        first = false;
    }
    printBoolChange("Detecting", before.bc12Detecting, after.bc12Detecting, yesNoName, first);
    printStatusLine("BC1.2", !first);
}

bool handleIrqOnce()
{
    uint64_t status = pmic.irq().readStatus(true);
    if (status == 0) {
        return false;
    }

    StatusSnapshot snapshot = readStatusSnapshot();

    Serial.println();
    Serial.print("[");
    Serial.print(millis());
    Serial.print(" ms] IRQ #");
    Serial.println(++irqCount);

    Serial.print("  RAW   pd=");
    printHex16(static_cast<uint16_t>(status >> 32));
    Serial.print(" pmic=");
    printHex32(static_cast<uint32_t>(status));
    Serial.print(" pin=");
    Serial.println(digitalRead(PMIC_IRQ) == LOW ? "LOW" : "HIGH");

    printPmicIrqLine(status);
    printPdAlertLine(static_cast<uint16_t>(status >> 32));
    printSnapshotChanges(lastSnapshot, snapshot);
    Serial.println();
    lastSnapshot = snapshot;
    return true;
}

void drainIrq()
{
    uint32_t start = millis();

    do {
        bool handled = handleIrqOnce();
        delay(2);

        if (digitalRead(PMIC_IRQ) != LOW) {
            return;
        }

        if (!handled) {
            return;
        }
    } while (millis() - start < IRQ_DRAIN_MS);

    if (digitalRead(PMIC_IRQ) == LOW) {
        Serial.println("PMIC_IRQ is still LOW after drain.");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== AXP517 Interrupt Example ===\n");

    if (!pmic.begin(Wire, AXP517_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL, PMIC_IRQ)) {
        Serial.println("Failed to init AXP517 or PMIC_IRQ pin!");
        while (1) delay(1000);
    }

    pmic.enableModule(PmicAXP517::Module::CHARGE, true);
    pmic.enableModule(PmicAXP517::Module::BC12, true);
    pmic.bc12().enableAutoDetect(true);

    uint64_t mask =
        AXP517Irq::IRQ_SOC_WARNING_LEVEL |
        AXP517Irq::IRQ_SOC_SHUTDOWN_LEVEL |
        AXP517Irq::IRQ_GAUGE_NEW_SOC |
        AXP517Irq::IRQ_CHARGE_TO_NORMAL |
        AXP517Irq::IRQ_BOOST_OVER_VOLTAGE |
        AXP517Irq::IRQ_VBUS_OVER_VOLTAGE |
        AXP517Irq::IRQ_VBUS_FAULT |
        AXP517Irq::IRQ_VBUS_INSERT |
        AXP517Irq::IRQ_VBUS_REMOVE |
        AXP517Irq::IRQ_BATTERY_INSERT |
        AXP517Irq::IRQ_BATTERY_REMOVE |
        AXP517Irq::IRQ_PWR_ON_SHORT_PRESS |
        AXP517Irq::IRQ_PWR_ON_LONG_PRESS |
        AXP517Irq::IRQ_PWR_ON_NEGATIVE_EDGE |
        AXP517Irq::IRQ_PWR_ON_POSITIVE_EDGE |
        AXP517Irq::IRQ_WATCHDOG_EXPIRE |
        AXP517Irq::IRQ_BAT_FET_OCP |
        AXP517Irq::IRQ_CHARGE_DONE |
        AXP517Irq::IRQ_CHARGE_START |
        AXP517Irq::IRQ_DIE_OVER_TEMP_LEVEL1 |
        AXP517Irq::IRQ_CHG_SAFETY_TIMER |
        AXP517Irq::IRQ_BAT_OVER_VOLTAGE |
        AXP517Irq::IRQ_BC12_DETECT_FINISHED |
        AXP517Irq::IRQ_BC12_DETECT_CHANGE |
        AXP517Irq::IRQ_BAT_OVER_TEMP_QUIT_CHG |
        AXP517Irq::IRQ_BAT_OVER_TEMP_CHG |
        AXP517Irq::IRQ_BAT_UNDER_TEMP_CHG |
        AXP517Irq::IRQ_BAT_OVER_TEMP_WORK |
        AXP517Irq::IRQ_BAT_UNDER_TEMP_WORK;

    if (!pmic.irq().enable(mask)) {
        Serial.println("Failed to enable AXP517 IRQ mask.");
    }
    if (!pmic.irq().clearStatus()) {
        Serial.println("Failed to clear AXP517 IRQ status.");
    }

    pinMode(PMIC_IRQ, INPUT_PULLUP);
    attachInterrupt(PMIC_IRQ, onPmicIrq, FALLING);

    Serial.print("SDA: ");
    Serial.println(PMIC_SDA);
    Serial.print("SCL: ");
    Serial.println(PMIC_SCL);
    Serial.print("PMIC_IRQ: ");
    Serial.println(PMIC_IRQ);
    lastSnapshot = readStatusSnapshot();
    Serial.println("\nInitial status:");
    printSnapshot(lastSnapshot);
    Serial.println("\nWaiting for AXP517 interrupts...");
}

void loop()
{
    static uint32_t lastPendingPrint = 0;

    if (irqTriggered || digitalRead(PMIC_IRQ) == LOW) {
        if (irqTriggered) {
            Serial.println("\nIRQ Triggered");
        } else if (millis() - lastPendingPrint > IRQ_PENDING_PRINT_MS) {
            lastPendingPrint = millis();
            Serial.println("\nIRQ Pending");
        }

        irqTriggered = false;
        drainIrq();
    }

    delay(20);
}

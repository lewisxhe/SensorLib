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
 * @file      axp517_pd_voltage.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-07-25
 */

#include <Arduino.h>
#include <Wire.h>
#include <PmicDrv.hpp>

#ifndef PMIC_SDA
#define PMIC_SDA  1
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
bool pdReady = false;

bool configurePdIrqOutput()
{
    auto &gpio = pmic.gpio();

    if (!PmicGpioBase::isOk(gpio.setDrive(0, PmicGpioBase::DriveType::OpenDrain))) {
        return false;
    }

    if (!PmicGpioBase::isOk(gpio.setDirection(0, PmicGpioBase::Direction::Output))) {
        return false;
    }

    return gpio.setOutputSource(0, AXP517Gpio::OutputSource::PdIrq);
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

void printVbus()
{
    uint16_t vbusMv = 0;

    if (pmic.tcpc().readVbusMv(vbusMv)) {
        Serial.print("TCPC VBUS: ");
        Serial.print(vbusMv);
        Serial.println(" mV");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== AXP517 PD Voltage Example ===\n");
    Serial.print("Target voltage: ");
    Serial.print(PD_TARGET_MV);
    Serial.println(" mV");

    if (!pmic.begin(Wire, AXP517_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL, PMIC_IRQ)) {
        Serial.println("Failed to init AXP517 or PMIC_IRQ pin!");
        while (1) delay(1000);
    }

    if (!configurePdIrqOutput()) {
        Serial.println("Failed to route AXP517 GPIO0 to PD_IRQ!");
        while (1) delay(1000);
    }

    Serial.print("PMIC_IRQ pin: ");
    Serial.println(PMIC_IRQ);
    Serial.println("Connect a USB-C PD source now.");

    pdReady = pmic.requestPd(PD_TARGET_MV, PD_TIMEOUT_MS, &sourceCaps);
    printSourceCaps(sourceCaps);

    Serial.println("\n--- PD Result ---");
    Serial.println(pdReady ? "Negotiation success." : "Negotiation failed.");
    printVbus();

    if (!pdReady) {
        Serial.println("Check PD source capability, CC wiring, and PMIC_IRQ wiring.");
    }
}

void loop()
{
    static uint32_t lastPrint = 0;

    if (millis() - lastPrint > 2000) {
        lastPrint = millis();

        Serial.println("\n--- Status ---");
        Serial.print("PD: ");
        Serial.println(pdReady ? "Ready" : "Not ready");
        printVbus();
    }

    delay(100);
}

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
 * @file      AXP517_Bc12.ino
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

const char *portTypeName(PmicBc12Base::PortType type)
{
    switch (type) {
    case PmicBc12Base::PortType::SDP: return "SDP (Standard Downstream Port)";
    case PmicBc12Base::PortType::CDP: return "CDP (Charging Downstream Port)";
    case PmicBc12Base::PortType::DCP: return "DCP (Dedicated Charging Port)";
    case PmicBc12Base::PortType::Apple_1A: return "Apple 1A";
    case PmicBc12Base::PortType::Apple_2A: return "Apple 2A";
    case PmicBc12Base::PortType::Apple_2_4A: return "Apple 2.4A";
    case PmicBc12Base::PortType::None: return "None";
    case PmicBc12Base::PortType::Unknown: return "Unknown";
    default: return "Invalid";
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n=== AXP517 BC1.2 Example ===\n");

    if (!pmic.begin(Wire, AXP517_SLAVE_ADDRESS, PMIC_SDA, PMIC_SCL)) {
        Serial.println("Failed to init AXP517!");
        while (1) delay(1000);
    }

    pmic.enableModule(PmicAXP517::Module::BC12, true);

    // Enable auto detection
    pmic.bc12().enableAutoDetect(true);
    Serial.println("BC1.2 Auto Detect: Enabled");

    Serial.println("\nConnect USB device to test...\n");
}

void loop()
{
    static uint32_t lastCheck = 0;
    static PmicBc12Base::PortType lastType = PmicBc12Base::PortType::Unknown;

    if (millis() - lastCheck > 500) {
        lastCheck = millis();

        auto result = pmic.bc12().readResult();

        if (result.type != lastType) {
            Serial.println("\n=== BC1.2 Result ===");
            Serial.print("Status: ");
            Serial.println(result.detecting ? "Detected" : "Not detected");
            Serial.print("Port Type: ");
            Serial.println(portTypeName(result.type));
            Serial.print("Raw Code: 0x");
            Serial.println(result.raw, HEX);
            lastType = result.type;
        }
    }

    delay(100);
}
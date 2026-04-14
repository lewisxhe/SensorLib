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
 * @file      IoExpanderPCA9570.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-03
 *
 */
#include <Arduino.h>
#include "IoExpanderPCA9570.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

IoExpanderPCA9570 expander;

void setup()
{
    Serial.begin(115200);

    if (!expander.begin(Wire, PCA9570_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        while (1) {
            Serial.println("Failed to find PCA9570 - check your wiring!");
            delay(1000);
        }
    }

    // Since PCA9570 pins are fixed as outputs, this call merely updates the software pin mode
    // cache and does not perform any actual hardware configuration.
    expander.configPins(IoExpanderPCA9570::PORT_ALL, OUTPUT);
}

void loop()
{
    // Set all pins to 1
    Serial.println("Set port HIGH");
    expander.digitalWritePort(0x0F, HIGH);
    delay(1000);

    Serial.println("Set port LOW");
    // Set all pins to 0
    expander.digitalWritePort(0x00, LOW);
    delay(1000);

    Serial.println("digitalWrite 0");
    expander.digitalWrite(0, HIGH);
    delay(1000);

    Serial.println("digitalToggle 0");
    expander.digitalToggle(0);
    delay(1000);
}

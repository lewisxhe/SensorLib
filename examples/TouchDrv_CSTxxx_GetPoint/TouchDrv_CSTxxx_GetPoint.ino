/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
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
 * @file      TouchDrv_CSTxxx_GetPoint.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-24
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "TouchDrvCSTXXX.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  8
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  10
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  5
#endif

#ifndef SENSOR_RST
#define SENSOR_RST  -1
#endif

TouchDrvCSTXXX touch;
int16_t x[5], y[5];

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    touch.setPins(SENSOR_RST, SENSOR_IRQ);
    touch.init(Wire, SENSOR_SDA, SENSOR_SCL, CSTXXX_SLAVE_ADDRESS);
    Serial.println("CSTxxx is not a standard I2C device, and it is impossible to read whether the device is online through any register, please ensure that the device is connected to the host");
}

void loop()
{
    uint8_t point = touch.getPoint(x, y, 5);
    if (point) {
        Serial.print("Point:");
        Serial.print(point);
        Serial.print(" ");
        uint8_t touched = touch.getPoint(x, y, 2);
        for (int i = 0; i < touched; ++i) {
            Serial.print("X[");
            Serial.print(i);
            Serial.print("]:");
            Serial.print(x[i]);
            Serial.print(" ");
            Serial.print(" Y[");
            Serial.print(i);
            Serial.print("]:");
            Serial.print(y[i]);
            Serial.print(" ");
        }
        Serial.println();
    }
    delay(5);
}




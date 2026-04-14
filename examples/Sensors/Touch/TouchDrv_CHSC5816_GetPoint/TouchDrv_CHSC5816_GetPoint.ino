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
 * @file      TouchDrv_CHSC5816_GetPoint.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-17
 *
 */
#include <TouchDrv.hpp>

#ifndef TOUCH_SDA
#define TOUCH_SDA  1
#endif

#ifndef TOUCH_SCL
#define TOUCH_SCL  2
#endif

#ifndef TOUCH_IRQ
#define TOUCH_IRQ  13
#endif

#ifndef TOUCH_RST
#define TOUCH_RST  14
#endif

TouchDrvCHSC5816 touch;

#ifdef ARDUINO_T_AMOLED_147
#include <XPowersAXP2101.tpp>   //PMU Library https://github.com/lewisxhe/XPowersLib.git
XPowersAXP2101 power;
#endif

void beginPower()
{
    // T_AMOLED_147 The PMU voltage needs to be turned on to use the sensor
#if defined(ARDUINO_T_AMOLED_147)
    bool ret = power.begin(Wire, AXP2101_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL);
    if (!ret) {
        Serial.println("PMU NOT FOUND!\n");
    }
    power.setALDO1Voltage(1800); power.enableALDO1();
    power.setALDO3Voltage(3300); power.enableALDO3();
    power.setBLDO1Voltage(1800); power.enableBLDO1();
#endif
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    beginPower();

    touch.setPins(TOUCH_RST, TOUCH_IRQ);
    if (!touch.begin(Wire, CHSC5816_SLAVE_ADDRESS, TOUCH_SDA, TOUCH_SCL)) {
        Serial.println("Failed to find CHSC5816 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("Init CHSC5816 Touch device success!");

    Serial.println("Touch Info:");
    Serial.print("Model: "); Serial.println(touch.getModelName());
    Serial.print("ID: 0x"); Serial.println(touch.getChipID(), HEX);
    Serial.print("Max Touch Points: "); Serial.println(touch.getSupportTouchPoint());
    uint16_t resX = touch.getResolutionX();
    uint16_t resY = touch.getResolutionY();
    if (resX == 0 || resY == 0) {
        Serial.println("The touch driver not support get touch resolution,please use setResolution() to set touch resolution.");
        // touch.setResolution(480, 320);
    } else {
        Serial.print("Resolution: "); Serial.print(resX); Serial.print(" x "); Serial.println(resY);
    }
    delay(3000);
}


void loop()
{
    if (touch.isPressed()) {
        TouchPoints touch_points = touch.getTouchPoints();
        if (touch_points.hasPoints()) {
            for (int i = 0; i < touch_points.getPointCount(); ++i) {
                const TouchPoint &point = touch_points.getPoint(i);
                Serial.print("X[");
                Serial.print(i);
                Serial.print("]:");
                Serial.print(point.x);
                Serial.print(" ");
                Serial.print(" Y[");
                Serial.print(i);
                Serial.print("]:");
                Serial.print(point.y);
                Serial.print(" ");
            }
            Serial.println();
        }
    }
}

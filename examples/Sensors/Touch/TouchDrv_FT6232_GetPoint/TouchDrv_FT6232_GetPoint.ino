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
 * @file      TouchDrv_FT6232_GetPoint.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-02
 *
 */
#include <TouchDrv.hpp>

#ifndef TOUCH_SDA
#define TOUCH_SDA  23
#endif

#ifndef TOUCH_SCL
#define TOUCH_SCL  32
#endif

#ifndef TOUCH_IRQ
#define TOUCH_IRQ  38
#endif

TouchDrvFT6X36 touch;


#ifdef ARDUINO_T_WATCH
#include <XPowersAXP202.tpp>   //PMU Library https://github.com/lewisxhe/XPowersLib.git
XPowersAXP202 power;
#endif

void beginPower()
{
    // T_AMOLED_147 The PMU voltage needs to be turned on to use the sensor
#if defined(ARDUINO_T_WATCH)
    bool ret = power.begin(Wire1, AXP202_SLAVE_ADDRESS, 21, 22);
    if (!ret) {
        Serial.println("PMU NOT FOUND!\n");
    }
    power.disableLDO2();
    power.setLDO2Voltage(3300); power.enableLDO2();
#endif
}


void setup()
{
    Serial.begin(115200);
    while (!Serial);

    beginPower();

    pinMode(TOUCH_IRQ, INPUT);

    if (!touch.begin(Wire, FT6X36_SLAVE_ADDRESS, TOUCH_SDA, TOUCH_SCL)) {
        Serial.println("Failed to find FT6X36 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
    touch.interruptPolling();

    // Set swap xy coordinates
    // touch.setSwapXY(true);

    // Set max coordinates
    // touch.setMaxCoordinates(320, 320);

    // Set mirror xy coordinates
    // touch.setMirrorXY(true, true);

    Serial.println("Touch Info:");
    Serial.print("Model: "); Serial.println(touch.getModelName());
    Serial.print("ID: 0x"); Serial.println(touch.getChipID(), HEX);
    /*
    * The default maximum number of touch fingers is the number of touch
    * fingers supported by the chip, not the maximum number of touch fingers
    * supported by the current device. The actual maximum number of touch
    * fingers is determined by the touch chip firmware and hardware.
    * */
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
    if (digitalRead(TOUCH_IRQ) == LOW) {
        TouchPoints touch_points = touch.getTouchPoints();
        if (touch_points.hasPoints()) {
            for (int i = 0; i < touch_points.getPointCount(); ++i) {
                const TouchPoint &point = touch_points.getPoint(i);
                Serial.print("ID: ");
                Serial.print(point.id);
                Serial.print(" ");
                Serial.print("X: ");
                Serial.print(point.x);
                Serial.print(" ");
                Serial.print("Y: ");
                Serial.print(point.y);
                Serial.print(" ");
                Serial.print("Pressure: ");
                Serial.print(point.pressure);
                Serial.print(" ");
                Serial.print("Event: ");
                switch (point.event) {
                case 0: Serial.print("Put Down"); break;
                case 1: Serial.print("Put Up"); break;
                case 2: Serial.print("Contact"); break;
                default: Serial.print("Unknown"); break;
                }
                Serial.println();
            }
            Serial.println();
        }
    }
    delay(50);
}

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
 * @file      GT911_GetPoint.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-12
 *
 */
#include <TouchDrv.hpp>

#ifndef TOUCH_SDA
#define TOUCH_SDA  18
#endif

#ifndef TOUCH_SCL
#define TOUCH_SCL  8
#endif

#ifndef TOUCH_IRQ
#define TOUCH_IRQ  16
#endif

#ifndef TOUCH_RST
#define TOUCH_RST  -1
#endif

TouchDrvGT911 touch;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // If the reset pin and interrupt pin can be controlled by GPIO, the device address can be set arbitrarily
    // If the interrupt and reset pins are not connected, you can pass in the -1 parameter and the library will automatically determine the address.
    touch.setPins(TOUCH_RST, TOUCH_IRQ);
    if (!touch.begin(Wire, GT911_SLAVE_ADDRESS_UNKNOWN, TOUCH_SDA, TOUCH_SCL)) {
        while (1) {
            Serial.println("Failed to find GT911 - check your wiring!");
            delay(1000);
        }
    }

    Serial.println("Init GT911 Sensor success!");

    // Set the center button to trigger the callback , Only for specific devices, e.g LilyGo-EPD47 S3 GT911
    touch.setHomeButtonCallback([](void *user_data) {
        Serial.println("Home button pressed!");
    }, NULL);


    /*
    *   GT911 Interrupt mode ,It is not recommended to modify any touch settings
    *   Please do not modify the touch interrupt mode without a touch screen configuration file,
    *   otherwise the touch screen may become unusable.
    * * */
    // Low level when idle, converts to high level when touched
    // touch.setInterruptMode(HIGH_LEVEL_QUERY);

    // Keep low level when idle, and trigger on the falling edge after touching, trigger once at a frequency of 100HZ, and keep high level for 10ms
    // touch.setInterruptMode(RISING);

    // Keep high level when idle, and switch to low level when touched
    // touch.setInterruptMode(LOW_LEVEL_QUERY);

    // Maintains high level when idle, and is triggered by the falling edge after being touched. The frequency is 100HZ and is triggered once. Maintains 10ms in the low level interval
    // touch.setInterruptMode(FALLING);

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
    delay(100);
}

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
 * @file      TouchDrv_GT9895_GetPoint_LilyGoP4.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-05
 *
 */
#include <TouchDrv.hpp>
#include <IoExpanderXL9555.hpp>
#include <SensorWireHelper.h>

// Pin definitions for the LilyGo-P4
#define P4_TOUCH_SDA  7
#define P4_TOUCH_SCL  8
#define P4_IO_EXPANDER_IRQ 5
#define IO_EXPANDER_TOUCH_RST   (3 | 0x80)
#define IO_EXPANDER_TOUCH_IRQ   (4 | 0x80)

#ifdef ARDUINO_ESP32P4_DEV

TouchDrvGT9895 touch;
IoExpanderXL9555 expander;
volatile bool ioExpanderInterruptTriggered = false;

void TouchDrvDigitalWrite(uint8_t gpio, uint8_t level)
{
    if (gpio & 0x80) {
        Serial.print("Expander DigitalWrite: "); Serial.print(gpio & 0x7F); Serial.print(" Level: "); Serial.println(level);
        expander.digitalWrite(gpio & 0x7F, level);
    } else {
        Serial.print("GPIO DigitalWrite: "); Serial.print(gpio); Serial.print(" Level: "); Serial.println(level);
        digitalWrite(gpio, level);
    }
}

uint8_t TouchDrvDigitalRead(uint8_t gpio)
{
    if (gpio & 0x80) {
        Serial.print("Expander DigitalRead: "); Serial.print(gpio & 0x7F); Serial.println();
        return expander.digitalRead(gpio & 0x7F);
    } else {
        Serial.print("GPIO DigitalRead: "); Serial.print(gpio); Serial.println();
        return digitalRead(gpio);
    }
}

void TouchDrvPinMode(uint8_t gpio, uint8_t mode)
{
    if (gpio & 0x80) {
        Serial.print("Expander PinMode: "); Serial.print(gpio & 0x7F); Serial.print(" Mode: "); Serial.println(mode);
        expander.pinMode(gpio & 0x7F, mode);
    } else {
        Serial.print("GPIO PinMode: "); Serial.print(gpio); Serial.print(" Mode: "); Serial.println(mode);
        pinMode(gpio, mode);
    }
}

void setup()
{
    Serial.begin(115200);

    while (!Serial);

    Serial.println("Sketch only works with LilyGo-T-Display-P4 AMOLED(GT9895) Version");

    Wire.begin(P4_TOUCH_SDA, P4_TOUCH_SCL);

    // Scan I2C bus for devices
    SensorWireHelper::dumpDevices(Wire);

    // The LilyGo-P4 relies on an external I/O extender to control the power supply of peripheral devices.
    // First, initialize the external extender.
    if (!expander.begin(Wire, XL9555_SLAVE_ADDRESS0)) {
        while (1) {
            Serial.println("Failed to find io expander - check your wiring!");
            delay(1000);
        }
    }
    Serial.println("Sensor Expander Initialized");

    // LilyGo-Display-P4 touch interrupt pin and touch reset pin aux to expander , use gpio custom callback
    touch.setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);

    // Set touch interrupt and reset Pin
    touch.setPins(IO_EXPANDER_TOUCH_RST, IO_EXPANDER_TOUCH_IRQ);

    if (!touch.begin(Wire, GT9895_SLAVE_ADDRESS_L)) {
        while (1) {
            Serial.println("Failed to find touch device - check your wiring!");
            delay(1000);
        }
    }

    Serial.println("GT9895 Touch Initialized");

    Serial.print("Chip ID : 0x"); Serial.println(touch.getChipID(), HEX);

    // Sleep touch
    // touch.sleep();

    // int i = 10;
    // while (i--) {
    //     Serial.print("Wake up after ");
    //     Serial.print(i);
    //     Serial.println(" seconds");
    //     delay(1000);
    // }

    // Wakeup touch
    // touch.wakeup();

    // Set the original touch panel resolution, LilyGo-P4 chip cannot automatically obtain the resolution;
    // the original resolution must be manually set.
    touch.setResolution(1060, 2400);

    // Set the target resolution, and the scaling factor will be calculated automatically.
    touch.setTargetResolution(568, 1232);

    // Set the maximum coordinates ，used for mirroring touch coordinates
    // touch.setMaxCoordinates(568, 1232);

    // Set swap xy
    // touch.setSwapXY(true);

    // Set mirror xy
    // touch.setMirrorXY(true, true);
}

void loop()
{
    /*
    * Methods to increase the refresh rate:
    * 1. Set the touch interrupt to a low level when touching, rather than a falling edge.
    *    Some are falling edge triggered, with a trigger period of about 10ms, depending on the firmware
    *    This requires the touch screen manufacturer to provide a firmware update, which is currently unavailable.
    * 2. Use polling registers instead of interrupts, but this will consume CPU
    */

    // * TouchPoints is configured with a 5-point touch point buffer by default,
    // * so it can return touch data from a maximum of 5 points. Although the GT9895
    // * supports 10-point touch
    TouchPoints touch_points = touch.getTouchPoints();
    if (touch_points.hasPoints()) {
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
                Serial.println();
            }
            Serial.println();
        }
    }
    delay(50);
}

#else
void setup()
{
    Serial.begin(115200);
}
void loop()
{
    Serial.println("Sketch only works with LilyGo-T-Display-P4 AMOLED(GT9895) Version");
    delay(1000);
}
#endif

/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
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
 * @file      TouchDrvInterface_Example.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-20
 * @note      TouchDrvInterface_Example use LilyGo T-RGB,T-RGB has three types of screens, each of which uses different touch driver chips.
 *            The example demonstrates using the touch interface class and one sketch is suitable for three types of touch chips.
 */
#include <TouchDrv.hpp>
#include <IoExpanderXL9555.hpp>
#include <SensorWireHelper.h>

#ifndef TOUCH_SDA
#define TOUCH_SDA  8
#endif

#ifndef TOUCH_SCL
#define TOUCH_SCL  48
#endif

#ifndef TOUCH_IRQ
#define TOUCH_IRQ  1
#endif

#ifndef TOUCH_RST
#define TOUCH_RST  1
#endif

// Use the TouchDrvInterface base class for automatic discovery and use of multi-touch devices
TouchDrvInterface *touchDrv;
// T-RGB uses XL9555 as the reset control of the touch screen
IoExpanderXL9555 expander;

int16_t x[5], y[5];

// Expander gpio 1
uint8_t expander_io_tp_reset = 1;

void TouchDrvDigitalWrite(uint8_t gpio, uint8_t level)
{
    if (gpio & 0x80) {
        expander.digitalWrite(gpio & 0x7F, level);
    } else {
        digitalWrite(gpio, level);
    }
}

uint8_t TouchDrvDigitalRead(uint8_t gpio)
{
    if (gpio & 0x80) {
        return expander.digitalRead(gpio & 0x7F);
    } else {
        return digitalRead(gpio);
    }
}

void TouchDrvPinMode(uint8_t gpio, uint8_t mode)
{
    if (gpio & 0x80) {
        expander.pinMode(gpio & 0x7F, mode);
    } else {
        pinMode(gpio, mode);
    }
}

bool setupTouchDrv()
{
    // Add the highest bit to indicate that the GPIO expander is used, not the ESP's GPIO
    const uint8_t touch_reset_pin = expander_io_tp_reset | 0x80;
    const uint8_t touch_irq_pin = TOUCH_IRQ;
    bool result = false;

    touchDrv = new TouchDrvCSTXXX();
    touchDrv->setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);
    touchDrv->setPins(touch_reset_pin, touch_irq_pin);
    result = touchDrv->begin(Wire, CST816_SLAVE_ADDRESS, TOUCH_SDA, TOUCH_SCL);
    if (result) {
        const char *model = touchDrv->getModelName();
        Serial.print("Successfully initialized "); Serial.print(model);
        Serial.print(",using "); Serial.print(model); Serial.println(" Driver!");
        return true;
    }
    delete touchDrv;

    touchDrv = new TouchDrvGT911();
    touchDrv->setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);
    touchDrv->setPins(touch_reset_pin, touch_irq_pin);
    result = touchDrv->begin(Wire, GT911_SLAVE_ADDRESS_L, TOUCH_SDA, TOUCH_SCL);
    if (result) {
        const char *model = touchDrv->getModelName();
        Serial.print("Successfully initialized "); Serial.print(model);
        Serial.print(",using "); Serial.print(model); Serial.println(" Driver!");
        return true;
    }
    delete touchDrv;

    touchDrv = new TouchDrvFT6X36();
    touchDrv->setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);
    touchDrv->setPins(touch_reset_pin, touch_irq_pin);
    result = touchDrv->begin(Wire, FT3267_SLAVE_ADDRESS, TOUCH_SDA, TOUCH_SCL);
    if (result) {
        TouchDrvFT6X36 *tmp = static_cast<TouchDrvFT6X36 *>(touchDrv);
        tmp->interruptPolling();
        const char *model = touchDrv->getModelName();
        Serial.print("Successfully initialized "); Serial.print(model);
        Serial.print(",using "); Serial.print(model); Serial.println(" Driver!");
        return true;
    }
    delete touchDrv;

    Serial.println("Unable to find touch device.");

    touchDrv = NULL;

    return false;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Start!");

#if (defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_STM32)) && !defined(ARDUINO_ARCH_MBED)
    Wire.setSCL(TOUCH_SCL);
    Wire.setSDA(TOUCH_SDA);
    Wire.begin();
#elif defined(ARDUINO_ARCH_NRF52)
    Wire.setPins(TOUCH_SDA, TOUCH_SCL);
    Wire.begin();
#elif defined(ARDUINO_ARCH_ESP32)
    Wire.begin(TOUCH_SDA, TOUCH_SCL);
#else
    Wire.begin();
#endif

    SensorWireHelper::dumpDevices(Wire);

    // Use unspecified address to test the address discovery function.
    // T-RGB XL9555 uses 0X20 device address
    if (!expander.begin(Wire, XL9555_UNKNOWN_ADDRESS, TOUCH_SDA, TOUCH_SCL)) {
        Serial.println("Failed to find XL9555 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    if (!setupTouchDrv()) {
        while (1) {
            delay(3000);
        }
    }

}

void loop()
{
    TouchPoints touch_points = touchDrv->getTouchPoints();
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
    delay(5);
}





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
 * @file      TouchDrvCSTXXX.tpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-24
 *
 */


#include "REG/CSTxxxConstants.h"
#include "TouchDrvInterface.hpp"
#include "SensorCommon.tpp"


class TouchDrvCSTXXX :
    public TouchDrvInterface,
    public SensorCommon<TouchDrvCSTXXX>
{
    friend class SensorCommon<TouchDrvCSTXXX>;
public:


#if defined(ARDUINO)
    TouchDrvCSTXXX(TwoWire &w,
                   int sda = DEFAULT_SDA,
                   int scl = DEFAULT_SCL,
                   uint8_t addr = CSTXXX_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __rst = SENSOR_PIN_NONE;
        __irq = SENSOR_PIN_NONE;
        __addr = addr;
    }
#endif

    TouchDrvCSTXXX()
    {
#if defined(ARDUINO)
        __wire = &Wire;
        __sda = DEFAULT_SDA;
        __scl = DEFAULT_SCL;
#endif
        __rst = SENSOR_PIN_NONE;
        __irq = SENSOR_PIN_NONE;
        __addr = CSTXXX_SLAVE_ADDRESS;
    }

    ~TouchDrvCSTXXX()
    {
        // deinit();
    }

#if defined(ARDUINO)
    bool init(TwoWire &w,
              int sda = DEFAULT_SDA,
              int scl = DEFAULT_SCL,
              uint8_t addr = CSTXXX_SLAVE_ADDRESS)
    {
        __wire = &w;
        __sda = sda;
        __scl = scl;
        __addr = addr;
        return begin();
    }
#endif

    bool init(int rst, int irq)
    {
        __rst = rst;
        __irq = irq;
        return initImpl();
    }

    void setPins(int rst, int irq)
    {
        __irq = irq;
        __rst = rst;
    }

    void reset()
    {
        if (__rst != SENSOR_PIN_NONE) {
            digitalWrite(__rst, LOW);
            delay(3);
            digitalWrite(__rst, HIGH);
            delay(5);
        }
    }

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point = 1)
    {
        uint8_t buffer[13];
        if (readRegister(CSTXXX_REG_STATUS, buffer, 13) == DEV_WIRE_ERR) {
            return 0;
        }
        if (!buffer[2] || !x_array || !y_array || !get_point) {
            return 0;
        }

        // buffer[3] >> 6;
        x_array[0] = ((buffer[((uint8_t)0x03)] & 0x0F) << 8 | buffer[((uint8_t)0x04)]);
        y_array[0] = ((buffer[((uint8_t)0x05)] & 0x0F) << 8 | buffer[((uint8_t)0x06)]);

        if (get_point == 2) {
            // buffer[9] >> 6;
            x_array[1] =  ((buffer[((uint8_t)0x09)] & 0x0F) << 8 | buffer[((uint8_t)0x10)]);
            y_array[1] =  ((buffer[((uint8_t)0x11)] & 0x0F) << 8 | buffer[((uint8_t)0x12)]);
        }
        return buffer[2] & 0x0F;
    }

    bool isPressed()
    {
        if (__irq != SENSOR_PIN_NONE) {
            return digitalRead(__irq) == LOW;
        }
        return getPoint(NULL, NULL);
    }

    bool enableInterrupt()
    {
        return false;
    }

    bool disableInterrupt()
    {
        return false;
    }

    uint8_t getChipID()
    {
        return false;
    }

    const char *getModelName()
    {
        return "CSTXXX";
    }

    //2uA
    void sleep()
    {
    }

    void wakeup()
    {
    }

    void idle()
    {

    }

    bool writeConfig(uint8_t *data, uint32_t size)
    {
        return false;
    }

    uint8_t getSupportTouchPoint()
    {
        return 1;
    }

    bool getResolution(int16_t *x, int16_t *y)
    {
        return true;
    }

    uint8_t getGesture()
    {
        return 0;
    }

private:
    bool initImpl()
    {
        setReadRegisterSendStop(false);
        setRegAddressLenght(1);

        if (__irq != SENSOR_PIN_NONE) {
            pinMode(__irq, INPUT);
        }

        if (__rst != SENSOR_PIN_NONE) {
            pinMode(__rst, OUTPUT);
        }

        reset();

        // CSTxxx is not a standard I2C device,
        // and it is impossible to read whether the device is online through any register,
        // please ensure that the device is connected to the host
        return true;
    }

    int getReadMaskImpl()
    {
        return -1;
    }


protected:
    int __rst = SENSOR_PIN_NONE;
    int __irq = SENSOR_PIN_NONE;
};




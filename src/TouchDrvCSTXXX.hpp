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
 * @note      230805: Test CST226SE
 *            230904: Test CST816T
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
        switch (__model) {
        case CST816T_MODEL_ID:
            return __getCST816T(x_array, y_array, get_point);
        case CST226SE_MODEL_ID:
            return __getCST226SE(x_array, y_array, get_point);
        default:
            return 0;
        }
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
        switch (__model) {
        case CST816T_MODEL_ID:
            return "CST816T";
        case CST226SE_MODEL_ID:
            return "CST226SE";
        default:
            return "UNKONW";
        }
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
        return false;
    }

    uint8_t getGesture()
    {
        return 0;
    }

    uint8_t getMaxDetectedPoint()
    {
        return __maxPoint;
    }


    void setHomeButtonCallback(home_button_callback_t cb, void *user_data = NULL)
    {
        this->__homeButtonCb = cb;
        this->user_data = user_data;
    }

private:

    uint8_t __getCST226SE(int16_t *x_array, int16_t *y_array, uint8_t get_point)
    {
        uint8_t buffer[26];
        uint8_t index = 0;

        if (!x_array && !y_array || !get_point) {
            return 0;
        }

        if (readRegister(CSTXXX_REG_STATUS, buffer, 26) == DEV_WIRE_ERR) {
            return 0;
        }

#ifdef LOG_PORT
        LOG_PORT.print("RAW:");
        for (int i = 0; i < 26; ++i) {
            LOG_PORT.print(buffer[i], HEX); LOG_PORT.print(",");
        }
        LOG_PORT.println();
#endif

        if (buffer[0] == 0x83 && buffer[1] == 0x17 && buffer[5] == 0x80) {
            if (__homeButtonCb) {
                __homeButtonCb(this->user_data);
            }
            return 1;
        }

        if (buffer[6] != 0xAB)return 0;
        if (buffer[0] == 0xAB)return 0;
        if (buffer[5] == 0x80) {
            return 0;
        }
        uint8_t point = buffer[5] & 0x7F;
        if (point > 5  || !point) {
            writeRegister(0x00, 0xAB);
            return 0;
        }

        for (int i = 0; i < point; i++) {
            x_array[i] = (int16_t)((buffer[index + 1] << 4) | ((buffer[index + 3] >> 4) & 0x0F));
            y_array[i] = (int16_t)((buffer[index + 2] << 4) | (buffer[index + 3] & 0x0F));
            index = (i == 0) ?  (index + 2) :  (index + 5);
        }

#ifdef LOG_PORT
        for (int i = 0; i < point; i++) {
            LOG_PORT.printf("[%d] --> X:%d Y:%d \n", i, x_array[i], y_array[i]);
        }
#endif
        return 0;
    }


    uint8_t __getCST816T(int16_t *x_array, int16_t *y_array, uint8_t get_point)
    {
        uint8_t buffer[13];
        if (readRegister(CSTXXX_REG_STATUS, buffer, 13) == DEV_WIRE_ERR) {
            return 0;
        }

        if (!buffer[2] || !x_array || !y_array || !get_point) {
            return 0;
        }

        uint8_t point = buffer[2] & 0x0F;
#ifdef LOG_PORT
        LOG_PORT.print("RAW:");
        for (int i = 0; i < 13; ++i) {
            LOG_PORT.print(buffer[i], HEX); LOG_PORT.print(",");
        }
        LOG_PORT.println();
#endif

        x_array[0] = ((buffer[((uint8_t)0x03)] & 0x0F) << 8 | buffer[((uint8_t)0x04)]);
        y_array[0] = ((buffer[((uint8_t)0x05)] & 0x0F) << 8 | buffer[((uint8_t)0x06)]);

        if (get_point == 2) {
            x_array[1] =  ((buffer[((uint8_t)0x09)] & 0x0F) << 8 | buffer[((uint8_t)0x10)]);
            y_array[1] =  ((buffer[((uint8_t)0x11)] & 0x0F) << 8 | buffer[((uint8_t)0x12)]);
        }

#ifdef LOG_PORT
        for (int i = 0; i < point; i++) {
            LOG_PORT.printf("[%d] --> X:%d Y:%d \n", i, x_array[i], y_array[i]);
        }
#endif

        return point;

    }

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

        //Model ID Register : 0xA8
        //CST816T   : 0X22 , One point
        //CST226SE  : 0X08 , five point
        __model =  readRegister(CSTXXX_REG_MODEL_ID);

        switch (__model) {
        case CST816T_MODEL_ID:
            Serial.println("Find CST816T");
            __maxPoint = 1;
            break;
        case CST226SE_MODEL_ID:
            Serial.println("Find CST226SE");
            __maxPoint = 5;
            break;
        default:
            Serial.print("Find device ID :"); Serial.println(__model);
            __maxPoint = 1;
            break;
        }

        return probe();
    }

    int getReadMaskImpl()
    {
        return -1;
    }


protected:
    int __rst = SENSOR_PIN_NONE;
    int __irq = SENSOR_PIN_NONE;
    uint8_t __model = 0xFF;
    uint8_t __maxPoint = 1;
    home_button_callback_t __homeButtonCb = NULL;
    void *user_data = NULL;
};




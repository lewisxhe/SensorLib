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
                   uint8_t addr = CSTXXX_SLAVE_ADDRESS): __swapXY(false), __mirrorX(false), __mirrorY(false), __resX(0), __resY(0)
    {
        __wire = & w;
        __sda = sda;
        __scl = scl;
        __rst = SENSOR_PIN_NONE;
        __irq = SENSOR_PIN_NONE;
        __addr = addr;
    }
#endif

    TouchDrvCSTXXX(): __swapXY(false), __mirrorX(false), __mirrorY(false), __resX(0), __resY(0)
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
            delay(30);
            digitalWrite(__rst, HIGH);
            delay(50);
        }
    }

    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point = 1)
    {
        uint8_t point = 0;
        if (!x_array && !y_array || !get_point) {
            return 0;
        }
        switch (__chipID) {
        case CST816S_CHIP_ID:
        case CST816T_CHIP_ID:
        case CST716_CHIP_ID:
            return updateCST8xx(x_array, y_array, get_point);
        case CST226SE_CHIP_ID:
            point = updateCST226SE();
            if (point) {
                for (int i = 0; i < get_point; i++) {
                    x_array[i] =  report.x[i];
                    y_array[i] =  report.y[i];
                }
            }
            break;
        default:
            return 0;
        }
        return point;
    }

    const TouchData getPoint()
    {
        uint8_t point = 0;
        TouchData data;
        switch (__chipID) {
        case CST816S_CHIP_ID:
        case CST816T_CHIP_ID:
        case CST716_CHIP_ID:
            // return updateCST8xx(x_array, y_array, get_point);
            //TODO:
            return TouchData();
        case CST226SE_CHIP_ID:
            point = updateCST226SE();
            if (point) {
                return report;
            }
        default:
            break;
        }
    }

    bool isPressed()
    {
        static uint32_t lastPulse = 0;
        if (__irq != SENSOR_PIN_NONE) {
            int val = digitalRead(__irq) == LOW;
            if (val) {
                //Filter low levels with intervals greater than 1000ms
                val = (millis() - lastPulse > 1000) ?  false : true;
                lastPulse = millis();
                return val;
            }
            return false;
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
        return __chipID;
    }

    const char *getModelName()
    {
        switch (__chipID) {
        case CST816S_CHIP_ID:
            return "CST816S";
        case CST816T_CHIP_ID:
            return "CST816T";
        case CST716_CHIP_ID:
            return "CST716";
        case CST226SE_CHIP_ID:
            return "CST226SE";
        default:
            return "UNKONW";
        }
    }

    void disableAutoSleep()
    {
        switch (__chipID) {
        case CST816S_CHIP_ID:
        case CST816T_CHIP_ID:
            reset();
            delay(50);
            writeRegister(0xFE, 0x01);
            break;
        case CST716_CHIP_ID:
        case CST226SE_CHIP_ID:
        default:
            break;
        }
    }

    void sleep()
    {
        switch (__chipID) {
        case CST816S_CHIP_ID:
        case CST816T_CHIP_ID:
            writeRegister(0xE5, 0x03);
            break;
        case CST716_CHIP_ID:
            writeRegister(0xA5, 0x03);
            break;
        case CST226SE_CHIP_ID:
            writeRegister(0xD1, 0x05);
            break;
        default:
            break;
        }

#ifdef ESP32
        if (__irq != SENSOR_PIN_NONE) {
            pinMode(__irq, OPEN_DRAIN);
        }
        if (__rst != SENSOR_PIN_NONE) {
            pinMode(__rst, OPEN_DRAIN);
        }
#endif

    }

    void wakeup()
    {
        reset();
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
        return __maxPoint;
    }

    bool getResolution(int16_t *x, int16_t *y)
    {
        *x = __resX;
        *y = __resY;
        return true;
    }

    uint8_t getGesture()
    {
        return 0;
    }

    void setHomeButtonCallback(home_button_callback_t cb, void *user_data = NULL)
    {
        this->__homeButtonCb = cb;
        this->user_data = user_data;
    }


    void setSwapXY(bool swap)
    {
        __swapXY = swap;
    }

    void setMirrorXY(bool mirrorX, bool mirrorY)
    {
        __mirrorX = mirrorX;
        __mirrorY = mirrorY;
    }

    void setMaxCoordinates(uint16_t x, uint16_t y)
    {
        __xMax = x;
        __yMax = y;
    }


private:

    void updateXY(uint8_t pointNum, int16_t *xBuffer, int16_t *yBuffer)
    {
        for (int i = 0; i < pointNum; ++i) {
            if (__swapXY) {
                uint16_t tmp = xBuffer[i];
                xBuffer[i] = yBuffer[i];
                yBuffer[i] = tmp;
            }
            if (__mirrorX && __xMax ) {
                xBuffer[i] = __xMax - xBuffer[i];
            }
            if (__mirrorY && __yMax) {
                yBuffer[i] = __yMax - yBuffer[i];
            }
        }
    }

    uint8_t updateCST226SE()
    {
        uint8_t buffer[CST226SE_BUFFER_NUM];
        uint8_t index = 0;

        if (readRegister(CSTXXX_REG_STATUS, buffer, CST226SE_BUFFER_NUM) == DEV_WIRE_ERR) {
            return 0;
        }

#ifdef LOG_PORT
        LOG_PORT.print("RAW:");
        for (int i = 0; i < CST226SE_BUFFER_NUM; ++i) {
            LOG_PORT.printf("%02X,", buffer[i]);
        }
        LOG_PORT.println();
#endif

        if (buffer[0] == 0x83 && buffer[1] == 0x17 && buffer[5] == 0x80) {
            if (__homeButtonCb) {
                __homeButtonCb(this->user_data);
            }
            return 0;
        }

        if (buffer[6] != 0xAB)return 0;
        if (buffer[0] == 0xAB)return 0;
        if (buffer[5] == 0x80)return 0;

        uint8_t point = buffer[5] & 0x7F;
        if (point > 5  || !point) {
            writeRegister(0x00, 0xAB);
            return 0;
        }

        for (int i = 0; i < point; i++) {
            report.id[i] = buffer[index] >> 4;
            report.status[i] = buffer[index] & 0x0F;
            report.x[i] = (uint16_t)((buffer[index + 1] << 4) | ((buffer[index + 3] >> 4) & 0x0F));
            report.y[i] = (uint16_t)((buffer[index + 2] << 4) | (buffer[index + 3] & 0x0F));
            report.pressure[i] = buffer[index + 4];
            index = (i == 0) ?  (index + 7) :  (index + 5);
        }

        updateXY(point, report.x, report.y);

#ifdef LOG_PORT
        for (int i = 0; i < 5; i++) {
            LOG_PORT.printf("[%d]-X:%u Y:%u P:%u sta:%u ", report.id[i], report.x[i], report.y[i], report.pressure[i], report.status[i]);
        }
        LOG_PORT.println();
#endif

        return point;
    }


    uint8_t updateCST8xx(int16_t *x_array, int16_t *y_array, uint8_t get_point)
    {
        uint8_t buffer[13];
        if (readRegister(CSTXXX_REG_STATUS, buffer, 13) == DEV_WIRE_ERR) {
            return 0;
        }

        if (!buffer[2] || !x_array || !y_array || !get_point) {
            return 0;
        }

        // Only suitable for AMOLED 1.91 inch
        // RAW:00,00,01,80,78,02,58,00,00,FF,FF,FF,FF,
        if (buffer[2] == 0x01 && buffer[3] == 0x80 && buffer[4] == 0x78 && buffer[5] == 0x02 && buffer[6] == 0x58) {
            if (__homeButtonCb) {
                __homeButtonCb(this->user_data);
            }
            return 0;
        }

        uint8_t point = buffer[2] & 0x0F;
#ifdef LOG_PORT
        LOG_PORT.print("RAW:");
        for (int i = 0; i < 13; ++i) {
            LOG_PORT.printf("%02X,", buffer[i]);
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
        updateXY(point, x_array, y_array);

        return point;

    }

    // CST8xx
    bool initSelfCapacitiveTouch()
    {
        setRegAddressLenght(1);

        int chip_id =   readRegister(0xA7);
        log_i("Chip ID:0x%x", chip_id);

        int version =   readRegister(0xA9);
        log_i("Version :0x%x", version);

        if (chip_id != CST816S_CHIP_ID &&
                chip_id != CST816T_CHIP_ID  &&
                chip_id != CST716_CHIP_ID) {
            return false;
        }
        __chipID = chip_id;
        return true;
    }

    // CST226SE
    bool initMutualCapacitiveTouch()
    {
        setRegAddressLenght(2);

        uint8_t buffer[8];
        // Enter Command mode
        writeRegister(0xD1, 0x01);
        delay(10);
        readRegister(0xD1FC, buffer, 4);
        uint32_t checkcode = 0;
        checkcode = buffer[3];
        checkcode <<= 8;
        checkcode |= buffer[2];
        checkcode <<= 8;
        checkcode |= buffer[1];
        checkcode <<= 8;
        checkcode |= buffer[0];

        log_i("Chip checkcode:0x%x.\r\n", checkcode);

        readRegister(0xD1F8, buffer, 4);
        __resX = ( buffer[1] << 8) | buffer[0];
        __resY = ( buffer[3] << 8) | buffer[2];
        log_i("Chip resolution X:%u Y:%u\r\n", __resX, __resY);

        readRegister(0xD204, buffer, 4);
        uint32_t chipType = buffer[3];
        chipType <<= 8;
        chipType |= buffer[2];


        uint32_t ProjectID = buffer[1];
        ProjectID <<= 8;
        ProjectID |= buffer[0];

        log_i("Chip type :0x%X, ProjectID:0X%x\r\n",
              chipType, ProjectID);


        readRegister(0xD208, buffer, 8);

        uint32_t fwVersion = buffer[3];
        fwVersion <<= 8;
        fwVersion |= buffer[2];
        fwVersion <<= 8;
        fwVersion |= buffer[1];
        fwVersion <<= 8;
        fwVersion |= buffer[0];

        uint32_t checksum = buffer[7];
        checksum <<= 8;
        checksum |= buffer[6];
        checksum <<= 8;
        checksum |= buffer[5];
        checksum <<= 8;
        checksum |= buffer[4];

        log_i("Chip ic version:0x%X, checksum:0x%X",
              fwVersion, checksum);

        if (fwVersion == 0xA5A5A5A5) {
            log_i("Chip ic don't have firmware. ");
            return false;
        }
        if ((checkcode & 0xffff0000) != 0xCACA0000) {
            log_i("Firmware info read error .");
            return false;
        }

        __chipID = chipType;

        // Exit Command mode
        writeRegister(0xD1, 0x09);
        return true;
    }

    bool initImpl()
    {
        bool rslt = false;

        setReadRegisterSendStop(false);

        if (__irq != SENSOR_PIN_NONE) {
            pinMode(__irq, INPUT);
        }

        if (__rst != SENSOR_PIN_NONE) {
            pinMode(__rst, OUTPUT);
        }

        reset();

        rslt = initMutualCapacitiveTouch();

        if (!rslt) {
            rslt = initSelfCapacitiveTouch();
        }

        switch (__chipID) {
        case CST816S_CHIP_ID:
            log_i("Find CST816S");
            __maxPoint = 1;
            // disableAutoSleep();
            break;
        case CST816T_CHIP_ID:
            log_i("Find CST816T");
            __maxPoint = 1;
            // disableAutoSleep();
            break;
        case CST716_CHIP_ID:
            log_i("Find CST716");
            __maxPoint = 1;
            break;
        case CST226SE_CHIP_ID:
            log_i("Find CST226SE");
            __maxPoint = 5;
            break;
        default: return false;
        }

        setRegAddressLenght(1);

        return probe();
    }

    int getReadMaskImpl()
    {
        return -1;
    }


protected:
    int __rst = SENSOR_PIN_NONE;
    int __irq = SENSOR_PIN_NONE;
    uint8_t __maxPoint = 1;
    uint8_t __chipID = 0xFF;
    home_button_callback_t __homeButtonCb = NULL;
    void *user_data = NULL;
    TouchData report;
    uint16_t __resX, __resY, __xMax, __yMax;
    bool __swapXY, __mirrorX, __mirrorY;
};




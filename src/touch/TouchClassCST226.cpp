/**
 *
 * @license MIT License
 *
 * Copyright (c) 2023 lewis he
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
 * @file      TouchClassCST226.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-06
 */
#include "TouchClassCST226.h"

#define CST2xx_REG_STATUS           (0x00)
#define CST226SE_BUFFER_NUM         (28)

TouchClassCST226::TouchClassCST226(PLATFORM_WIRE_TYPE &wire, int sda, int scl, uint8_t address)
{
    __wire = &wire;
    __sda = sda;
    __scl = scl;
    __addr = address;
}

bool TouchClassCST226::init()
{
    return begin();
}

void TouchClassCST226::reset()
{
    if (__rst != SENSOR_PIN_NONE) {
        digitalWrite(__rst, LOW);
        delay(30);
        digitalWrite(__rst, HIGH);
        delay(50);
    }
}

uint8_t TouchClassCST226::getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point)
{
    uint8_t buffer[CST226SE_BUFFER_NUM];
    uint8_t index = 0;

    if (readRegister(CST2xx_REG_STATUS, buffer, CST226SE_BUFFER_NUM) == DEV_WIRE_ERR) {
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
            __homeButtonCb(__userData);
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
    for (int i = 0; i < point; i++) {
        LOG_PORT.printf("[%d]-X:%u Y:%u P:%u sta:%u ", report.id[i], report.x[i], report.y[i], report.pressure[i], report.status[i]);
    }
    LOG_PORT.println();
#endif

    if (point) {
        for (int i = 0; i < get_point; i++) {
            x_array[i] =  report.x[i];
            y_array[i] =  report.y[i];
        }
    }

    return point;
}

bool TouchClassCST226::isPressed()
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
    return getPoint(NULL, NULL, 1);
}


const char *TouchClassCST226::getModelName()
{
    return "CST226SE";
}

void TouchClassCST226::sleep()
{
    writeRegister(0xD1, 0x05);
#ifdef ESP32
    if (__irq != SENSOR_PIN_NONE) {
        pinMode(__irq, OPEN_DRAIN);
    }
    if (__rst != SENSOR_PIN_NONE) {
        pinMode(__rst, OPEN_DRAIN);
    }
#endif
}

void TouchClassCST226::wakeup()
{
    reset();
}

void TouchClassCST226::idle()
{

}

uint8_t TouchClassCST226::getSupportTouchPoint()
{
    return 5;
}

bool TouchClassCST226::getResolution(int16_t *x, int16_t *y)
{
    return false;
}

void TouchClassCST226::setHomeButtonCallback(home_button_callback_t cb, void *user_data)
{
    __homeButtonCb = cb;
    __userData = user_data;
}

bool TouchClassCST226::initImpl()
{
    setRegAddressLenght(2);

    if (__rst != SENSOR_PIN_NONE) {
        pinMode(__rst, OUTPUT);
    }

    reset();

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

    setRegAddressLenght(1);

    return true;
}

int TouchClassCST226::getReadMaskImpl()
{
    return -1;
}
















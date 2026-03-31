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
 * @file      TouchDrvCST226.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-06
 */
#include "TouchDrvCST226.h"

void TouchDrvCST226::reset()
{
    if (_pinsCfg.rstPin != -1) {
        hal->pinMode(_pinsCfg.rstPin, OUTPUT);
        hal->digitalWrite(_pinsCfg.rstPin, LOW);
        hal->delay(100);
        hal->digitalWrite(_pinsCfg.rstPin, HIGH);
        hal->delay(100);
    } else {
        writeReg(0xD1, 0x0E);
        hal->delay(20);
    }
}

const TouchPoints &TouchDrvCST226::getTouchPoints()
{
    static constexpr uint8_t POINT_BUFFER_SIZE = 28;
    static constexpr uint8_t STATUS_REG   = 0x00;
    uint8_t buffer[POINT_BUFFER_SIZE] = {0};
    uint8_t index = 0;

    // Clear cached touch points
    _touchPoints.clear();

    if (readRegBuff(STATUS_REG, buffer, POINT_BUFFER_SIZE) == 0) {

        if (buffer[0] == 0x83 && buffer[1] == 0x17 && buffer[5] == 0x80) {
            if (_HButtonCallback) {
                _HButtonCallback(_userData);
            }
            return _touchPoints;
        }

        if (buffer[6] != 0xAB)return _touchPoints; // Return zero points
        if (buffer[0] == 0xAB)return _touchPoints; // Return zero points
        if (buffer[5] == 0x80)return _touchPoints; // Return zero points

        uint8_t numPoints = buffer[5] & 0x7F; // Get number of touch points (lower 7 bits)

        // Validate number of touch points
        if (numPoints == 0 || numPoints > MAX_FINGER_NUM) {
            writeReg(0x00, 0xAB);
            return _touchPoints; // Return zero points
        }

        // If not center button, add point
        for (int i = 0; i < numPoints; i++) {
            const uint16_t x = (buffer[index + 1] << 4) | ((buffer[index + 3] >> 4) & 0x0F);
            const uint16_t y = (buffer[index + 2] << 4) | (buffer[index + 3] & 0x0F);
            const uint16_t pressure = buffer[index + 4];
            const uint8_t id = (buffer[index] & 0xF0) >> 4;
            const uint8_t event = buffer[index] & 0x0F;
            _touchPoints.addPoint(x, y, pressure, id, event);
            index = (i == 0) ?  (index + 7) :  (index + 5);
        }
        // Swap XY or mirroring coordinates,if set
        updateXY(_touchPoints);
    }
    return _touchPoints;
}

const char *TouchDrvCST226::getModelName()
{
    switch (_chipID) {
    case CST226SE_CHIPTYPE:
        return "CST226SE";
    case CST328_CHIPTYPE:
        return "CST328";
    default:
        break;
    }
    return "Unknown";
}

void TouchDrvCST226::sleep()
{
    writeReg(0xD1, 0x05);
#ifdef ARDUINO_ARCH_ESP32
    if (_pinsCfg.irqPin != -1) {
        hal->pinMode(_pinsCfg.irqPin, OPEN_DRAIN);
    }
    if (_pinsCfg.rstPin != -1) {
        hal->pinMode(_pinsCfg.rstPin, OPEN_DRAIN);
    }
#endif
}

bool TouchDrvCST226::initImpl(uint8_t)
{
    uint8_t buffer[8];
    // Enter Command mode
    writeReg(0xD1, 0x01);
    hal->delay(10);
    uint8_t write_buffer[2] = {0xD1, 0xFC};
    writeThenRead(write_buffer, 2, buffer, 4);

    uint32_t checkcode = (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
    log_i("Chip checkcode:0x%lX.", checkcode);

    write_buffer[0] = 0xD1;
    write_buffer[1] = 0xF8;
    writeThenRead(write_buffer, 2, buffer, 4);
    _touchConfig.resolutionX = ( buffer[1] << 8) | buffer[0];
    _touchConfig.resolutionY = ( buffer[3] << 8) | buffer[2];
    log_i("Chip resolution X:%u Y:%u", _touchConfig.resolutionX, _touchConfig.resolutionY);

    write_buffer[0] = 0xD2;
    write_buffer[1] = 0x04;
    writeThenRead(write_buffer, 2, buffer, 4);
    // uint32_t chipType = buffer[3];
    // chipType <<= 8;
    // chipType |= buffer[2];
    uint32_t chipType = buffer[2];
    // log_i("Chip ID high byte:0x%lx. low byte:0x%lx", buffer[3], buffer[2]);

    uint32_t ProjectID = buffer[1];
    ProjectID <<= 8;
    ProjectID |= buffer[0];
    log_i("Chip type :0x%lx, ProjectID:0X%lx",
          chipType, ProjectID);


    write_buffer[0] = 0xD2;
    write_buffer[1] = 0x08;
    writeThenRead(write_buffer, 2, buffer, 8);

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

    log_i("Chip ic version:0x%lx, checksum:0x%lx",
          fwVersion, checksum);

    if (fwVersion == 0xA5A5A5A5) {
        log_e("Chip ic don't have firmware.");
        return false;
    }
    if ((checkcode & 0xffff0000) != 0xCACA0000) {
        log_e("Firmware info read error.");
        return false;
    }

    if (chipType != CST226SE_CHIPTYPE && chipType != CST328_CHIPTYPE) {
        log_e("Chip ID does not match, should be 0x%02" PRIX8 " ,but is 0x%02" PRIX32, CST226SE_CHIPTYPE, chipType);
        return false;
    }

    _chipID = chipType;

    // Exit Command mode
    writeReg(0xD1, 0x09);

    _maxTouchPoints = 5;

    return true;
}

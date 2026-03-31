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
 * @file      TouchDrvCHSC5816.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-05
 *
 */
#include "TouchDrvCHSC5816.hpp"

void TouchDrvCHSC5816::sleep()
{
    uint8_t CHSC5816_REG_SLEEP[] = {
        0x20, 0x00, 0x00, 0x00, // CHSC5816_REG_CMD_BUFF
        0xF8, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE9
    };
    writeBuff(CHSC5816_REG_SLEEP, arraySize(CHSC5816_REG_SLEEP));
}

const TouchPoints &TouchDrvCHSC5816::getTouchPoints()
{
    uint8_t buffer[POINT_BUFFER_SIZE] = {0};

    _touchPoints.clear(); // Clear cached touch points

    addrToBeBuf(REG_POINT_EVENT, buffer);
    if (writeThenRead(buffer, COMMAND_SIZE, buffer, POINT_BUFFER_SIZE) == 0) {

        // Get number of touch points
        uint8_t numPoints = buffer[1];

        if (numPoints > MAX_FINGER_NUM || numPoints == 0) {
            return _touchPoints;
        }
        const uint16_t x  = buffer[2] | ((buffer[5] & 0x0F) << 8);
        const uint16_t y  = buffer[3] | (((buffer[5] >> 4) & 0x0F) << 8);
        const uint8_t pressure = buffer[4];
        const uint8_t eventFlag = (buffer[6] & 0x0F);
        const uint8_t id = (buffer[6] >> 4) & 0x0F;
        _touchPoints.addPoint(x, y, pressure, id, eventFlag);

        // Swap XY or mirroring coordinates,if set
        updateXY(_touchPoints);
    }
    return _touchPoints;
}

const char *TouchDrvCHSC5816::getModelName()
{
    return "CHSC5816";
}

bool TouchDrvCHSC5816::initImpl(uint8_t)
{
    hal->delay(10);

    uint8_t buffer[16] = {0};

    addrToBeBuf(REG_BOOT_STATE, buffer);
    if (writeBuff(buffer, arraySize(buffer)) < 0) {
        log_e("writeBuff clean boot state failed!");
        return false;
    }

    hal->delay(10);

    bool success = false;
    int retry = 5;
    do {
        addrToBeBuf(REG_BOOT_STATE, buffer);
        if (writeThenRead(buffer, 4, buffer, 4) < 0) {
            return false;
        }

        hal->delay(20);

        // Check if the signature matches
        // When device is startup , reg will report 0x43 0x48 0x53 0x43
        if (buffer[0] == 0x43 && buffer[1] == 0x48 && buffer[2] == 0x53 && buffer[3] == 0x43) {
            log_d("Signature matched!");
            success = true;
            break;

        }
    } while (retry--);

    if (success) {
        _maxTouchPoints = 1;
        _chipID = 0x5816;   // Fixed chip ID
        return true;
    }

    return false;
}

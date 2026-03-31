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
 * @file      TouchDrvCST816.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-06
 */
#include "TouchDrvCST816.h"

const TouchPoints &TouchDrvCST816::getTouchPoints()
{
    static constexpr uint8_t POINT_BUFFER_SIZE = 13;
    uint8_t buffer[POINT_BUFFER_SIZE] = {0};
    uint16_t x = 0, y = 0;

    // Clear cached touch points
    _touchPoints.clear();

    if (readRegBuff(CST8xx_REG_STATUS, buffer, POINT_BUFFER_SIZE) == 0) {

        // Some CST816T will return all 0xFF after turning off automatic sleep.
        if (buffer[2] == 0x00 || buffer[2] == 0xFF) {
            return _touchPoints;
        }

        uint8_t numPoints = buffer[2] & 0x0F; // Get number of touch points (lower 4 bits)

        // CST816 only supports single touch
        if (numPoints > MAX_FINGER_NUM || numPoints == 0 ) {
            return _touchPoints;
        }

        x = ((buffer[3] & 0x0F) << 8 | buffer[4]);
        y = ((buffer[5] & 0x0F) << 8 | buffer[6]);

        // Depends on touch screen firmware
        if (x == _center_btn_x && y == _center_btn_y && _HButtonCallback) {
            _HButtonCallback(_userData);
            return _touchPoints; // Return zero points
        }
        // If not center button, add point
        _touchPoints.addPoint(x, y);
        // Swap XY or mirroring coordinates,if set
        updateXY(_touchPoints);
    }

    return _touchPoints;
}

const char *TouchDrvCST816::getModelName()
{
    switch (_chipID) {
    case CST816S_CHIP_ID:
        return "CST816S";
    case CST816T_CHIP_ID:
        return "CST816T";
    case CST716_CHIP_ID:
        return "CST716";
    case CST820_CHIP_ID:
        return "CST820";
    case CST816D_CHIP_ID:
        return "CST816D";
    default:
        break;
    }
    return "UNKNOWN";
}

void TouchDrvCST816::sleep()
{
    writeReg(CST8xx_REG_SLEEP, 0x03);
}

void TouchDrvCST816::disableAutoSleep()
{
    switch (_chipID) {
    case CST816S_CHIP_ID:
    case CST816T_CHIP_ID:
    case CST820_CHIP_ID:
    case CST816D_CHIP_ID:
        reset();
        hal->delay(50);
        writeReg(CST8xx_REG_DIS_AUTOSLEEP, 0x01);
        break;
    case CST716_CHIP_ID:
    default:
        break;
    }
}

void TouchDrvCST816::enableAutoSleep()
{
    switch (_chipID) {
    case CST816S_CHIP_ID:
    case CST816T_CHIP_ID:
    case CST820_CHIP_ID:
    case CST816D_CHIP_ID:
        reset();
        hal->delay(50);
        writeReg(CST8xx_REG_DIS_AUTOSLEEP, (uint8_t)0x00);
        break;
    case CST716_CHIP_ID:
    default:
        break;
    }
}

bool TouchDrvCST816::initImpl(uint8_t)
{
    int chip_id = readReg(CST8xx_REG_CHIP_ID);
    log_i("Chip ID:0x%x", chip_id);

    int version = readReg(CST8xx_REG_FW_VERSION);
    log_i("Version :0x%x", version);

    // CST716  : 0x20
    // CST816S : 0xB4
    // CST816T : 0xB5
    // CST816D : 0xB6
    // CST226SE : A7 = 0X20
    if (chip_id != CST816S_CHIP_ID &&
            chip_id != CST816T_CHIP_ID  &&
            chip_id != CST820_CHIP_ID &&
            chip_id != CST816D_CHIP_ID &&
            (chip_id != CST716_CHIP_ID || version == 0)) {
        log_e("Chip ID does not match, should be CST816S:0X%02X , CST816T:0X%02X , CST816D:0X%02X , CST820:0X%02X , CST716:0X%02X",
              CST816S_CHIP_ID, CST816T_CHIP_ID, CST816D_CHIP_ID, CST820_CHIP_ID, CST716_CHIP_ID);
        return false;
    }

    _chipID = chip_id;

    log_i("Touch type:%s", getModelName());

    _maxTouchPoints = 1;

    return true;
}

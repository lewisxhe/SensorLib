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
 * @file      TouchDrvFT6X36.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-05
 *
 */
#include "TouchDrvFT6X36.hpp"

void TouchDrvFT6X36::sleep()
{
    writeReg(FT6X36_REG_POWER_MODE, PMODE_DEEP_SLEEP);
}

const TouchPoints &TouchDrvFT6X36::getTouchPoints()
{
    static constexpr size_t POINT_BUFFER_SIZE = MAX_FINGER_NUM * BYTES_PER_POINT;
    uint8_t buffer[POINT_BUFFER_SIZE] = {0};
    _touchPoints.clear(); // Clear cached touch points
    int numPoints = readReg(FT6X36_REG_STATUS);
    if (numPoints < 0) {
        log_e("Failed to read touch points: %d", numPoints);
        return _touchPoints;
    }
    if (numPoints == 0 || numPoints > MAX_FINGER_NUM) {
        return _touchPoints;
    }
    uint8_t expectedBytes = numPoints * BYTES_PER_POINT; // 6 bytes per touch point

    // Read all touch points data in a single burst read for efficiency
    if (readRegBuff(FT6X36_REG_TOUCH1_XH, buffer, expectedBytes) == 0) {
        for (int i = 0; i < numPoints; i++) {
            const uint8_t eventFlag = (buffer[0 + i * BYTES_PER_POINT] & 0xC0) >> 6;
            const uint8_t touchId = (buffer[2 + i * BYTES_PER_POINT]  >> 4) & 0x0F;
            const uint16_t x = ((buffer[0 + i * BYTES_PER_POINT] & 0x0F) << 8) | buffer[1 + i * BYTES_PER_POINT];
            const uint16_t y = ((buffer[2 + i * BYTES_PER_POINT] & 0x0F) << 8) | buffer[3 + i * BYTES_PER_POINT];
            const uint8_t weight = buffer[4 + i * BYTES_PER_POINT];
            // Not used in FT6206, but valid in FT6236/FT6336U, representing the touch area size.
            // The value is typically 0-15, where higher values indicate a larger touch area,
            // which can be useful for distinguishing between a fingertip and a larger object like a
            // stylus or palm. In some applications, this information can be used to improve touch accuracy
            // or to implement features like palm rejection.
            // const uint8_t area = (buffer[5 + i * BYTES_PER_POINT] & 0xF0) >> 4;
            // Serial.printf("Point %d: ID=%d, Event=%d, X=%d, Y=%d, Weight=%d, Area=%d\n", i, touchId, eventFlag, x, y, weight, area);
            _touchPoints.addPoint(x, y, weight, touchId, eventFlag);
        }
        // Swap XY or mirroring coordinates, if set
        updateXY(_touchPoints);
    }
    return _touchPoints;
}

const char *TouchDrvFT6X36::getModelName()
{
    switch (_chipID) {
    case FT6206_CHIP_ID: return "FT6206";
    case FT6236_CHIP_ID: return "FT6236";
    case FT6336U_CHIP_ID: return "FT6336U";
    case FT3267_CHIP_ID: return "FT3267";
    case FT5336_CHIP_ID: return "FT5336";
    case FT3068_CHIP_ID: return "FT3068";
    default: return "UNKNOWN";
    }
}

uint8_t TouchDrvFT6X36::getDeviceMode(void)
{
    return readReg(FT6X36_REG_MODE) & 0x03;
}

uint8_t TouchDrvFT6X36::getGesture()
{
    int val = readReg(FT6X36_REG_GEST);
    switch (val) {
    case 0x10: return MOVE_UP;
    case 0x14: return MOVE_RIGHT;
    case 0x18: return MOVE_DOWN;
    case 0x1C: return MOVE_LEFT;
    case 0x48: return ZOOM_IN;
    case 0x49: return ZOOM_OUT;
    default: break;
    }
    return NO_GESTURE;
}

void TouchDrvFT6X36::setDeviceMode(uint8_t value)
{
    writeReg(FT6X36_REG_MODE, value);
}

uint8_t TouchDrvFT6X36::getThreshold(void)
{
    return readReg(FT6X36_REG_THRESHOLD);
}

void TouchDrvFT6X36::setThreshold(uint8_t value)
{
    writeReg(FT6X36_REG_THRESHOLD, value);
}

uint8_t TouchDrvFT6X36::getMonitorTime(void)
{
    return readReg(FT6X36_REG_MONITOR_TIME);
}

void TouchDrvFT6X36::setMonitorTime(uint8_t sec)
{
    writeReg(FT6X36_REG_MONITOR_TIME, sec);
}

uint16_t TouchDrvFT6X36::getLibraryVersion()
{
    uint8_t buffer[2] = {0};
    readRegBuff(FT6X36_REG_LIB_VERSION_H, buffer, 2);
    return (buffer[0] << 8) | buffer[1];
}

void TouchDrvFT6X36::interruptPolling(void)
{
    writeReg(FT6X36_REG_INT_STATUS, 0x00);
}

void TouchDrvFT6X36::interruptTrigger(void)
{
    writeReg(FT6X36_REG_INT_STATUS, 0x01);
}

void TouchDrvFT6X36::setPowerMode(PowerMode mode)
{
    writeReg(FT6X36_REG_POWER_MODE, mode);
}

uint8_t TouchDrvFT6X36::getVendorID(void)
{
    return readReg(FT6X36_REG_VENDOR1_ID);
}

uint8_t TouchDrvFT6X36::getErrorCode(void)
{
    return readReg(FT6X36_REG_ERROR_STATUS);
}

bool TouchDrvFT6X36::initImpl(uint8_t)
{
    uint8_t vendId = readReg(FT6X36_REG_VENDOR1_ID);

    switch (vendId) {
    case FT_VEND_ID1:
    case FT_VEND_ID2:
    case FT_VEND_ID3:
        break;
    default:
        log_e("Vendor id: 0x%X not match!", vendId);
        return false;
    }

    _chipID = readReg(FT6X36_REG_CHIP_ID);

    switch (_chipID) {
    case FT3267_CHIP_ID:
    case FT5336_CHIP_ID:
    case FT6206_CHIP_ID:
    case FT6236_CHIP_ID:
    case FT6336U_CHIP_ID:
    case FT3068_CHIP_ID:
        break;
    default:
        log_e("Chip ID: 0x%lx not match!", _chipID);
        return false;
    }

    log_i("Vend ID: 0x%X", vendId);
    log_i("Chip ID: 0x%lx", _chipID);
    log_i("Firmware Version: 0x%X", readReg(FT6X36_REG_FIRM_VERS));
    log_i("Report Rate Hz: %u", readReg(FT6X36_REG_PERIOD_ACTIVE));
    log_i("Threshold : %u", readReg(FT6X36_REG_THRESHOLD));
    log_i("Library version : 0x%x", getLibraryVersion());
    // This register describes period of monitor status, it should not less than 30.
    log_i("Chip period of monitor status : 0x%x", readReg(FT6X36_REG_PERIOD_MONITOR));
    _maxTouchPoints = MAX_FINGER_NUM;

    return true;
}


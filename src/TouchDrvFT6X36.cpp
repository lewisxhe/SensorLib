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

void TouchDrvFT6X36::reset()
{
    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(10);
        hal->digitalWrite(_rst, LOW);
        hal->delay(30);
        hal->digitalWrite(_rst, HIGH);
        // For the variant of GPIO extended RST,
        // communication and hal->delay are carried out simultaneously, and 160ms is measured in T-RGB esp-idf new api
        hal->delay(160);
    }
}

void TouchDrvFT6X36::sleep()
{
    comm->writeRegister(FT6X36_REG_POWER_MODE, PMODE_DEEP_SLEEP);
}

void TouchDrvFT6X36::TouchDrvFT6X36::wakeup()
{
    reset();
}

const TouchPoints &TouchDrvFT6X36::getTouchPoints()
{
    static TouchPoints points;
    uint8_t buffer[16];
    points.clear(); // Clear cached touch points
    if (comm->readRegister(FT6X36_REG_MODE, buffer, 16) == 0) {
        uint8_t numPoints = buffer[2] & 0x0F;   // Get number of touch points
        if (numPoints == 0 || numPoints > MAX_FINGER_NUM) {
            return points;
        }
        for (int i = 0; i < numPoints; i++) {
            points.addPoint(((buffer[3 + i * BYTES_PER_POINT] & 0x0F) << 8) | buffer[4 + i * BYTES_PER_POINT],
                            ((buffer[5 + i * BYTES_PER_POINT] & 0x0F) << 8) | buffer[6 + i * BYTES_PER_POINT],
                            (buffer[5 + i * BYTES_PER_POINT] >> 4));
        }
        // Swap XY or mirroring coordinates,if set
        updateXY(points);
    }
    return points;
}

bool TouchDrvFT6X36::isPressed()
{
    if (_irq != -1) {
        return hal->digitalRead(_irq) == LOW;
    }
    return comm->readRegister(FT6X36_REG_STATUS) & 0x0F;
}

const char *TouchDrvFT6X36::getModelName()
{
    switch (_chipID) {
    case FT6206_CHIP_ID: return "FT6206";
    case FT6236_CHIP_ID: return "FT6236";
    case FT6236U_CHIP_ID: return "FT6236U";
    case FT3267_CHIP_ID: return "FT3267";
    default: return "UNKNOWN";
    }
}

uint8_t TouchDrvFT6X36::getDeviceMode(void)
{
    return comm->readRegister(FT6X36_REG_MODE) & 0x03;
}

uint8_t TouchDrvFT6X36::getGesture()
{
    int val = comm->readRegister(FT6X36_REG_GEST);
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
    comm->writeRegister(FT6X36_REG_MODE, value);
}

uint8_t TouchDrvFT6X36::getThreshold(void)
{
    return comm->readRegister(FT6X36_REG_THRESHOLD);
}

uint8_t TouchDrvFT6X36::getMonitorTime(void)
{
    return comm->readRegister(FT6X36_REG_MONITOR_TIME);
}

void TouchDrvFT6X36::setMonitorTime(uint8_t sec)
{
    comm->writeRegister(FT6X36_REG_MONITOR_TIME, sec);
}

uint16_t TouchDrvFT6X36::getLibraryVersion()
{
    uint8_t buffer[2];
    comm->readRegister(FT6X36_REG_LIB_VERSION_H, buffer, 2);
    return (buffer[0] << 8) | buffer[1];
}

void TouchDrvFT6X36::interruptPolling(void)
{
    comm->writeRegister(FT6X36_REG_INT_STATUS, 0x00);
}

void TouchDrvFT6X36::interruptTrigger(void)
{
    comm->writeRegister(FT6X36_REG_INT_STATUS, 0x01);
}

void TouchDrvFT6X36::setPowerMode(PowerMode mode)
{
    comm->writeRegister(FT6X36_REG_POWER_MODE, mode);
}

uint8_t TouchDrvFT6X36::getVendorID(void)
{
    return comm->readRegister(FT6X36_REG_VENDOR1_ID);
}

uint8_t TouchDrvFT6X36::getErrorCode(void)
{
    return comm->readRegister(FT6X36_REG_ERROR_STATUS);
}

bool TouchDrvFT6X36::initImpl(uint8_t addr)
{
    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }

    reset();

    uint8_t vendId = comm->readRegister(FT6X36_REG_VENDOR1_ID);

    if (vendId != FT6X36_VEND_ID) {
        log_e("Vendor id is 0x%X not match!", vendId);
        return false;
    }

    _chipID = comm->readRegister(FT6X36_REG_CHIP_ID);

    if ((_chipID != FT6206_CHIP_ID) &&
            (_chipID != FT6236_CHIP_ID) &&
            (_chipID != FT6236U_CHIP_ID)  &&
            (_chipID != FT3267_CHIP_ID)) {
        log_e("Vendor id is not match!");
        log_e("ChipID:0x%lx should be 0x06 or 0x36 or 0x64", _chipID);
        return false;
    }

    log_i("Vend ID: 0x%X", vendId);
    log_i("Chip ID: 0x%lx", _chipID);
    log_i("Firm Version: 0x%X", comm->readRegister(FT6X36_REG_FIRM_VERS));
    log_i("Point Rate Hz: %u", comm->readRegister(FT6X36_REG_PERIOD_ACTIVE));
    log_i("Thresh : %u", comm->readRegister(FT6X36_REG_THRESHOLD));

    // change threshold to be higher/lower
    comm->writeRegister(FT6X36_REG_THRESHOLD, 60);

    log_i("Chip library version : 0x%x", getLibraryVersion());

    // This register describes period of monitor status, it should not less than 30.
    log_i("Chip period of monitor status : 0x%x", comm->readRegister(FT6X36_REG_PERIOD_MONITOR));

    // This register describes the period of active status, it should not less than 12

    _maxTouchPoints = 1;

    return true;
}


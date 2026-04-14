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
 * @file      SensorPawA350.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-11
 * @brief     PAW-A350 Optical Finger Navigation Sensor Implementation
 */

#include "SensorPawA350.hpp"
#include "SensorPawA350_Reg.hpp"

using namespace Paw350Regs;

const char *SensorPawA350::getChipName() const
{
    return "PAW-A350";
}

int SensorPawA350::getProductID() const
{
    return readReg(REG_PRODUCT_ID);
}

int SensorPawA350::getRevisionID() const
{
    return readReg(REG_REVISION_ID);
}

bool SensorPawA350::softReset()
{
    int ret = writeReg(REG_SOFT_RESET, SOFTRESET_VAL);
    if (ret != 0) return false;
    return getProductID() == PAW_A350_DEFAULT_PID;
}

bool SensorPawA350::checkMotion(MotionStatus &status)
{
    int val = readReg(REG_EVENT);
    if (val < 0) return false;
    status = (val & MASK_MOTION) ? MotionStatus::MOTION_DETECTED : MotionStatus::NO_MOTION;
    return true;
}

bool SensorPawA350::getMotionData(MotionData &data)
{
    uint8_t buffer[2] = {0};
    buffer[0] = readReg(REG_DELTA_X);
    buffer[1] = readReg(REG_DELTA_Y);
    data.delta_x = static_cast<int8_t>(buffer[0]);
    data.delta_y = static_cast<int8_t>(buffer[1]);
    writeReg(REG_EVENT, 0x00);
    return true;
}

bool SensorPawA350::setCpi(CpiResolution cpi)
{
    return writeReg(REG_CPI_SEL, static_cast<uint8_t>(cpi)) == 0;
}

SensorPawA350::CpiResolution SensorPawA350::getCpi() const
{
    int val = readReg(REG_CPI_SEL);
    if (val < 0) return CpiResolution::CPI_500;
    return static_cast<CpiResolution>(val & MASK_CPI_SEL);
}

bool SensorPawA350::setRunDownshiftTime(uint8_t rd)
{
    return writeReg(REG_RUN_DOWNSHIFT, rd) == 0;
}

int SensorPawA350::getRunDownshiftTime() const
{
    return readReg(REG_RUN_DOWNSHIFT);
}

bool SensorPawA350::setRest1Period(uint8_t period)
{
    // Note: Register 0x11 has a hardware read issue on some PAW-A350 variants
    // where reading back the written value returns 0. As a workaround, we use an
    // internal cache to track the setting. The register write is still attempted.
    bool rslt = writeReg(REG_REST1_PERIOD, period) == 0;
    if (rslt) {
        cached_rest1_period = period;
    }
    return rslt;
}

int SensorPawA350::getRest1Period() const
{
    // Returns cached value because register 0x11 readback returns 0 on some sensors.
    // Use setRest1Period() to update this cached value.
    return cached_rest1_period;
}

bool SensorPawA350::setRest1DownshiftTime(uint8_t r1d)
{
    return writeReg(REG_REST1_DOWNSHIFT, r1d) == 0;
}

int SensorPawA350::getRest1DownshiftTime() const
{
    return readReg(REG_REST1_DOWNSHIFT);
}

bool SensorPawA350::setRest2Period(uint8_t period)
{
    return writeReg(REG_REST2_PERIOD, period) == 0;
}

int SensorPawA350::getRest2Period() const
{
    return readReg(REG_REST2_PERIOD);
}

bool SensorPawA350::setRest2DownshiftTime(uint8_t r2d)
{
    return writeReg(REG_REST2_DOWNSHIFT, r2d) == 0;
}

int SensorPawA350::getRest2DownshiftTime() const
{
    return readReg(REG_REST2_DOWNSHIFT);
}

bool SensorPawA350::setRest3Period(uint8_t period)
{
    return writeReg(REG_REST3_PERIOD, period) == 0;
}

int SensorPawA350::getRest3Period() const
{
    return readReg(REG_REST3_PERIOD);
}

bool SensorPawA350::setLedOn(bool enable)
{
    int val = readReg(REG_LED_CTRL);
    if (val < 0) return false;
    return writeReg(REG_LED_CTRL, enable ? (val | MASK_LED_ON) : (val & ~MASK_LED_ON)) == 0;
}

bool SensorPawA350::getLedOn() const
{
    int val = readReg(REG_LED_CTRL);
    return (val >= 0) && (val & MASK_LED_ON);
}

bool SensorPawA350::setLedDriveCurrent(uint8_t current)
{
    int val = readReg(REG_LED_CTRL);
    if (val < 0) return false;
    return writeReg(REG_LED_CTRL, (val & ~MASK_LED_DRV) | (current & MASK_LED_DRV)) == 0;
}

int SensorPawA350::getLedDriveCurrent() const
{
    int val = readReg(REG_LED_CTRL);
    return (val >= 0) ? (val & MASK_LED_DRV) : -1;
}

bool SensorPawA350::setShutterMax(uint16_t value)
{
    int hi = writeReg(REG_SHUTTER_MAX_HI, (value >> 8) & 0xFF);
    int lo = writeReg(REG_SHUTTER_MAX_LO, value & 0xFF);
    return hi == 0 && lo == 0;
}

int SensorPawA350::getShutterMax() const
{
    int hi = readReg(REG_SHUTTER_MAX_HI);
    int lo = readReg(REG_SHUTTER_MAX_LO);
    if (hi < 0 || lo < 0) return -1;
    return (hi << 8) | lo;
}

bool SensorPawA350::setMotionInterruptThreshold(uint8_t threshold)
{
    int val = readReg(REG_MOTION_CTRL);
    if (val < 0) return false;
    return writeReg(REG_MOTION_CTRL, (val & ~MASK_MOTION_INT_THRES) | (threshold & MASK_MOTION_INT_THRES)) == 0;
}

int SensorPawA350::getMotionInterruptThreshold() const
{
    int val = readReg(REG_MOTION_CTRL);
    return (val >= 0) ? (val & MASK_MOTION_INT_THRES) : -1;
}

bool SensorPawA350::clearFingerState()
{
    int val = readReg(REG_FPD_FLAG);
    return val >= 0;
}

bool SensorPawA350::isMotionDetected()
{
    MotionStatus status;
    return checkMotion(status) && status == MotionStatus::MOTION_DETECTED;
}

uint32_t SensorPawA350::getRunDownshiftTimeMs() const
{
    int rd = getRunDownshiftTime();
    return (rd > 0) ? (rd * 8 * 8) : 0;
}

uint32_t SensorPawA350::getRest1PeriodMs() const
{
    int r1r = getRest1Period();
    return (r1r > 0) ? ((r1r + 1) * 10) : 0;
}

uint32_t SensorPawA350::getRest1DownshiftTimeMs() const
{
    int r1d = getRest1DownshiftTime();
    int period = getRest1Period();
    return (r1d > 0 && period > 0) ? (r1d * 16 * (period + 1) * 10) : 0;
}

uint32_t SensorPawA350::getRest2PeriodMs() const
{
    int r2r = getRest2Period();
    return (r2r > 0) ? ((r2r + 1) * 10) : 0;
}

uint32_t SensorPawA350::getRest2DownshiftTimeMs() const
{
    int r2d = getRest2DownshiftTime();
    int period = getRest2Period();
    return (r2d > 0 && period > 0) ? (r2d * 128 * (period + 1) * 10) : 0;
}

uint32_t SensorPawA350::getRest3PeriodMs() const
{
    int r3r = getRest3Period();
    return (r3r > 0) ? ((r3r + 1) * 10) : 0;
}

bool SensorPawA350::initImpl(uint8_t param)
{
    (void)param;
    int pid = getProductID();
    if (pid != PAW_A350_DEFAULT_PID) {
        return false;
    }
    cached_rest1_period = 1;
    return true;
}

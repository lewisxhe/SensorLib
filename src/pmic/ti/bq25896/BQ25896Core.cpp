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
 * @file      BQ25896Core.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 */
#include "BQ25896Core.hpp"
#include "BQ25896Regs.hpp"

using namespace BQ25896Regs;

void BQ25896Core::end() {}

bool BQ25896Core::isValid() const
{
    uint8_t rev = getDeviceRevision();
    return rev == BQ25896_DEV_REV;
}

uint8_t BQ25896Core::getDeviceRevision() const
{
    int val = readReg(REG_DEVICE_REV);
    return (val >= 0) ? (val & MASK_DEV_REV) : 0xFF;
}

uint8_t BQ25896Core::getDeviceConfig()
{
    int val = readReg(REG_DEVICE_REV);
    return (val >= 0) ? ((val >> SHIFT_PN) & 0x07) : 0xFF;
}

void BQ25896Core::reset()
{
    setRegBit(REG_DEVICE_REV, 7);
}

bool BQ25896Core::isVbusPresent()
{
    int val = readReg(REG_ADC_BUSV);
    return (val >= 0) && (val & MASK_VBUS_GD);
}

uint8_t BQ25896Core::getBusStatus()
{
    int val = readReg(REG_STATUS);
    return (val >= 0) ? ((val >> SHIFT_VBUS_STAT) & 0x07) : 0;
}

uint8_t BQ25896Core::getChargeStatus()
{
    int val = readReg(REG_STATUS);
    return (val >= 0) ? ((val >> SHIFT_CHRG_STAT) & 0x03) : 0xFF;
}

bool BQ25896Core::isPowerGood()
{
    int val = readReg(REG_STATUS);
    return (val >= 0) && (val & MASK_PG_STAT);
}

bool BQ25896Core::isCharging()
{
    return getChargeStatus() != CHARGE_STATE_NO_CHARGE;
}

bool BQ25896Core::isChargeDone()
{
    return getChargeStatus() == CHARGE_STATE_DONE;
}

bool BQ25896Core::isOtg()
{
    return getBusStatus() == BUS_STATE_OTG;
}

bool BQ25896Core::isVsysInRegulation()
{
    int val = readReg(REG_STATUS);
    return (val >= 0) && (val & MASK_VSYS_STAT);
}

bool BQ25896Core::isVindpmActive()
{
    int val = readReg(REG_IDPM_STATUS);
    return (val >= 0) && (val & MASK_VDPM_STAT);
}

bool BQ25896Core::isIindpmActive()
{
    int val = readReg(REG_IDPM_STATUS);
    return (val >= 0) && (val & MASK_IDPM_STAT);
}

bool BQ25896Core::getFaultStatus(uint8_t &fault)
{
    int val = readReg(REG_FAULT);
    if (val < 0) return false;
    fault = static_cast<uint8_t>(val);
    return true;
}

bool BQ25896Core::isInputCurrentOptimizerOptimized()
{
    int val = readReg(REG_DEVICE_REV);
    return (val >= 0) && (val & MASK_ICO_OPTIMIZED);
}

bool BQ25896Core::initImpl(uint8_t param)
{
    (void)param;
    uint8_t rev = getDeviceRevision();
    if (rev != BQ25896_DEV_REV) {
        log_e("Device revision mismatch: expected 0x%02X, got 0x%02X", BQ25896_DEV_REV, rev);
        return false;
    }

    // Perform a reset to ensure device is in known state
    reset();

    while (getRegBit(REG_DEVICE_REV, 7)) {
        // Wait for reset bit to clear, indicating reset complete
        for (volatile int i = 0; i < 100000; ++i); // Simple delay loop
    }

    // Disable watchdog timer by default for safety (can be enabled via API)
    return updateBits(REG_CHG_TIMER, MASK_WATCHDOG, 0) >= 0;
}

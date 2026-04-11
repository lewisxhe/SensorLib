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
 * @file      BQ25896Power.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 */
#include "BQ25896Power.hpp"
#include "BQ25896Regs.hpp"

using namespace BQ25896Regs;

BQ25896Power::BQ25896Power(BQ25896Core &core) : _core(core) {}

bool BQ25896Power::setMinimumSystemVoltage(uint32_t mv)
{
    uint32_t closest = SYS_VOFF_VOL_MIN + ((mv - SYS_VOFF_VOL_MIN + SYS_VOL_STEPS / 2) / SYS_VOL_STEPS) * SYS_VOL_STEPS;
    if (closest < SYS_VOFF_VOL_MIN) {
        closest = SYS_VOFF_VOL_MIN;
    }
    if (closest > SYS_VOFF_VOL_MAX) {
        closest = SYS_VOFF_VOL_MAX;
    }
    uint8_t val = ((closest - SYS_VOFF_VOL_MIN) / SYS_VOL_STEPS) << SHIFT_SYS_MIN;
    return _core.updateBits(REG_CHG_CTRL, MASK_SYS_MIN, val) >= 0;
}

uint32_t BQ25896Power::getMinimumSystemVoltage() const
{
    int val = _core.readReg(REG_CHG_CTRL);
    if (val < 0) return 0;
    val = (val >> SHIFT_SYS_MIN) & 0x07;
    return SYS_VOFF_VOL_MIN + (val * SYS_VOL_STEPS);
}

bool BQ25896Power::setInputVoltageLimit(uint32_t mv)
{
    uint32_t closest = VINDPM_VOL_BASE + ((mv - VINDPM_VOL_BASE + VINDPM_VOL_STEPS / 2) / VINDPM_VOL_STEPS) * VINDPM_VOL_STEPS;
    if (closest < VINDPM_VOL_MIN) {
        closest = VINDPM_VOL_MIN;
    }
    if (closest > VINDPM_VOL_MAX) {
        closest = VINDPM_VOL_MAX;
    }
    uint8_t val = ((closest - VINDPM_VOL_BASE) / VINDPM_VOL_STEPS) & MASK_VINDPM;
    return _core.updateBits(REG_VINDPM, MASK_VINDPM, val) >= 0;
}

uint32_t BQ25896Power::getInputVoltageLimit() const
{
    int val = _core.readReg(REG_VINDPM);
    if (val < 0) return 0;
    val = val & MASK_VINDPM;
    uint32_t result = VINDPM_VOL_BASE + (val * VINDPM_VOL_STEPS);
    return (result < VINDPM_VOL_MIN) ? VINDPM_VOL_MIN : result;
}

bool BQ25896Power::setInputCurrentLimit(uint32_t mA)
{
    uint32_t closest = IN_CURRENT_MIN_VAL + ((mA - IN_CURRENT_MIN_VAL + IN_CURRENT_STEP / 2) / IN_CURRENT_STEP) * IN_CURRENT_STEP;
    if (closest < IN_CURRENT_MIN_VAL) {
        closest = IN_CURRENT_MIN_VAL;
    }
    if (closest > IN_CURRENT_MAX_VAL) {
        closest = IN_CURRENT_MAX_VAL;
    }
    uint8_t val = ((closest - IN_CURRENT_MIN_VAL) / IN_CURRENT_STEP) & MASK_IINLIM;
    return _core.updateBits(REG_PWR_INPUT, MASK_IINLIM, val) >= 0;
}

uint32_t BQ25896Power::getInputCurrentLimit() const
{
    int val = _core.readReg(REG_PWR_INPUT);
    if (val < 0) return 0;
    return ((val & MASK_IINLIM) * IN_CURRENT_STEP) + IN_CURRENT_MIN_VAL;
}

bool BQ25896Power::enableBoost(bool enable)
{
    if (enable) {
        if (_core.isVbusPresent()) {
            return false;
        }
        return _core.updateBits(REG_CHG_CTRL, MASK_OTG_CONFIG, MASK_OTG_CONFIG) >= 0;
    } else {
        return _core.updateBits(REG_CHG_CTRL, MASK_OTG_CONFIG, 0) >= 0;
    }
}

bool BQ25896Power::isBoostEnabled() const
{
    int val = _core.readReg(REG_CHG_CTRL);
    return (val >= 0) && (val & MASK_OTG_CONFIG);
}

bool BQ25896Power::setBoostVoltage(uint16_t mv)
{
    uint16_t closest = BOOST_VOL_BASE + ((mv - BOOST_VOL_BASE + BOOST_VOL_STEP / 2) / BOOST_VOL_STEP) * BOOST_VOL_STEP;
    if (closest < BOOST_VOL_MIN_VAL) {
        closest = BOOST_VOL_MIN_VAL;
    }
    if (closest > BOOST_VOL_MAX_VAL) {
        closest = BOOST_VOL_MAX_VAL;
    }
    uint8_t val = (((closest - BOOST_VOL_BASE) / BOOST_VOL_STEP) << SHIFT_BOOSTV) & MASK_BOOSTV;
    return _core.updateBits(REG_BOOST_CTRL, MASK_BOOSTV, val) >= 0;
}

uint16_t BQ25896Power::getBoostVoltage() const
{
    int val = _core.readReg(REG_BOOST_CTRL);
    if (val < 0) return 0;
    val = (val >> SHIFT_BOOSTV) & 0x0F;
    return BOOST_VOL_BASE + (val * BOOST_VOL_STEP);
}

bool BQ25896Power::enableShipMode(bool enable)
{
    uint8_t val = enable ? MASK_BATFET_DIS : 0;
    return _core.updateBits(REG_MISC_CTRL, MASK_BATFET_DIS, val) >= 0;
}

bool BQ25896Power::isShipModeEnabled() const
{
    int val = _core.readReg(REG_MISC_CTRL);
    return (val >= 0) && (val & MASK_BATFET_DIS);
}

bool BQ25896Power::enableWatchdog(bool enable)
{
    if (enable) {
        return _core.updateBits(REG_CHG_TIMER, MASK_WATCHDOG, (1 << SHIFT_WATCHDOG)) >= 0;
    } else {
        return _core.updateBits(REG_CHG_TIMER, MASK_WATCHDOG, 0) >= 0;
    }
}

bool BQ25896Power::isWatchdogEnabled() const
{
    int val = _core.readReg(REG_CHG_TIMER);
    if (val < 0) return false;
    return ((val >> SHIFT_WATCHDOG) & 0x03) != 0;
}

bool BQ25896Power::setWatchdogTimeout(uint16_t timeout_s)
{
    uint8_t val = 0;
    if (timeout_s == 0) {
        val = 0;
    } else if (timeout_s <= 40) {
        val = 1 << SHIFT_WATCHDOG;
    } else if (timeout_s <= 80) {
        val = 2 << SHIFT_WATCHDOG;
    } else {
        val = 3 << SHIFT_WATCHDOG;
    }
    return _core.updateBits(REG_CHG_TIMER, MASK_WATCHDOG, val) >= 0;
}

uint16_t BQ25896Power::getWatchdogTimeout() const
{
    int val = _core.readReg(REG_CHG_TIMER);
    if (val < 0) return 0;
    uint8_t wd = (val >> SHIFT_WATCHDOG) & 0x03;
    if (wd == 0) return 0;
    return wd * 40;
}

bool BQ25896Power::resetWatchdog()
{
    return _core.updateBits(REG_CHG_CTRL, MASK_WD_RST, MASK_WD_RST) >= 0;
}

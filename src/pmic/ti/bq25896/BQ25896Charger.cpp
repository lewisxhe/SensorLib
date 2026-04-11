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
 * @file      BQ25896Charger.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 */
#include "BQ25896Charger.hpp"
#include "BQ25896Regs.hpp"

using namespace BQ25896Regs;

BQ25896Charger::BQ25896Charger(BQ25896Core &core) : _core(core) {}

bool BQ25896Charger::enableCharging(bool enable)
{
    if (enable) {
        _userDisableCharge = false;
        return _core.updateBits(REG_CHG_CTRL, MASK_CHG_CONFIG, MASK_CHG_CONFIG) >= 0;
    } else {
        _userDisableCharge = true;
        return _core.updateBits(REG_CHG_CTRL, MASK_CHG_CONFIG, 0) >= 0;
    }
}

bool BQ25896Charger::isCharging()
{
    return _core.isCharging();
}

bool BQ25896Charger::setPreChargeCurrent(uint16_t mA)
{
    uint16_t closest = PRE_CHG_CUR_BASE + ((mA - PRE_CHG_CUR_BASE + PRE_CHG_CUR_STEP / 2) / PRE_CHG_CUR_STEP) * PRE_CHG_CUR_STEP;
    if (closest < PRE_CHG_CURRENT_MIN) {
        closest = PRE_CHG_CURRENT_MIN;
    }
    if (closest > PRE_CHG_CURRENT_MAX) {
        closest = PRE_CHG_CURRENT_MAX;
    }
    uint8_t val = ((closest - PRE_CHG_CUR_BASE) / PRE_CHG_CUR_STEP) << SHIFT_IPRECHG;
    return _core.updateBits(REG_CHG_PRE_CURR, MASK_IPRECHG, val) >= 0;
}

uint16_t BQ25896Charger::getPreChargeCurrent()
{
    int val = _core.readReg(REG_CHG_PRE_CURR);
    if (val < 0) return 0;
    val = (val >> SHIFT_IPRECHG) & 0x0F;
    return PRE_CHG_CUR_BASE + (val * PRE_CHG_CUR_STEP);
}

bool BQ25896Charger::setFastChargeCurrent(uint16_t mA)
{
    uint16_t closest;
    if (mA <= FAST_CHG_CUR_STEP / 2) {
        closest = 0;
    } else {
        closest = ((mA + FAST_CHG_CUR_STEP / 2) / FAST_CHG_CUR_STEP) * FAST_CHG_CUR_STEP;
    }
    if (closest > FAST_CHG_CURRENT_MAX) {
        closest = FAST_CHG_CURRENT_MAX;
    }
    uint8_t val = (closest / FAST_CHG_CUR_STEP) & MASK_ICHG;
    return _core.updateBits(REG_CHG_CURRENT, MASK_ICHG, val) >= 0;
}

uint16_t BQ25896Charger::getFastChargeCurrent()
{
    int val = _core.readReg(REG_CHG_CURRENT);
    if (val < 0) return 0;
    return (val & MASK_ICHG) * FAST_CHG_CUR_STEP;
}

bool BQ25896Charger::setTerminationCurrent(uint16_t mA)
{
    uint16_t closest = TERM_CHG_CUR_BASE + ((mA - TERM_CHG_CUR_BASE + TERM_CHG_CUR_STEP / 2) / TERM_CHG_CUR_STEP) * TERM_CHG_CUR_STEP;
    if (closest < TERM_CHG_CURRENT_MIN) {
        closest = TERM_CHG_CURRENT_MIN;
    }
    if (closest > TERM_CHG_CURRENT_MAX) {
        closest = TERM_CHG_CURRENT_MAX;
    }
    uint8_t val = ((closest - TERM_CHG_CUR_BASE) / TERM_CHG_CUR_STEP) & MASK_ITERM;
    return _core.updateBits(REG_CHG_PRE_CURR, MASK_ITERM, val) >= 0;
}

uint16_t BQ25896Charger::getTerminationCurrent()
{
    int val = _core.readReg(REG_CHG_PRE_CURR);
    if (val < 0) return 0;
    val = val & MASK_ITERM;
    return TERM_CHG_CUR_BASE + (val * TERM_CHG_CUR_STEP);
}

bool BQ25896Charger::setChargeVoltage(uint16_t mV)
{
    uint16_t closest = CHG_VOL_BASE + ((mV - CHG_VOL_BASE + CHG_VOL_STEP / 2) / CHG_VOL_STEP) * CHG_VOL_STEP;
    if (closest < FAST_CHG_VOL_MIN) {
        closest = FAST_CHG_VOL_MIN;
    }
    if (closest > FAST_CHG_VOL_MAX) {
        closest = FAST_CHG_VOL_MAX;
    }
    uint8_t val = (((closest - CHG_VOL_BASE) / CHG_VOL_STEP) << SHIFT_VREG) & MASK_VREG;
    return _core.updateBits(REG_CHG_VOLT, MASK_VREG, val) >= 0;
}

uint16_t BQ25896Charger::getChargeVoltage()
{
    int val = _core.readReg(REG_CHG_VOLT);
    if (val < 0) return 0;
    val = (val >> SHIFT_VREG) & 0x3F;
    if (val > 0x30) {
        return FAST_CHG_VOL_MAX;
    }
    return CHG_VOL_BASE + (val * CHG_VOL_STEP);
}

PmicChargerBase::Status BQ25896Charger::getStatus()
{
    Status status;
    status.online = _core.isValid();
    status.vbusPresent = _core.isVbusPresent();
    status.charging = _core.isCharging();
    status.chargeDone = _core.isChargeDone();
    uint8_t tmp = _core.getChargeStatus();
    switch (tmp) {
    case 0x00:
        status.chargingStatus = ChargingStatus::NO_CHARGING;
        break;
    case 0x01:
        status.chargingStatus = ChargingStatus::PRE_CHARGE;
        break;
    case 0x02:
        status.chargingStatus = ChargingStatus::FAST_CHARGE;
        break;
    case 0x03:
        status.chargingStatus = ChargingStatus::TERMINATION;
        break;
    default:
        status.chargingStatus = ChargingStatus::UNKNOWN;
        break;
    }
    uint8_t fault;
    if (_core.getFaultStatus(fault)) {
        status.fault = (fault != 0);
        status.faultCode = fault;
    }

    return status;
}

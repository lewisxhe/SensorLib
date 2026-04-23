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
 * @file      AXP517Charger.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#include "AXP517Charger.hpp"
#include "AXP517Regs.hpp"

using namespace axp517::regs;

AXP517Charger::AXP517Charger(AXP517Core &core) : _core(core)
{

}

bool AXP517Charger::enableCharging(bool enable)
{
    return _core.updateBits(ctrl::MODULE_EN1, 0x02, enable ? 0x02 : 0x00);
}

bool AXP517Charger::isCharging()
{
    return getStatus().charging;
}

bool AXP517Charger::setPreChargeCurrent(uint16_t mA)
{
    return setCurrentWithStep(chg::IPRECHG_ITRICHG, mA, 960, 64, 0x0F);
}

uint16_t AXP517Charger::getPreChargeCurrent()
{
    return getCurrentWithStep(chg::IPRECHG_ITRICHG, 64, 0x0F);
}

bool AXP517Charger::setFastChargeCurrent(uint16_t mA)
{
    return setCurrentWithStep(chg::ICC_SETTING, mA, 5120, 64, 0x7F);
}

uint16_t AXP517Charger::getFastChargeCurrent()
{
    return getCurrentWithStep(chg::ICC_SETTING, 64, 0x7F);
}

bool AXP517Charger::setTerminationCurrent(uint16_t mA)
{
    return setCurrentWithStep(chg::ITERM_CTRL, mA, 960, 64, 0x0F);
}

uint16_t AXP517Charger::getTerminationCurrent()
{
    return getCurrentWithStep(chg::ITERM_CTRL, 64, 0x0F);
}

bool AXP517Charger::setChargeVoltage(uint16_t mV)
{
    uint8_t voltCode = 0;
    int regValue = 0;
    switch (mV) {
    case 4000: voltCode = 0x00; break;
    case 4100: voltCode = 0x01; break;
    case 4200: voltCode = 0x02; break;
    case 4350: voltCode = 0x03; break;
    case 4400: voltCode = 0x04; break;
    case 3800: voltCode = 0x05; break;
    case 3600: voltCode = 0x06; break;
    case 5000: voltCode = 0x07; break;
    default: return false;
    }
    regValue =  _core.readReg(chg::CV_CHG_VOLTAGE);
    if (regValue < 0) {
        return false;
    }
    regValue = (regValue &  0xF8) | voltCode;
    if (_core.writeReg(chg::CV_CHG_VOLTAGE, regValue) < 0) {
        return false;
    }
    return true;
}

uint16_t AXP517Charger::getChargeVoltage()
{
    int regValue = _core.readReg(chg::CV_CHG_VOLTAGE);
    if (regValue < 0) {
        return 0;
    }
    uint8_t voltCode = regValue & 0x07;
    switch (voltCode) {
    case 0x00: return 4000;
    case 0x01: return 4100;
    case 0x02: return 4200;
    case 0x03: return 4350;
    case 0x04: return 4400;
    case 0x05: return 3800;
    case 0x06: return 3600;
    case 0x07: return 5000;
    default: break;
    }
    return 0;
}

AXP517Charger::Status AXP517Charger::getStatus()
{
    Status status {};
    uint8_t buffer[2] {};
    if (_core.readRegBuff(bmu::STATUS0, buffer, sizeof(buffer)) < 0) {
        return status;
    }
    status.online = true;
    status.vbusPresent = (buffer[0] >> 5) & 0x01;
    status.batteryPresent = (buffer[0] >> 3) & 0x01;
    ///< 000: tri_charge
    ///< 001: pre_charge
    ///< 010: constant charge(CC)
    ///< 011: constant voltage(CV)
    ///< 100: charge done
    ///< 101: not charging
    ///< 11X: Reserved
    if (status.batteryPresent) {
        uint8_t charging_status = buffer[1] & 0x03;
        status.charging = charging_status < 3;
        status.chargeDone = charging_status == 4;
    } else {
        status.charging = false;
        status.chargeDone = false;
    }
    status.fault = false; //TODO: Implement fault detection
    return status;
}

bool AXP517Charger::setCurrentWithStep(uint8_t reg, uint16_t milliamps,
                                       uint16_t max, uint16_t step, uint8_t mask)
{
    if (milliamps > max) milliamps = max;
    uint8_t code = milliamps / step;
    milliamps = code * step;
    int regValue = _core.readReg(reg);
    if (regValue < 0) {
        return false;
    }
    regValue = (regValue & ~mask) | code;
    if (_core.writeReg(reg, regValue) < 0) {
        return false;
    }
    return true;
}

uint16_t AXP517Charger::getCurrentWithStep(uint8_t reg, uint16_t step, uint8_t mask)
{
    int regValue = _core.readReg(reg);
    if (regValue < 0) {
        return 0;
    }
    return (regValue & mask) * step;
}

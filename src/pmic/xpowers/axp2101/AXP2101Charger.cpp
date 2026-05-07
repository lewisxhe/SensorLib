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
 * @file      AXP2101Charger.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Charger Control
 *
 */
#include "AXP2101Charger.hpp"
#include "AXP2101Regs.hpp"


AXP2101Charger::AXP2101Charger(AXP2101Core &core) : _core(core)
{
}

bool AXP2101Charger::enableCharging(bool enable)
{
    return _core.enableModule(AXP2101Core::Module::CELL_CHARGE, enable);
}

bool AXP2101Charger::isCharging()
{
    return getStatus().charging;
}

bool AXP2101Charger::setPreChargeCurrent(uint16_t mA)
{
    if (mA > 200) mA = 200;
    uint8_t code = mA / 25;
    int regVal = _core.readReg(axp2101_regs::chg::IPRECHG_SET);
    if (regVal < 0) return false;
    regVal = (regVal & 0xF0) | code;
    return _core.writeReg(axp2101_regs::chg::IPRECHG_SET, static_cast<uint8_t>(regVal)) >= 0;
}

uint16_t AXP2101Charger::getPreChargeCurrent()
{
    int val = _core.readReg(axp2101_regs::chg::IPRECHG_SET);
    if (val < 0) return 0;
    return (val & 0x0F) * 25;
}

bool AXP2101Charger::setFastChargeCurrent(uint16_t mA)
{
    uint8_t code;
    if (mA > 1000) mA = 1000;
    if (mA <= 200) {
        code = mA / 25;
    } else {
        code = (mA - 200) / 100 + 8;
    }
    int regVal = _core.readReg(axp2101_regs::chg::ICC_SETTING);
    if (regVal < 0) return false;
    regVal = (regVal & 0xE0) | code;
    return _core.writeReg(axp2101_regs::chg::ICC_SETTING, static_cast<uint8_t>(regVal)) >= 0;
}

uint16_t AXP2101Charger::getFastChargeCurrent()
{
    int val = _core.readReg(axp2101_regs::chg::ICC_SETTING);
    if (val < 0) return 0;
    uint8_t code = val & 0x1F;
    if (code <= 8) {
        return code * 25;
    }
    return 200 + (code - 8) * 100;
}

bool AXP2101Charger::setTerminationCurrent(uint16_t mA)
{
    if (mA > 200) mA = 200;
    uint8_t code = mA / 25;
    int regVal = _core.readReg(axp2101_regs::chg::ITERM_CTRL);
    if (regVal < 0) return false;
    regVal = (regVal & 0xF0) | code;
    return _core.writeReg(axp2101_regs::chg::ITERM_CTRL, static_cast<uint8_t>(regVal)) >= 0;
}

uint16_t AXP2101Charger::getTerminationCurrent()
{
    int val = _core.readReg(axp2101_regs::chg::ITERM_CTRL);
    if (val < 0) return 0;
    return (val & 0x0F) * 25;
}

bool AXP2101Charger::setChargeVoltage(uint16_t mV)
{
    uint8_t code;
    switch (mV) {
    case 5000: code = 0; break;
    case 4000: code = 1; break;
    case 4100: code = 2; break;
    case 4200: code = 3; break;
    case 4350: code = 4; break;
    case 4400: code = 5; break;
    default: return false;
    }
    int regVal = _core.readReg(axp2101_regs::chg::CV_CHG_VOLTAGE);
    if (regVal < 0) return false;
    regVal = (regVal & 0xF8) | code;
    return _core.writeReg(axp2101_regs::chg::CV_CHG_VOLTAGE, static_cast<uint8_t>(regVal)) >= 0;
}

uint16_t AXP2101Charger::getChargeVoltage()
{
    int val = _core.readReg(axp2101_regs::chg::CV_CHG_VOLTAGE);
    if (val < 0) return 0;
    switch (val & 0x07) {
    case 0:  return 5000;
    case 1:  return 4000;
    case 2:  return 4100;
    case 3:  return 4200;
    case 4:  return 4350;
    case 5:  return 4400;
    default: return 0;
    }
}

AXP2101Charger::Status AXP2101Charger::getStatus()
{
    Status status{};
    int status1 = _core.readReg(axp2101_regs::bmu::STATUS1);
    int status2 = _core.readReg(axp2101_regs::bmu::STATUS2);
    if (status1 < 0 || status2 < 0) {
        return status;
    }
    status.online = true;
    status.vbusPresent = (status1 >> 5) & 0x01;
    status.batteryPresent = (status1 >> 3) & 0x01;
    uint8_t chargePhase = status2 & 0x07;
    switch (chargePhase) {
    case 1:
        status.chargingStatus = ChargingStatus::PRE_CHARGE;
        break;
    case 2:
    case 3:
        status.chargingStatus = ChargingStatus::FAST_CHARGE;
        break;
    case 4:
        status.chargingStatus = ChargingStatus::TERMINATION;
        break;
    default:
        status.chargingStatus = ChargingStatus::NO_CHARGING;
        break;
    }
    status.chargeDone = (chargePhase == 4);
    status.charging = (chargePhase >= 1 && chargePhase <= 3);
    status.fault = (status1 & 0x01) != 0;
    return status;
}

// ---- Termination Control ----

void AXP2101Charger::enableChargeTermination()
{
    int val = _core.readReg(axp2101_regs::chg::ITERM_CTRL);
    if (val < 0) return;
    _core.writeReg(axp2101_regs::chg::ITERM_CTRL, val | 0x10);
}

void AXP2101Charger::disableChargeTermination()
{
    int val = _core.readReg(axp2101_regs::chg::ITERM_CTRL);
    if (val < 0) return;
    _core.writeReg(axp2101_regs::chg::ITERM_CTRL, val & 0xEF);
}

bool AXP2101Charger::isChargeTerminationEnabled()
{
    int val = _core.readReg(axp2101_regs::chg::ITERM_CTRL);
    if (val < 0) return false;
    return (val >> 4) & 0x01;
}

// ---- Button Battery Charge ----

void AXP2101Charger::enableButtonBatteryCharge()
{
    _core.setRegBit(axp2101_regs::ctrl::MODULE_EN, 2);
}

void AXP2101Charger::disableButtonBatteryCharge()
{
    _core.clrRegBit(axp2101_regs::ctrl::MODULE_EN, 2);
}

bool AXP2101Charger::isButtonBatteryChargeEnabled()
{
    return _core.getRegBit(axp2101_regs::ctrl::MODULE_EN, 2);
}

bool AXP2101Charger::setButtonBatteryChargeVoltage(uint16_t millivolt)
{
    if (millivolt < 2600 || millivolt > 3300 || (millivolt % 100))
        return false;
    int val = _core.readReg(axp2101_regs::chg::BTN_BAT_CHG_VOL);
    if (val < 0) return false;
    val &= 0xF8;
    val |= (millivolt - 2600) / 100;
    return _core.writeReg(axp2101_regs::chg::BTN_BAT_CHG_VOL, val) >= 0;
}

uint16_t AXP2101Charger::getButtonBatteryChargeVoltage()
{
    int val = _core.readReg(axp2101_regs::chg::BTN_BAT_CHG_VOL);
    if (val < 0) return 0;
    return ((val & 0x07) * 100) + 2600;
}

// ---- Battery Detection ----

void AXP2101Charger::enableBatteryDetection()
{
    _core.setRegBit(axp2101_regs::chg::BAT_DETECT_CTRL, 0);
}

void AXP2101Charger::disableBatteryDetection()
{
    _core.clrRegBit(axp2101_regs::chg::BAT_DETECT_CTRL, 0);
}

bool AXP2101Charger::isBatteryDetectionEnabled()
{
    return _core.getRegBit(axp2101_regs::chg::BAT_DETECT_CTRL, 0);
}

// ---- Charger Safety Timer ----

void AXP2101Charger::setPreChargeSafetyTimer(uint8_t opt)
{
    if (opt > 3) opt = 3;
    int val = _core.readReg(axp2101_regs::chg::CHG_TIMER_CFG);
    if (val < 0) return;
    val &= 0xFC;
    val |= opt;
    _core.writeReg(axp2101_regs::chg::CHG_TIMER_CFG, val);
}

void AXP2101Charger::setChargeDoneSafetyTimer(uint8_t opt)
{
    if (opt > 3) opt = 3;
    int val = _core.readReg(axp2101_regs::chg::CHG_TIMER_CFG);
    if (val < 0) return;
    val &= 0xCF;
    val |= (opt << 4);
    _core.writeReg(axp2101_regs::chg::CHG_TIMER_CFG, val);
}

// ---- Thermal Regulation ----

void AXP2101Charger::setThermalRegulationThreshold(uint8_t opt)
{
    if (opt > 3) return;
    int val = _core.readReg(axp2101_regs::chg::THERMAL_REG_THRESH);
    if (val < 0) return;
    val &= 0xFC;
    val |= opt;
    _core.writeReg(axp2101_regs::chg::THERMAL_REG_THRESH, val);
}

uint8_t AXP2101Charger::getThermalRegulationThreshold()
{
    int val = _core.readReg(axp2101_regs::chg::THERMAL_REG_THRESH);
    if (val < 0) return 0;
    return val & 0x03;
}

// ---- JEITA Temperature Protection ----

bool AXP2101Charger::enableJeita(bool enable)
{
    return enable ? _core.setRegBit(axp2101_regs::jeita::EN_CTRL, 0)
                  : _core.clrRegBit(axp2101_regs::jeita::EN_CTRL, 0);
}

bool AXP2101Charger::isJeitaEnabled()
{
    return _core.getRegBit(axp2101_regs::jeita::EN_CTRL, 0);
}

// -- Temperature Fault Thresholds --

bool AXP2101Charger::setChargeLowTempFaultThreshold(uint8_t val)
{
    return _core.writeReg(axp2101_regs::jeita::VLTF_CHG, val) >= 0;
}

uint8_t AXP2101Charger::getChargeLowTempFaultThreshold()
{
    int val = _core.readReg(axp2101_regs::jeita::VLTF_CHG);
    return (val < 0) ? 0 : static_cast<uint8_t>(val);
}

bool AXP2101Charger::setChargeHighTempFaultThreshold(uint8_t val)
{
    return _core.writeReg(axp2101_regs::jeita::VHTF_CHG, val) >= 0;
}

uint8_t AXP2101Charger::getChargeHighTempFaultThreshold()
{
    int val = _core.readReg(axp2101_regs::jeita::VHTF_CHG);
    return (val < 0) ? 0 : static_cast<uint8_t>(val);
}

bool AXP2101Charger::setWorkLowTempFaultThreshold(uint8_t val)
{
    return _core.writeReg(axp2101_regs::jeita::VLTF_WORK, val) >= 0;
}

uint8_t AXP2101Charger::getWorkLowTempFaultThreshold()
{
    int val = _core.readReg(axp2101_regs::jeita::VLTF_WORK);
    return (val < 0) ? 0 : static_cast<uint8_t>(val);
}

bool AXP2101Charger::setWorkHighTempFaultThreshold(uint8_t val)
{
    return _core.writeReg(axp2101_regs::jeita::VHTF_WORK, val) >= 0;
}

uint8_t AXP2101Charger::getWorkHighTempFaultThreshold()
{
    int val = _core.readReg(axp2101_regs::jeita::VHTF_WORK);
    return (val < 0) ? 0 : static_cast<uint8_t>(val);
}

// -- Temperature Hysteresis --

bool AXP2101Charger::setLowTempHysteresis(uint8_t val)
{
    return _core.writeReg(axp2101_regs::ts::HYSL2H, val) >= 0;
}

uint8_t AXP2101Charger::getLowTempHysteresis()
{
    int val = _core.readReg(axp2101_regs::ts::HYSL2H);
    return (val < 0) ? 0 : static_cast<uint8_t>(val);
}

bool AXP2101Charger::setHighTempHysteresis(uint8_t val)
{
    return _core.writeReg(axp2101_regs::ts::HYSH2L, val) >= 0;
}

uint8_t AXP2101Charger::getHighTempHysteresis()
{
    int val = _core.readReg(axp2101_regs::ts::HYSH2L);
    return (val < 0) ? 0 : static_cast<uint8_t>(val);
}

// -- JEITA CV Configuration (REG 59) --

bool AXP2101Charger::setJeitaWarmCurrentReduction(bool reduce)
{
    return reduce ? _core.setRegBit(axp2101_regs::jeita::SET0, 6)
                  : _core.clrRegBit(axp2101_regs::jeita::SET0, 6);
}

bool AXP2101Charger::getJeitaWarmCurrentReduction()
{
    return _core.getRegBit(axp2101_regs::jeita::SET0, 6);
}

bool AXP2101Charger::setJeitaCoolCurrentReduction(bool reduce)
{
    return reduce ? _core.setRegBit(axp2101_regs::jeita::SET0, 4)
                  : _core.clrRegBit(axp2101_regs::jeita::SET0, 4);
}

bool AXP2101Charger::getJeitaCoolCurrentReduction()
{
    return _core.getRegBit(axp2101_regs::jeita::SET0, 4);
}

bool AXP2101Charger::setJeitaWarmVoltageDrop(uint8_t levels)
{
    if (levels > 2) return false;
    int val = _core.readReg(axp2101_regs::jeita::SET0);
    if (val < 0) return false;
    val &= 0xF3;
    val |= (levels << 2);
    return _core.writeReg(axp2101_regs::jeita::SET0, static_cast<uint8_t>(val)) >= 0;
}

uint8_t AXP2101Charger::getJeitaWarmVoltageDrop()
{
    int val = _core.readReg(axp2101_regs::jeita::SET0);
    if (val < 0) return 0;
    return (val >> 2) & 0x03;
}

bool AXP2101Charger::setJeitaCoolVoltageDrop(uint8_t levels)
{
    if (levels > 2) return false;
    int val = _core.readReg(axp2101_regs::jeita::SET0);
    if (val < 0) return false;
    val &= 0xFC;
    val |= levels;
    return _core.writeReg(axp2101_regs::jeita::SET0, static_cast<uint8_t>(val)) >= 0;
}

uint8_t AXP2101Charger::getJeitaCoolVoltageDrop()
{
    int val = _core.readReg(axp2101_regs::jeita::SET0);
    if (val < 0) return 0;
    return val & 0x03;
}

// -- JEITA Temperature Thresholds (REG 5B) --

bool AXP2101Charger::setJeitaCoolThreshold(uint8_t val)
{
    // T2: VHTF = val * 16mV, stored in lower nibble of REG 0x5B
    int reg = _core.readReg(axp2101_regs::jeita::SET2);
    if (reg < 0) return false;
    reg &= 0xF0;
    reg |= (val & 0x0F);
    return _core.writeReg(axp2101_regs::jeita::SET2, static_cast<uint8_t>(reg)) >= 0;
}

uint8_t AXP2101Charger::getJeitaCoolThreshold()
{
    int val = _core.readReg(axp2101_regs::jeita::SET2);
    if (val < 0) return 0;
    return val & 0x0F;
}

bool AXP2101Charger::setJeitaWarmThreshold(uint8_t val)
{
    // T3: VHTF = val * 8mV, stored in upper nibble of REG 0x5B
    int reg = _core.readReg(axp2101_regs::jeita::SET2);
    if (reg < 0) return false;
    reg &= 0x0F;
    reg |= ((val & 0x0F) << 4);
    return _core.writeReg(axp2101_regs::jeita::SET2, static_cast<uint8_t>(reg)) >= 0;
}

uint8_t AXP2101Charger::getJeitaWarmThreshold()
{
    int val = _core.readReg(axp2101_regs::jeita::SET2);
    if (val < 0) return 0;
    return (val >> 4) & 0x0F;
}

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
 * @file      AXP517Adc.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#include "../../../SensorBuildOpt.h"
#if !SENSORLIB_EXCLUDE_PMIC_AXP517

#include "AXP517Adc.hpp"
#include "AXP517Regs.hpp"


AXP517Adc::AXP517Adc(AXP517Core &core) : _core(core)
{
}

namespace
{
bool readRegisterPair(AXP517Core &core, uint8_t highReg, uint8_t lowReg,
                      uint8_t &high, uint8_t &low)
{
    // The AXP517 PMIC register window does not auto-increment on a burst read.
    // Read each address explicitly and retry if the high byte changed at a
    // conversion boundary.
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        int highBefore = core.readReg(highReg);
        int lowValue = core.readReg(lowReg);
        int highAfter = core.readReg(highReg);
        if (highBefore < 0 || lowValue < 0 || highAfter < 0) return false;
        if (highBefore == highAfter) {
            high = static_cast<uint8_t>(highBefore);
            low = static_cast<uint8_t>(lowValue);
            return true;
        }
    }
    return false;
}

bool selectAndReadAdcData(AXP517Core &core, uint8_t selector,
                          uint8_t &high, uint8_t &low)
{
    int dataSelect = core.readReg(axp517_regs::adc::DATA_SEL);
    if (dataSelect < 0) return false;

    if ((dataSelect & 0x0F) != selector) {
        const uint8_t next = static_cast<uint8_t>((dataSelect & 0xF0) | selector);
        if (core.writeReg(axp517_regs::adc::DATA_SEL, next) < 0) return false;

        // REG9C/9D retains the previous multiplexed channel briefly after
        // REG9B changes. Allow a fresh conversion before consuming the data.
        core.delayMs(10);
    }

    return readRegisterPair(core, axp517_regs::adc::DATA_H,
                            axp517_regs::adc::DATA_L, high, low);
}
} // namespace

static uint8_t axp517_channelToMask(PmicAdcBase::Channel ch)
{
    switch (ch) {
    case PmicAdcBase::Channel::BAT_VOLTAGE:      return 0x01;
    case PmicAdcBase::Channel::BAT_TEMPERATURE:  return 0x02;
    case PmicAdcBase::Channel::VBUS_VOLTAGE:     return 0x04;
    case PmicAdcBase::Channel::VSYS_VOLTAGE:     return 0x08;
    case PmicAdcBase::Channel::DIE_TEMPERATURE:  return 0x10;
    case PmicAdcBase::Channel::BAT_CURRENT:      return 0x60; // BAT_CHARGE | BAT_DISCHARGE
    case PmicAdcBase::Channel::VBUS_CURRENT:     return 0x80;
    default:                                     return 0;
    }
}

bool AXP517Adc::enableChannels(uint32_t mask)
{
    uint8_t hwMask = 0;
    forEachChannel(mask, [&](uint32_t bit) {
        auto ch = static_cast<Channel>(bit);
        hwMask |= axp517_channelToMask(ch);
    });
    if (hwMask == 0) return false;
    int regVal = _core.readReg(axp517_regs::adc::ENABLE);
    if (regVal < 0) return false;
    regVal |= hwMask;
    return _core.writeReg(axp517_regs::adc::ENABLE, static_cast<uint8_t>(regVal)) == 0;
}

bool AXP517Adc::disableChannels(uint32_t mask)
{
    uint8_t hwMask = 0;
    forEachChannel(mask, [&](uint32_t bit) {
        auto ch = static_cast<Channel>(bit);
        hwMask |= axp517_channelToMask(ch);
    });
    if (hwMask == 0) return false;
    int regVal = _core.readReg(axp517_regs::adc::ENABLE);
    if (regVal < 0) return false;
    regVal &= ~hwMask;
    return _core.writeReg(axp517_regs::adc::ENABLE, static_cast<uint8_t>(regVal)) == 0;
}

bool AXP517Adc::read(Channel ch, float &out)
{
    int16_t adcVal;
    uint8_t buffer[2] = {0, 0};
    out = NAN;  // Default to NaN for unsupported channels or read errors

    if (ch == Channel::VBUS_VOLTAGE || ch == Channel::VBUS_CURRENT ||
            ch == Channel::BAT_VOLTAGE || ch == Channel::BAT_CURRENT) {

        int status0 = _core.readReg(axp517_regs::bmu::STATUS0);
        if (status0 < 0) {
            return false;
        }
        bool vbusPresent = (status0 >> 5) & 0x01;
        bool batPresent = (status0 >> 3) & 0x01;

        if (ch == Channel::VBUS_VOLTAGE || ch == Channel::VBUS_CURRENT) {
            if (!vbusPresent) {
                // VBUS is not present
                out = 0.0f;
                return true;
            }
        }

        if (ch == Channel::BAT_VOLTAGE || ch == Channel::BAT_CURRENT) {
            if (!batPresent) {
                // Battery is not present
                out = 0.0f;
                return true;
            }
        }
    }

    switch (ch) {
    case Channel::VBUS_VOLTAGE:
        if (!readRegisterPair(_core, axp517_regs::adc::VBUS_VOLT_H,
                              axp517_regs::adc::VBUS_VOLT_L, buffer[0], buffer[1]))
            return false;
        out = ((buffer[0] & 0x3F) << 8 | buffer[1]) * axp517_regs::factory::FACTORY_VBUS_VOLTAGE;
        break;
    case Channel::VBUS_CURRENT:
        if (!readRegisterPair(_core, axp517_regs::adc::VBUS_CURR_H,
                              axp517_regs::adc::VBUS_CURR_L, buffer[0], buffer[1]))
            return false;
        adcVal = (static_cast<int16_t>((buffer[0] & 0x3F) << 8 | buffer[1]));
        out = adcVal * axp517_regs::factory::FACTORY_VBUS_CURRENT;
        break;
    case Channel::BAT_VOLTAGE:
        if (!readRegisterPair(_core, axp517_regs::adc::BAT_VOLT_H,
                              axp517_regs::adc::BAT_VOLT_L, buffer[0], buffer[1]))
            return false;
        out = ((buffer[0] & 0x3F) << 8 | buffer[1]) * axp517_regs::factory::FACTORY_VBAT_VOLTAGE;
        break;
    case Channel::BAT_CURRENT:
        if (!readRegisterPair(_core, axp517_regs::adc::BAT_CURR_H,
                              axp517_regs::adc::BAT_CURR_L, buffer[0], buffer[1]))
            return false;
        adcVal = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
        out = static_cast<float>(adcVal) * axp517_regs::factory::FACTORY_BAT_CURRENT;
        break;
    case Channel::VSYS_VOLTAGE:
        if (!selectAndReadAdcData(_core, 0x01, buffer[0], buffer[1])) return false;
        adcVal = ((buffer[0] & 0x3F) << 8) | buffer[1];
        out = static_cast<int16_t>((buffer[0] & 0x3F) << 8 | buffer[1]) * axp517_regs::factory::FACTORY_VSYS_VOLTAGE;
        break;
    case Channel::DIE_TEMPERATURE:
        if (!selectAndReadAdcData(_core, 0x00, buffer[0], buffer[1])) return false;
        adcVal = ((buffer[0] & 0x3F) << 8) | buffer[1];
        out = (3552.0f - adcVal) / 1.79f + 25.0f;
        break;
    case Channel::BAT_TEMPERATURE:
        if (_core.readRegBuff(axp517_regs::gauge::BAT_TEMP, buffer, 1) < 0) {
            return false;
        }
        out = static_cast<float>(buffer[0]);
        break;
    case Channel::BAT_PERCENTAGE:
        if (_core.readRegBuff(axp517_regs::gauge::BAT_PERCENT, buffer, 1) < 0) {
            return false;
        }
        out = static_cast<float>(buffer[0]);
        break;
    default:
        return false;
    }
    return true;
}

#endif

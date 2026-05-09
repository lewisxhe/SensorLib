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
#include "AXP517Adc.hpp"
#include "AXP517Regs.hpp"


AXP517Adc::AXP517Adc(AXP517Core &core) : _core(core)
{
}

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

        if (_core.readRegBuff(axp517_regs::bmu::STATUS0, buffer, sizeof(buffer)) < 0) {
            return false;
        }
        bool vbusPresent = (buffer[0] >> 5) & 0x01;
        bool batPresent = (buffer[0] >> 3) & 0x01;

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
        if (_core.readRegBuff(axp517_regs::adc::VBUS_VOLT_H, buffer, sizeof(buffer)) < 0)
            return false;
        out = ((buffer[0] & 0x3F) << 8 | buffer[1]) * axp517_regs::factory::FACTORY_VBUS_VOLTAGE;
        break;
    case Channel::VBUS_CURRENT:
        if (_core.readRegBuff(axp517_regs::adc::VBUS_CURR_H, buffer, sizeof(buffer)) < 0)
            return false;
        adcVal = (static_cast<int16_t>((buffer[0] & 0x3F) << 8 | buffer[1]));
        out = adcVal * axp517_regs::factory::FACTORY_VBUS_CURRENT;
        break;
    case Channel::BAT_VOLTAGE:
        if (_core.readRegBuff(axp517_regs::adc::BAT_VOLT_H, buffer, sizeof(buffer)) < 0)
            return false;
        out = ((buffer[0] & 0x3F) << 8 | buffer[1]) * axp517_regs::factory::FACTORY_VBAT_VOLTAGE;
        break;
    case Channel::BAT_CURRENT:
        if (_core.readRegBuff(axp517_regs::adc::BAT_CURR_H, buffer, sizeof(buffer)) < 0)
            return false;
        adcVal = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
        out = static_cast<float>(adcVal) * axp517_regs::factory::FACTORY_BAT_CURRENT;
        break;
    case Channel::VSYS_VOLTAGE:
        if (_core.writeReg(axp517_regs::adc::DATA_SEL, 0x01) < 0) return false;
        if (_core.readRegBuff(axp517_regs::adc::DATA_H, buffer, 2) < 0) return false;
        adcVal = ((buffer[0] & 0x3F) << 8) | buffer[1];
        out = static_cast<int16_t>((buffer[0] & 0x3F) << 8 | buffer[1]) * axp517_regs::factory::FACTORY_VSYS_VOLTAGE;
        break;
    case Channel::DIE_TEMPERATURE:
        if (_core.writeReg(axp517_regs::adc::DATA_SEL, 0x00) < 0) return false;
        if (_core.readRegBuff(axp517_regs::adc::DATA_H, buffer, 2) < 0) return false;
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

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
 * @file      AXP2101Adc.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 ADC Control
 *
 */
#include "AXP2101Adc.hpp"
#include "AXP2101Regs.hpp"
#include <math.h>


AXP2101Adc::AXP2101Adc(AXP2101Core &core) : _core(core)
{
}

static uint8_t axp2101_channelToMask(PmicAdcBase::Channel ch)
{
    switch (ch) {
    case PmicAdcBase::Channel::BAT_VOLTAGE:      return 0x01;
    case PmicAdcBase::Channel::BAT_TEMPERATURE:  return 0x02;
    case PmicAdcBase::Channel::VBUS_VOLTAGE:     return 0x04;
    case PmicAdcBase::Channel::VSYS_VOLTAGE:     return 0x08;
    case PmicAdcBase::Channel::DIE_TEMPERATURE:  return 0x10;
    default:                                     return 0;
    }
}

bool AXP2101Adc::enableChannels(uint32_t mask)
{
    uint8_t hwMask = 0;
    forEachChannel(mask, [&](uint32_t bit) {
        auto ch = static_cast<Channel>(bit);
        hwMask |= axp2101_channelToMask(ch);
    });
    if (hwMask == 0) return false;

    // TS pin special handling
    if (hwMask & 0x02) {
        int tsCtrl = _core.readReg(axp2101_regs::ts::TS_PIN_CTRL);
        if (tsCtrl < 0) return false;
        tsCtrl &= 0xE0;
        tsCtrl |= 0x07;
        if (_core.writeReg(axp2101_regs::ts::TS_PIN_CTRL, static_cast<uint8_t>(tsCtrl)) < 0) return false;
    }

    int regVal = _core.readReg(axp2101_regs::adc::CHANNEL_CTRL);
    if (regVal < 0) return false;
    regVal |= hwMask;
    return _core.writeReg(axp2101_regs::adc::CHANNEL_CTRL, static_cast<uint8_t>(regVal)) >= 0;
}

bool AXP2101Adc::disableChannels(uint32_t mask)
{
    uint8_t hwMask = 0;
    forEachChannel(mask, [&](uint32_t bit) {
        auto ch = static_cast<Channel>(bit);
        hwMask |= axp2101_channelToMask(ch);
    });
    if (hwMask == 0) return false;

    if (hwMask & 0x02) {
        int tsCtrl = _core.readReg(axp2101_regs::ts::TS_PIN_CTRL);
        if (tsCtrl < 0) return false;
        tsCtrl &= 0xF0;
        tsCtrl |= 0x10;
        if (_core.writeReg(axp2101_regs::ts::TS_PIN_CTRL, static_cast<uint8_t>(tsCtrl)) < 0) return false;
    }

    int regVal = _core.readReg(axp2101_regs::adc::CHANNEL_CTRL);
    if (regVal < 0) return false;
    regVal &= ~hwMask;
    return _core.writeReg(axp2101_regs::adc::CHANNEL_CTRL, static_cast<uint8_t>(regVal)) >= 0;
}

bool AXP2101Adc::read(Channel ch, float &out)
{
    uint8_t buffer[2] = {0, 0};
    out = NAN;

    if (ch == Channel::VBUS_VOLTAGE || ch == Channel::VBUS_CURRENT ||
            ch == Channel::BAT_VOLTAGE || ch == Channel::BAT_CURRENT) {
        int status1 = _core.readReg(axp2101_regs::bmu::STATUS1);
        if (status1 < 0) return false;
        bool vbusPresent = (status1 >> 5) & 0x01;
        bool batPresent = (status1 >> 3) & 0x01;
        if (ch == Channel::VBUS_VOLTAGE || ch == Channel::VBUS_CURRENT) {
            if (!vbusPresent) {
                out = 0.0f;
                return true;
            }
        }
        if (ch == Channel::BAT_VOLTAGE || ch == Channel::BAT_CURRENT) {
            if (!batPresent) {
                out = 0.0f;
                return true;
            }
        }
    }

    switch (ch) {
    case Channel::VBUS_VOLTAGE:
        if (_core.readRegBuff(axp2101_regs::adc::VBUS_VOLT_H, buffer, 2) < 0)
            return false;
        out = static_cast<float>(((buffer[0] & 0x3F) << 8 | buffer[1])) * axp2101_regs::factory::VBUS_STEP;
        break;
    case Channel::VBUS_CURRENT:
        out = 0.0f;
        return true;
    case Channel::BAT_VOLTAGE:
        if (_core.readRegBuff(axp2101_regs::adc::BAT_VOLT_H, buffer, 2) < 0)
            return false;
        out = static_cast<float>(((buffer[0] & 0x1F) << 8 | buffer[1])) * axp2101_regs::factory::VBAT_STEP;
        break;
    case Channel::BAT_CURRENT:
        out = 0.0f;
        return true;
    case Channel::VSYS_VOLTAGE:
        if (_core.readRegBuff(axp2101_regs::adc::VSYS_VOLT_H, buffer, 2) < 0)
            return false;
        out = static_cast<float>(((buffer[0] & 0x3F) << 8 | buffer[1])) * axp2101_regs::factory::VSYS_STEP;
        break;
    case Channel::DIE_TEMPERATURE:
        if (_core.readRegBuff(axp2101_regs::adc::DIE_TEMP_H, buffer, 2) < 0)
            return false;
        {
            uint16_t raw = ((buffer[0] & 0x3F) << 8) | buffer[1];
            // formula: (22.0 + (7274 - raw) / 20.0)
            out = 22.0f + (7274.0f - static_cast<float>(raw)) / 20.0f;
        }
        break;
    case Channel::BAT_TEMPERATURE:
        if (_core.readRegBuff(axp2101_regs::adc::TS_VOLT_H, buffer, 2) < 0)
            return false;
        out = static_cast<float>(((buffer[0] & 0x3F) << 8 | buffer[1])) * axp2101_regs::factory::TS_STEP;
        break;
    case Channel::BAT_PERCENTAGE:
        {
            int val = _core.readReg(axp2101_regs::gauge::BAT_PERCENT);
            if (val < 0) return false;
            out = static_cast<float>(val);
        }
        break;
    default:
        return false;
    }
    return true;
}

// ---- TS Temperature (Steinhart-Hart) ----

float AXP2101Adc::getTsTemperature(float SteinhartA, float SteinhartB, float SteinhartC)
{
    float voltage_mV = 0.0f;
    if (!read(Channel::BAT_TEMPERATURE, voltage_mV)) return 0.0f;

    if (voltage_mV == 4095.0f) return 0.0f;

    float voltage_V = voltage_mV / 1000.0f;
    float resistance = voltage_V / 5e-5f;
    float ln_r = logf(resistance);
    float t_inv = SteinhartA + SteinhartB * ln_r + SteinhartC * powf(ln_r, 3.0f);
    return (1.0f / t_inv) - 273.15f;
}

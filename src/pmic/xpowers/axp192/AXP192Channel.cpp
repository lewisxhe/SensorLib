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
 * @file      AXP192Channel.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 */
#include "AXP192Channel.hpp"
#include "AXP192Regs.hpp"


AXP192Channel::AXP192Channel(AXP192Core &core) : _core(core)
{
}

static uint8_t getEnableBit(uint8_t channel)
{
    switch (channel) {
    case 0:  return 0;
    case 1:  return 4;
    case 2:  return 1;
    case 3:  return 2;
    case 4:  return 3;
    case 5:  return 0xFF;
    case 6:  return 6;
    default: return 0xFF;
    }
}

bool AXP192Channel::enable(uint8_t channel, bool enable)
{
    if (channel >= CHANNEL_COUNT) return false;

    if (channel == CH_LDOio) {
        int val = _core.readReg(axp192_regs::gpio::GPIO0_CTL);
        if (val < 0) return false;
        val &= 0xF8;
        if (enable) {
            val |= 0x02;
        }
        return _core.writeReg(axp192_regs::gpio::GPIO0_CTL, static_cast<uint8_t>(val)) >= 0;
    }

    uint8_t bit = getEnableBit(channel);
    if (bit == 0xFF) return false;
    return enable ? _core.setRegBit(axp192_regs::pwr::LDO23_DC123_EXT_CTL, bit)
           : _core.clrRegBit(axp192_regs::pwr::LDO23_DC123_EXT_CTL, bit);
}

bool AXP192Channel::isEnabled(uint8_t channel)
{
    if (channel >= CHANNEL_COUNT) return false;

    if (channel == CH_LDOio) {
        int val = _core.readReg(axp192_regs::gpio::GPIO0_CTL);
        if (val < 0) return false;
        return (val & 0x02) != 0;
    }

    uint8_t bit = getEnableBit(channel);
    if (bit == 0xFF) return false;
    return _core.getRegBit(axp192_regs::pwr::LDO23_DC123_EXT_CTL, bit);
}

bool AXP192Channel::setVoltage(uint8_t channel, uint16_t mV)
{
    if (channel >= CHANNEL_COUNT) return false;

    switch (channel) {
    case CH_DCDC1: {
        if (mV < 700) mV = 700;
        if (mV > 3500) mV = 3500;
        uint8_t code = (mV - 700) / 25;
        return _core.updateBits(axp192_regs::pwr::DC1_VOLTAGE, 0x7F, code) >= 0;
    }
    case CH_DCDC2: {
        if (mV < 700) mV = 700;
        if (mV > 2275) mV = 2275;
        uint8_t code = (mV - 700) / 25;
        return _core.updateBits(axp192_regs::pwr::DC2OUT_VOL, 0x3F, code) >= 0;
    }
    case CH_DCDC3: {
        if (mV < 700) mV = 700;
        if (mV > 3500) mV = 3500;
        uint8_t code = (mV - 700) / 25;
        return _core.updateBits(axp192_regs::pwr::DC3OUT_VOL, 0x7F, code) >= 0;
    }
    case CH_LDO2: {
        if (mV < 1800) mV = 1800;
        if (mV > 3300) mV = 3300;
        uint8_t code = (mV - 1800) / 100;
        return _core.updateBits(axp192_regs::pwr::LDO23OUT_VOL, 0xF0, code << 4) >= 0;
    }
    case CH_LDO3: {
        if (mV < 1800) mV = 1800;
        if (mV > 3300) mV = 3300;
        uint8_t code = (mV - 1800) / 100;
        return _core.updateBits(axp192_regs::pwr::LDO23OUT_VOL, 0x0F, code) >= 0;
    }
    case CH_LDOio: {
        if (mV < 1800) mV = 1800;
        if (mV > 3300) mV = 3300;
        uint8_t code = (mV - 1800) / 100;
        return _core.updateBits(axp192_regs::gpio::GPIO0_VOL, 0xF0, code << 4) >= 0;
    }
    case CH_EXTEN:
        return false;
    default:
        return false;
    }
}

uint16_t AXP192Channel::getVoltage(uint8_t channel)
{
    if (channel >= CHANNEL_COUNT) return 0;

    switch (channel) {
    case CH_DCDC1: {
        int val = _core.readReg(axp192_regs::pwr::DC1_VOLTAGE);
        if (val < 0) return 0;
        return ((val & 0x7F) * 25) + 700;
    }
    case CH_DCDC2: {
        int val = _core.readReg(axp192_regs::pwr::DC2OUT_VOL);
        if (val < 0) return 0;
        return ((val & 0x3F) * 25) + 700;
    }
    case CH_DCDC3: {
        int val = _core.readReg(axp192_regs::pwr::DC3OUT_VOL);
        if (val < 0) return 0;
        return (val * 25) + 700;
    }
    case CH_LDO2: {
        int val = _core.readReg(axp192_regs::pwr::LDO23OUT_VOL);
        if (val < 0) return 0;
        uint8_t code = (val >> 4) & 0x0F;
        return (code * 100) + 1800;
    }
    case CH_LDO3: {
        int val = _core.readReg(axp192_regs::pwr::LDO23OUT_VOL);
        if (val < 0) return 0;
        uint8_t code = val & 0x0F;
        return (code * 100) + 1800;
    }
    case CH_LDOio: {
        int val = _core.readReg(axp192_regs::gpio::GPIO0_VOL);
        if (val < 0) return 0;
        uint8_t code = (val >> 4) & 0x0F;
        return (code * 100) + 1800;
    }
    case CH_EXTEN:
        return 0;
    default:
        return 0;
    }
}

uint8_t AXP192Channel::count() const
{
    return CHANNEL_COUNT;
}

PmicChannelBase::Info AXP192Channel::getInfo(uint8_t channel) const
{
    static const Info channelInfo[] = {
        {CH_DCDC1, Type::DCDC, 700,  3500, 25,  "DCDC1"},
        {CH_DCDC2, Type::DCDC, 700,  2275, 25,  "DCDC2"},
        {CH_DCDC3, Type::DCDC, 700,  3500, 25,  "DCDC3"},
        {CH_LDO2,  Type::LDO,  1800, 3300, 100, "LDO2"},
        {CH_LDO3,  Type::LDO,  1800, 3300, 100, "LDO3"},
        {CH_LDOio, Type::LDO,  1800, 3300, 100, "LDOio"},
        {CH_EXTEN, Type::LDO,  0,    0,    0,   "EXTEN"},
    };
    if (channel >= CHANNEL_COUNT) {
        return {0, Type::DCDC, 0, 0, 0, "UNKNOWN"};
    }
    return channelInfo[channel];
}

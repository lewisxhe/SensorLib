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
 * @file      AXP202Channel.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 */
#include "AXP202Channel.hpp"
#include "AXP202Regs.hpp"


const uint16_t AXP202Channel::ldo4Table_[16] = {
    1250, 1300, 1400, 1500, 1600, 1700, 1800, 1900,
    2000, 2500, 2700, 2800, 3000, 3100, 3200, 3300
};

AXP202Channel::AXP202Channel(AXP202Core &core) : _core(core)
{
}

bool AXP202Channel::enable(uint8_t channel, bool enable)
{
    if (channel >= CHANNEL_COUNT) return false;

    switch (channel) {
    case CH_DCDC2:
        return enable ? _core.setRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_DCDC2)
                      : _core.clrRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_DCDC2);
    case CH_DCDC3:
        return enable ? _core.setRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_DCDC3)
                      : _core.clrRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_DCDC3);
    case CH_LDO2:
        return enable ? _core.setRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_LDO2)
                      : _core.clrRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_LDO2);
    case CH_LDO3:
        return enable ? _core.setRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_LDO3)
                      : _core.clrRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_LDO3);
    case CH_LDO4:
        return enable ? _core.setRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_LDO4)
                      : _core.clrRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_LDO4);
    case CH_EXTEN:
        return enable ? _core.setRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_EXTEN)
                      : _core.clrRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_EXTEN);
    case CH_LDOio: {
        int val = _core.readReg(axp202_regs::gpio::GPIO0_CTL);
        if (val < 0) return false;
        if (enable) {
            val = (val & 0xF8) | 0x03;
        } else {
            val = (val & 0xF8);
        }
        return _core.writeReg(axp202_regs::gpio::GPIO0_CTL, static_cast<uint8_t>(val)) >= 0;
    }
    default:
        return false;
    }
}

bool AXP202Channel::isEnabled(uint8_t channel)
{
    if (channel >= CHANNEL_COUNT) return false;

    switch (channel) {
    case CH_DCDC2:
        return _core.getRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_DCDC2);
    case CH_DCDC3:
        return _core.getRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_DCDC3);
    case CH_LDO2:
        return _core.getRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_LDO2);
    case CH_LDO3:
        return _core.getRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_LDO3);
    case CH_LDO4:
        return _core.getRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_LDO4);
    case CH_EXTEN:
        return _core.getRegBit(axp202_regs::power::LDO234_DC23_CTL, axp202_regs::BIT_EXTEN);
    case CH_LDOio: {
        int val = _core.readReg(axp202_regs::gpio::GPIO0_CTL);
        if (val < 0) return false;
        return (val & 0x07) == 0x03;
    }
    default:
        return false;
    }
}

bool AXP202Channel::setVoltage(uint8_t channel, uint16_t mV)
{
    if (channel >= CHANNEL_COUNT) return false;

    switch (channel) {
    case CH_DCDC2: {
        if (mV < 700) mV = 700;
        if (mV > 2275) mV = 2275;
        uint8_t code = (mV - 700) / 25;
        return _core.updateBits(axp202_regs::power::DC2OUT_VOL, 0x3F, code) >= 0;
    }
    case CH_DCDC3: {
        if (mV < 700) mV = 700;
        if (mV > 3500) mV = 3500;
        uint8_t code = (mV - 700) / 25;
        return _core.updateBits(axp202_regs::power::DC3OUT_VOL, 0x7F, code) >= 0;
    }
    case CH_LDO2: {
        if (mV < 1800) mV = 1800;
        if (mV > 3300) mV = 3300;
        uint8_t code = (mV - 1800) / 100;
        return _core.updateBits(axp202_regs::power::LDO24OUT_VOL, 0xF0, code << 4) >= 0;
    }
    case CH_LDO3: {
        if (mV < 700) mV = 700;
        if (mV > 3500) mV = 3500;
        uint8_t code = (mV - 700) / 25;
        return _core.updateBits(axp202_regs::power::LDO3OUT_VOL, 0x7F, code) >= 0;
    }
    case CH_LDO4: {
        int index = 0;
        uint16_t bestDiff = static_cast<uint16_t>(ldo4Table_[0] > mV ? (ldo4Table_[0] - mV) : (mV - ldo4Table_[0]));
        for (int i = 0; i < 16; ++i) {
            uint16_t diff = static_cast<uint16_t>(ldo4Table_[i] > mV ? (ldo4Table_[i] - mV) : (mV - ldo4Table_[i]));
            if (diff < bestDiff) {
                bestDiff = diff;
                index = i;
            }
        }
        return _core.updateBits(axp202_regs::power::LDO24OUT_VOL, 0x0F, static_cast<uint8_t>(index)) >= 0;
    }
    case CH_LDOio: {
        if (mV < 1800) mV = 1800;
        if (mV > 3300) mV = 3300;
        uint8_t code = (mV - 1800) / 100;
        return _core.updateBits(axp202_regs::gpio::GPIO0_VOL, 0xF0, code << 4) >= 0;
    }
    case CH_EXTEN:
        return false;
    default:
        return false;
    }
}

uint16_t AXP202Channel::getVoltage(uint8_t channel)
{
    if (channel >= CHANNEL_COUNT) return 0;

    switch (channel) {
    case CH_DCDC2: {
        int val = _core.readReg(axp202_regs::power::DC2OUT_VOL);
        if (val < 0) return 0;
        return ((val & 0x3F) * 25) + 700;
    }
    case CH_DCDC3: {
        int val = _core.readReg(axp202_regs::power::DC3OUT_VOL);
        if (val < 0) return 0;
        return ((val & 0x7F) * 25) + 700;
    }
    case CH_LDO2: {
        int val = _core.readReg(axp202_regs::power::LDO24OUT_VOL);
        if (val < 0) return 0;
        uint8_t code = (val >> 4) & 0x0F;
        return (code * 100) + 1800;
    }
    case CH_LDO3: {
        int val = _core.readReg(axp202_regs::power::LDO3OUT_VOL);
        if (val < 0) return 0;
        if (val & 0x80) {
            return 0;
        }
        return ((val & 0x7F) * 25) + 700;
    }
    case CH_LDO4: {
        int val = _core.readReg(axp202_regs::power::LDO24OUT_VOL);
        if (val < 0) return 0;
        uint8_t idx = val & 0x0F;
        if (idx >= 16) return 0;
        return ldo4Table_[idx];
    }
    case CH_LDOio: {
        int val = _core.readReg(axp202_regs::gpio::GPIO0_VOL);
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

uint8_t AXP202Channel::count() const
{
    return CHANNEL_COUNT;
}

PmicChannelBase::Info AXP202Channel::getInfo(uint8_t channel) const
{
    static const Info channelInfo[] = {
        {CH_DCDC2, Type::DCDC,  700,  2275, 25,  "DCDC2"},
        {CH_DCDC3, Type::DCDC,  700,  3500, 25,  "DCDC3"},
        {CH_LDO2,  Type::LDO,   1800, 3300, 100, "LDO2"},
        {CH_LDO3,  Type::LDO,   700,  3500, 25,  "LDO3"},
        {CH_LDO4,  Type::LDO,   1250, 3300, 0,   "LDO4"},
        {CH_LDOio, Type::LDO,   1800, 3300, 100, "LDOio"},
        {CH_EXTEN, Type::LDO,   0,    0,    0,   "EXTEN"},
    };
    if (channel >= CHANNEL_COUNT) {
        return {0, Type::DCDC, 0, 0, 0, "UNKNOWN"};
    }
    return channelInfo[channel];
}

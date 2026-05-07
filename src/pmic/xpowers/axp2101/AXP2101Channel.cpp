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
 * @file      AXP2101Channel.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Power Output Channel Control
 *
 */
#include "AXP2101Channel.hpp"
#include "AXP2101Regs.hpp"

AXP2101Channel::AXP2101Channel(AXP2101Core &core) : _core(core)
{
}

bool AXP2101Channel::enable(uint8_t channel, bool enable)
{
    if (channel >= CHANNEL_COUNT) return false;

    if (isDCDC(channel)) {
        return enable ? _core.setRegBit(axp2101_regs::dcdc::ONOFF_DVM_CTRL, channel)
                      : _core.clrRegBit(axp2101_regs::dcdc::ONOFF_DVM_CTRL, channel);
    }

    uint8_t ldoIdx = channel - DCDC_COUNT;
    uint8_t reg = (ldoIdx <= 7) ? axp2101_regs::ldo::ONOFF_CTRL0 : axp2101_regs::ldo::ONOFF_CTRL1;
    uint8_t bit = (ldoIdx <= 7) ? ldoIdx : 0;
    return enable ? _core.setRegBit(reg, bit) : _core.clrRegBit(reg, bit);
}

bool AXP2101Channel::isEnabled(uint8_t channel)
{
    if (channel >= CHANNEL_COUNT) return false;

    if (isDCDC(channel)) {
        return _core.getRegBit(axp2101_regs::dcdc::ONOFF_DVM_CTRL, channel);
    }

    uint8_t ldoIdx = channel - DCDC_COUNT;
    uint8_t reg = (ldoIdx <= 7) ? axp2101_regs::ldo::ONOFF_CTRL0 : axp2101_regs::ldo::ONOFF_CTRL1;
    uint8_t bit = (ldoIdx <= 7) ? ldoIdx : 0;
    return _core.getRegBit(reg, bit);
}

bool AXP2101Channel::setVoltage(uint8_t channel, uint16_t mV)
{
    if (channel >= CHANNEL_COUNT) return false;

    static const uint8_t dcdcRegs[] = {
        axp2101_regs::dcdc::VOL0_CTRL, axp2101_regs::dcdc::VOL1_CTRL, axp2101_regs::dcdc::VOL2_CTRL,
        axp2101_regs::dcdc::VOL3_CTRL, axp2101_regs::dcdc::VOL4_CTRL
    };
    static const uint8_t ldoRegs[] = {
        axp2101_regs::ldo::VOL0_CTRL, axp2101_regs::ldo::VOL1_CTRL, axp2101_regs::ldo::VOL2_CTRL,
        axp2101_regs::ldo::VOL3_CTRL, axp2101_regs::ldo::VOL4_CTRL, axp2101_regs::ldo::VOL5_CTRL,
        axp2101_regs::ldo::VOL6_CTRL, axp2101_regs::ldo::VOL7_CTRL, axp2101_regs::ldo::VOL8_CTRL
    };

    if (isDCDC(channel)) {
        uint8_t reg = dcdcRegs[channel];
        switch (channel) {
        case CH_DCDC1: {
            if (mV < 1500) mV = 1500;
            if (mV > 3400) mV = 3400;
            return _core.updateBits(reg, 0x1F, (mV - 1500) / 100) >= 0;
        }
        case CH_DCDC2: {
            if (mV >= 500 && mV <= 1200) {
                return _core.updateBits(reg, 0x7F, (mV - 500) / 10) >= 0;
            } else if (mV >= 1220 && mV <= 1540) {
                return _core.updateBits(reg, 0x7F, (mV - 1220) / 20 + 71) >= 0;
            }
            return false;
        }
        case CH_DCDC3: {
            if (mV >= 500 && mV <= 1200) {
                return _core.updateBits(reg, 0x7F, (mV - 500) / 10) >= 0;
            } else if (mV >= 1220 && mV <= 1540) {
                return _core.updateBits(reg, 0x7F, (mV - 1220) / 20 + 71) >= 0;
            } else if (mV >= 1600 && mV <= 3400) {
                return _core.updateBits(reg, 0x7F, (mV - 1600) / 100 + 88) >= 0;
            }
            return false;
        }
        case CH_DCDC4: {
            if (mV >= 500 && mV <= 1200) {
                return _core.updateBits(reg, 0x7F, (mV - 500) / 10) >= 0;
            } else if (mV >= 1220 && mV <= 1840) {
                return _core.updateBits(reg, 0x7F, (mV - 1220) / 20 + 71) >= 0;
            }
            return false;
        }
        case CH_DCDC5: {
            if (mV == 1200) {
                return _core.updateBits(reg, 0x1F, 0x19) >= 0;
            }
            if (mV < 1400) mV = 1400;
            if (mV > 3700) mV = 3700;
            return _core.updateBits(reg, 0x1F, (mV - 1400) / 100) >= 0;
        }
        }
        return false;
    }

    uint8_t ldoIdx = channel - DCDC_COUNT;
    uint8_t reg = ldoRegs[ldoIdx];

    // ALDO1-4 (ldoIdx 0-3) and BLDO1-2 (ldoIdx 4-5): 500-3500mV, 100mV steps
    if (ldoIdx <= 5) {
        if (mV < 500) mV = 500;
        if (mV > 3500) mV = 3500;
        return _core.updateBits(reg, 0x1F, (mV - 500) / 100) >= 0;
    }

    if (ldoIdx == 6) {
        if (mV < 500) mV = 500;
        if (mV > 1400) mV = 1400;
        return _core.updateBits(reg, 0x1F, (mV - 500) / 50) >= 0;
    }

    if (ldoIdx == 7) { // DLDO1
        if (mV < 500) mV = 500;
        if (mV > 3300) mV = 3300;
        return _core.updateBits(reg, 0x1F, (mV - 500) / 100) >= 0;
    }

    if (ldoIdx == 8) {  // DLDO2: 0.5-1.4V, 50mV steps
        if (mV < 500) mV = 500;
        if (mV > 1400) mV = 1400;
        return _core.updateBits(reg, 0x1F, (mV - 500) / 50) >= 0;
    }

    return false;
}

uint16_t AXP2101Channel::getVoltage(uint8_t channel)
{
    if (channel >= CHANNEL_COUNT) return 0;

    static const uint8_t dcdcRegs[] = {
        axp2101_regs::dcdc::VOL0_CTRL, axp2101_regs::dcdc::VOL1_CTRL, axp2101_regs::dcdc::VOL2_CTRL,
        axp2101_regs::dcdc::VOL3_CTRL, axp2101_regs::dcdc::VOL4_CTRL
    };
    static const uint8_t ldoRegs[] = {
        axp2101_regs::ldo::VOL0_CTRL, axp2101_regs::ldo::VOL1_CTRL, axp2101_regs::ldo::VOL2_CTRL,
        axp2101_regs::ldo::VOL3_CTRL, axp2101_regs::ldo::VOL4_CTRL, axp2101_regs::ldo::VOL5_CTRL,
        axp2101_regs::ldo::VOL6_CTRL, axp2101_regs::ldo::VOL7_CTRL, axp2101_regs::ldo::VOL8_CTRL
    };

    if (isDCDC(channel)) {
        int val = _core.readReg(dcdcRegs[channel]);
        if (val < 0) return 0;
        val &= 0x7F;
        switch (channel) {
        case CH_DCDC1:
            return (val & 0x1F) * 100 + 1500;
        case CH_DCDC2:
            if (val < 71) return val * 10 + 500;
            return (val - 71) * 20 + 1220;
        case CH_DCDC3:
            if (val < 71) return val * 10 + 500;
            if (val < 88) return (val - 71) * 20 + 1220;
            return (val - 88) * 100 + 1600;
        case CH_DCDC4:
            if (val < 71) return val * 10 + 500;
            return (val - 71) * 20 + 1220;
        case CH_DCDC5:
            val &= 0x1F;
            if (val == 0x19) return 1200;
            return val * 100 + 1400;
        }
        return 0;
    }

    uint8_t ldoIdx = channel - DCDC_COUNT;
    int val = _core.readReg(ldoRegs[ldoIdx]);
    if (val < 0) return 0;
    uint8_t code = val & 0x1F;

    if (ldoIdx == 6) {
        return code * 50 + 500;
    }
    if (ldoIdx == 8) {
        // DLDO2: 500-1400mV, 50mV steps
        return code * 50 + 500;
    }
    return code * 100 + 500;
}

uint8_t AXP2101Channel::count() const
{
    return CHANNEL_COUNT;
}

PmicChannelBase::Info AXP2101Channel::getInfo(uint8_t channel) const
{
    static const Info channelInfo[] = {
        {CH_DCDC1,   Type::DCDC, 1500, 3400, 100, "DCDC1"},
        {CH_DCDC2,   Type::DCDC, 500,  1540, 10,  "DCDC2"},
        {CH_DCDC3,   Type::DCDC, 500,  3400, 10,  "DCDC3"},
        {CH_DCDC4,   Type::DCDC, 500,  1840, 10,  "DCDC4"},
        {CH_DCDC5,   Type::DCDC, 1200, 3700, 100, "DCDC5"},
        {CH_ALDO1,   Type::LDO,  500,  3500, 100, "ALDO1"},
        {CH_ALDO2,   Type::LDO,  500,  3500, 100, "ALDO2"},
        {CH_ALDO3,   Type::LDO,  500,  3500, 100, "ALDO3"},
        {CH_ALDO4,   Type::LDO,  500,  3500, 100, "ALDO4"},
        {CH_BLDO1,   Type::LDO,  500,  3500, 100, "BLDO1"},
        {CH_BLDO2,   Type::LDO,  500,  3500, 100, "BLDO2"},
        {CH_CPUSLDO, Type::LDO,  500,  1400, 50,  "CPUSLDO"},
        {CH_DLDO1,   Type::LDO,  500,  3300, 100, "DLDO1"},
        {CH_DLDO2,   Type::LDO,  500,  1400, 50,  "DLDO2"},
    };
    if (channel >= CHANNEL_COUNT) {
        return {0, Type::DCDC, 0, 0, 0, "UNKNOWN"};
    }
    return channelInfo[channel];
}

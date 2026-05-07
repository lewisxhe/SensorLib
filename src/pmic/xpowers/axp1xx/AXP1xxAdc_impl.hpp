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
 * @file      AXP1xxAdc_impl.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-28
 *
 */
#pragma once

#include "AXP1xxAdc.hpp"

namespace axp1xx {

template<typename CoreType, typename RegTraits>
bool AXP1xxAdc<CoreType, RegTraits>::enableChannels(uint32_t mask)
{
    uint8_t en1 = mask & 0xFF;
    uint8_t en2 = (mask >> 8) & 0xFF;

    // Update ADC_EN1 register if mask has bits in lower byte
    if (en1 || (!en2 && mask)) {  // Always try EN1 if any mask bits present
        int val = _core.readReg(RegTraits::ADC_EN1_ADDR);
        if (val < 0) return false;
        val |= en1;
        if (_core.writeReg(RegTraits::ADC_EN1_ADDR, static_cast<uint8_t>(val)) < 0) {
            return false;
        }
    }

    // Update ADC_EN2 register if mask has bits in upper byte
    if (en2) {
        int val = _core.readReg(RegTraits::ADC_EN2_ADDR);
        if (val < 0) return false;
        val |= en2;
        if (_core.writeReg(RegTraits::ADC_EN2_ADDR, static_cast<uint8_t>(val)) < 0) {
            return false;
        }
    }

    // Handle special enablement requirements (e.g., TS pin on AXP2101)
    for (int i = 0; i < 10; ++i) {
        PmicAdcBase::Channel ch = static_cast<PmicAdcBase::Channel>(i);
        uint32_t chMask = RegTraits::getChannelMask(ch);
        if ((mask & chMask) && RegTraits::hasSpecialEnablement(ch)) {
            if (!RegTraits::enableSpecialChannel(_core, ch)) {
                return false;
            }
        }
    }

    return true;
}

template<typename CoreType, typename RegTraits>
bool AXP1xxAdc<CoreType, RegTraits>::disableChannels(uint32_t mask)
{
    uint8_t en1 = mask & 0xFF;
    uint8_t en2 = (mask >> 8) & 0xFF;

    // Update ADC_EN1 register
    if (en1 || (!en2 && mask)) {
        int val = _core.readReg(RegTraits::ADC_EN1_ADDR);
        if (val < 0) return false;
        val &= ~en1;
        if (_core.writeReg(RegTraits::ADC_EN1_ADDR, static_cast<uint8_t>(val)) < 0) {
            return false;
        }
    }

    // Update ADC_EN2 register
    if (en2) {
        int val = _core.readReg(RegTraits::ADC_EN2_ADDR);
        if (val < 0) return false;
        val &= ~en2;
        if (_core.writeReg(RegTraits::ADC_EN2_ADDR, static_cast<uint8_t>(val)) < 0) {
            return false;
        }
    }

    // Handle special disablement requirements
    for (int i = 0; i < 10; ++i) {
        PmicAdcBase::Channel ch = static_cast<PmicAdcBase::Channel>(i);
        uint32_t chMask = RegTraits::getChannelMask(ch);
        if ((mask & chMask) && RegTraits::hasSpecialEnablement(ch)) {
            if (!RegTraits::disableSpecialChannel(_core, ch)) {
                return false;
            }
        }
    }

    return true;
}

template<typename CoreType, typename RegTraits>
uint16_t AXP1xxAdc<CoreType, RegTraits>::readH8L4(uint8_t regH, uint8_t regL)
{
    int h = _core.readReg(regH);
    int l = _core.readReg(regL);
    if (h < 0 || l < 0) return 0;
    return (static_cast<uint16_t>(h) << 4) | (l & 0x0F);
}

template<typename CoreType, typename RegTraits>
uint16_t AXP1xxAdc<CoreType, RegTraits>::readH8L5(uint8_t regH, uint8_t regL)
{
    int h = _core.readReg(regH);
    int l = _core.readReg(regL);
    if (h < 0 || l < 0) return 0;
    return (static_cast<uint16_t>(h) << 5) | (l & 0x1F);
}

template<typename CoreType, typename RegTraits>
bool AXP1xxAdc<CoreType, RegTraits>::read(PmicAdcBase::Channel ch, float &out)
{
    out = NAN;
    
    // Delegate channel-specific read logic to RegTraits
    return RegTraits::readChannel(_core, ch, out);
}

} // namespace axp1xx

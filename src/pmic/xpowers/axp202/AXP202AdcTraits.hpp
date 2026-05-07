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
 * @file      AXP202AdcTraits.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-05-02
 *
 * @brief AXP202-specific ADC traits for template specialization
 *
 * Provides AXP202 register addresses, channel mappings, and ADC reading logic
 * for use with the AXP1xxAdc template class.
 *
 * @note AXP202 has more channels than AXP192 (includes GPIO0, GPIO1, BAT_PERCENTAGE)
 *       and uses the same register layout but with extended support.
 *
 * @see AXP1xxAdc.hpp, AXP202Regs.hpp, AXP202Core.hpp
 */
#pragma once

#include "AXP202Core.hpp"
#include "AXP202Regs.hpp"
#include "../axp1xx/AXP1xxAdcTraits.hpp"

namespace axp1xx {

/**
 * @brief AXP202 ADC traits specialization
 *
 * Implements the AdcTraitsBase interface for AXP202, providing:
 * - Register addresses for ADC control
 * - Channel mask definitions (includes GPIO0, GPIO1, BAT_PERCENTAGE)
 * - ADC reading logic for all supported channels
 *
 * @note AXP202 uses a hybrid readRegBuff approach for some channels
 *       and supports additional GPIO/percentage channels.
 */
template<>
class AdcTraitsBase<AXP202Core>
{
public:
    static constexpr uint8_t ADC_EN1_ADDR = axp202_regs::adc::ADC_EN1;
    static constexpr uint8_t ADC_EN2_ADDR = axp202_regs::adc::ADC_EN2;

    /**
     * @brief Map PmicAdcBase::Channel to AXP202 ADC enable mask
     */
    static uint32_t getChannelMask(PmicAdcBase::Channel ch);

    /**
     * @brief AXP202 does not require special channel enablement
     */
    static bool hasSpecialEnablement(PmicAdcBase::Channel ch)
    {
        (void)ch;
        return false;
    }

    /**
     * @brief No special enablement needed for AXP202
     */
    static bool enableSpecialChannel(AXP202Core &core, PmicAdcBase::Channel ch)
    {
        (void)core;
        (void)ch;
        return true;
    }

    /**
     * @brief No special disablement needed for AXP202
     */
    static bool disableSpecialChannel(AXP202Core &core, PmicAdcBase::Channel ch)
    {
        (void)core;
        (void)ch;
        return true;
    }

    /**
     * @brief Read a single ADC channel value for AXP202
     */
    static bool readChannel(AXP202Core &core, PmicAdcBase::Channel ch, float &out);

private:
    /**
     * @brief Helper to read 12-bit ADC value from H8L4 register pair using readRegBuff
     */
    static uint16_t readRegisterH8L4(AXP202Core &core, uint8_t regH, uint8_t regL);
};

} // namespace axp1xx

// Forward include AXP202Regs for the implementation
#include "AXP202Regs.hpp"

// Include inline implementation
#include "AXP202AdcTraits_impl.hpp"

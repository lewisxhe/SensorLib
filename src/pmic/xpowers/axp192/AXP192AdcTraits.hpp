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
 * @file      AXP192AdcTraits.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP192-specific ADC traits for template specialization
 *
 * Provides AXP192 register addresses, channel mappings, and ADC reading logic
 * for use with the AXP1xxAdc template class.
 *
 * @see AXP1xxAdc.hpp, AXP192Regs.hpp, AXP192Core.hpp
 */
#pragma once

#include "AXP192Core.hpp"
#include "AXP192Regs.hpp"
#include "../axp1xx/AXP1xxAdcTraits.hpp"

namespace axp1xx {

/**
 * @brief AXP192 ADC traits specialization
 *
 * Implements the AdcTraitsBase interface for AXP192, providing:
 * - Register addresses for ADC control
 * - Channel mask definitions
 * - ADC reading logic for all supported channels
 *
 * @note AXP192 supports 9 logical ADC channels (VBUS, BAT, APS, TS, TEMP, ACIN, etc.)
 *       with 12-bit H8L4 and 13-bit H8L5 register formats.
 */
template<>
class AdcTraitsBase<AXP192Core>
{
public:
    static constexpr uint8_t ADC_EN1_ADDR = axp192_regs::adc::ADC_EN1;
    static constexpr uint8_t ADC_EN2_ADDR = axp192_regs::adc::ADC_EN2;

    /**
     * @brief Map PmicAdcBase::Channel to AXP192 ADC enable mask
     *
     * Returns a bitmask to enable the corresponding ADC channel.
     * Lower 8 bits go to ADC_EN1, upper 8 bits to ADC_EN2.
     */
    static uint32_t getChannelMask(PmicAdcBase::Channel ch);

    /**
     * @brief AXP192 does not require special channel enablement
     */
    static bool hasSpecialEnablement(PmicAdcBase::Channel ch)
    {
        (void)ch;
        return false;
    }

    /**
     * @brief No special enablement needed for AXP192
     */
    static bool enableSpecialChannel(AXP192Core &core, PmicAdcBase::Channel ch)
    {
        (void)core;
        (void)ch;
        return true;
    }

    /**
     * @brief No special disablement needed for AXP192
     */
    static bool disableSpecialChannel(AXP192Core &core, PmicAdcBase::Channel ch)
    {
        (void)core;
        (void)ch;
        return true;
    }

    /**
     * @brief Read a single ADC channel value for AXP192
     *
     * Implements channel-specific register reads, device state checks,
     * and ADC value conversion using the AXP192's conversion factors.
     *
     * @param core Reference to AXP192Core
     * @param ch Channel to read
     * @param out Reference to output float
     * @return true on success
     */
    static bool readChannel(AXP192Core &core, PmicAdcBase::Channel ch, float &out);

private:
    /**
     * @brief Helper to read 12-bit ADC value from H8L4 register pair
     */
    static uint16_t readH8L4(AXP192Core &core, uint8_t regH, uint8_t regL);

    /**
     * @brief Helper to read 13-bit ADC value from H8L5 register pair
     */
    static uint16_t readH8L5(AXP192Core &core, uint8_t regH, uint8_t regL);
};

} // namespace axp1xx

// Include inline implementation
#include "AXP192AdcTraits_impl.hpp"

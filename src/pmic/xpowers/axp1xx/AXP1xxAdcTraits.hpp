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
 * @file      AXP1xxAdcTraits.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-28
 *
 */
#pragma once

#include <stdint.h>
#include "../../PmicAdcBase.hpp"

namespace axp1xx {

/**
 * @brief Base traits class for AXP1xxAdc template specialization
 *
 * Specializations of this class must provide:
 * 1. Static register addresses (ADC_EN1_ADDR, ADC_EN2_ADDR)
 * 2. Channel mask mapping function (getChannelMask)
 * 3. Special enablement/disablement handlers (for TS pin, etc.)
 * 4. Main channel reading logic (readChannel)
 *
 * The design uses static polymorphism (templates) rather than virtual functions
 * to avoid vtable overhead and enable compiler optimizations.
 *
 * @tparam CoreType The I2C communication core type (unused in base, required for specializations)
 */
template<typename CoreType>
class AdcTraitsBase
{
public:
    /**
     * @brief ADC_EN1 register address
     *
     * This is the primary ADC enable register, typically containing
     * channel enable bits for 0-7 (VBUS, VBUS_CUR, APS, TS, BAT, ACIN, etc.)
     */
    static constexpr uint8_t ADC_EN1_ADDR = 0x82;

    /**
     * @brief ADC_EN2 register address
     *
     * Optional secondary ADC enable register for additional channels.
     * Set to same value as ADC_EN1_ADDR if not used.
     */
    static constexpr uint8_t ADC_EN2_ADDR = 0x83;

    /**
     * @brief Map a Channel enum to its hardware enable mask
     *
     * Returns the bitmask(s) that must be set in the ADC enable registers
     * to activate the given channel. The mask is split into:
     * - Lower 8 bits: bits for ADC_EN1 register
     * - Upper 8 bits: bits for ADC_EN2 register
     *
     * @param ch Channel from PmicAdcBase::Channel enum
     * @return Enable mask, or 0 if channel is not supported
     */
    static uint32_t getChannelMask(PmicAdcBase::Channel ch)
    {
        (void)ch;
        return 0;  // Default: no channels supported
    }

    /**
     * @brief Check if a channel requires special enablement logic
     *
     * Some channels need custom handling outside the standard
     * ADC enable register writes. For example, AXP2101's TS pin
     * requires configuration in a separate TS_PIN_CTRL register.
     *
     * @param ch The channel to check
     * @return true if enableSpecialChannel() must be called
     */
    static bool hasSpecialEnablement(PmicAdcBase::Channel ch)
    {
        (void)ch;
        return false;  // Default: no special handling
    }

    /**
     * @brief Perform special channel enablement
     *
     * Called by AXP1xxAdc::enableChannels() for channels where
     * hasSpecialEnablement() returns true. Implementations should
     * perform chip-specific register writes outside the standard
     * ADC enable procedure.
     *
     * Default implementation is a no-op.
     *
     * @param core Reference to I2C communication core
     * @param ch The channel to enable
     * @return true on success
     */
    static bool enableSpecialChannel(CoreType &core, PmicAdcBase::Channel ch)
    {
        (void)core;
        (void)ch;
        return true;
    }

    /**
     * @brief Perform special channel disablement
     *
     * Called by AXP1xxAdc::disableChannels() for channels where
     * hasSpecialEnablement() returns true.
     *
     * @param core Reference to I2C communication core
     * @param ch The channel to disable
     * @return true on success
     */
    static bool disableSpecialChannel(CoreType &core, PmicAdcBase::Channel ch)
    {
        (void)core;
        (void)ch;
        return true;
    }

    /**
     * @brief Main ADC channel reading implementation
     *
     * This is the workhorse method that handles all channel-specific
     * register reads, conversions, and state checks. Must be specialized
     * for each PMIC variant.
     *
     * Implementations should:
     * 1. Check device state (VBUS present, battery present, charging, etc.)
     * 2. Read ADC registers (using readH8L4, readH8L5 helpers)
     * 3. Apply conversion factors
     * 4. Return true on success, false on error
     *
     * @param core Reference to I2C communication core
     * @param ch Channel to read
     * @param out Reference to output float
     * @return true on success
     */
    static bool readChannel(CoreType &core, PmicAdcBase::Channel ch, float &out)
    {
        (void)core;
        (void)ch;
        (void)out;
        return false;  // Default: unsupported
    }
};

} // namespace axp1xx

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
 * @file      PmicAdcBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-13
 *
 * @brief PMIC analog data interface.
 *
 * This interface standardizes the most common ADC controls across PMICs.
 *
 * Design notes:
 *  - Channel is a bitmask enum. Common channels can be combined with |.
 *  - enableChannels(uint32_t) is the virtual interface. Subclasses accept
 *    both Channel values AND chip-specific masks (e.g. AXP202 GPIO ADC).
 *  - Not all chips support all channels; unsupported channels return false.
 */
#pragma once

#include <stdint.h>
#include <math.h>

class PmicAdcBase
{
public:
    /**
     * @brief ADC channel bitmask.
     *
     * Each common channel occupies a unique bit position so they can be
     * combined with bitwise OR for batch enable/disable operations.
     *
     * These values are passed directly to enableChannels(uint32_t).
     * Subclasses may define additional chip-specific mask values beyond
     * what is defined here (e.g. GPIO ADC channels).
     *
     * @note Values 0-9 are reserved for common channels.
     *       Subclasses may use higher bits for chip-specific channels.
     */
    enum class Channel : uint16_t {
        VBUS_VOLTAGE    = (1 << 0),   ///< VBUS voltage
        VBUS_CURRENT    = (1 << 1),   ///< VBUS current
        BAT_VOLTAGE     = (1 << 2),   ///< Battery voltage
        BAT_CURRENT     = (1 << 3),   ///< Battery current
        VSYS_VOLTAGE    = (1 << 4),   ///< VSYS voltage
        DIE_TEMPERATURE = (1 << 5),   ///< Die temperature
        BAT_TEMPERATURE = (1 << 6),   ///< Battery temperature
        BAT_PERCENTAGE  = (1 << 7),   ///< Battery percentage
        ACIN_VOLTAGE    = (1 << 8),   ///< ACIN voltage
        ACIN_CURRENT    = (1 << 9),   ///< ACIN current
    };

    virtual ~PmicAdcBase() = default;

    /**
     * @brief Enable one or more ADC channels by Channel enum.
     *
     * Convenience overload that accepts Channel values directly
     * without explicit cast. Delegates to enableChannels(uint32_t).
     *
     * @param mask Bitwise OR of Channel values to enable.
     * @return True on success, false on failure.
     */
    bool enableChannels(Channel mask)
    {
        return enableChannels(static_cast<uint32_t>(mask));
    }

    /**
     * @brief Disable one or more ADC channels by Channel enum.
     * @param mask Bitwise OR of Channel values to disable.
     * @return True on success, false on failure.
     */
    bool disableChannels(Channel mask)
    {
        return disableChannels(static_cast<uint32_t>(mask));
    }

    /**
     * @brief Enable one or more ADC channels.
     *
     * Accepts Channel bitmask values. Common channels (Channel::XXX) can be
     * combined with bitwise OR. Subclasses may also accept chip-specific
     * mask values for channels not in the Channel enum.
     *
     * @param mask Bitmask of channels to enable.
     * @return True on success, false on failure.
     */
    virtual bool enableChannels(uint32_t mask) = 0;

    /**
     * @brief Disable one or more ADC channels.
     * @param mask Bitmask of channels to disable.
     * @return True on success, false on failure.
     */
    virtual bool disableChannels(uint32_t mask) = 0;

    /**
     * @brief Read a single ADC channel value.
     * @param ch Single channel to read (must be a single bit, not a combination).
     * @param out Output variable for the channel value in physical units.
     * @return True on success, false on failure.
     */
    virtual bool read(Channel ch, float &out) = 0;

protected:
    /**
     * @brief Helper to iterate over set bits in a bitmask.
     *
     * Usage in subclass implementations:
     * @code
     *   forEachBit(mask, [](uint32_t bit) {
     *       // handle each individual bit
     *   });
     * @endcode
     *
     * @param mask Bitmask to iterate
     * @param fn Callback invoked for each set bit
     */
    template<typename Func>
    static void forEachChannel(uint32_t mask, Func fn)
    {
        while (mask) {
            uint32_t bit = mask & (-mask);  // isolate lowest set bit
            fn(bit);
            mask &= mask - 1;               // clear lowest set bit
        }
    }
};

/**
 * @brief Bitwise OR operator for combining Channel values.
 *
 * Enables: Channel::BAT_VOLTAGE | Channel::VBUS_VOLTAGE
 */
inline PmicAdcBase::Channel operator|(PmicAdcBase::Channel a, PmicAdcBase::Channel b)
{
    return static_cast<PmicAdcBase::Channel>(
        static_cast<uint16_t>(a) | static_cast<uint16_t>(b));
}

/**
 * @brief Bitwise OR assignment operator for Channel values.
 */
inline PmicAdcBase::Channel &operator|=(PmicAdcBase::Channel &a, PmicAdcBase::Channel b)
{
    a = a | b;
    return a;
}

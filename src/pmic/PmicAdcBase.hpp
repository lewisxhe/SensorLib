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
 * This interface standardizes the most common adc controls across PMICs.
 *
 * Notes:
 *  - Some PMICs do not support analog data; implementations
 *    should return false.
 */
#pragma once

#include <stdint.h>

class PmicAdcBase
{
public:
    enum class Channel : uint8_t {
        VBUS_VOLTAGE,       ///< VBUS voltage
        VBUS_CURRENT,       ///< VBUS current
        BAT_VOLTAGE,        ///< Battery voltage
        BAT_CURRENT,        ///< Battery current
        VSYS_VOLTAGE,       ///< VSYS voltage
        DIE_TEMPERATURE,    ///< Die temperature
        BAT_TEMPERATURE,    ///< Battery temperature
        BAT_PERCENTAGE,     ///< Battery percentage
    };

    virtual ~PmicAdcBase() = default;
    /**
     * @brief Enable ADC channels.
     * @param mask Bitmask of channels to enable.
     * @note The mask value is different for each chip and should be defined in the subclass.
     * @return True on success, false on failure.
     */
    virtual bool enableChannels(uint32_t mask) = 0;

    /**
     * @brief Disable ADC channels.
     * @param mask Bitmask of channels to disable.
     * @note The mask value is different for each chip and should be defined in the subclass.
     * @return True on success, false on failure.
     */
    virtual bool disableChannels(uint32_t mask) = 0;

    /**
     * @brief Read ADC channel value.
     * @param ch Channel to read.
     * @param out Output variable for the channel value.
     * @note Use float as the output data type uniformly.
     * @return True on success, false on failure.
     */
    virtual bool read(Channel ch, float &out) = 0;
};

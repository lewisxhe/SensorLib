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
 * @file      AXP1xxAdc.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-28
 *
 * @brief Generic ADC Template for AXP1xx-based PMIC Drivers
 */
#pragma once

#include <stdint.h>
#include <math.h>
#include "../../PmicAdcBase.hpp"

namespace axp1xx {

/**
 * @brief Generic ADC implementation for AXP1xx-based PMICs
 *
 * Provides common ADC functionality with chip-specific behavior delegated
 * to a RegTraits template parameter. This eliminates code duplication while
 * maintaining full API compatibility with PmicAdcBase.
 *
 * @tparam CoreType I2C communication core (must have readReg/writeReg/readRegBuff methods)
 * @tparam RegTraits Chip-specific register and conversion factor configuration
 */
template<typename CoreType, typename RegTraits>
class AXP1xxAdc : public PmicAdcBase
{
public:
    /**
     * @brief Construct ADC interface
     * @param core Reference to the I2C communication core
     */
    explicit AXP1xxAdc(CoreType &core) : _core(core) {}

    ~AXP1xxAdc() = default;

    /**
     * @brief Enable one or more ADC channels
     *
     * Maps each Channel bit to chip-specific enable mask via RegTraits,
     * then sets the appropriate bits in ADC enable registers.
     * Supports single or combined channels:
     *   adc.enableChannels(Channel::BAT_VOLTAGE | Channel::VBUS_VOLTAGE);
     *
     * @param mask Bitwise OR of Channel values or chip-specific mask constants
     * @return true on success, false if channel not supported or I2C error
     */
    bool enableChannels(uint32_t mask) override;

    /**
     * @brief Disable one or more ADC channels
     *
     * @param mask Bitwise OR of Channel values or chip-specific mask constants
     * @return true on success, false if channel not supported or I2C error
     */
    bool disableChannels(uint32_t mask) override;

    /**
     * @brief Read a single ADC channel value
     *
     * Reads the raw ADC data from registers and converts to physical units
     * (mV for voltage, mA for current, degC for temperature). Implements
     * device-state checks (e.g., VBUS present) to return 0 when appropriate.
     *
     * @param ch The channel to read (from PmicAdcBase::Channel enum)
     * @param[out] out Output value in physical units
     * @return true on success, false on read error
     */
    bool read(Channel ch, float &out) override;

protected:
    CoreType &_core;  ///< Reference to I2C communication core

private:
    /**
     * @brief Read 12-bit ADC value from H8L4 register pair
     *
     * Reads two consecutive registers: 8 bits from high register,
     * lower 4 bits of low register, combined as: (H << 4) | (L & 0x0F)
     *
     * @param regH Register address for high 8 bits
     * @param regL Register address for low 4 bits
     * @return 12-bit unsigned value, or 0 on error
     */
    uint16_t readH8L4(uint8_t regH, uint8_t regL);

    /**
     * @brief Read 13-bit ADC value from H8L5 register pair
     *
     * Reads two consecutive registers: 8 bits from high register,
     * lower 5 bits of low register, combined as: (H << 5) | (L & 0x1F)
     *
     * Used for battery discharge current measurement on some chips.
     *
     * @param regH Register address for high 8 bits
     * @param regL Register address for low 5 bits
     * @return 13-bit unsigned value, or 0 on error
     */
    uint16_t readH8L5(uint8_t regH, uint8_t regL);
};

} // namespace axp1xx

#include "AXP1xxAdc_impl.hpp"

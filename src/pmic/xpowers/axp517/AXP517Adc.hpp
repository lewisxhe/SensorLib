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
 * @file      AXP517Adc.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 * @brief     ADC channel control and measurement interface for AXP517.
 */
#pragma once

#include "../../PmicAdcBase.hpp"
#include "AXP517Core.hpp"

/**
 * @brief AXP517 ADC driver.
 *
 * Provides PMIC ADC channel enable/disable control and converts raw AXP517 ADC
 * readings into millivolts, milliamps, or degrees Celsius according to the
 * selected PmicAdcBase::Channel.
 */
class AXP517Adc : public PmicAdcBase
{
public:
    /** @brief Hardware ADC enable bit for VBUS current. */
    static constexpr uint8_t ADC_VBUS_CURRENT_HW   = 0x80;

    /** @brief Hardware ADC enable bit for battery discharge current. */
    static constexpr uint8_t ADC_BAT_DISCHARGE_HW  = 0x40;

    /** @brief Hardware ADC enable bit for battery charge current. */
    static constexpr uint8_t ADC_BAT_CHARGE_HW     = 0x20;

    /** @brief Hardware ADC enable bit for die temperature. */
    static constexpr uint8_t ADC_DIE_TEMP_HW       = 0x10;

    /** @brief Hardware ADC enable bit for system voltage. */
    static constexpr uint8_t ADC_SYSTEM_VOLTAGE_HW = 0x08;

    /** @brief Hardware ADC enable bit for VBUS voltage. */
    static constexpr uint8_t ADC_VBUS_VOLTAGE_HW   = 0x04;

    /** @brief Hardware ADC enable bit for TS pin voltage. */
    static constexpr uint8_t ADC_TS_PIN_HW         = 0x02;

    /** @brief Hardware ADC enable bit for battery voltage. */
    static constexpr uint8_t ADC_BAT_VOLTAGE_HW    = 0x01;

    /**
     * @brief Construct an AXP517 ADC driver.
     * @param core Shared AXP517 register transport.
     */
    explicit AXP517Adc(AXP517Core &core);

    ~AXP517Adc() = default;

    using PmicAdcBase::enableChannels;
    using PmicAdcBase::disableChannels;

    /**
     * @brief  Enable one or more ADC channels.
     * @param  mask: Bitwise OR of Channel values to enable.
     * @retval true on success, false on register access failure.
     */
    bool enableChannels(uint32_t mask) override;

    /**
     * @brief  Disable one or more ADC channels.
     * @param  mask: Bitwise OR of Channel values to disable.
     * @retval true on success, false on register access failure.
     */
    bool disableChannels(uint32_t mask) override;

    /**
     * @brief  Read an ADC channel.
     * @note   This function reads the specified ADC channel.
     * @param  ch: The ADC channel to read.
     * @param  out: Receives the converted ADC value.
     * @retval true on success, false on invalid channel or register access failure.
     */
    bool read(Channel ch, float &out) override;

private:
    AXP517Core &_core; ///< Shared AXP517 register transport.
};

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
 * @file      AXP192Adc.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP192 ADC Interface
 *
 * Provides access to the AXP192's internal 12-bit ADC, which can measure
 * VBUS voltage/current, battery voltage/current, APS voltage, TS pin
 * temperature, internal die temperature, and GPIO pin voltages. Channels
 * are individually enabled via bitmask and read in real-world units
 * (mV, mA, degC).
 *
 * @note This is a thin wrapper around the template AXP1xxAdc class,
 *       parameterized with AXP192-specific register traits.
 *
 * @see AXP192Regs.hpp, AXP1xxAdc.hpp, AXP192AdcTraits.hpp
 */
#pragma once
#include "AXP192Core.hpp"
#include "../axp1xx/AXP1xxAdc.hpp"
#include "AXP192AdcTraits.hpp"

/**
 * @brief AXP192 ADC Interface (template wrapper)
 *
 * Instantiates the generic AXP1xxAdc template with AXP192Core and AXP192 traits.
 * Provides the same interface as before while benefiting from reduced code duplication.
 *
 * @deprecated This class is now a template wrapper. Use AXP1xxAdc<AXP192Core, AdcTraitsBase<AXP192Core>>
 *             directly if more flexibility is needed.
 */
class AXP192Adc : public axp1xx::AXP1xxAdc<AXP192Core, axp1xx::AdcTraitsBase<AXP192Core>>
{
public:
    // Hardware bitmask constants (for direct register control)
    static constexpr uint32_t ADC_VBUS_VOLTAGE_HW = 0x08;   // REG82H bit 3
    static constexpr uint32_t ADC_VBUS_CURRENT_HW = 0x04;   // REG82H bit 2
    static constexpr uint32_t ADC_ACIN_VOLTAGE_HW = 0x20;   // REG82H bit 5
    static constexpr uint32_t ADC_ACIN_CURRENT_HW = 0x10;   // REG82H bit 4
    static constexpr uint32_t ADC_BAT_VOLTAGE_HW  = 0x80;   // REG82H bit 7
    static constexpr uint32_t ADC_BAT_CURRENT_HW  = 0x40;   // REG82H bit 6
    static constexpr uint32_t ADC_APS_VOLTAGE_HW  = 0x02;   // REG82H bit 1
    static constexpr uint32_t ADC_TS_PIN_HW       = 0x01;   // REG82H bit 0
    static constexpr uint32_t ADC_TEMPERATURE_HW  = 0x8000; // REG83H bit 7

    /**
     * @brief Construct ADC interface
     * @param core Reference to AXP192 core communication object
     */
    explicit AXP192Adc(AXP192Core &core)
        : axp1xx::AXP1xxAdc<AXP192Core, axp1xx::AdcTraitsBase<AXP192Core>>(core) {}

    ~AXP192Adc() = default;
};

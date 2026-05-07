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
 * @file      AXP2101Adc.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 ADC Control
 *
 */
#pragma once
#include "../../PmicAdcBase.hpp"
#include "AXP2101Core.hpp"

/**
 * @brief ADC control for the AXP2101 PMIC.
 *
 * Provides access to the 14-bit SAR ADC for measuring battery voltage,
 * VBUS voltage, VSYS voltage, TS pin voltage (thermistor), die temperature,
 * and battery percentage via the fuel gauge.
 *
 * @section ADC Channels
 * | Channel           | Register  | Resolution | Unit   |
 * |-------------------|-----------|------------|--------|
 * | VBAT voltage      | 0x34-0x35 | 14-bit     | 1 mV   |
 * | VBUS voltage      | 0x38-0x39 | 14-bit     | 1 mV   |
 * | VSYS voltage      | 0x3A-0x3B | 14-bit     | 1 mV   |
 * | TS pin voltage    | 0x36-0x37 | 14-bit     | 0.5 mV |
 * | Die temperature   | 0x3C-0x3D | 14-bit     | formula|
 * | Battery percentage| 0xA4      | 8-bit      | 1 %    |
 */
class AXP2101Adc : public PmicAdcBase
{
public:
    /** @brief ADC channel enable bit for die temperature (REG 0x30 bit 4) */
    static constexpr uint8_t ADC_DIE_TEMP        = 0x10;
    /** @brief ADC channel enable bit for VSYS voltage (REG 0x30 bit 3) */
    static constexpr uint8_t ADC_SYSTEM_VOLTAGE  = 0x08;
    /** @brief ADC channel enable bit for VBUS voltage (REG 0x30 bit 2) */
    static constexpr uint8_t ADC_VBUS_VOLTAGE    = 0x04;
    /** @brief ADC channel enable bit for TS pin (REG 0x30 bit 1) */
    static constexpr uint8_t ADC_TS_PIN          = 0x02;
    /** @brief ADC channel enable bit for battery voltage (REG 0x30 bit 0) */
    static constexpr uint8_t ADC_BAT_VOLTAGE     = 0x01;

    explicit AXP2101Adc(AXP2101Core &core);
    ~AXP2101Adc() = default;

    /**
     * @brief Enable one or more ADC channels (REG 0x30).
     * @param mask Bitmask of ADC channel bits to enable. Use ADC_xxx constants.
     *             Only bits [5:0] are used; higher bits are ignored.
     * @return true on success.
     */
    bool enableChannels(uint32_t mask) override;

    /**
     * @brief Disable one or more ADC channels (REG 0x30).
     * @param mask Bitmask of ADC channel bits to disable. Use ADC_xxx constants.
     *             Only bits [5:0] are used; higher bits are ignored.
     * @return true on success.
     */
    bool disableChannels(uint32_t mask) override;

    /**
     * @brief Read an ADC channel value.
     *
     * For VBUS/BAT voltage channels, returns 0.0 if the source is not present.
     * VBUS_CURRENT and BAT_CURRENT are not supported (return 0.0).
     *
     * @param ch Channel to read.
     * @param[out] out Result value. Units depend on channel:
     *   - Voltage channels: millivolts (mV)
     *   - DIE_TEMPERATURE: degrees Celsius
     *   - BAT_TEMPERATURE: millivolts (raw TS pin voltage * 0.5)
     *   - BAT_PERCENTAGE: percentage (0-100)
     * @return true on success.
     */
    bool read(Channel ch, float &out) override;

    /**
     * @brief Read TS pin temperature using Steinhart-Hart equation.
     *
     * Converts the raw TS ADC voltage to temperature in Celsius using
     * the Steinhart-Hart thermistor equation:
     *   1/T = A + B*ln(R) + C*ln(R)^3
     * where R = V_ts / I_source (default 50uA).
     *
     * @param SteinhartA Coefficient A (default 1.126e-3 for 10k NTC).
     * @param SteinhartB Coefficient B (default 2.38e-4 for 10k NTC).
     * @param SteinhartC Coefficient C (default 8.5e-8 for 10k NTC).
     * @return Temperature in degrees Celsius. Returns 0.0f on error
     *         or if the ADC reading is saturated.
     */
    float getTsTemperature(float SteinhartA = 1.126e-3f,
                           float SteinhartB = 2.38e-4f,
                           float SteinhartC = 8.5e-8f);

private:
    AXP2101Core &_core;
};

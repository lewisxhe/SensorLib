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
 * @file      BQ25896Adc.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 * @brief BQ25896 ADC (Analog-to-Digital Converter) Module Interface
 *
 * This class provides access to the BQ25896's integrated 10-bit ADC for monitoring
 * various system parameters including battery voltage, system voltage, VBUS voltage,
 * charge current, and NTC (Negative Temperature Coefficient) thermistor readings.
 *
 * @section ADC Channels
 * The BQ25896 ADC supports the following measurement channels:
 * - VBUS_VOLTAGE: USB input voltage (2600-15300mV, 100mV resolution)
 * - BAT_VOLTAGE: Battery voltage (2304-4608mV, 20mV resolution)
 * - VSYS_VOLTAGE: System voltage (2304-3700mV, 20mV resolution)
 * - BAT_CURRENT: Charge current (0-6350mA, 50mA resolution, requires charging)
 * - BAT_TEMPERATURE: NTC thermistor (21%-80%, percentage, NOT temperature!)
 * - DIE_TEMPERATURE: Thermal status flag (0=normal, 1=thermal regulation active)
 *
 * @section Important Note About NTC Temperature
 * @warning The BAT_TEMPERATURE channel returns NTC percentage, NOT temperature!
 * 
 * The ADC reads the NTC thermistor voltage as a percentage of REGN (the internal
 * voltage reference used in the NTC divider circuit).
 * - Formula: NTC/REGN = 21% + [TSPCT] * 0.465%
 * - Range: 21% (cold) to 80% (hot)
 * 
 * This value cannot be directly converted to temperature without knowing:
 * 1. The NTC thermistor's Beta value (typically 3380, 3950, 4100, etc.)
 * 2. The pull-up resistor value in your specific battery pack circuit
 * 3. The REGN voltage which varies based on charger configuration
 * 
 * Different battery packs use different NTC specifications (e.g., 10kΩ at 25°C
 * with different Beta values), so no universal conversion is possible.
 * 
 * To convert to temperature, you would need to use the Steinhart-Hart equation
 * with parameters from your battery pack's specification sheet.
 *
 * @section ADC Operation
 * The ADC can operate in two modes:
 * - One-shot mode: Single conversion on demand (default)
 * - Continuous mode: Repeated conversions at 1Hz
 *
 * ADC conversions are controlled via REG_PWR_ONOFF (0x02):
 * - Bit 7 (CONV_START): Initiates ADC conversion
 * - Bit 6 (CONV_RATE): Sets continuous mode if set, one-shot if clear
 *
 *
 * @see BQ25896Adc.cpp for implementation details
 * @see BQ25896Regs.hpp for ADC register definitions and step values
 * @see BQ25896RegisterMap.md for detailed ADC specifications
 */
#pragma once

#include "../../PmicAdcBase.hpp"
#include "BQ25896Core.hpp"

class BQ25896Adc : public PmicAdcBase
{
public:
    /**
     * @brief ADC control flags
     * @{
     */
    static constexpr uint8_t ADC_CONV_START   = 0x80;  ///< Start ADC conversion
    static constexpr uint8_t ADC_CONV_RATE    = 0x40;  ///< Enable continuous conversion mode (1Hz)
    /** @} */

    /**
     * @brief Construct ADC interface
     * @param core Reference to BQ25896 core communication
     */
    explicit BQ25896Adc(BQ25896Core &core);

    ~BQ25896Adc() = default;

    /**
     * @brief Enable ADC channels for conversion
     *
     * Enables the ADC and optionally sets continuous conversion mode.
     * The ADC must be enabled before reading values.
     *
     * @param mask Bitmask of flags:
     *        - ADC_CONV_START (0x80): Start conversion (required)
     *        - ADC_CONV_RATE (0x40): Enable continuous mode (1Hz)
     * @return true on success, false on I2C error
     *
     * @note The mask is used to pass control flags, not channel selection.
     *       All ADC channels are always enabled in the BQ25896.
     *
     * @code
     * // One-shot mode - single conversion
     * adc.enableChannels(BQ25896Adc::ADC_CONV_START);
     * 
     * // Continuous mode - auto-update at 1Hz
     * adc.enableChannels(BQ25896Adc::ADC_CONV_START | BQ25896Adc::ADC_CONV_RATE);
     * @endcode
     *
     * @see disableChannels()
     * @see setContinuousMode()
     */
    bool enableChannels(uint32_t mask) override;

    /**
     * @brief Disable ADC channels
     *
     * Stops ADC conversions by clearing CONV_START and CONV_RATE bits.
     * This saves power when ADC readings are not needed.
     *
     * @param mask Parameter kept for API compatibility (not used)
     * @return true on success, false on I2C error
     */
    bool disableChannels(uint32_t mask) override;

    /**
     * @brief Read ADC value for specified channel
     *
     * Reads the raw ADC value from the appropriate register and converts it
     * to the physical value using the step values defined in BQ25896Regs.hpp.
     *
     * @param ch ADC channel to read
     * @param out Reference to store the converted value
     *        - Voltage channels (VBUS, BAT, VSYS): millivolts (mV)
     *        - Current channel (BAT_CURRENT): milliamps (mA)
     *        - Temperature channel (BAT_TEMPERATURE): percentage (21-80%) - NOT temperature!
     *        - DIE_TEMPERATURE: flag (0.0 = normal, 1.0 = thermal regulation active)
     * @return true on success, false on unsupported channel or I2C error
     *
     * @section Channel Details
     * - VBUS_VOLTAGE: Returns 0 if VBUS not present (requires isVbusPresent())
     * - BAT_VOLTAGE: Returns 0 if battery not connected (ADC value = 0)
     * - VSYS_VOLTAGE: Always returns valid value
     * - BAT_CURRENT: Returns 0 if not charging or VBAT < VBATSHORT
     * - BAT_TEMPERATURE: Returns NTC percentage (21-80%), see important note above!
     * - DIE_TEMPERATURE: Returns 1.0 if thermal regulation active, 0.0 otherwise
     *
     * @note For VBUS voltage, ensure VBUS is present first using isVbusPresent()
     *       or check that the returned value is non-zero.
     */
    bool read(Channel ch, float &out) override;

    /**
     * @brief Start a single ADC conversion
     *
     * Sets the CONV_START bit to trigger a one-shot ADC conversion.
     * This is automatically called by read() if needed, but can be called
     * explicitly to pre-trigger conversion before reading multiple values.
     *
     * @return true on success, false on I2C error
     *
     * @note For continuous readings, use setContinuousMode(true) instead.
     */
    bool startConversion();

    /**
     * @brief Enable or disable continuous ADC mode
     *
     * In continuous mode, the ADC performs conversions automatically at 1Hz.
     * This is useful for real-time monitoring applications where you need
     * the latest values available immediately when calling read().
     *
     * @param enable true to enable continuous mode, false for one-shot mode
     * @return true on success, false on I2C error
     *
     * @note Continuous mode consumes more power - disable when not needed.
     * @note When in continuous mode, CONV_START is automatically set during conversion.
     */
    bool setContinuousMode(bool enable);

private:
    BQ25896Core &_core;
};

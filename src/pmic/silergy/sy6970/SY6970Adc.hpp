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
 * @file      SY6970Adc.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 * @brief SY6970 ADC (Analog-to-Digital Converter) Module Interface
 *
 * This class provides access to the SY6970's integrated 12-bit ADC for monitoring
 * various system parameters including battery voltage, system voltage, VBUS voltage,
 * charge current, and NTC (Negative Temperature Coefficient) thermistor readings.
 *
 * @section ADC Channels
 * The SY6970 ADC supports the following measurement channels:
 * - VBAT_VOLTAGE: Battery voltage (2304-4608mV, 20mV resolution)
 * - VSYS_VOLTAGE: System voltage (2304-3700mV, 20mV resolution)
 * - VBUS_VOLTAGE: USB input voltage (2600-15400mV, 100mV resolution)
 * - BAT_CURRENT: Charge current (0-5056mA, 50mA resolution)
 * - BAT_TEMPERATURE: NTC thermistor (21-100%, 0.465% resolution)
 * - DIE_TEMPERATURE: Die temperature flag
 *
 * @section ADC Operation
 * The ADC can operate in two modes:
 * - One-shot mode: Single conversion on demand (default)
 * - Continuous mode: Repeated conversions at 1Hz or 8Hz
 *
 * ADC conversions are controlled via REG_PWR_ONOFF (0x02):
 * - Bit 7 (CONV_START): Initiates ADC conversion
 * - Bit 6 (CONV_RATE): Sets continuous mode if set, one-shot if clear
 *
 * @section Usage Example
 * @code
 * SY6970Adc &adc = pmic.adc();
 * float vbat;
 * adc.read(SY6970Adc::Channel::BAT_VOLTAGE, vbat);
 * Serial.print("VBAT: ");
 * Serial.print(vbat);
 * Serial.println(" mV");
 * @endcode
 *
 * @see SY6970Adc.cpp for implementation details
 * @see SY6970Regs.hpp for ADC register definitions and step values
 */
#pragma once

#include "../../PmicAdcBase.hpp"
#include "SY6970Core.hpp"

class SY6970Adc : public PmicAdcBase
{
public:
    /**
     * @brief ADC control flags
     * @{
     */
    static constexpr uint8_t ADC_CONV_START   = 0x80;  ///< Start ADC conversion
    static constexpr uint8_t ADC_CONV_RATE    = 0x40;   ///< Enable continuous conversion mode
    /** @} */

    /**
     * @brief Construct ADC interface
     * @param core Reference to SY6970 core communication
     */
    explicit SY6970Adc(SY6970Core &core);

    ~SY6970Adc() = default;

    /**
     * @brief Enable ADC channels for conversion
     *
     * Enables specified ADC channels. The ADC must be running to read values.
     * This method sets the CONV_START bit in REG_PWR_ONOFF to initiate conversions.
     *
     * @param mask Bitmask of channels to enable (from PmicAdcBase::Channel)
     * @return true on success, false on I2C error
     *
     * @note The mask parameter is retained for compatibility but currently enables
     *       the ADC conversion (start bit) rather than selecting specific channels.
     *       All ADC channels are always enabled in the SY6970.
     *
     * @see disableChannels()
     * @see startConversion()
     */
    bool enableChannels(uint32_t mask) override;

    /**
     * @brief Disable ADC channels
     *
     * Stops ADC conversions by clearing CONV_START and CONV_RATE bits.
     *
     * @param mask Bitmask of channels to disable (not used, for compatibility)
     * @return true on success, false on I2C error
     */
    bool disableChannels(uint32_t mask) override;

    /**
     * @brief Read ADC value for specified channel
     *
     * Reads the raw ADC value from the appropriate register and converts it
     * to the physical value using the step values defined in SY6970Regs.hpp.
     *
     * @param ch ADC channel to read
     * @param out Reference to store the converted value
     *        - Voltage channels: millivolts (mV)
     *        - Current channels: milliamps (mA)
     *        - Temperature: percentage (0-100%) or flag
     * @return true on success, false on I2C error or unsupported channel
     *
     * @section Channel Details
     * - VBUS_VOLTAGE: Returns 0 if VBUS not present
     * - BAT_VOLTAGE: Returns 0 if battery not connected (ADC value = 0)
     * - BAT_CURRENT: Returns 0 if not charging
     * - BAT_TEMPERATURE: NTC percentage (21-100%)
     * - DIE_TEMPERATURE: 1.0 if thermal warning, 0.0 otherwise
     */
    bool read(Channel ch, float &out) override;

    /**
     * @brief Start a single ADC conversion
     *
     * Sets the CONV_START bit to trigger a one-shot ADC conversion.
     * This is automatically called by read() if needed.
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
     * This is useful for real-time monitoring applications.
     *
     * @param enable true to enable continuous mode, false for one-shot mode
     * @return true on success, false on I2C error
     *
     * @note Continuous mode consumes more power - disable when not needed.
     */
    bool setContinuousMode(bool enable);

private:
    SY6970Core &_core;
};

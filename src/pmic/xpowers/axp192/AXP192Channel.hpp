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
 * @file      AXP192Channel.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP192 Power Output Channel Control
 *
 * Manages the AXP192's voltage regulator output channels: three DC-DC
 * converters (DCDC1/2/3), two low-dropout regulators (LDO2/3), one GPIO
 * configurable as an LDO (LDOio), and an external enable pin (EXTEN).
 *
 * @section Voltage Parameters
 * - DCDC1/2/3: 700-3500mV, 25mV steps
 * - LDO2/3:    1800-3300mV, 100mV steps
 * - LDOio:     1800-3300mV, 100mV steps (via GPIO0)
 * - EXTEN:     External enable pin, no voltage control
 *
 * @note LDOio shares the GPIO0 pin. When used as a voltage output, the
 *       GPIO must be configured for LDO mode via the AXP192Gpio class.
 *
 * @see AXP192Gpio.hpp
 * @see AXP192Regs.hpp (pwr struct)
 */
#pragma once
#include "AXP192Core.hpp"
#include "../../PmicChannelBase.hpp"

/**
 * @brief AXP192 Power Output Channel Control
 *
 * Provides enable/disable, voltage get/set, and channel enumeration for
 * all 7 power output channels (3 DCDC + 4 LDO-type). Voltage setters
 * automatically quantize to the nearest valid step. Inherits from
 * PmicChannelBase for polymorphic PMIC integration.
 */
class AXP192Channel : public PmicChannelBase
{
public:
    static constexpr uint8_t DCDC_COUNT = 3;
    static constexpr uint8_t LDO_COUNT  = 4;
    static constexpr uint8_t CHANNEL_COUNT = 7;

    static constexpr uint8_t CH_DCDC1 = 0;
    static constexpr uint8_t CH_DCDC2 = 1;
    static constexpr uint8_t CH_DCDC3 = 2;
    static constexpr uint8_t CH_LDO2  = 3;
    static constexpr uint8_t CH_LDO3  = 4;
    static constexpr uint8_t CH_LDOio = 5;
    static constexpr uint8_t CH_EXTEN = 6;

    /**
     * @brief Construct channel control interface
     * @param core Reference to AXP192 core communication object
     */
    explicit AXP192Channel(AXP192Core &core);
    ~AXP192Channel() = default;

    /**
     * @brief Enable or disable a power output channel
     * @param channel Channel identifier (CH_DCDC1 through CH_EXTEN)
     * @param enable true to turn on, false to turn off
     * @return true on success, false on I2C error
     */
    bool enable(uint8_t channel, bool enable) override;

    /**
     * @brief Check whether a power output channel is enabled
     * @param channel Channel identifier (CH_DCDC1 through CH_EXTEN)
     * @return true if enabled, false otherwise
     */
    bool isEnabled(uint8_t channel) override;

    /**
     * @brief Set output voltage for a channel
     *
     * Valid ranges and step sizes:
     * - DCDC1/2/3: 700-3500mV, 25mV steps
     * - LDO2/3:    1800-3300mV, 100mV steps
     * - LDOio:     1800-3300mV, 100mV steps
     * - EXTEN:     No voltage control (returns false)
     *
     * @param channel Channel identifier
     * @param mV Desired voltage in millivolts
     * @return true on success, false if channel has no voltage control
     *         or on I2C error
     */
    bool setVoltage(uint8_t channel, uint16_t mV) override;

    /**
     * @brief Get current output voltage for a channel
     * @param channel Channel identifier
     * @return Voltage in millivolts, or 0 on error / non-configurable channel
     */
    uint16_t getVoltage(uint8_t channel) override;

    /**
     * @brief Get total number of power output channels
     * @return Always 7 (3 DCDC + 4 LDO-type)
     */
    uint8_t count() const override;

    /**
     * @brief Get channel information (type, name, min/max/step)
     * @param channel Channel identifier
     * @return Info struct containing channel metadata
     */
    Info getInfo(uint8_t channel) const override;

private:
    bool isDCDC(uint8_t channel) const { return channel < DCDC_COUNT; }

    AXP192Core &_core;
};

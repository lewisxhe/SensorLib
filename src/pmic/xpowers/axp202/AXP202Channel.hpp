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
 * LIABILITY, WHETHER IN AN ACTION OF SETTING, OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      AXP202Channel.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP202 Power Output Channel Control
 *
 * Manages the seven power output channels of the AXP202 PMIC: DCDC2,
 * DCDC3, LDO2, LDO3, LDO4, LDOio (via GPIO0), and EXTEN (external
 * enable pin). Each channel can be individually enabled/disabled and
 * its output voltage configured.
 *
 * @section Channel Voltage Ranges
 * - DCDC2: 700-2275mV, 25mV steps (register 0x23)
 * - DCDC3: 700-3500mV, 25mV steps (register 0x27)
 * - LDO2:  1800-3300mV, 100mV steps (via LDO24OUT_VOL bits[3:0])
 * - LDO3:  700-3500mV, 25mV steps (register 0x29)
 * - LDO4:  1250-3300mV, fixed lookup table (21 entries)
 * - LDOio: 1800-3300mV, 100mV steps (register 0x91, via GPIO0)
 * - EXTEN: External enable pin only, no voltage control
 *
 * @note LDO4 uses a discrete lookup table. Setting a value not in the
 *       table quantizes to the nearest entry.
 * @note EXTEN is an on/off control only; setVoltage/getVoltage are
 *       not applicable.
 */
#pragma once
#include "AXP202Core.hpp"
#include "../../PmicChannelBase.hpp"

/**
 * @brief AXP202 power channel enable and voltage control
 *
 * Implements PmicChannelBase to provide enable/disable and voltage
 * setting for all seven AXP202 output rails. Each channel maps to
 * a unique register and bit position in the power control register map.
 */
class AXP202Channel : public PmicChannelBase
{
public:
    static constexpr uint8_t CHANNEL_COUNT = 7;

    static constexpr uint8_t CH_DCDC2 = 0;
    static constexpr uint8_t CH_DCDC3 = 1;
    static constexpr uint8_t CH_LDO2  = 2;
    static constexpr uint8_t CH_LDO3  = 3;
    static constexpr uint8_t CH_LDO4  = 4;
    static constexpr uint8_t CH_LDOio = 5;
    static constexpr uint8_t CH_EXTEN = 6;

    /**
     * @brief Construct AXP202 channel control interface
     * @param core Reference to AXP202 core communication
     */
    explicit AXP202Channel(AXP202Core &core);

    ~AXP202Channel() = default;

    /**
     * @brief Enable or disable a power output channel
     * @param channel Channel identifier (CH_DCDC2 .. CH_EXTEN)
     * @param enable true to turn on, false to turn off
     * @return true on success
     */
    bool enable(uint8_t channel, bool enable) override;

    /**
     * @brief Check whether a power output channel is enabled
     * @param channel Channel identifier
     * @return true if the channel is currently enabled
     */
    bool isEnabled(uint8_t channel) override;

    /**
     * @brief Set the output voltage of a power channel
     *
     * @param channel Channel identifier (DCDC2, DCDC3, LDO2, LDO3, LDO4, or LDOio)
     * @param mV Desired voltage in millivolts
     *
     * @par Valid ranges per channel:
     * - DCDC2: 700-2275mV, 25mV steps
     * - DCDC3: 700-3500mV, 25mV steps
     * - LDO2:  1800-3300mV, 100mV steps
     * - LDO3:  700-3500mV, 25mV steps
     * - LDO4:  1250-3300mV, discrete table (clamped to nearest entry)
     * - LDOio: 1800-3300mV, 100mV steps
     * - EXTEN: Not applicable (returns false)
     *
     * @return true on success
     */
    bool setVoltage(uint8_t channel, uint16_t mV) override;

    /**
     * @brief Get the current output voltage of a power channel
     * @param channel Channel identifier
     * @return Voltage in millivolts, or 0 on error
     */
    uint16_t getVoltage(uint8_t channel) override;

    /**
     * @brief Get the number of available power channels
     * @return Always 7 (CHANNEL_COUNT)
     */
    uint8_t count() const override;

    /**
     * @brief Get information about a specific channel
     * @param channel Channel identifier
     * @return Info struct describing the channel capabilities
     */
    Info getInfo(uint8_t channel) const override;

private:
    /**
     * @brief LDO4 voltage lookup table (indexed by register value)
     *
     * Entries: 1250,1300,1400,1500,1600,1700,1800,1900,2000,
     *          2100,2200,2300,2400,2500,2600,2700 mV
     */
    static const uint16_t ldo4Table_[16];

    AXP202Core &_core;
};

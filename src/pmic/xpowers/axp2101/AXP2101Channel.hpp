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
 * @file      AXP2101Channel.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Power Output Channel Control
 *
 */
#pragma once
#include "AXP2101Core.hpp"
#include "../../PmicChannelBase.hpp"

/**
 * @brief Power output channel (DCDC / LDO) control for the AXP2101 PMIC.
 *
 * The AXP2101 provides 5 DCDC converters and 9 LDO regulators (14 channels total).
 * Each channel can be independently enabled/disabled and its output voltage set.
 *
 * @section Voltage Ranges and Steps
 * - DCDC1: 1500-3400mV, 100mV steps
 * - DCDC2: 500-1540mV, dual-range (10mV / 20mV steps)
 * - DCDC3: 500-3400mV, triple-range (10mV / 20mV / 100mV steps)
 * - DCDC4: 500-1840mV, dual-range (10mV / 20mV steps)
 * - DCDC5: 1200-3700mV, 100mV steps
 * - ALDO1-4: 500-3500mV, 100mV steps
 * - BLDO1-2: 500-3500mV, 100mV steps
 * - CPUSLDO: 500-1400mV, 50mV steps
 * - DLDO1:   500-3300mV, 100mV steps, This only applies to the linear version; it does not apply to the DC charging version.
 * - DLDO2:   500-1400mV, 50mV steps, This only applies to the linear version; it does not apply to the DC charging version.
 *
 * @note Input voltages outside a channel's valid range will be clamped
 * and/or quantized to the nearest valid register value.
 */
class AXP2101Channel : public PmicChannelBase
{
public:
    /** Number of DCDC converters */
    static constexpr uint8_t DCDC_COUNT = 5;
    /** Number of LDO regulators */
    static constexpr uint8_t LDO_COUNT  = 9;
    /** Total number of power channels */
    static constexpr uint8_t CHANNEL_COUNT = DCDC_COUNT + LDO_COUNT;

    static constexpr uint8_t CH_DCDC1 = 0;
    static constexpr uint8_t CH_DCDC2 = 1;
    static constexpr uint8_t CH_DCDC3 = 2;
    static constexpr uint8_t CH_DCDC4 = 3;
    static constexpr uint8_t CH_DCDC5 = 4;
    static constexpr uint8_t CH_ALDO1 = 5;
    static constexpr uint8_t CH_ALDO2 = 6;
    static constexpr uint8_t CH_ALDO3 = 7;
    static constexpr uint8_t CH_ALDO4 = 8;
    static constexpr uint8_t CH_BLDO1 = 9;
    static constexpr uint8_t CH_BLDO2 = 10;
    static constexpr uint8_t CH_CPUSLDO = 11;
    static constexpr uint8_t CH_DLDO1 = 12;
    static constexpr uint8_t CH_DLDO2 = 13;

    explicit AXP2101Channel(AXP2101Core &core);
    ~AXP2101Channel() = default;

    /**
     * @brief Enable or disable a power output channel.
     * @param channel Channel index (0-13).
     * @param enable true to power on, false to power off.
     * @return true on success.
     */
    bool enable(uint8_t channel, bool enable) override;

    /**
     * @brief Check whether a power output channel is enabled.
     * @param channel Channel index (0-13).
     * @return true if the channel is powered on.
     */
    bool isEnabled(uint8_t channel) override;

    /**
     * @brief Set the output voltage for a DCDC or LDO channel.
     *
     * Value will be clamped to the channel's valid range and quantized
     * to the nearest valid step size.
     *
     * @param channel Channel index (0-13).
     * @param mV Desired output voltage in millivolts.
     * @return true on success.
     */
    bool setVoltage(uint8_t channel, uint16_t mV) override;

    /**
     * @brief Get the current output voltage setting for a channel.
     * @param channel Channel index (0-13).
     * @return Output voltage in millivolts.
     */
    uint16_t getVoltage(uint8_t channel) override;

    /**
     * @brief Get the total number of available power channels.
     * @return Channel count (14).
     */
    uint8_t count() const override;

    /**
     * @brief Get information about a specific power channel.
     * @param channel Channel index (0-13).
     * @return Info structure containing type, voltage range, and step.
     */
    Info getInfo(uint8_t channel) const override;

private:
    /**
     * @brief Check if a channel index refers to a DCDC converter.
     * @param channel Channel index.
     * @return true if channel < DCDC_COUNT.
     */
    bool isDCDC(uint8_t channel) const { return channel < DCDC_COUNT; }

    AXP2101Core &_core;
};

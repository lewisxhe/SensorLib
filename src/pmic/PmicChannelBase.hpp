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
 * @file      PmicChannelBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 *
 * @brief PMIC power output channel interface.
 *
 * This interface abstracts PMIC power output channels (DCDC converters and LDO
 * regulators). Each channel can be independently enabled/disabled and have its
 * output voltage configured. The interface uses a unified uint8_t channel index
 * that maps to chip-specific power rails.
 *
 * @section Design
 * Instead of exposing chip-specific DCDC/LDO methods, this interface provides a
 * single entry point (getChannel()) that accepts a channel index. Each chip
 * implementation defines its own channel layout and provides metadata via
 * getInfo() and count().
 *
 */
#pragma once

#include <stdint.h>

/**
 * @brief PMIC power output channel base interface.
 *
 * Provides unified access to DCDC buck converters and LDO regulators.
 * Each channel is identified by a uint8_t index; the channel-to-rail mapping
 * is chip-specific and documented in the implementation's getInfo() table.
 *
 * Usage:
 * @code
 * pmic.getChannel()->enable(0, true);           // Enable DCDC1
 * pmic.getChannel()->setVoltage(0, 3300);       // Set DCDC1 to 3.3V
 * auto info = pmic.getChannel()->getInfo(0);    // Query range & name
 * @endcode
 */
class PmicChannelBase
{
public:
    /**
     * @brief Regulator type classification.
     */
    enum class Type : uint8_t {
        DCDC = 0,   ///< Switching buck/boost converter
        LDO  = 1,   ///< Linear low-dropout regulator
    };

    /**
     * @brief Static metadata for a single power output channel.
     */
    struct Info {
        uint8_t channel;            ///< Channel index (0-based)
        Type    type;               ///< Regulator type (DCDC/LDO)
        uint16_t minMillivolt;      ///< Minimum output voltage in mV
        uint16_t maxMillivolt;      ///< Maximum output voltage in mV
        uint16_t stepMillivolt;     ///< Voltage adjustment step in mV
        const char *name;           ///< Name (e.g. "DCDC1")
    };

    /**
     * @brief Virtual destructor.
     */
    virtual ~PmicChannelBase() = default;

    /**
     * @brief Enable or disable a power output channel.
     * @param channel Channel index (0-based, see implementation for mapping).
     * @param enable true to turn on, false to turn off.
     * @return true on success, false if channel is invalid or I2C error.
     */
    virtual bool enable(uint8_t channel, bool enable) = 0;

    /**
     * @brief Check whether a power output channel is currently enabled.
     * @param channel Channel index.
     * @return true if the channel is enabled, false otherwise.
     */
    virtual bool isEnabled(uint8_t channel) = 0;

    /**
     * @brief Set the output voltage of a channel.
     * @param channel Channel index.
     * @param mV Desired voltage in millivolts. Value will be quantized to the
     *           nearest valid step within the channel's range.
     * @return true on success, false if channel is invalid or voltage out of range.
     *
     * @note Each channel has a specific range and step defined in getInfo().
     *       Call getVoltage() after setVoltage() to read back the quantized value.
     */
    virtual bool setVoltage(uint8_t channel, uint16_t mV) = 0;

    /**
     * @brief Get the current output voltage of a channel.
     * @param channel Channel index.
     * @return Output voltage in millivolts, or 0 on error or invalid channel.
     */
    virtual uint16_t getVoltage(uint8_t channel) = 0;

    /**
     * @brief Get the number of power output channels available on this PMIC.
     * @return Channel count.
     */
    virtual uint8_t count() const = 0;

    /**
     * @brief Get static metadata for a specific channel.
     * @param channel Channel index.
     * @return Info struct containing type, voltage range, step, and name.
     *         Returns an UNKNOWN entry if the channel index is out of range.
     */
    virtual Info getInfo(uint8_t channel) const = 0;
};


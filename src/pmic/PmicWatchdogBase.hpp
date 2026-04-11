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
 * @file      PmicWatchdogBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 * @brief PMIC Watchdog interface.
 *
 * This interface defines methods for controlling the I2C watchdog timer.
 * The watchdog monitors I2C communication and will trigger a fault if no
 * I2C communication occurs within the configured timeout period.
 *
 * Note: When watchdog expires, the PMIC will reset register settings to defaults.
 *       Regular watchdog reset (feeding) is required to prevent this.
 *       Disable watchdog functionality during default initialization of subclasses.
 *       Enable and configure watchdog after initialization if needed.
 */
#pragma once

#include <stdint.h>

class PmicWatchdogBase
{
public:

    /**
     * @brief Virtual destructor.
     */
    virtual ~PmicWatchdogBase() = default;

    /**
     * @brief Enable or disable the watchdog timer.
     *
     * When disabled, the PMIC will not monitor I2C communication.
     * When enabled, the watchdog must be regularly reset (fed) to prevent timeout.
     *
     * @param enable True to enable watchdog, false to disable
     * @return true on success, false if not supported or I2C error
     */
    virtual bool enableWatchdog(bool enable) = 0;

    /**
     * @brief Check if watchdog is enabled.
     * @return true if watchdog is enabled
     */
    virtual bool isWatchdogEnabled() const = 0;

    /**
     * @brief Set the watchdog timeout duration.
     *
     * Sets how long the watchdog will wait before triggering a fault.
     * Must be called after enableWatchdog(true).
     *
     * @param timeout_s Timeout in seconds (chip-specific valid values, typically 0=disabled, 40, 80, 160)
     * @return true on success, false if not supported or I2C error
     *
     * @note Valid timeout values are chip-specific. Common values: 0=disabled, 40, 80, 160 seconds.
     *       Other chips may use different values - implementation should check and clamp.
     * @see getWatchdogTimeout()
     */
    virtual bool setWatchdogTimeout(uint16_t timeout_s) = 0;

    /**
     * @brief Get the current watchdog timeout setting.
     * @return Timeout in seconds (0 if disabled, or chip-specific timeout value)
     */
    virtual uint16_t getWatchdogTimeout() const = 0;

    /**
     * @brief Reset the watchdog timer (feed the watchdog).
     *
     * This must be called regularly before the watchdog timeout expires
     * to prevent the PMIC from resetting due to I2C communication timeout.
     *
     * @return true on success, false if not supported or I2C error
     *
     * @note Should be called at least once before the watchdog timeout.
     *       Recommended to call every 20-30 seconds for 40s timeout.
     */
    virtual bool resetWatchdog() = 0;

};

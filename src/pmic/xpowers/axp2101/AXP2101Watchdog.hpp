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
 * @file      AXP2101Watchdog.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Watchdog Timer
 *
 */
#pragma once
#include "AXP2101Core.hpp"
#include "../../PmicWatchdogBase.hpp"

/**
 * @brief I2C watchdog timer control for the AXP2101 PMIC.
 *
 * The watchdog monitors I2C bus activity. If no communication occurs
 * within the configured timeout, the PMIC resets suspect registers to
 * their default values. Regular calls to resetWatchdog() are required
 * to prevent timeout expiry.
 *
 * Supported timeout values: 1, 2, 4, 8, 16, 32, 64, 128 seconds.
 * Watchdog behaviour (IRQ only / IRQ+reset) is configured via
 * AXP2101Core::setWatchdogConfig(), default is IRQ Only.
 */
class AXP2101Watchdog : public PmicWatchdogBase
{
public:
    explicit AXP2101Watchdog(AXP2101Core &core);
    ~AXP2101Watchdog() = default;

    /**
     * @brief Enable or disable the watchdog timer.
     * @param enable true to enable (default 64s timeout), false to disable.
     * @return true on success.
     */
    bool enableWatchdog(bool enable) override;

    /**
     * @brief Check whether the watchdog timer is enabled.
     * @return true if watchdog is active.
     */
    bool isWatchdogEnabled() const override;

    /**
     * @brief Set the watchdog timeout duration.
     *
     * Valid values: 1, 2, 4, 8, 16, 32, 64, 128 seconds.
     * Other values will be rejected (return false).
     *
     * @param timeout_s Timeout in seconds.
     * @return true on success.
     */
    bool setWatchdogTimeout(uint16_t timeout_s) override;

    /**
     * @brief Get the current watchdog timeout setting.
     * @return Timeout in seconds (1, 2, 4, 8, 16, 32, 64, or 128).
     */
    uint16_t getWatchdogTimeout() const override;

    /**
     * @brief Reset (pet) the watchdog timer.
     *
     * Must be called at least once before the timeout expires
     * to prevent register reset. Recommended interval is half
     * the configured timeout period.
     *
     * @return true on success.
     */
    bool resetWatchdog() override;

private:
    AXP2101Core &_core;
};

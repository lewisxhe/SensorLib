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
 * @file      GaugeBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-25
 */
#pragma once

#include <stdint.h>

/**
 * @brief Base class for battery gauge implementations
 * @details Provides a common interface for various battery gauge ICs
 *         to read battery status information such as voltage, current,
 *         temperature, state of charge, and time estimates.
 */
class GaugeBase
{
public:
    virtual ~GaugeBase() = default;

    GaugeBase(const GaugeBase &) = delete;
    GaugeBase &operator=(const GaugeBase &) = delete;

    /**
     * @brief Refresh cached gauge data
     * @return true if refresh successful, false otherwise
     */
    virtual bool refresh() = 0;

    /**
     * @brief Get battery temperature
     * @return Temperature in Celsius
     */
    virtual float getTemperature() = 0;

    /**
     * @brief Get battery voltage
     * @return Voltage in millivolts
     */
    virtual uint16_t getVoltage() = 0;

    /**
     * @brief Get battery current
     * @return Current in milliamps (positive = charging, negative = discharging)
     */
    virtual float getCurrent() = 0;

    /**
     * @brief Get state of charge
     * @return SOC percentage (0-100)
     */
    virtual uint16_t getStateOfCharge() = 0;

    /**
     * @brief Get time to empty
     * @return Time to empty in minutes
     */
    virtual uint16_t getTimeToEmpty() = 0;

    /**
     * @brief Get time to full
     * @return Time to full charge in minutes
     */
    virtual uint16_t getTimeToFull() = 0;

    /**
     * @brief Get chip ID
     * @return Chip identifier
     */
    virtual uint16_t getChipID() = 0;

    /**
     * @brief Reset gauge to initial state
     */
    virtual void reset() = 0;

    /**
     * @brief Check if battery is charging
     * @return true if charging, false otherwise
     */
    virtual bool isCharging() = 0;

    /**
     * @brief Check if battery is discharging
     * @return true if discharging, false otherwise
     */
    virtual bool isDischarging() = 0;

protected:
    GaugeBase() = default;
};

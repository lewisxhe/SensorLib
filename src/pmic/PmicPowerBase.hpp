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
 * @file      PmicPowerBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 * @brief PMIC Power interface.
 *
 * This interface defines methods for setting and getting input voltage and current limits,
 * as well as enabling boost functionality.
 *
 * Notes:
 *  - Some PMICs do not support some functions; implementations
 *    should return false.
 */
#pragma once

#include <stdint.h>

class PmicPowerBase
{
public:

    /**
     * @brief  Set the minimum system voltage.
     * @note   This function sets the minimum voltage required for the system to operate.
     * @param  mv: Minimum voltage in millivolts.
     * @retval True if the operation was successful, false otherwise.
     */
    virtual bool setMinimumSystemVoltage(uint32_t mv) = 0;

    /**
     * @brief  Get the minimum system voltage.
     * @note   This function retrieves the minimum voltage required for the system to operate.
     * @retval Minimum voltage in millivolts.
     */
    virtual uint32_t getMinimumSystemVoltage() const = 0;

    /**
     * @brief Set the input voltage limit.
     * @param mv Voltage limit in millivolts.
     */
    virtual bool setInputVoltageLimit(uint32_t mv) = 0;

    /** @brief Get the input voltage limit.
     * @return Input voltage limit in millivolts.
     */
    virtual uint32_t getInputVoltageLimit() const = 0;

    /**
     * * @brief Set the input current limit.
     * @param ma Current limit in milliamperes.
     */
    virtual bool setInputCurrentLimit(uint32_t ma) = 0;

    /** @brief Get the input current limit.
     * @return Input current limit in milliamperes.
     */
    virtual uint32_t getInputCurrentLimit() const = 0;

    /**
     * @brief Enable boost functionality.
     * @param enable True to enable, false to disable.
     */
    virtual bool enableBoost(bool enable) = 0;

    /**
     * @brief Check if boost functionality is enabled.
     * @return True if boost is enabled, false otherwise.
     */
    virtual bool isBoostEnabled() const = 0;

    /**
    * @brief Set the boost voltage.
    * @param mv Boost voltage in millivolts.
    */
    virtual bool setBoostVoltage(uint16_t mv) = 0;

    /**
     * @brief Get the boost voltage.
     * @return Boost voltage in millivolts.
     */
    virtual uint16_t getBoostVoltage() const = 0;

    /**
     * @brief Enable ship mode (battery disconnect)
     *
     * When enabled, the BATFET is turned off to disconnect the battery
     * from the system. This is used for shipping mode or to completely
     * power off the device when no external power is available.
     *
     * @param enable True to enter ship mode (battery disconnected), false to exit
     * @return true on success, false if not supported or I2C error
     *
     * @note When ship mode is enabled, the device cannot be powered from battery.
     *       To exit ship mode, VBUS must be connected to power the system.
     */
    virtual bool enableShipMode(bool enable) = 0;

    /**
     * @brief Check if ship mode is enabled.
     * @return true if ship mode is active (battery disconnected)
     */
    virtual bool isShipModeEnabled() const = 0;

    /**
     * @brief Virtual destructor.
     */
    virtual ~PmicPowerBase() = default;
};

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
 * @file      AXP517Power.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#pragma once

#include "AXP517Core.hpp"
#include "../../PmicPowerBase.hpp"

class AXP517Power : public PmicPowerBase
{
public:
    AXP517Power(AXP517Core &core);

    ~AXP517Power() = default;

    /**
    * @brief  Set the minimum system voltage.
    * @note   This function sets the minimum voltage required for the system to operate.
    * @param  mv: Minimum voltage in millivolts.
    *         Range: 1000mV ~ 3800mV, steps: 100mV
    * @retval True if the operation was successful, false otherwise.
    */
    bool setMinimumSystemVoltage(uint32_t mv) override;

    /**
    * @brief  Get the minimum system voltage.
    * @note   This function retrieves the minimum voltage required for the system to operate.
    * @retval Minimum voltage in millivolts.
    */
    uint32_t getMinimumSystemVoltage() const override;

    /**
    * @brief Set the input voltage limit.
    * @param mv Voltage limit in millivolts.
    *        Range: 3600mV ~ 16200mV, steps:100mV
    * @return True on success, false on failure.
    */
    bool setInputVoltageLimit(uint32_t mv) override;

    /**
    * @brief Get the input voltage limit.
    * @return Input voltage limit in millivolts.
    */
    uint32_t getInputVoltageLimit() const override;

    /**
    * @brief Set the input current limit.
    * @param mA Current limit in milliamperes.
    *        Range: 100mA ~ 3250mA, steps:50mA
    * @return True on success, false on failure.
    */
    bool setInputCurrentLimit(uint32_t mA) override;

    /**
    * @brief Get the input current limit.
    * @return Input current limit in milliamperes.
    */
    uint32_t getInputCurrentLimit() const override;

    /**
    * @brief Enable boost functionality.
    * @param enable True to enable, false to disable.
    * @return True on success, false on failure.
    */
    bool enableBoost(bool enable) override;

    /**
     * @brief Check if boost functionality is enabled.
     * @return True if boost is enabled, false otherwise.
     */
    bool isBoostEnabled() const override;

    /**
    * @brief Set the boost voltage.
    * @param mv Boost voltage in millivolts.
    * @return True on success, false on failure.
    */
    bool setBoostVoltage(uint16_t mv) override;

    /**
     * @brief Get the boost voltage.
     * @return Boost voltage in millivolts.
     */
    uint16_t getBoostVoltage() const override;

    /**
     * @brief Enter ship mode.
     * @param enable: True to enter ship mode, false to exit.
     * @retval True on success, false on failure.
     */
    bool enableShipMode(bool enable) override;

    /**
     * @brief Check if ship mode is enabled.
     * @retval True if ship mode is enabled, false otherwise.
     */
    bool isShipModeEnabled() const override;
    
private:
    AXP517Core &_core;
};

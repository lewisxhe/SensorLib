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
 * @brief     Power-path, boost, and ship-mode control interface for AXP517.
 */
#pragma once

#include "AXP517Core.hpp"
#include "../../PmicPowerBase.hpp"

/**
 * @brief AXP517 power-path driver.
 *
 * Provides input current/voltage limits, minimum system voltage, boost output,
 * ship mode, and reverse-blocking FET controls.
 */
class AXP517Power : public PmicPowerBase
{
public:
    /**
     * @brief Construct an AXP517 power-path driver.
     * @param core Shared AXP517 register transport.
     */
    AXP517Power(AXP517Core &core);

    ~AXP517Power() = default;

    /**
     * @brief  Set the minimum system voltage.
     * @note   This function sets the minimum voltage required for the system to operate.
     * @param  mv Minimum voltage in millivolts, range 1000-3800mV, steps 100mV.
     * @retval true on success, false on register access failure.
     */
    bool setMinimumSystemVoltage(uint32_t mv) override;

    /**
     * @brief  Get the minimum system voltage.
     * @note   This function retrieves the minimum voltage required for the system to operate.
     * @return Minimum system voltage in millivolts.
     */
    uint32_t getMinimumSystemVoltage() const override;

    /**
     * @brief Set the input voltage limit.
     * @param mv Voltage limit in millivolts, range 3600-16200mV, steps 100mV.
     * @retval true on success, false on register access failure.
     */
    bool setInputVoltageLimit(uint32_t mv) override;

    /**
     * @brief Get the input voltage limit.
     * @return Input voltage limit in millivolts.
     */
    uint32_t getInputVoltageLimit() const override;

    /**
     * @brief Set the input current limit.
     * @param mA Current limit in milliamps, range 100-3250mA, steps 50mA.
     * @retval true on success, false on register access failure.
     */
    bool setInputCurrentLimit(uint32_t mA) override;

    /**
     * @brief Get the input current limit.
     * @return Input current limit in milliamps.
     */
    uint32_t getInputCurrentLimit() const override;

    /**
     * @brief Enable boost functionality.
     * @param enable true to enable, false to disable.
     * @retval true on success, false on register access failure.
     */
    bool enableBoost(bool enable) override;

    /**
     * @brief Check if boost functionality is enabled.
     * @retval true if boost is enabled, false otherwise.
     */
    bool isBoostEnabled() const override;

    /**
     * @brief Set the boost voltage.
     * @param mv Boost voltage in millivolts.
     * @retval true on success, false on register access failure.
     */
    bool setBoostVoltage(uint16_t mv) override;

    /**
     * @brief Get the boost voltage.
     * @return Boost voltage in millivolts.
     */
    uint16_t getBoostVoltage() const override;

    /**
     * @brief Enter ship mode.
     * @param enable true to enter ship mode, false to exit.
     * @retval true on success, false on register access failure.
     */
    bool enableShipMode(bool enable) override;

    /**
     * @brief Check if ship mode is enabled.
     * @retval true if ship mode is enabled, false otherwise.
     */
    bool isShipModeEnabled() const override;

    /**
     * @brief Enable or disable the RBFET.
     * @param enable true to enable, false to disable.
     * @retval true on success, false on register access failure.
     */
    bool enableRBFET(bool enable) const;

    /**
     * @brief Check if RBFET is enabled.
     * @retval true if RBFET is enabled, false otherwise.
     */
    bool isRBFETEnabled() const;

private:
    AXP517Core &_core; ///< Shared AXP517 register transport.
};

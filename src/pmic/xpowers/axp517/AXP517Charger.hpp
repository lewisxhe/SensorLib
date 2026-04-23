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
 * @file      AXP517Charger.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#pragma once

#include "../../PmicChargerBase.hpp"
#include "AXP517Core.hpp"

class AXP517Charger : public PmicChargerBase
{
public:
    explicit AXP517Charger(AXP517Core &core);

    ~AXP517Charger() = default;

    /**
    * @brief Enable/disable charging.
    */
    bool enableCharging(bool enable) override;

    /**
     * @brief Check whether charger is currently charging (best-effort).
     * @return true if charging, false otherwise.
     */
    bool isCharging() override;

    /**
     * @brief Set pre-charge current.
     * @param mA Desired current in milliamps, range 0-960mA, steps 64mA.
     * @return true on success, false on failure.
     */
    bool setPreChargeCurrent(uint16_t mA) override;

    /**
     * @brief Get pre-charge current.
     */
    uint16_t getPreChargeCurrent() override;

    /**
     * @brief Set fast charge (constant-current) current.
     * @param mA Desired current in milliamps, range 0-5120mA, steps 64mA.
     * @return true on success, false on failure.
     */
    bool setFastChargeCurrent(uint16_t mA) override;

    /**
     * @brief Get fast charge (constant-current) current.
     */
    uint16_t getFastChargeCurrent() override;

    /**
     * @brief Set termination current.
     * @param mA Desired current in milliamps, range 0-960mA, steps 64mA.
     * @return true on success, false on failure.
     */
    bool setTerminationCurrent(uint16_t mA) override;

    /**
     * @brief Get termination current.
     */
    uint16_t getTerminationCurrent() override;

    /**
     * @brief Set charge voltage (CV).
     * @param mV Desired voltage in millivolts, range 3600-5000mV.
     * [mV]:3600,3800,4000,4100,4200,4350,4400,5000
     * @return true on success, false on failure.
     */
    bool setChargeVoltage(uint16_t mV) override;

    /**
     * @brief Get charge voltage (CV).
     */
    uint16_t getChargeVoltage() override;

    /**
     * @brief Get current charger status (best-effort).
     */
    Status getStatus() override;

private:

    /**
     * @brief Set current with step.
     * @param reg Register to modify.
     * @param milliamps Desired current in milliamps.
     * @param max Maximum allowed current in milliamps.
     * @param step Step size for adjusting current.
     * @param mask Bitmask for the relevant bits in the register.
     * @return true on success, false on failure.
     */
    bool setCurrentWithStep(uint8_t reg, uint16_t milliamps,
                            uint16_t max, uint16_t step, uint8_t mask);
    /**
     * @brief Get current with step.
     * @param reg Register to read.
     * @param step Step size for adjusting current.
     * @param mask Bitmask for the relevant bits in the register.
     * @return Current in milliamps.
     */
    uint16_t getCurrentWithStep(uint8_t reg, uint16_t step, uint8_t mask);

    AXP517Core &_core;
};

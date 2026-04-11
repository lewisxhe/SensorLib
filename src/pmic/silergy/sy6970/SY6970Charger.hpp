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
 * @file      SY6970Charger.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 */
#pragma once

#include "../../PmicChargerBase.hpp"
#include "SY6970Core.hpp"

class SY6970Charger : public PmicChargerBase
{
public:
    explicit SY6970Charger(SY6970Core &core);

    ~SY6970Charger() = default;

    /**
     * @brief Enable/disable charging.
     * @param enable true to enable charging, false to disable.
     * @return true on success.
     */
    bool enableCharging(bool enable) override;

    /**
     * @brief Check whether charger is currently charging (best-effort).
     * @return true if charging.
     */
    bool isCharging() override;

    /**
     * @brief Set pre-charge current.
     * @param mA Pre-charge current in milliamps. Value will be quantized to nearest valid step (64mA).
     * @return true on success.
     */
    bool setPreChargeCurrent(uint16_t mA) override;

    /**
     * @brief Get pre-charge current.
     * @return Pre-charge current in milliamps.
     */
    uint16_t getPreChargeCurrent() override;

    /**
     * @brief Set fast charge (constant-current) current.
     * @param mA Fast charge current in milliamps. Value will be quantized to nearest valid step (64mA).
     * @return true on success.
     */
    bool setFastChargeCurrent(uint16_t mA) override;

    /**
     * @brief Get fast charge (constant-current) current.
     * @return Fast charge current in milliamps.
     */
    uint16_t getFastChargeCurrent() override;

    /**
     * @brief Set termination current.
     * @param mA Termination current in milliamps. Value will be quantized to nearest valid step (64mA).
     * @return true on success.
     */
    bool setTerminationCurrent(uint16_t mA) override;

    /**
     * @brief Get termination current.
     * @return Termination current in milliamps.
     */
    uint16_t getTerminationCurrent() override;

    /**
     * @brief Set charge voltage (CV).
     * @param mV Charge voltage in millivolts. Value will be quantized to nearest valid step (16mV).
     * @return true on success.
     */
    bool setChargeVoltage(uint16_t mV) override;

    /**
     * @brief Get charge voltage (CV).
     * @return Charge voltage in millivolts.
     */
    uint16_t getChargeVoltage() override;

    /**
     * @brief Get charger status.
     * @return Status structure containing charger state.
     */
    Status getStatus() override;

private:
    SY6970Core &_core;
    bool _userDisableCharge = false;
};

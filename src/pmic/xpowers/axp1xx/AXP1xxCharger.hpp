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
 * @file      AXP1xxCharger.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-28
 *
 * @brief AXP192/AXP202 Battery Charger Control Interface
 */
#pragma once
#include "../../PmicChargerBase.hpp"
#include "../../../platform/SensorCommWrapper.hpp"

template<typename Regs>
class AXP1xxCharger : public PmicChargerBase
{
public:
    /**
     * @brief Construct charger control interface.
     * @param core Reference to the I2C communication wrapper.
     */
    explicit AXP1xxCharger(SensorCommWrapper &core) : _core(core) {}

    /**
     * @brief Enable or disable battery charging.
     *
     * Controls CHARGE1 register bit 7.
     *
     * @param enable true to enable charging, false to disable.
     * @retval true  Charging was configured successfully.
     * @retval false I2C communication failed.
     */
    bool enableCharging(bool enable) override
    {
        return enable ? _core.setRegBit(Regs::chg::CHARGE1, 7)
               : _core.clrRegBit(Regs::chg::CHARGE1, 7);
    }

    /**
     * @brief Check whether the battery is currently charging.
     * @return true if charging, false otherwise.
     */
    bool isCharging() override
    {
        return getStatus().charging;
    }

    /**
     * @brief Set pre-charge current (not supported by AXP1xx).
     * @param mA Pre-charge current in milliamps (ignored).
     * @return Always false.
     */
    bool setPreChargeCurrent(uint16_t mA) override
    {
        (void)mA;
        return false;
    }

    /**
     * @brief Get pre-charge current (not supported by AXP1xx).
     * @return Always 0.
     */
    uint16_t getPreChargeCurrent() override
    {
        return 0;
    }

    /**
     * @brief Set fast charge (constant-current) current.
     *
     * Programs CHARGE1 register bits 0-3. The value is clamped to the
     * valid range.
     * axp192: Set in 90mA steps, close to the set value.
     * axp202: Set in 100mA steps.
     * @param mA Desired fast charge current in milliamps
     * @retval true  Current was set successfully.
     * @retval false I2C communication failed.
     */
    bool setFastChargeCurrent(uint16_t mA) override
    {
        if (mA < Regs::chg::CHG_CUR_MIN) mA = Regs::chg::CHG_CUR_MIN;
        if (mA > Regs::chg::CHG_CUR_MAX) mA = Regs::chg::CHG_CUR_MAX;
        uint8_t code = (mA - Regs::chg::CHG_CUR_MIN) / Regs::chg::CHG_CUR_STEP;
        int regVal = _core.readReg(Regs::chg::CHARGE1);
        if (regVal < 0) return false;
        regVal = (regVal & 0xF0) | code;
        return _core.writeReg(Regs::chg::CHARGE1, static_cast<uint8_t>(regVal)) >= 0;
    }

    /**
     * @brief Get the fast charge (constant-current) current.
     * @return Fast charge current in milliamps, or 0 on error.
     */
    uint16_t getFastChargeCurrent() override
    {
        int val = _core.readReg(Regs::chg::CHARGE1);
        if (val < 0) return 0;
        uint8_t code = val & 0x0F;
        return code * Regs::chg::CHG_CUR_STEP + Regs::chg::CHG_CUR_MIN;
    }

    /**
     * @brief Set the termination current threshold.
     *
     * Controls CHARGE1 register bit 4. When set, termination current is
     * approximately 150mA.
     *
     * @param mA Desired termination current in milliamps (>=150 sets threshold).
     * @retval true  Threshold was set successfully.
     * @retval false I2C communication failed.
     */
    bool setTerminationCurrent(uint16_t mA) override
    {
        bool set = (mA >= 150);
        return _core.updateBits(Regs::chg::CHARGE1, 0x10, set ? 0x10 : 0x00) >= 0;
    }

    /**
     * @brief Get the termination current threshold.
     * @return Termination current in milliamps (0 if unknown).
     */
    uint16_t getTerminationCurrent() override
    {
        int val = _core.readReg(Regs::chg::CHARGE1);
        if (val < 0) return 0;
        return (val & 0x10) ? 150 : 0;
    }

    /**
     * @brief Set the charge target voltage (CV).
     *
     * Programs CHARGE1 register bits 5-6 using a lookup table.
     *
     * @param mV Desired charge voltage in millivolts.
     *           Values <= 4100 set 4100mV.
     *           Values <= 4150 set 4150mV.
     *           Values <= 4200 set 4200mV.
     *           Values >  4200 set 4360mV.
     * @retval true  Voltage was set successfully.
     * @retval false I2C communication failed.
     */
    bool setChargeVoltage(uint16_t mV) override
    {
        uint8_t code;
        if (mV <= 4100) {
            code = 0;
        } else if (mV <= 4150) {
            code = 1;
        } else if (mV <= 4200) {
            code = 2;
        } else {
            code = 3;
        }
        return _core.updateBits(Regs::chg::CHARGE1, 0x60, code << 5) >= 0;
    }

    /**
     * @brief Get the charge target voltage (CV).
     * @return Charge voltage in millivolts (4100, 4150, 4200, or 4360), or 0 on error.
     */
    uint16_t getChargeVoltage() override
    {
        int val = _core.readReg(Regs::chg::CHARGE1);
        if (val < 0) return 0;
        uint8_t code = (val >> 5) & 0x03;
        switch (code) {
        case 0:  return 4100;
        case 1:  return 4150;
        case 2:  return 4200;
        case 3:  return 4360;
        default: return 0;
        }
    }

    /**
     * @brief Get the charger status.
     *
     * Reads the STATUS and MODE_CHGSTATUS registers to determine VBUS
     * presence, battery presence, charging state, and fault conditions.
     *
     * @return Status structure with current charger state.
     */
    Status getStatus() override
    {
        Status status{};
        int s0 = _core.readReg(Regs::bmu::STATUS);
        int s1 = _core.readReg(Regs::bmu::MODE_CHGSTATUS);
        if (s0 < 0 || s1 < 0) {
            return status;
        }
        status.online = true;
        status.vbusPresent = (s0 >> 5) & 0x01;
        status.batteryPresent = (s1 >> 5) & 0x01;
        status.charging = (s1 >> 6) & 0x01;
        status.chargeDone = false;
        status.fault = (s1 >> 7) & 0x01;
        if (status.charging) {
            status.chargingStatus = ChargingStatus::FAST_CHARGE;
        } else {
            status.chargingStatus = ChargingStatus::NO_CHARGING;
        }
        return status;
    }

protected:
    SensorCommWrapper &_core;
};

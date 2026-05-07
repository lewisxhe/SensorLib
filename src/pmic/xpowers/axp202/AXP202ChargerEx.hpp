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
 * @file      AXP202ChargerEx.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * Features not covered by AXP1xxCharger template: battery temperature
 * protection thresholds, backup battery charger, charge timeout control,
 * external channel charging.
 */

#pragma once
#include "AXP202Core.hpp"
#include "AXP202Regs.hpp"

class AXP202ChargerEx
{
public:
    explicit AXP202ChargerEx(AXP202Core &core) : _core(core) {}

    // ---- Charge Temperature Thresholds ----
    /**
     * @brief Set charge low-temperature fault threshold (REG 38H)
     * @param val Raw 8-bit value. VLTF = val * 12.8mV
     * @return true on success
     */
    bool setChargeLowTempThreshold(uint8_t val)
    {
        return _core.writeReg(axp202_regs::chg::VLTF_CHGSET, val) >= 0;
    }

    uint8_t getChargeLowTempThreshold()
    {
        int val = _core.readReg(axp202_regs::chg::VLTF_CHGSET);
        return (val < 0) ? 0 : static_cast<uint8_t>(val);
    }

    /**
     * @brief Set charge high-temperature fault threshold (REG 39H)
     * @param val Raw 8-bit value. VHTF = val * 12.8mV
     * @return true on success
     */
    bool setChargeHighTempThreshold(uint8_t val)
    {
        return _core.writeReg(axp202_regs::chg::VHTF_CHGSET, val) >= 0;
    }

    uint8_t getChargeHighTempThreshold()
    {
        int val = _core.readReg(axp202_regs::chg::VHTF_CHGSET);
        return (val < 0) ? 0 : static_cast<uint8_t>(val);
    }

    // ---- Discharge Temperature Thresholds ----
    /**
     * @brief Set discharge low-temperature fault threshold (REG 3CH)
     * @param val Raw 8-bit value
     * @return true on success
     */
    bool setDischargeLowTempThreshold(uint8_t val)
    {
        return _core.writeReg(axp202_regs::chg::TLTF_DISCHGSET, val) >= 0;
    }

    uint8_t getDischargeLowTempThreshold()
    {
        int val = _core.readReg(axp202_regs::chg::TLTF_DISCHGSET);
        return (val < 0) ? 0 : static_cast<uint8_t>(val);
    }

    /**
     * @brief Set discharge high-temperature fault threshold (REG 3DH)
     * @param val Raw 8-bit value
     * @return true on success
     */
    bool setDischargeHighTempThreshold(uint8_t val)
    {
        return _core.writeReg(axp202_regs::chg::THTF_DISCHGSET, val) >= 0;
    }

    uint8_t getDischargeHighTempThreshold()
    {
        int val = _core.readReg(axp202_regs::chg::THTF_DISCHGSET);
        return (val < 0) ? 0 : static_cast<uint8_t>(val);
    }

    // ---- Backup Battery Charger (REG 35H) ----

    /**
     * @brief Enable backup battery charger (REG 35H bit 7)
     * @param enable true to enable
     * @return true on success
     */
    bool enableBackupCharger(bool enable)
    {
        return enable ? _core.setRegBit(axp202_regs::chg::BACKUP_CHG, 7)
               : _core.clrRegBit(axp202_regs::chg::BACKUP_CHG, 7);
    }

    bool isBackupChargerEnabled()
    {
        return _core.getRegBit(axp202_regs::chg::BACKUP_CHG, 7);
    }

    /**
     * @brief Set backup battery charge voltage (REG 35H bits 6:5)
     * @param opt 0=3.1V, 1=3.0V, 2=3.6V, 3=2.5V
     * @return true on success
     */
    bool setBackupChargerVoltage(uint8_t opt)
    {
        if (opt > 3) return false;
        return _core.updateBits(axp202_regs::chg::BACKUP_CHG, 0x60, opt << 5) >= 0;
    }

    uint8_t getBackupChargerVoltage()
    {
        int val = _core.readReg(axp202_regs::chg::BACKUP_CHG);
        return (val < 0) ? 0 : static_cast<uint8_t>((val >> 5) & 0x03);
    }

    /**
     * @brief Set backup battery charge current (REG 35H bits 1:0)
     * @param opt 0=50uA, 1=100uA, 2=200uA, 3=400uA
     * @return true on success
     */
    bool setBackupChargerCurrent(uint8_t opt)
    {
        if (opt > 3) return false;
        return _core.updateBits(axp202_regs::chg::BACKUP_CHG, 0x03, opt) >= 0;
    }

    uint8_t getBackupChargerCurrent()
    {
        int val = _core.readReg(axp202_regs::chg::BACKUP_CHG);
        return (val < 0) ? 0 : static_cast<uint8_t>(val & 0x03);
    }

    // ---- Charge Timeout (REG 34H) ----

    /**
     * @brief Set precharge timeout (REG 34H bits 7:6)
     * @param opt 0=40min, 1=50min, 2=60min, 3=70min
     * @return true on success
     */
    bool setPrechargeTimeout(uint8_t opt)
    {
        if (opt > 3) return false;
        return _core.updateBits(axp202_regs::chg::CHARGE2, 0xC0, opt << 6) >= 0;
    }

    uint8_t getPrechargeTimeout()
    {
        int val = _core.readReg(axp202_regs::chg::CHARGE2);
        return (val < 0) ? 0 : static_cast<uint8_t>((val >> 6) & 0x03);
    }

    /**
     * @brief Set constant-current charge timeout (REG 34H bits 1:0)
     * @param opt 0=6h, 1=8h, 2=10h, 3=12h
     * @return true on success
     */
    bool setConstantCurrentTimeout(uint8_t opt)
    {
        if (opt > 3) return false;
        return _core.updateBits(axp202_regs::chg::CHARGE2, 0x03, opt) >= 0;
    }

    uint8_t getConstantCurrentTimeout()
    {
        int val = _core.readReg(axp202_regs::chg::CHARGE2);
        return (val < 0) ? 0 : static_cast<uint8_t>(val & 0x03);
    }

    // NOTE: External channel charge functions removed - bits 2 and 4:3 of
    // REG 34H are reserved. Using them corrupts CHGLED mode configuration.

private:
    AXP202Core &_core;
};

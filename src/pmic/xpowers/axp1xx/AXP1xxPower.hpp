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
 * @file      AXP1xxPower.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-28
 *
 * @brief AXP192/AXP202 Power Management Interface
 *
 * This template class provides control over power management functions:
 * - Minimum system voltage (VOFF_SET register, 2600-3300mV, 100mV steps)
 * - Input current limit (IPS_SET register bits 1-0, encoding chip-dependent)
 * - Ship mode control (OFF_CTL register bit 7)
 *
 * @note Voltage/current limit set-get methods not supported by the AXP1xx
 *       hardware (VCOND, boost) return false/0 as stubs.
 *
 * @tparam Regs Register map struct type (AXP192Regs or AXP202Regs)
 */
#pragma once
#include "../../PmicPowerBase.hpp"
#include "../../../platform/SensorCommWrapper.hpp"

template<typename Regs>
class AXP1xxPower : public PmicPowerBase
{
public:
    /**
     * @brief Construct power management interface.
     * @param core Reference to the I2C communication wrapper.
     */
    explicit AXP1xxPower(SensorCommWrapper &core) : _core(core) {}

    /**
     * @brief Set minimum system voltage.
     *
     * Configures the minimum system voltage (VSYS) threshold via the VOFF_SET
     * register. The value is clamped to the valid range and quantized to 100mV
     * steps.
     *
     * @param mv Desired minimum system voltage in millivolts (2600-3300mV).
     * @retval true  Voltage was set successfully.
     * @retval false I2C communication failed.
     */
    bool setMinimumSystemVoltage(uint32_t mv) override
    {
        if (mv < 2600) mv = 2600;
        if (mv > 3300) mv = 3300;
        uint8_t code = (mv - 2600) / 100;
        return _core.updateBits(Regs::pmu::VOFF_SET, 0x07, code) >= 0;
    }

    /**
     * @brief Get minimum system voltage.
     * @return Minimum system voltage in millivolts, or 0 on error.
     */
    uint32_t getMinimumSystemVoltage() const override
    {
        int val = _core.readReg(Regs::pmu::VOFF_SET);
        if (val < 0) return 0;
        return 2600 + (val & 0x07) * 100;
    }

    /**
     * @brief Set input voltage limit (not supported by AXP1xx).
     * @param mv Voltage in millivolts (ignored).
     * @return Always false.
     */
    bool setInputVoltageLimit(uint32_t mv) override
    {
        (void)mv;
        return false;
    }

    /**
     * @brief Get input voltage limit (not supported by AXP1xx).
     * @return Always 0.
     */
    uint32_t getInputVoltageLimit() const override
    {
        return 0;
    }

    /**
     * @brief Set input current limit.
     *
     * Configures the maximum input current drawn from VBUS via the IPS_SET
     * register (bits 1-0). Exact encoding differs between AXP192 and AXP202
     * and is provided by Regs::pmu::encodeInputCurrentLimit().
     *
     * @param mA Input current limit in milliamps.
     *           Request values are quantized to chip-supported levels.
     * @retval true  Current limit was set successfully.
     * @retval false I2C communication failed.
     */
    bool setInputCurrentLimit(uint32_t mA) override
    {
        uint8_t code = Regs::pmu::encodeInputCurrentLimit(mA);
        return _core.updateBits(Regs::pmu::IPS_SET, Regs::pmu::INPUT_CURR_MASK, code) >= 0;
    }

    /**
     * @brief Get input current limit.
     * @return Input current limit in milliamps (0, 100, 500, or 900), or 0 on error.
     */
    uint32_t getInputCurrentLimit() const override
    {
        int val = _core.readReg(Regs::pmu::IPS_SET);
        if (val < 0) return 0;
        return Regs::pmu::decodeInputCurrentLimit(static_cast<uint8_t>(val));
    }

    /**
     * @brief Enable or disable ship mode.
     *
     * Ship mode disconnects the battery from the system by setting
     * OFF_CTL register bit 7. When enabled and no external power is present,
     * the system is completely powered down.
     *
     * @param enable true to enter ship mode, false to exit.
     * @retval true  Mode was set successfully.
     * @retval false I2C communication failed.
     */
    bool enableShipMode(bool enable) override
    {
        if (enable) {
            return _core.setRegBit(Regs::pmu::OFF_CTL, 7);
        }
        return _core.clrRegBit(Regs::pmu::OFF_CTL, 7);
    }

    /**
     * @brief Check if ship mode is enabled.
     * @return true if ship mode is active, false otherwise.
     */
    bool isShipModeEnabled() const override
    {
        return _core.getRegBit(Regs::pmu::OFF_CTL, 7);
    }

private:
    SensorCommWrapper &_core;
};

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
 * @file      AXP1xxLed.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-28
 *
 * @brief AXP192/AXP202 Charge Indicator LED Control Interface
 *
 * This template class manages the charge indicator LED via the OFF_CTL
 * register. Supports automatic charge indication, manual level control,
 * and blinking modes at 1 Hz or 4 Hz.
 *
 * @tparam Regs Register map struct type (AXP192Regs or AXP202Regs)
 */
#pragma once
#include "../../PmicLedBase.hpp"
#include "../../../platform/SensorCommWrapper.hpp"

template<typename Regs>
class AXP1xxLed : public PmicLedBase
{
public:
    /**
     * @brief Construct LED control interface.
     * @param core Reference to the I2C communication wrapper.
     */
    explicit AXP1xxLed(SensorCommWrapper &core) : _core(core) {}

    /**
     * @brief Set LED output type (no-op, always succeeds).
     * @param type Output type (ignored).
     * @return Always true.
     */
    bool setOutputType(OutputType type) override
    {
        (void)type;
        return true;
    }

    /**
     * @brief Set LED operating mode.
     *
     * Controls OFF_CTL register bit 3.
     * - AUTO:    LED indicates charge state automatically (bit 3 = 0).
     * - MANUAL:  LED controlled by setManualState() (bit 3 = 1).
     * - DISABLE: LED forced high-impedance (bit 3 = 1, control bits = 00).
     *
     * @param mode LED operating mode.
     * @retval true  Mode was set successfully.
     * @retval false Invalid mode or I2C communication failed.
     */
    bool setMode(Mode mode) override
    {
        switch (mode) {
        case Mode::AUTO:
            return _core.clrRegBit(Regs::pmu::OFF_CTL, 3);
        case Mode::MANUAL:
            return _core.setRegBit(Regs::pmu::OFF_CTL, 3);
        case Mode::DISABLE:
            return _core.setRegBit(Regs::pmu::OFF_CTL, 3) &&
                   _core.updateBits(Regs::pmu::OFF_CTL, 0x30, 0x00) >= 0;
        default:
            return false;
        }
    }

    /**
     * @brief Get the current LED operating mode.
     * @return Current Mode value.
     */
    Mode getMode() override
    {
        if (!_core.getRegBit(Regs::pmu::OFF_CTL, 3)) {
            return Mode::AUTO;
        }
        return Mode::MANUAL;
    }

    /**
     * @brief Set the LED state in manual mode.
     *
     * Automatically switches to manual mode if not already set.
     * Programs bits 5-4 of the OFF_CTL register.
     *
     * @param state Desired LED state.
     *              LEVEL_LOW:  drive pin low (0x30).
     *              BLINK_1HZ:  blink at 1 Hz (0x10).
     *              BLINK_4HZ:  blink at 4 Hz (0x20).
     *              HiZ:        high-impedance (0x00).
     *
     * @note ManualState::LEVEL_HIGH is not supported by AXP1xx CHGLED hardware.
     * @retval true  State was set successfully.
     * @retval false Invalid state or I2C communication failed.
     */
    bool setManualState(ManualState state) override
    {
        if (!_core.setRegBit(Regs::pmu::OFF_CTL, 3)) {
            return false;
        }
        uint8_t val;
        switch (state) {
        case ManualState::HiZ:        val = 0x00; break;
        case ManualState::BLINK_1HZ:  val = 0x10; break;
        case ManualState::BLINK_4HZ:  val = 0x20; break;
        case ManualState::LEVEL_LOW:  val = 0x30; break;
        case ManualState::LEVEL_HIGH: return false;
        default: return false;
        }
        return _core.updateBits(Regs::pmu::OFF_CTL, 0x30, val) >= 0;
    }

    /**
     * @brief Get the current LED state in manual mode.
     * @return Current ManualState, or ManualState::UNDEFINED if in auto mode
     *         or on error.
     */
    ManualState getManualState() override
    {
        if (!_core.getRegBit(Regs::pmu::OFF_CTL, 3)) {
            return ManualState::UNDEFINED;
        }
        int val = _core.readReg(Regs::pmu::OFF_CTL);
        if (val < 0) return ManualState::UNDEFINED;
        uint8_t manualBits = val & 0x30;
        switch (manualBits) {
        case 0x00: return ManualState::HiZ;
        case 0x10: return ManualState::BLINK_1HZ;
        case 0x20: return ManualState::BLINK_4HZ;
        case 0x30: return ManualState::LEVEL_LOW;
        default:   return ManualState::UNDEFINED;
        }
    }

private:
    SensorCommWrapper &_core;
};

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
 * @file      AXP1xxTimer.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-05-04
 *
 * @brief AXP192/AXP202 Timer Interface (REG 8AH)
 *
 * Template implementation of PmicTimerBase for AXP192 and AXP202.
 * Controls the 7-bit countdown timer in REG 8AH.
 *
 * REG 8AH layout:
 *   Bit 7   : Timeout status (W1C) — set when timer reaches zero
 *   Bits 6-0: Timer value in minutes (0 = off, 1-127 = active)
 *
 * @tparam Regs Register map struct type (axp192_regs or axp202_regs)
 */
#pragma once

#include "../../PmicTimerBase.hpp"
#include "../../../platform/SensorCommWrapper.hpp"

template<typename Regs>
class AXP1xxTimer : public PmicTimerBase
{
public:
    explicit AXP1xxTimer(SensorCommWrapper &core) : _core(core) {}

    /**
     * @brief Start the timer with the given duration, or stop it.
     *
     * Writes the minute value to REG8AH bits[6:0].
     * Also clears any pending timeout status (bit 7) by writing 1.
     *
     * @param minutes 0 to stop, 1-127 to start.
     * @retval true  Success.
     * @retval false Out of range or I2C error.
     */
    bool setTimer(uint8_t minutes) override
    {
        if (minutes > 127) return false;
        // Write timer value to bits[6:0]; also clear timeout flag (bit 7 = W1C)
        uint8_t val = minutes | 0x80;
        return _core.writeReg(Regs::timer::TIMER_CTL, val) >= 0;
    }

    /**
     * @brief Read the current timer value from REG8AH bits[6:0].
     * @return Timer value in minutes (0 = stopped), or 0 on error.
     */
    uint8_t getTimer() const override
    {
        int val = _core.readReg(Regs::timer::TIMER_CTL);
        if (val < 0) return 0;
        return static_cast<uint8_t>(val & 0x7F);
    }

    /**
     * @brief Check whether the timer has expired (bit 7 of REG8AH).
     * @retval true  Timer has timed out.
     * @retval false Still running, stopped, or I2C error.
     */
    bool isTimedOut() const override
    {
        return _core.getRegBit(Regs::timer::TIMER_CTL, 7);
    }

    /**
     * @brief Clear the timeout status flag (write 1 to bit 7 of REG8AH).
     * @retval true  Cleared successfully.
     * @retval false I2C error.
     */
    bool clearTimeout() override
    {
        return _core.setRegBit(Regs::timer::TIMER_CTL, 7);
    }

private:
    SensorCommWrapper &_core;
};

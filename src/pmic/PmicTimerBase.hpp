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
 * @file      PmicTimerBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-30
 *
 * @brief PMIC Timer interface.
 *
 * This interface controls the general-purpose countdown timer
 */
#pragma once

#include <stdint.h>

class PmicTimerBase
{
public:
    virtual ~PmicTimerBase() = default;

    /**
     * @brief Start the timer with the given duration, or stop it.
     *
     * Writing a non-zero value starts (or restarts) the countdown.
     * Writing 0 stops the timer.
     *
     * @param minutes Timeout in minutes.
     * @retval true  Timer was configured successfully.
     * @retval false Value out of range or I2C error.
     */
    virtual bool setTimer(uint8_t minutes) = 0;

    /**
     * @brief Read the current timer value.
     * @return Timer value in minutes (0 = stopped), or 0 on error.
     */
    virtual uint8_t getTimer() const = 0;

    /**
     * @brief Check whether the timer has expired.
     *
     * @retval true  Timer has counted down to zero.
     * @retval false Timer is still running, or stopped, or I2C error.
     */
    virtual bool isTimedOut() const = 0;

    /**
     * @brief Clear the timeout status flag.
     *
     * @retval true  Flag was cleared successfully.
     * @retval false I2C communication failed.
     */
    virtual bool clearTimeout() = 0;
};

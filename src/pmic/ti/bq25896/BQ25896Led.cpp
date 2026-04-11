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
 * @file      BQ25896Led.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-11
 *
 */
#include "BQ25896Led.hpp"
#include "BQ25896Regs.hpp"

using namespace BQ25896Regs;

BQ25896Led::BQ25896Led(BQ25896Core &core) : _core(core) {}

bool BQ25896Led::setOutputType(OutputType type)
{
    (void)type;
    return false;
}

bool BQ25896Led::setMode(Mode mode)
{
    if (mode == Mode::DISABLE) {
        return _core.updateBits(REG_CHG_TIMER, MASK_STAT_DIS, MASK_STAT_DIS) >= 0;
    } else if (mode == Mode::AUTO) {
        return _core.updateBits(REG_CHG_TIMER, MASK_STAT_DIS, 0) >= 0;
    }
    return false;
}

PmicLedBase::Mode BQ25896Led::getMode()
{
    int val = _core.readReg(REG_CHG_TIMER);
    if (val < 0) return Mode::VENDOR;
    return (val & MASK_STAT_DIS) ? Mode::DISABLE : Mode::AUTO;
}

bool BQ25896Led::setManualState(ManualState state)
{
    (void)state;
    return false;
}

PmicLedBase::ManualState BQ25896Led::getManualState()
{
    return ManualState::UNDEFINED;
}

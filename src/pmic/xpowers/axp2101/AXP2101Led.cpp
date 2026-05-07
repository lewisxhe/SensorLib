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
 * @file      AXP2101Led.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Charging LED Control
 *
 */
#include "AXP2101Led.hpp"
#include "AXP2101Regs.hpp"


AXP2101Led::AXP2101Led(AXP2101Core &core) : _core(core)
{
}

// Bit 6 (0x40) is RO per datasheet — CHGLED is always open-drain.
bool AXP2101Led::setOutputType(OutputType type)
{
    (void)type;
    return false;
}

bool AXP2101Led::setMode(Mode mode)
{
    switch (mode) {
    case Mode::AUTO:
        return _core.updateBits(axp2101_regs::chg::CHGLED_CFG, 0x06, 0x00) >= 0;
    case Mode::MANUAL:
        return _core.updateBits(axp2101_regs::chg::CHGLED_CFG, 0x06, 0x04) >= 0;
    case Mode::DISABLE:
        return _core.updateBits(axp2101_regs::chg::CHGLED_CFG, 0x01, 0x00) >= 0;
    default:
        return false;
    }
}

AXP2101Led::Mode AXP2101Led::getMode()
{
    int val = _core.readReg(axp2101_regs::chg::CHGLED_CFG);
    if (val < 0) return Mode::VENDOR;
    if ((val & 0x01) == 0) return Mode::DISABLE;
    switch (val & 0x06) {
    case 0x04: return Mode::MANUAL;
    default:   return Mode::AUTO;
    }
}

// Datasheet REG69H bits 5:4:
//   00 = HiZ        01 = 1 Hz blink    10 = 4 Hz blink    11 = drive low
// On open-drain: LEVEL_HIGH and HiZ are both HiZ (LED off),
// LEVEL_LOW = drive low (LED on).
// Readback aliases 0x00 → LEVEL_HIGH so toggle patterns work.
bool AXP2101Led::setManualState(ManualState state)
{
    uint8_t code;
    switch (state) {
    case ManualState::LEVEL_HIGH:
    case ManualState::HiZ:        code = 0x00; break;
    case ManualState::BLINK_1HZ:  code = 0x01; break;
    case ManualState::BLINK_4HZ:  code = 0x02; break;
    case ManualState::LEVEL_LOW:  code = 0x03; break;
    default: return false;
    }
    if (_core.updateBits(axp2101_regs::chg::CHGLED_CFG, 0x06, 0x04) < 0) {
        return false;
    }
    return _core.updateBits(axp2101_regs::chg::CHGLED_CFG, 0x30, code << 4) >= 0;
}

AXP2101Led::ManualState AXP2101Led::getManualState()
{
    int val = _core.readReg(axp2101_regs::chg::CHGLED_CFG);
    if (val < 0) return ManualState::UNDEFINED;
    if ((val & 0x06) != 0x04) {
        return ManualState::UNDEFINED;
    }
    uint8_t code = (val >> 4) & 0x03;
    switch (code) {
    case 0x00: return ManualState::LEVEL_HIGH;
    case 0x01: return ManualState::BLINK_1HZ;
    case 0x02: return ManualState::BLINK_4HZ;
    case 0x03: return ManualState::LEVEL_LOW;
    default:   return ManualState::UNDEFINED;
    }
}

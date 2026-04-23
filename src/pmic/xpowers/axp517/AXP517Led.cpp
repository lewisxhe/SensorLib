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
 * @file      AXP517Led.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#include "AXP517Led.hpp"
#include "AXP517Regs.hpp"

using namespace axp517::regs;


AXP517Led::AXP517Led(AXP517Core &core) : _core(core)
{
}

bool AXP517Led::setOutputType(OutputType type)
{
    switch (type) {
    case OutputType::OpenDrain:
        return _core.updateBits(led::CHGLED_CFG, 0x80, 0x00);
    case OutputType::PushPull:
        return _core.updateBits(led::CHGLED_CFG, 0x80, 0x80);
    default:
        break;
    }
    return false;
}

bool AXP517Led::setMode(Mode mode)
{
    if (mode == Mode::DISABLE) {
        return _core.clrRegBit(ctrl::MODULE_EN1, 2); // Disable LED module
    }
    _core.setRegBit(ctrl::MODULE_EN1, 2); // Ensure LED module is enabled
    switch (mode) {
    case Mode::AUTO:
        return _core.updateBits(led::CHGLED_CFG, 0x47, 0x00);
    case Mode::MANUAL:
        return _core.updateBits(led::CHGLED_CFG, 0x47, 0x06);
    case Mode::BREATH:
        return _core.updateBits(led::CHGLED_CFG, 0x47, 0x43);
    default:
        return false;
    }
}

AXP517Led::Mode AXP517Led::getMode()
{
    if (!_core.getRegBit(ctrl::MODULE_EN1, 2)) {
        return Mode::DISABLE; // LED module is disabled
    }
    int regVal = _core.readReg(led::CHGLED_CFG);
    if (regVal < 0) return Mode::VENDOR;
    regVal &= 0x47;
    switch (regVal) {
    case 0x00: return Mode::AUTO;
    case 0x06: return Mode::MANUAL;
    case 0x43: return Mode::BREATH;
    default: break;
    }
    return Mode::VENDOR;
}

bool AXP517Led::setManualState(ManualState state)
{
    uint8_t base = 0x06;
    int val;
    switch (state) {
    case ManualState::HiZ:        val = 0x00; break;
    case ManualState::BLINK_1HZ:  val = 0x08; break;
    case ManualState::BLINK_4HZ:  val = 0x10; break;
    case ManualState::LEVEL_LOW:  val = 0x18; break;
    case ManualState::LEVEL_HIGH: val = 0x20; break;
    default: return false;
    }
    return _core.updateBits(led::CHGLED_CFG, 0x3F, base | val);
}

AXP517Led::ManualState AXP517Led::getManualState()
{
    int regVal = _core.readReg(led::CHGLED_CFG);
    if (regVal < 0) return ManualState::UNDEFINED;
    uint8_t modeBits = regVal & 0x07;
    if (modeBits != 0x06) {
        return ManualState::UNDEFINED;
    }
    uint8_t manualBits = (regVal >> 3) & 0x07;
    switch (manualBits) {
    case 0x00: return ManualState::HiZ;
    case 0x01: return ManualState::BLINK_1HZ;
    case 0x02: return ManualState::BLINK_4HZ;
    case 0x03: return ManualState::LEVEL_LOW;
    case 0x04: return ManualState::LEVEL_HIGH;
    default:   return ManualState::UNDEFINED;
    }
}

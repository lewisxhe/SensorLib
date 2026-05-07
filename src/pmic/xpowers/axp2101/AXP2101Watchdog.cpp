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
 * @file      AXP2101Watchdog.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Watchdog Timer
 *
 */
#include "AXP2101Watchdog.hpp"
#include "AXP2101Regs.hpp"


AXP2101Watchdog::AXP2101Watchdog(AXP2101Core &core) : _core(core)
{
}

bool AXP2101Watchdog::enableWatchdog(bool enable)
{
    return _core.enableModule(AXP2101Core::Module::WATCHDOG, enable);
}

bool AXP2101Watchdog::isWatchdogEnabled() const
{
    return _core.isModuleEnabled(AXP2101Core::Module::WATCHDOG);
}

bool AXP2101Watchdog::setWatchdogTimeout(uint16_t timeout_s)
{
    uint8_t val;
    switch (timeout_s) {
    case 1:   val = 0; break;
    case 2:   val = 1; break;
    case 4:   val = 2; break;
    case 8:   val = 3; break;
    case 16:  val = 4; break;
    case 32:  val = 5; break;
    case 64:  val = 6; break;
    case 128: val = 7; break;
    default: return false;
    }
    return _core.updateBits(axp2101_regs::ctrl::WDT_CTRL, 0x07, val) >= 0;
}

uint16_t AXP2101Watchdog::getWatchdogTimeout() const
{
    int val = _core.readReg(axp2101_regs::ctrl::WDT_CTRL);
    if (val < 0) return 0;
    const uint16_t timeouts[] = {1, 2, 4, 8, 16, 32, 64, 128};
    uint8_t idx = val & 0x07;
    if (idx >= 8) return 0;
    return timeouts[idx];
}

bool AXP2101Watchdog::resetWatchdog()
{
    return _core.setRegBit(axp2101_regs::ctrl::WDT_CTRL, 3);
}

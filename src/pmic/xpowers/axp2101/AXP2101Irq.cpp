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
 * @file      AXP2101Irq.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Interrupt Controller
 *
 */
#include "AXP2101Irq.hpp"
#include "AXP2101Regs.hpp"


AXP2101Irq::AXP2101Irq(AXP2101Core &core) : _core(core)
{
}

bool AXP2101Irq::enable(uint64_t mask)
{
    uint8_t buffer[3];
    if (_core.readRegBuff(axp2101_regs::irq::ENABLE1, buffer, sizeof(buffer)) < 0) {
        return false;
    }
    uint64_t current = (static_cast<uint64_t>(buffer[2]) << 16) |
                       (static_cast<uint64_t>(buffer[1]) << 8) |
                       (static_cast<uint64_t>(buffer[0]));
    current |= mask;
    buffer[2] = static_cast<uint8_t>((current >> 16) & 0xFF);
    buffer[1] = static_cast<uint8_t>((current >> 8) & 0xFF);
    buffer[0] = static_cast<uint8_t>(current & 0xFF);
    return _core.writeRegBuff(axp2101_regs::irq::ENABLE1, buffer, sizeof(buffer)) >= 0;
}

bool AXP2101Irq::disable(uint64_t mask)
{
    uint8_t buffer[3];
    if (_core.readRegBuff(axp2101_regs::irq::ENABLE1, buffer, sizeof(buffer)) < 0) {
        return false;
    }
    uint64_t current = (static_cast<uint64_t>(buffer[2]) << 16) |
                       (static_cast<uint64_t>(buffer[1]) << 8) |
                       (static_cast<uint64_t>(buffer[0]));
    current &= ~mask;
    buffer[2] = static_cast<uint8_t>((current >> 16) & 0xFF);
    buffer[1] = static_cast<uint8_t>((current >> 8) & 0xFF);
    buffer[0] = static_cast<uint8_t>(current & 0xFF);
    return _core.writeRegBuff(axp2101_regs::irq::ENABLE1, buffer, sizeof(buffer)) >= 0;
}

uint64_t AXP2101Irq::readStatus(bool clear)
{
    uint8_t buffer[3];
    if (_core.readRegBuff(axp2101_regs::irq::STATUS1, buffer, sizeof(buffer)) < 0) {
        return 0;
    }
    uint64_t mask = (static_cast<uint64_t>(buffer[2]) << 16) |
                    (static_cast<uint64_t>(buffer[1]) << 8) |
                    (static_cast<uint64_t>(buffer[0]));
    if (clear && mask != 0) {
        // Only clear the bits that were actually read to avoid losing
        // IRQs that arrived between the read and the clear (W1C race).
        _core.writeRegBuff(axp2101_regs::irq::STATUS1, buffer, sizeof(buffer));
    }
    return mask;
}

bool AXP2101Irq::clearStatus()
{
    uint8_t buffer[3] = {0xFF, 0xFF, 0xFF};
    return _core.writeRegBuff(axp2101_regs::irq::STATUS1, buffer, sizeof(buffer)) >= 0;
}

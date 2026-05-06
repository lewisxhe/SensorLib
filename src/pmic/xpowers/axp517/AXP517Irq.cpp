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
 * @file      AXP517Irq.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#include "AXP517Irq.hpp"
#include "AXP517Regs.hpp"


AXP517Irq::AXP517Irq(AXP517Core &core) : _core(core)
{
}

bool AXP517Irq::enable(uint64_t mask)
{
    mask &= ~IRQ_RESERVED_MASK;

    uint8_t buffer[4];

    if (_core.readRegBuff(axp517_regs::irq::ENABLE0, buffer, sizeof(buffer)) < 0) {
        log_e("Failed to read IRQ enable registers");
        return false;
    }
    uint64_t current = (static_cast<uint64_t>(buffer[0]) << 24) |
                       (static_cast<uint64_t>(buffer[1]) << 16) |
                       (static_cast<uint64_t>(buffer[2]) << 8) |
                       (static_cast<uint64_t>(buffer[3]));
    current &= (~IRQ_RESERVED_MASK);
    current |= mask;
    buffer[0] = static_cast<uint8_t>((current >> 24) & 0xFF);
    buffer[1] = static_cast<uint8_t>((current >> 16) & 0xFF);
    buffer[2] = static_cast<uint8_t>((current >> 8) & 0xFF);
    buffer[3] = static_cast<uint8_t>(current & 0xFF);
    if (_core.writeRegBuff(axp517_regs::irq::ENABLE0, buffer, sizeof(buffer)) < 0) {
        return false;
    }
    return true;
}

bool AXP517Irq::disable(uint64_t mask)
{
    mask &= ~IRQ_RESERVED_MASK;

    uint8_t buffer[4];
    if (_core.readRegBuff(axp517_regs::irq::ENABLE0, buffer, sizeof(buffer)) < 0) {
        log_e("Failed to read IRQ enable registers");
        return false;
    }
    uint64_t current = (static_cast<uint64_t>(buffer[0]) << 24) |
                       (static_cast<uint64_t>(buffer[1]) << 16) |
                       (static_cast<uint64_t>(buffer[2]) << 8) |
                       (static_cast<uint64_t>(buffer[3]));

    current &= (~IRQ_RESERVED_MASK);
    current &= (~mask);
    buffer[0] = static_cast<uint8_t>((current >> 24) & 0xFF);
    buffer[1] = static_cast<uint8_t>((current >> 16) & 0xFF);
    buffer[2] = static_cast<uint8_t>((current >> 8) & 0xFF);
    buffer[3] = static_cast<uint8_t>(current & 0xFF);
    if (_core.writeRegBuff(axp517_regs::irq::ENABLE0, buffer, sizeof(buffer)) < 0) {
        return false;
    }
    return true;
}

uint64_t AXP517Irq::readStatus(bool clear)
{
    uint8_t buffer[4];
    if (_core.readRegBuff(axp517_regs::irq::STATUS0, buffer, sizeof(buffer)) < 0) {
        log_e("Failed to read IRQ status registers");
        return 0;
    }
    uint64_t mask = (static_cast<uint64_t>(buffer[0]) << 24) |
                    (static_cast<uint64_t>(buffer[1]) << 16) |
                    (static_cast<uint64_t>(buffer[2]) << 8) |
                    (static_cast<uint64_t>(buffer[3]));
    mask &= (~IRQ_RESERVED_MASK);
    if (clear && mask != 0) {
        this->clearStatus();
    }
    return mask;
}

bool AXP517Irq::clearStatus()
{
    uint8_t buffer[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    if (_core.writeRegBuff(axp517_regs::irq::STATUS0, buffer, sizeof(buffer)) < 0) {
        log_e("Failed to clear IRQ status registers");
        return false;
    }
    return true;
}

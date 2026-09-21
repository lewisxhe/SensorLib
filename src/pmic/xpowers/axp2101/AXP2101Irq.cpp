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
#include "../../../SensorBuildOpt.h"
#if !SENSORLIB_EXCLUDE_PMIC_AXP2101

#include "AXP2101Irq.hpp"
#include "AXP2101Regs.hpp"


AXP2101Irq::AXP2101Irq(AXP2101Core &core) : _core(core)
{
}

bool AXP2101Irq::enable(uint64_t mask)
{
    for (uint8_t i = 0; i < 3; ++i) {
        const uint8_t byte_mask = static_cast<uint8_t>((mask >> (i * 8)) & 0xFF);
        if (byte_mask == 0) continue;

        const int current = _core.readReg(axp2101_regs::irq::ENABLE1 + i);
        if (current < 0) return false;

        const uint8_t next = static_cast<uint8_t>(current) | byte_mask;
        if (next != static_cast<uint8_t>(current) &&
                _core.writeReg(axp2101_regs::irq::ENABLE1 + i, next) < 0) {
            return false;
        }
    }
    return true;
}

bool AXP2101Irq::disable(uint64_t mask)
{
    for (uint8_t i = 0; i < 3; ++i) {
        const uint8_t byte_mask = static_cast<uint8_t>((mask >> (i * 8)) & 0xFF);
        if (byte_mask == 0) continue;

        const int current = _core.readReg(axp2101_regs::irq::ENABLE1 + i);
        if (current < 0) return false;

        const uint8_t next = static_cast<uint8_t>(current) & ~byte_mask;
        if (next != static_cast<uint8_t>(current) &&
                _core.writeReg(axp2101_regs::irq::ENABLE1 + i, next) < 0) {
            return false;
        }
    }
    return true;
}

uint64_t AXP2101Irq::readStatus(bool clear)
{
    uint8_t buffer[3];
    for (uint8_t i = 0; i < 3; ++i) {
        const int value = _core.readReg(axp2101_regs::irq::STATUS1 + i);
        if (value < 0) return 0;
        buffer[i] = static_cast<uint8_t>(value);
    }
    uint64_t mask = (static_cast<uint64_t>(buffer[2]) << 16) |
                    (static_cast<uint64_t>(buffer[1]) << 8) |
                    (static_cast<uint64_t>(buffer[0]));
    if (clear && mask != 0) {
        // Only clear the bits that were actually read to avoid losing
        // IRQs that arrived between the read and the clear (W1C race).
        for (uint8_t i = 0; i < 3; ++i) {
            if (buffer[i] != 0) {
                _core.writeReg(axp2101_regs::irq::STATUS1 + i, buffer[i]);
            }
        }
    }
    return mask;
}

bool AXP2101Irq::clearStatus()
{
    for (uint8_t i = 0; i < 3; ++i) {
        if (_core.writeReg(axp2101_regs::irq::STATUS1 + i, 0xFF) < 0) {
            return false;
        }
    }
    return true;
}

#endif

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
 * @file      AXP517Pwron.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#include "AXP517Pwron.hpp"
#include "AXP517Regs.hpp"

using namespace axp517::regs;

AXP517Pwron::AXP517Pwron(AXP517Core &core)
    : _core(core)
{
}

bool AXP517Pwron::updateBits(uint8_t mask, uint8_t value)
{
    int v = _core.readReg(ctrl::PWRON_CFG);
    if (v < 0) return false;
    uint8_t cur = (uint8_t)v;
    uint8_t next = (uint8_t)((cur & ~mask) | (value & mask));
    if (next == cur) return true;
    return _core.writeReg(ctrl::PWRON_CFG, next) >= 0;
}


bool AXP517Pwron::setOnLevel(uint16_t ms)
{
    uint8_t idx = (ms >= 2000) ? 3 :
                  (ms >= 1000) ? 2 :
                  (ms >= 512)  ? 1 : 0;
    return updateBits(0x03, idx);
}

bool AXP517Pwron::getOnLevel(uint16_t &ms) const
{
    // ONLEVEL: 00=128ms, 01=512ms, 10=1000ms, 11=2000ms
    const uint16_t kOnLevelMs[] = {128, 512, 1000, 2000};
    int v = _core.readReg(ctrl::PWRON_CFG);
    if (v < 0) return false;
    ms = kOnLevelMs[((uint8_t)v) & 0x03];
    return true;
}

bool AXP517Pwron::setOffLevel(uint16_t ms)
{
    uint8_t idx = (ms >= 10000) ? 3 :
                  (ms >= 8000)  ? 2 :
                  (ms >= 6000)  ? 1 : 0;
    return updateBits(0x0C, idx << 2);
}

bool AXP517Pwron::getOffLevel(uint16_t &ms) const
{
    // OFFLEVEL: 00=4000ms, 01=6000ms, 10=8000ms, 11=10000ms
    const uint16_t kOffLevelMs[] = {4000, 6000, 8000, 10000};
    int v = _core.readReg(ctrl::PWRON_CFG);
    if (v < 0) return false;
    ms = kOffLevelMs[(((uint8_t)v) >> 2) & 0x03];
    return true;
}


bool AXP517Pwron::setIrqLevel(uint16_t ms)
{
    uint8_t idx = (ms >= 2500) ? 3 :
                  (ms >= 2000) ? 2 :
                  (ms >= 1500) ? 1 : 0;
    return updateBits(0x30, idx << 4);
}

bool AXP517Pwron::getIrqLevel(uint16_t &ms) const
{
    // IRQLEVEL: 00=1000ms, 01=1500ms, 10=2000ms, 11=2500ms
    const uint16_t kIrqLevelMs[] = {1000, 1500, 2000, 2500};
    int v = _core.readReg(ctrl::PWRON_CFG);
    if (v < 0) return false;
    ms = kIrqLevelMs[(((uint8_t)v) >> 4) & 0x03];
    return true;
}

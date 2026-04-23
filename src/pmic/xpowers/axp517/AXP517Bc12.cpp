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
 * @file      AXP517Bc12.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#include "AXP517Bc12.hpp"
#include "AXP517Regs.hpp"

using namespace axp517::regs;

AXP517Bc12::AXP517Bc12(AXP517Core &core)
    : _core(core)
{
}

bool AXP517Bc12::enableAutoDetect(bool enable)
{
    // REG 2BH: BC1.2 control3, bit6: auto dpdm detect enable
    const uint8_t reg = bc12::CTRL3;
    const uint8_t bit = 6;
    return enable ? _core.setRegBit(reg, bit) : _core.clrRegBit(reg, bit);
}

bool AXP517Bc12::triggerDetect()
{
    // REG 2BH: BC1.2 control3, bit7: force dpdm detection (write 1, self-clear)
    const uint8_t reg = bc12::CTRL3;
    int cur = _core.readReg(reg);
    if (cur < 0) return false;
    cur |= 0x80;
    return _core.writeReg(reg, (uint8_t)cur) >= 0;
}

bool AXP517Bc12::isDetecting()
{
    const uint8_t reg = bc12::CTRL0;
    int cur = _core.readReg(reg);
    if (cur < 0) return false;
    return (cur & 0x04) != 0;
}

PmicBc12Base::Result AXP517Bc12::readResult()
{
    Result out{};
    out.detecting = isDetecting();
    int v = _core.readReg(bmu::BC_DETECT);
    if (v < 0) {
        out.type = PortType::Unknown;
        return out;
    }

    uint8_t code = (uint8_t)((v >> 5) & 0x07);
    out.raw = code;
    out.type = mapResult(code);
    return out;
}

PmicBc12Base::PortType AXP517Bc12::mapResult(uint8_t code)
{
    switch (code) {
    case 0b001: return PortType::SDP;
    case 0b010: return PortType::CDP;
    case 0b011: return PortType::DCP;
    case 0b000: return PortType::None;
    default:    return PortType::Unknown;
    }
}

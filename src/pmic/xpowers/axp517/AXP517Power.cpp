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
 * @file      AXP517Power.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#include "AXP517Power.hpp"
#include "AXP517Regs.hpp"


AXP517Power::AXP517Power(AXP517Core &core) : _core(core)
{
}

bool AXP517Power::setMinimumSystemVoltage(uint32_t mv)
{
    if (mv < 1000) mv = 1000;
    if (mv > 3800) mv = 3800;
    uint8_t code = (mv - 1000) / 100;
    return _core.updateBits(axp517_regs::ctrl::MIN_SYS_VOLTAGE, 0x1F, code);
}

uint32_t AXP517Power::getMinimumSystemVoltage() const
{
    int regVal = _core.readReg(axp517_regs::ctrl::MIN_SYS_VOLTAGE);
    if (regVal < 0) return 0;
    uint8_t code = regVal & 0x1F;
    return 1000 + code * 100;
}

bool AXP517Power::setInputVoltageLimit(uint32_t mv)
{
    if (mv < 3600) mv = 3600;
    if (mv > 16200) mv = 16200;
    uint8_t code = (mv - 3600) / 100 + 1;
    return _core.updateBits(axp517_regs::ctrl::INPUT_VOLT_LIMIT, 0x7F, code);
}

uint32_t AXP517Power::getInputVoltageLimit() const
{
    int regVal = _core.readReg(axp517_regs::ctrl::INPUT_VOLT_LIMIT);
    if (regVal < 0) return 0;
    uint8_t code = regVal & 0x7F;
    if (code == 0) return 3600;
    return 3600 + (code - 1) * 100;
}

bool AXP517Power::setInputCurrentLimit(uint32_t mA)
{
    if (mA < 100) mA = 100;
    if (mA > 3250) mA = 3250;
    uint8_t code = (mA - 100) / 50;
    return _core.updateBits(axp517_regs::ctrl::INPUT_CURR_LIMIT, 0xFC, code << 2);
}

uint32_t AXP517Power::getInputCurrentLimit() const
{
    int regVal = _core.readReg(axp517_regs::ctrl::INPUT_CURR_LIMIT);
    if (regVal < 0) return 0;
    uint8_t code = (regVal >> 2) & 0x3F;
    return 100 + code * 50;
}

bool AXP517Power::enableBoost(bool enable)
{
    return _core.updateBits(axp517_regs::ctrl::MODULE_EN1, 0x10, enable ? 0x10 : 0x00);
}

bool AXP517Power::isBoostEnabled() const
{
    return _core.getRegBit(axp517_regs::ctrl::MODULE_EN1, 4);
}

bool AXP517Power::setBoostVoltage(uint16_t mv)
{
    if (mv < 4550) mv = 4550;
    if (mv > 5510) mv = 5510;
    uint8_t n = ((mv - 4550) / 64);
    if (n > 15) n = 15;
    int regVal = _core.readReg(axp517_regs::ctrl::BOOST_CFG);
    if (regVal < 0) return false;
    return _core.updateBits(axp517_regs::ctrl::BOOST_CFG, 0xF0, n << 4);
}

uint16_t AXP517Power::getBoostVoltage() const
{
    int regVal = _core.readReg(axp517_regs::ctrl::BOOST_CFG);
    if (regVal < 0) return 0;
    uint8_t n = (regVal >> 4) & 0x0F;
    return 4550 + n * 64;
}

bool AXP517Power::enableShipMode(bool enable)
{
    return _core.setRegBit(axp517_regs::ctrl::BATFET_CTRL, 2);
}

bool AXP517Power::isShipModeEnabled() const
{
    return _core.getRegBit(axp517_regs::bmu::STATUS0, 4);
}

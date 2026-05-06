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
 * @file      AXP517Core.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#include "AXP517Core.hpp"
#include "AXP517Regs.hpp"
#include "AXP517TcpcRegs.hpp"


bool AXP517Core::enableModule(Module module, bool enable)
{
    uint8_t reg = 0, bit = 0;
    switch (module) {
    case Module::BC12:            reg = axp517_regs::bmu::MODULE_EN0;  bit = 4; break;
    case Module::TYPEC:           reg = axp517_regs::bmu::MODULE_EN0;  bit = 3; break;
    case Module::GAUGE:           reg = axp517_regs::bmu::MODULE_EN0;  bit = 2; break;
    case Module::WATCHDOG:        {
        bit = 0;
        reg = axp517_regs::bmu::MODULE_EN0;
        enable ? setRegBit(reg, bit) : clrRegBit(reg, bit);
        reg = axp517_regs::ctrl::MODULE_EN1;
        enable ? setRegBit(reg, bit) : clrRegBit(reg, bit);
    }
    break;
    case Module::GAUGE_LOW_POWER: reg = axp517_regs::ctrl::MODULE_EN1; bit = 6; break;
    case Module::BOOST:           reg = axp517_regs::ctrl::MODULE_EN1; bit = 4; break;
    case Module::BUCK:            reg = axp517_regs::ctrl::MODULE_EN1; bit = 3; break;
    case Module::CHGLED:          reg = axp517_regs::ctrl::MODULE_EN1; bit = 2; break;
    case Module::CHARGE:          reg = axp517_regs::ctrl::MODULE_EN1; bit = 1; break;
    case Module::MPPT:            reg = axp517_regs::ctrl::MPPT_CFG;   bit = 0; break;
    default:
        return false;
    }
    return enable ? setRegBit(reg, bit) : clrRegBit(reg, bit);
}

bool AXP517Core::isModuleEnabled(Module module)
{
    uint8_t reg = 0, bit = 0;
    switch (module) {
    case Module::BC12:            reg = axp517_regs::bmu::MODULE_EN0;  bit = 4; break;
    case Module::TYPEC:           reg = axp517_regs::bmu::MODULE_EN0;  bit = 3; break;
    case Module::GAUGE:           reg = axp517_regs::bmu::MODULE_EN0;  bit = 2; break;
    case Module::WATCHDOG:        {
        return getRegBit(axp517_regs::bmu::MODULE_EN0, 0) && getRegBit(axp517_regs::ctrl::MODULE_EN1, 0);
    }
    break;
    case Module::GAUGE_LOW_POWER: reg = axp517_regs::ctrl::MODULE_EN1; bit = 6; break;
    case Module::BOOST:           reg = axp517_regs::ctrl::MODULE_EN1; bit = 4; break;
    case Module::BUCK:            reg = axp517_regs::ctrl::MODULE_EN1; bit = 3; break;
    case Module::CHGLED:          reg = axp517_regs::ctrl::MODULE_EN1; bit = 2; break;
    case Module::CHARGE:          reg = axp517_regs::ctrl::MODULE_EN1; bit = 1; break;
    case Module::MPPT:            {
        int val = readReg(axp517_regs::ctrl::MPPT_CFG);
        if (val < 0)return false;
        return (val & 0x03) == 0x03;
    }
    default:
        return false;
    }
    return getRegBit(reg, bit);
}

bool AXP517Core::initImpl(uint8_t param)
{
    // todo: fix
#if 0
    uint8_t buffer[2] = {0};
    uint16_t vendorId = 0;

    if (readRegBuff(axp517::tcpc::TCPC_VENDOR_ID, buffer, sizeof(buffer)) < 0) {
        return false;
    }
    vendorId = (buffer[1] << 8) | buffer[0];
    if (vendorId != axp517::tcpc::VENDOR_ID) {
        log_e("AXP517 not found, vendor ID: 0x%04X", vendorId);
        return false;
    }
#endif
    // Enable CC module clock (REG 0BH bit3 = Type-C CC detect enable)
    log_d("AXP517: Enabling CC module clock...");
    if (!enableModule(Module::TYPEC, true)) {
        log_e("Failed to enable CC clock");
        // Continue anyway - may not be fatal
    }
    // Wait for CC module ready
    hal->delay(20);

    log_d("AXP517: Init complete, initializing...");


    int cur = readReg(axp517_regs::ctrl::INPUT_VOLT_LIMIT);
    if (cur < 0) return false;
    cur &= 0x80;
    // Set volt limit to 4700mV
    if (cur < 12) {
        if (writeReg(axp517_regs::ctrl::INPUT_VOLT_LIMIT, cur | 12) < 0) {
            return false;
        }
    }
    cur = readReg(axp517_regs::ctrl::INPUT_VOLT_LIMIT);
    if (cur < 0) return false;
    cur &= 0x7F;
    return cur == 12;
}

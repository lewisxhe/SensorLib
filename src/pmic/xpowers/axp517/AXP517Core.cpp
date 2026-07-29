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

namespace
{
static constexpr uint16_t AXP517_VENDOR_ID_ALT = 0xE0C5;
static constexpr uint8_t TCPC_POWER_STATUS_UNINIT = 0x40;
} // namespace


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
    uint8_t buffer[2] = {0};
    uint16_t vendorId = 0;

    // Enable CC module clock (REG 0BH bit3 = Type-C CC detect enable)
    // SENSORLIB_LOG_D("AXP517: Enabling CC module clock...");
    if (!enableModule(Module::TYPEC, true)) {
        SENSORLIB_LOG_E("Failed to enable CC clock");
        // Continue anyway - may not be fatal
    }
    // Wait for CC module ready
    hal->delay(20);

    if (writeReg(axp517::tcpc::CC_GENERAL_CONTROL, axp517::tcpc::SW_RESET) < 0) {
        SENSORLIB_LOG_E("Failed to assert TCPC software reset");
        return false;
    }
    hal->delay(50);
    if (writeReg(axp517::tcpc::CC_GENERAL_CONTROL, 0x00) < 0) {
        SENSORLIB_LOG_E("Failed to release TCPC software reset");
        return false;
    }
    hal->delay(300);

    int powerStatus = 0;
    bool tcpc_initial = false;
    for (uint8_t i = 0; i < 40; ++i) {
        powerStatus = readReg(axp517::tcpc::POWER_STATUS);
        if (powerStatus < 0) {
            SENSORLIB_LOG_E("Failed to read TCPC power status");
            return false;
        }
        tcpc_initial = (powerStatus & TCPC_POWER_STATUS_UNINIT) == 0;
        if (tcpc_initial) {
            break;
        }
        hal->delay(50);
    }

    if (!tcpc_initial) {
        SENSORLIB_LOG_E("TCPC initialization failed, POWER_STATUS: 0x%" PRIx32,
                        static_cast<uint32_t>(powerStatus));
        return false;
    }

    if (readRegBuff(axp517::tcpc::TCPC_VENDOR_ID, buffer, sizeof(buffer)) < 0) {
        SENSORLIB_LOG_E("Failed to read TCPC vendor ID");
        return false;
    }
    vendorId = (buffer[1] << 8) | buffer[0];
    if (vendorId != axp517::tcpc::VENDOR_ID && vendorId != AXP517_VENDOR_ID_ALT) {
        SENSORLIB_LOG_E("AXP517 not found, vendor ID: 0x%" PRIx32,
                        static_cast<uint32_t>(vendorId));
        return false;
    }

    SENSORLIB_LOG_D("AXP517: Init complete, initializing... Vendor ID: 0x%" PRIx32,
                    static_cast<uint32_t>(vendorId));

    return true;
}

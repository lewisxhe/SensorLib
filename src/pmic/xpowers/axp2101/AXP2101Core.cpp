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
 * @file      AXP2101Core.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Core I2C Communication and Module Control
 *
 */
#include "AXP2101Core.hpp"
#include "AXP2101Regs.hpp"

bool AXP2101Core::enableModule(Module module, bool enable)
{
    switch (module) {
    case Module::CELL_CHARGE:
        return enable ? setRegBit(axp2101_regs::ctrl::MODULE_EN, 1) : clrRegBit(axp2101_regs::ctrl::MODULE_EN, 1);
    case Module::BTN_CHARGE:
        return enable ? setRegBit(axp2101_regs::ctrl::MODULE_EN, 2) : clrRegBit(axp2101_regs::ctrl::MODULE_EN, 2);
    case Module::WATCHDOG:
        return enable ? setRegBit(axp2101_regs::ctrl::MODULE_EN, 0) : clrRegBit(axp2101_regs::ctrl::MODULE_EN, 0);
    case Module::GAUGE:
        return enable ? setRegBit(axp2101_regs::ctrl::MODULE_EN, 3) : clrRegBit(axp2101_regs::ctrl::MODULE_EN, 3);
    case Module::BAT_DETECT:
        return enable ? setRegBit(axp2101_regs::chg::BAT_DETECT_CTRL, 0) : clrRegBit(axp2101_regs::chg::BAT_DETECT_CTRL, 0);
    case Module::GENERAL_ADC:
        return enable ? setRegBit(axp2101_regs::adc::CHANNEL_CTRL, 5) : clrRegBit(axp2101_regs::adc::CHANNEL_CTRL, 5);
    case Module::TEMP_MEASURE:
        return enable ? setRegBit(axp2101_regs::adc::CHANNEL_CTRL, 4) : clrRegBit(axp2101_regs::adc::CHANNEL_CTRL, 4);
    case Module::VSYS_MEASURE:
        return enable ? setRegBit(axp2101_regs::adc::CHANNEL_CTRL, 3) : clrRegBit(axp2101_regs::adc::CHANNEL_CTRL, 3);
    case Module::VBUS_MEASURE:
        return enable ? setRegBit(axp2101_regs::adc::CHANNEL_CTRL, 2) : clrRegBit(axp2101_regs::adc::CHANNEL_CTRL, 2);
    case Module::TS_MEASURE:
        return enable ? setRegBit(axp2101_regs::adc::CHANNEL_CTRL, 1) : clrRegBit(axp2101_regs::adc::CHANNEL_CTRL, 1);
    case Module::BAT_VOLT_MEASURE:
        return enable ? setRegBit(axp2101_regs::adc::CHANNEL_CTRL, 0) : clrRegBit(axp2101_regs::adc::CHANNEL_CTRL, 0);
    default:
        return false;
    }
}

bool AXP2101Core::isModuleEnabled(Module module)
{
    switch (module) {
    case Module::CELL_CHARGE:
        return getRegBit(axp2101_regs::ctrl::MODULE_EN, 1);
    case Module::BTN_CHARGE:
        return getRegBit(axp2101_regs::ctrl::MODULE_EN, 2);
    case Module::WATCHDOG:
        return getRegBit(axp2101_regs::ctrl::MODULE_EN, 0);
    case Module::GAUGE:
        return getRegBit(axp2101_regs::ctrl::MODULE_EN, 3);
    case Module::BAT_DETECT:
        return getRegBit(axp2101_regs::chg::BAT_DETECT_CTRL, 0);
    case Module::GENERAL_ADC:
        return getRegBit(axp2101_regs::adc::CHANNEL_CTRL, 5);
    case Module::TEMP_MEASURE:
        return getRegBit(axp2101_regs::adc::CHANNEL_CTRL, 4);
    case Module::VSYS_MEASURE:
        return getRegBit(axp2101_regs::adc::CHANNEL_CTRL, 3);
    case Module::VBUS_MEASURE:
        return getRegBit(axp2101_regs::adc::CHANNEL_CTRL, 2);
    case Module::TS_MEASURE:
        return getRegBit(axp2101_regs::adc::CHANNEL_CTRL, 1);
    case Module::BAT_VOLT_MEASURE:
        return getRegBit(axp2101_regs::adc::CHANNEL_CTRL, 0);
    default:
        return false;
    }
}

bool AXP2101Core::setWatchdogConfig(WatchdogConfig config)
{
    // Bit 5:4 : 00 = IRQ only, 01 = IRQ + reset, 10 = IRQ + reset + pull down PWROK, 11 = IRQ + reset after power off
    return updateBits(axp2101_regs::ctrl::WDT_CTRL, static_cast<uint8_t>(config), 0x30);
}

bool AXP2101Core::initImpl(uint8_t param)
{
    int id = readReg(axp2101_regs::bmu::IC_TYPE);
    if (id < 0 || id != axp2101_regs::CHIP_ID) {
        return false;
    }
    int val = readReg(axp2101_regs::ts::TS_PIN_CTRL);
    if (val >= 0) {
        val &= 0xF0;
        writeReg(axp2101_regs::ts::TS_PIN_CTRL, val | 0x10);
        clrRegBit(axp2101_regs::adc::CHANNEL_CTRL, 1);
    }
    return true;
}

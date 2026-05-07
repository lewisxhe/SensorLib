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
 * @file      AXP202AdcTraits_impl.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-05-02
 *
 * @brief Implementation of AXP202-specific ADC traits
 *
 * This file contains the inline/template implementations for AXP202AdcTraits.
 * Included by AXP202AdcTraits.hpp.
 */
#pragma once

#include "AXP202AdcTraits.hpp"
#include "AXP202Regs.hpp"

namespace axp1xx {

inline uint32_t AdcTraitsBase<AXP202Core>::getChannelMask(PmicAdcBase::Channel ch)
{
    switch (ch) {
    case PmicAdcBase::Channel::VBUS_VOLTAGE:
    case PmicAdcBase::Channel::VBUS_CURRENT:
        return (ch == PmicAdcBase::Channel::VBUS_VOLTAGE) ? 0x08 : 0x04;
    case PmicAdcBase::Channel::ACIN_VOLTAGE:
    case PmicAdcBase::Channel::ACIN_CURRENT:
        return (ch == PmicAdcBase::Channel::ACIN_VOLTAGE) ? 0x20 : 0x10;
    case PmicAdcBase::Channel::BAT_VOLTAGE:
    case PmicAdcBase::Channel::BAT_CURRENT:
        return (ch == PmicAdcBase::Channel::BAT_VOLTAGE) ? 0x80 : 0x40;
    case PmicAdcBase::Channel::VSYS_VOLTAGE:
        return 0x02;  // APS_VOLTAGE
    case PmicAdcBase::Channel::DIE_TEMPERATURE:
        return 0x8000;  // REG83H bit 7
    case PmicAdcBase::Channel::BAT_PERCENTAGE:
        return 0;  // No enable mask needed for battery percentage
    default:
        return 0;
    }
}

inline uint16_t AdcTraitsBase<AXP202Core>::readRegisterH8L4(AXP202Core &core, uint8_t regH, uint8_t regL)
{
    int h = core.readReg(regH);
    int l = core.readReg(regL);
    if (h < 0 || l < 0) {
        return 0xFFFF;
    }
    return (static_cast<uint16_t>(h) << 4) | (static_cast<uint16_t>(l) & 0x0F);
}

inline bool AdcTraitsBase<AXP202Core>::readChannel(AXP202Core &core, PmicAdcBase::Channel ch, float &out)
{
    // Check device state for VBUS and BAT channels
    if (ch == PmicAdcBase::Channel::VBUS_VOLTAGE || ch == PmicAdcBase::Channel::VBUS_CURRENT) {
        int s0 = core.readReg(axp202_regs::status::STATUS);
        if (s0 < 0) return false;
        if (!((s0 >> 5) & 0x01)) {
            out = 0.0f;
            return true;
        }
    }
    if (ch == PmicAdcBase::Channel::BAT_VOLTAGE || ch == PmicAdcBase::Channel::BAT_CURRENT) {
        int s1 = core.readReg(axp202_regs::status::MODE_CHGSTATUS);
        if (s1 < 0) return false;
        if (!((s1 >> 5) & 0x01)) {
            out = 0.0f;
            return true;
        }
    }

    switch (ch) {
    case PmicAdcBase::Channel::VBUS_VOLTAGE:
    {
        uint16_t raw = readRegisterH8L4(core, axp202_regs::adc::VBUS_VOL_H, axp202_regs::adc::VBUS_VOL_L);
        if (raw == 0xFFFF) return false;
        out = static_cast<float>(raw) * axp202_regs::factory::FACTORY_VBUS_VOLTAGE;
        break;
    }

    case PmicAdcBase::Channel::VBUS_CURRENT:
    {
        uint16_t raw = readRegisterH8L4(core, axp202_regs::adc::VBUS_CUR_H, axp202_regs::adc::VBUS_CUR_L);
        if (raw == 0xFFFF) return false;
        out = static_cast<float>(raw) * axp202_regs::factory::FACTORY_VBUS_CURRENT;
        break;
    }

    case PmicAdcBase::Channel::BAT_VOLTAGE:
    {
        uint16_t raw = readRegisterH8L4(core, axp202_regs::adc::BAT_AVERVOL_H, axp202_regs::adc::BAT_AVERVOL_L);
        if (raw == 0xFFFF) return false;
        out = static_cast<float>(raw) * axp202_regs::factory::FACTORY_VBAT_VOLTAGE;
        break;
    }

    case PmicAdcBase::Channel::BAT_CURRENT:
    {
        int s0 = core.readReg(axp202_regs::status::STATUS);
        if (s0 < 0) return false;
        bool charging = (s0 >> 2) & 0x01;
        uint8_t regH = charging ? axp202_regs::adc::BAT_AVERCHGCUR_H : axp202_regs::adc::BAT_AVERDISCHGCUR_H;
        uint16_t raw;
        if (charging) {
            raw = readRegisterH8L4(core, regH, axp202_regs::adc::BAT_AVERCHGCUR_L);
            if (raw == 0xFFFF) return false;
        } else {
            int h = core.readReg(regH);
            int l = core.readReg(axp202_regs::adc::BAT_AVERDISCHGCUR_L);
            if (h < 0 || l < 0) return false;
            raw = (static_cast<uint16_t>(h) << 5) | (static_cast<uint16_t>(l) & 0x1F);
        }
        out = static_cast<float>(raw) * axp202_regs::factory::FACTORY_BAT_CURRENT;
        if (!charging) {
            out = -out;  // Negative for discharge
        }
        break;
    }

    case PmicAdcBase::Channel::VSYS_VOLTAGE:
    {
        uint16_t raw = readRegisterH8L4(core, axp202_regs::adc::APS_AVERVOL_H, axp202_regs::adc::APS_AVERVOL_L);
        if (raw == 0xFFFF) return false;
        out = static_cast<float>(raw) * axp202_regs::factory::FACTORY_APS_VOLTAGE;
        break;
    }

    case PmicAdcBase::Channel::DIE_TEMPERATURE:
    {
        uint16_t raw = readRegisterH8L4(core, axp202_regs::adc::INTERNAL_TEMP_H, axp202_regs::adc::INTERNAL_TEMP_L);
        if (raw == 0xFFFF) return false;
        out = static_cast<float>(raw) * axp202_regs::factory::FACTORY_INTERNAL_TEMP - axp202_regs::factory::FACTORY_INTERNAL_OFFSET;
        break;
    }

    case PmicAdcBase::Channel::BAT_TEMPERATURE:
        return false;

    case PmicAdcBase::Channel::BAT_PERCENTAGE:
    {
        int val = core.readReg(axp202_regs::adc::BATT_PERCENTAGE);
        if (val < 0) return false;
        out = static_cast<float>(val & 0x7F);
        break;
    }

    default:
        return false;
    }
    return true;
}

} // namespace axp1xx

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
 * @file      AXP192AdcTraits_impl.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief Implementation of AXP192-specific ADC traits
 *
 * This file contains the inline/template implementations for AXP192AdcTraits.
 * Included by AXP192AdcTraits.hpp.
 */
#pragma once

#include "AXP192AdcTraits.hpp"
#include "AXP192Regs.hpp"

namespace axp1xx
{

inline uint32_t AdcTraitsBase<AXP192Core>::getChannelMask(PmicAdcBase::Channel ch)
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
    default:
        return 0;
    }
}

inline uint16_t AdcTraitsBase<AXP192Core>::readH8L4(AXP192Core &core, uint8_t regH, uint8_t regL)
{
    int h = core.readReg(regH);
    int l = core.readReg(regL);
    if (h < 0 || l < 0) return 0;
    return (static_cast<uint16_t>(h) << 4) | (l & 0x0F);
}

inline uint16_t AdcTraitsBase<AXP192Core>::readH8L5(AXP192Core &core, uint8_t regH, uint8_t regL)
{
    int h = core.readReg(regH);
    int l = core.readReg(regL);
    if (h < 0 || l < 0) return 0;
    return (static_cast<uint16_t>(h) << 5) | (l & 0x1F);
}

inline bool AdcTraitsBase<AXP192Core>::readChannel(AXP192Core &core, PmicAdcBase::Channel ch, float &out)
{
    switch (ch) {
    case PmicAdcBase::Channel::VBUS_VOLTAGE: {
        int s = core.readReg(axp192_regs::bmu::STATUS);
        if (s < 0) return false;
        if (!((s >> 5) & 0x01)) {
            out = 0.0f;
            return true;
        }
        uint16_t raw = readH8L4(core, axp192_regs::adc::VBUS_VOL_H8, axp192_regs::adc::VBUS_VOL_L4);
        out = static_cast<float>(raw) * axp192_regs::factory::VBUS_VOLTAGE_STEP;
        break;
    }
    case PmicAdcBase::Channel::VBUS_CURRENT: {
        int s = core.readReg(axp192_regs::bmu::STATUS);
        if (s < 0) return false;
        if (!((s >> 5) & 0x01)) {
            out = 0.0f;
            return true;
        }
        uint16_t raw = readH8L4(core, axp192_regs::adc::VBUS_CUR_H8, axp192_regs::adc::VBUS_CUR_L4);
        out = static_cast<float>(raw) * axp192_regs::factory::VBUS_CURRENT_STEP;
        break;
    }
    case PmicAdcBase::Channel::BAT_VOLTAGE: {
        int s = core.readReg(axp192_regs::bmu::MODE_CHGSTATUS);
        if (s < 0) return false;
        if (!((s >> 5) & 0x01)) {
            out = 0.0f;
            return true;
        }
        uint16_t raw = readH8L4(core, axp192_regs::adc::BAT_AVERVOL_H8, axp192_regs::adc::BAT_AVERVOL_L4);
        out = static_cast<float>(raw) * axp192_regs::factory::BAT_VOLTAGE_STEP;
        break;
    }
    case PmicAdcBase::Channel::BAT_CURRENT: {
        int s = core.readReg(axp192_regs::bmu::MODE_CHGSTATUS);
        if (s < 0) return false;
        if (!((s >> 5) & 0x01)) {
            out = 0.0f;
            return true;
        }
        bool charging = (s >> 6) & 0x01;
        if (charging) {
            uint16_t raw = readH8L5(core, axp192_regs::adc::BAT_AVERCHGCUR_H8, axp192_regs::adc::BAT_AVERCHGCUR_L5);
            out = static_cast<float>(raw) * axp192_regs::factory::BAT_CHARGE_CUR_STEP;
        } else {
            uint16_t raw = readH8L5(core, axp192_regs::adc::BAT_AVERDISCHGCUR_H8, axp192_regs::adc::BAT_AVERDISCHGCUR_L5);
            out = -static_cast<float>(raw) * axp192_regs::factory::BAT_DISCHARGE_CUR_STEP;
        }
        break;
    }
    case PmicAdcBase::Channel::VSYS_VOLTAGE: {
        uint16_t raw = readH8L4(core, axp192_regs::adc::APS_AVERVOL_H8, axp192_regs::adc::APS_AVERVOL_L4);
        out = static_cast<float>(raw) * axp192_regs::factory::APS_VOLTAGE_STEP;
        break;
    }
    case PmicAdcBase::Channel::DIE_TEMPERATURE: {
        uint16_t raw = readH8L4(core, axp192_regs::adc::INTERNAL_TEMP_H8, axp192_regs::adc::INTERNAL_TEMP_L4);
        out = static_cast<float>(raw) * axp192_regs::factory::INTERNAL_TEMP_STEP - axp192_regs::factory::INTERNAL_TEMP_OFFSET;
        break;
    }
    case PmicAdcBase::Channel::ACIN_VOLTAGE: {
        int s = core.readReg(axp192_regs::bmu::STATUS);
        if (s < 0) return false;
        if (!((s >> 7) & 0x01)) {
            out = 0.0f;
            return true;
        }
        uint16_t raw = readH8L4(core, axp192_regs::ac::ACIN_VOL_H8, axp192_regs::ac::ACIN_VOL_L4);
        out = static_cast<float>(raw) * axp192_regs::factory::ACIN_VOLTAGE_STEP;
        break;
    }
    case PmicAdcBase::Channel::ACIN_CURRENT: {
        int s = core.readReg(axp192_regs::bmu::STATUS);
        if (s < 0) return false;
        if (!((s >> 7) & 0x01)) {
            out = 0.0f;
            return true;
        }
        uint16_t raw = readH8L4(core, axp192_regs::ac::ACIN_CUR_H8, axp192_regs::ac::ACIN_CUR_L4);
        out = static_cast<float>(raw) * axp192_regs::factory::ACIN_CURRENT_STEP;
        break;
    }
    case PmicAdcBase::Channel::BAT_TEMPERATURE:
    case PmicAdcBase::Channel::BAT_PERCENTAGE:
    default:
        return false;
    }
    return true;
}

} // namespace axp1xx

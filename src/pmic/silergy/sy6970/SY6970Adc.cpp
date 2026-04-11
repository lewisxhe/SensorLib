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
 * @file      SY6970Adc.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 */
#include "SY6970Adc.hpp"
#include "SY6970Regs.hpp"

using namespace SY6970Regs;

SY6970Adc::SY6970Adc(SY6970Core &core) : _core(core) {}

bool SY6970Adc::enableChannels(uint32_t mask)
{
    uint8_t val = MASK_CONV_START;
    if (mask & ADC_CONV_RATE) {
        val |= MASK_CONV_RATE;
    }
    return _core.updateBits(REG_PWR_ONOFF, MASK_CONV_START | MASK_CONV_RATE, val) >= 0;
}

bool SY6970Adc::disableChannels(uint32_t mask)
{
    (void)mask;
    return _core.updateBits(REG_PWR_ONOFF, MASK_CONV_START | MASK_CONV_RATE, 0) >= 0;
}

bool SY6970Adc::read(Channel ch, float &out)
{
    out = 0.0f;

    // Check if ADC conversion is in progress
    // CONV_START stays high during conversion and is read-only when CONV_RATE=1
    int ctrl = _core.readReg(REG_PWR_ONOFF);
    if (ctrl >= 0 && !(ctrl & MASK_CONV_START)) {
        // ADC not busy, trigger a one-shot conversion
        startConversion();
    }

    switch (ch) {
    case Channel::VBUS_VOLTAGE: {
        if (!_core.isVbusPresent()) {
            out = 0.0f;
            return true;
        }
        int val = _core.readReg(REG_ADC_BUSV);
        if (val < 0) return false;
        out = ((val & MASK_BUSV) * VBUS_VOL_STEP) + VBUS_BASE_VAL;
        break;
    }
    case Channel::BAT_VOLTAGE: {
        int val = _core.readReg(REG_ADC_BATV);
        if (val < 0) return false;
        val = val & MASK_BATV;
        if (val == 0) {
            out = 0.0f;
            return true;
        }
        out = (val * VBAT_VOL_STEP) + VBAT_BASE_VAL;
        break;
    }
    case Channel::VSYS_VOLTAGE: {
        int val = _core.readReg(REG_ADC_SYSV);
        if (val < 0) return false;
        out = ((val & MASK_SYSV) * VSYS_VOL_STEP) + VSYS_BASE_VAL;
        break;
    }
    case Channel::BAT_CURRENT: {
        if (!_core.isCharging()) {
            out = 0.0f;
            return true;
        }
        int val = _core.readReg(REG_ADC_ICHGR);
        if (val < 0 || (val & MASK_ICHGR) == 0) {
            out = 0.0f;
            return true;
        }
        out = (val & MASK_ICHGR) * CHG_STEP_VAL;
        break;
    }
    case Channel::BAT_TEMPERATURE: {
        int val = _core.readReg(REG_ADC_NTC);
        if (val < 0) return false;
        out = ((val & MASK_NTPCPT) * NTC_VOL_STEP) + NTC_BASE_VAL;
        break;
    }
    case Channel::DIE_TEMPERATURE: {
        int val = _core.readReg(REG_ADC_BATV);
        if (val < 0) return false;
        out = (val & MASK_THERM_STAT) ? 1.0f : 0.0f;
        break;
    }
    case Channel::VBUS_CURRENT:
    case Channel::BAT_PERCENTAGE:
    default:
        return false;
    }
    return true;
}

bool SY6970Adc::startConversion()
{
    return _core.updateBits(REG_PWR_ONOFF, MASK_CONV_START, MASK_CONV_START) >= 0;
}

bool SY6970Adc::setContinuousMode(bool enable)
{
    uint8_t val = MASK_CONV_START;
    if (enable) {
        val |= MASK_CONV_RATE;
    }
    return _core.updateBits(REG_PWR_ONOFF, MASK_CONV_START | MASK_CONV_RATE, val) >= 0;
}

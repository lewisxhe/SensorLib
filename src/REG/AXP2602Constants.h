/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
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
 * @file      AXP2602Constants.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-12-05
 *
 */

#pragma once

#include <stdint.h>

class AXP2602Constants
{
protected:
    // AXP2602 registers
    static constexpr uint8_t REG_ID = 0x00;
    static constexpr uint8_t REG_BROM = 0x01;
    static constexpr uint8_t REG_MODE = 0x02;
    static constexpr uint8_t REG_PARA_CONFIG = 0x03;
    static constexpr uint8_t REG_VBAT_ADC_HIGH = 0x04;
    static constexpr uint8_t REG_VBAT_ADC_LOW = 0x05;
    static constexpr uint8_t REG_TEMP_RESULT = 0x06;
    static constexpr uint8_t REG_SOH = 0x07;
    static constexpr uint8_t REG_SOC = 0x08;

    static constexpr uint8_t REG_TIME_TO_EMPTY_HIGH = 0x0A;
    static constexpr uint8_t REG_TIME_TO_EMPTY_LOW = 0x0B;
    static constexpr uint8_t REG_TIME_TO_FULL_HIGH = 0x0C;
    static constexpr uint8_t REG_TIME_TO_FULL_LOW = 0x0D;
    static constexpr uint8_t REG_LOW_SOC_THLD = 0x0E;
    static constexpr uint8_t REG_OT_THLD = 0x0F;

    static constexpr uint8_t REG_COMM_CONFIG = 0x11;
    static constexpr uint8_t REG_IBAT_ADC_HIGH = 0x14;
    static constexpr uint8_t REG_IBAT_ADC_LOW = 0x15;
    static constexpr uint8_t REG_RDC25_CONFIG_HIGH = 0x1A;
    static constexpr uint8_t REG_RDC25_CONFIG_LOW = 0x1B;
    static constexpr uint8_t REG_QMAX_CONFIG_HIGH = 0x1E;
    static constexpr uint8_t REG_QMAX_CONFIG_LOW = 0x1F;

    static constexpr uint8_t REG_IRQ_STATUS = 0x20;
    static constexpr uint8_t REG_IRQ_ENABLE = 0x21;
    static constexpr uint8_t REG_TDIE_ADC_HIGH = 0xC2;
    static constexpr uint8_t REG_TDIE_ADC_LOW = 0xC3;
    static constexpr uint8_t REG_RDC25_QMAX_SEL = 0xC5;
    static constexpr uint8_t REG_OPERATING_MODE = 0xC6;

    static constexpr uint8_t AXP2602_CHIP_ID = 0x1C;
};













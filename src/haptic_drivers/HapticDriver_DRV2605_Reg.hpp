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
 * @file      HapticDriver_DRV2605_Reg.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-01
 *
 */
#pragma once

#include <stdint.h>

// -----------------------
// Chip IDs (STATUS[7:5])
// -----------------------

static constexpr uint8_t DRV2604_CHIP_ID = 0x04;
static constexpr uint8_t DRV2605_CHIP_ID = 0x03;
static constexpr uint8_t DRV2604L_CHIP_ID = 0x06;
static constexpr uint8_t DRV2605L_CHIP_ID = 0x07;
static constexpr uint8_t DRV2605X_CHIP_ID = 0x05;

// -----------------------
// Register addresses
// -----------------------

static constexpr uint8_t DRV2605_REG_STATUS = 0x00;
static constexpr uint8_t DRV2605_REG_MODE = 0x01;
static constexpr uint8_t DRV2605_REG_RTPIN = 0x02;
static constexpr uint8_t DRV2605_REG_LIBRARY = 0x03;
static constexpr uint8_t DRV2605_REG_WAVESEQ1 = 0x04;
static constexpr uint8_t DRV2605_REG_WAVESEQ2 = 0x05;
static constexpr uint8_t DRV2605_REG_WAVESEQ3 = 0x06;
static constexpr uint8_t DRV2605_REG_WAVESEQ4 = 0x07;
static constexpr uint8_t DRV2605_REG_WAVESEQ5 = 0x08;
static constexpr uint8_t DRV2605_REG_WAVESEQ6 = 0x09;
static constexpr uint8_t DRV2605_REG_WAVESEQ7 = 0x0A;
static constexpr uint8_t DRV2605_REG_WAVESEQ8 = 0x0B;
static constexpr uint8_t DRV2605_REG_GO = 0x0C;
static constexpr uint8_t DRV2605_REG_OVERDRIVE = 0x0D;
static constexpr uint8_t DRV2605_REG_SUSTAINPOS = 0x0E;
static constexpr uint8_t DRV2605_REG_SUSTAINNEG = 0x0F;
static constexpr uint8_t DRV2605_REG_BREAK = 0x10;
static constexpr uint8_t DRV2605_REG_AUDIOCTRL = 0x11;
static constexpr uint8_t DRV2605_REG_AUDIOLVL = 0x12;
static constexpr uint8_t DRV2605_REG_AUDIOMAX = 0x13;
static constexpr uint8_t DRV2605_REG_AUDIOOUTMIN = 0x14;
static constexpr uint8_t DRV2605_REG_AUDIOOUTMAX = 0x15;
static constexpr uint8_t DRV2605_REG_RATEDV = 0x16;
static constexpr uint8_t DRV2605_REG_CLAMPV = 0x17;
static constexpr uint8_t DRV2605_REG_AUTOCALCOMP = 0x18;
static constexpr uint8_t DRV2605_REG_AUTOCALEMP = 0x19;
static constexpr uint8_t DRV2605_REG_FEEDBACK = 0x1A;
static constexpr uint8_t DRV2605_REG_CONTROL1 = 0x1B;
static constexpr uint8_t DRV2605_REG_CONTROL2 = 0x1C;
static constexpr uint8_t DRV2605_REG_CONTROL3 = 0x1D;
static constexpr uint8_t DRV2605_REG_CONTROL4 = 0x1E;
static constexpr uint8_t DRV2605_REG_VBAT = 0x21;
static constexpr uint8_t DRV2605_REG_LRARESON = 0x22;

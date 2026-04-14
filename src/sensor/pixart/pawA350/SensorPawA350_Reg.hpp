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
 * @file      SensorPawA350_Reg.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-10
 *
 */
#pragma once

#include <stdint.h>

namespace Paw350Regs
{

#ifdef _BV
#undef _BV
#endif
#define _BV(bit)  (1U << (bit))

static constexpr uint8_t SOFTRESET_VAL = 0x5A;

static constexpr uint8_t PAW_A350_DEFAULT_PID = 0x88;

static constexpr uint8_t CPI_DEFAULT = 0x22;

static constexpr uint8_t LED_DRV_13MA = 0;
static constexpr uint8_t LED_DRV_9_6MA = 2;
static constexpr uint8_t LED_DRV_27MA = 7;

static constexpr uint16_t SHUTTER_MAX_DEFAULT = 0x0B71;

static constexpr uint8_t MOTION_CTRL_THRESHOLD_MIN = 0;
static constexpr uint8_t MOTION_CTRL_THRESHOLD_MAX = 7;

static constexpr uint8_t REST1_PERIOD_MIN = 1;
static constexpr uint16_t REST1_PERIOD_MAX = 256;

static constexpr uint8_t REST2_PERIOD_MIN = 1;
static constexpr uint16_t REST2_PERIOD_MAX = 256;

static constexpr uint8_t REST3_PERIOD_MIN = 1;
static constexpr uint16_t REST3_PERIOD_MAX = 256;

static constexpr uint8_t DOWNSHIFT_TIME_MIN = 2;
static constexpr uint8_t DOWNSHIFT_TIME_MAX = 242;

// ==============================================
// 0x00 (RO)
// ==============================================
// Register : PRODUCT_ID
static constexpr uint8_t REG_PRODUCT_ID  = 0x00;
static constexpr uint8_t MASK_PID        = 0xFF;
static constexpr uint8_t SHIFT_PID       = 0;

// ==============================================
// 0x01 (RO)
// ==============================================
// Register : REVISION_ID
static constexpr uint8_t REG_REVISION_ID = 0x01;
static constexpr uint8_t MASK_RID        = 0xFF;
static constexpr uint8_t SHIFT_RID       = 0;

// ==============================================
// 0x02 (R/WC)
// ==============================================
// Register : EVENT
static constexpr uint8_t REG_EVENT       = 0x02;
static constexpr uint8_t MASK_MOTION      = _BV(7);
static constexpr uint8_t MASK_RESET_ST    = _BV(3);
static constexpr uint8_t MASK_FPD_ST     = _BV(0);

// ==============================================
// 0x03 (RO)
// ==============================================
// Register : DELTA_X
static constexpr uint8_t REG_DELTA_X     = 0x03;
static constexpr uint8_t MASK_DX          = 0xFF;
static constexpr uint8_t SHIFT_DX         = 0;

// ==============================================
// 0x04 (RO)
// ==============================================
// Register : DELTA_Y
static constexpr uint8_t REG_DELTA_Y     = 0x04;
static constexpr uint8_t MASK_DY          = 0xFF;
static constexpr uint8_t SHIFT_DY         = 0;

// ==============================================
// 0x11 (RW)
// ==============================================
// Register : REST1_PERIOD
static constexpr uint8_t REG_REST1_PERIOD = 0x11;
static constexpr uint8_t MASK_R1R         = 0xFF;
static constexpr uint8_t SHIFT_R1R         = 0;

// ==============================================
// 0x13 (RW)
// ==============================================
// Register : RUN_DOWNSHIFT
static constexpr uint8_t REG_RUN_DOWNSHIFT = 0x13;
static constexpr uint8_t MASK_RD           = 0xFF;
static constexpr uint8_t SHIFT_RD          = 0;

// ==============================================
// 0x15 (RW)
// ==============================================
// Register : REST1_DOWNSHIFT
static constexpr uint8_t REG_REST1_DOWNSHIFT = 0x15;
static constexpr uint8_t MASK_R1D           = 0xFF;
static constexpr uint8_t SHIFT_R1D           = 0;

// ==============================================
// 0x16 (RW)
// ==============================================
// Register : REST2_PERIOD
static constexpr uint8_t REG_REST2_PERIOD = 0x16;
static constexpr uint8_t MASK_R2R         = 0xFF;
static constexpr uint8_t SHIFT_R2R        = 0;

// ==============================================
// 0x17 (RW)
// ==============================================
// Register : REST2_DOWNSHIFT
static constexpr uint8_t REG_REST2_DOWNSHIFT = 0x17;
static constexpr uint8_t MASK_R2D           = 0xFF;
static constexpr uint8_t SHIFT_R2D           = 0;

// ==============================================
// 0x18 (RW)
// ==============================================
// Register : REST3_PERIOD
static constexpr uint8_t REG_REST3_PERIOD = 0x18;
static constexpr uint8_t MASK_R3R         = 0xFF;
static constexpr uint8_t SHIFT_R3R        = 0;

// ==============================================
// 0x1A (RW)
// ==============================================
// Register : LED_CTRL
static constexpr uint8_t REG_LED_CTRL    = 0x1A;
static constexpr uint8_t MASK_LED_ON     = _BV(3);
static constexpr uint8_t MASK_LED_DRV     = 0x07;
static constexpr uint8_t SHIFT_LED_DRV    = 0;

// ==============================================
// 0x3A (WO)
// ==============================================
// Register : SOFT_RESET
static constexpr uint8_t REG_SOFT_RESET  = 0x3A;
static constexpr uint8_t MASK_SFRST      = 0xFF;

// ==============================================
// 0x3B (RW)
// ==============================================
// Register : SHUTTER_MAX_HI
static constexpr uint8_t REG_SHUTTER_MAX_HI = 0x3B;
static constexpr uint8_t MASK_SMH             = 0xFF;
static constexpr uint8_t SHIFT_SMH            = 0;

// ==============================================
// 0x3C (RW)
// ==============================================
// Register : SHUTTER_MAX_LO
static constexpr uint8_t REG_SHUTTER_MAX_LO = 0x3C;
static constexpr uint8_t MASK_SML             = 0xFF;
static constexpr uint8_t SHIFT_SML            = 0;

// ==============================================
// 0x62 (RW)
// ==============================================
// Register : CPI_SEL
static constexpr uint8_t REG_CPI_SEL     = 0x62;
static constexpr uint8_t MASK_CPI_SEL    = 0x0F;
static constexpr uint8_t SHIFT_CPI_SEL   = 0;

// ==============================================
// 0x74 (RW)
// ==============================================
// Register : MOTION_CTRL
static constexpr uint8_t REG_MOTION_CTRL = 0x74;
static constexpr uint8_t MASK_MOTION_INT_THRES = 0x07;
static constexpr uint8_t SHIFT_MOTION_INT_THRES = 0;

// ==============================================
// 0x7A (RO)
// ==============================================
// Register : FPD_FLAG
static constexpr uint8_t REG_FPD_FLAG    = 0x7A;
static constexpr uint8_t MASK_FPD        = _BV(0);

} // namespace Paw350Regs

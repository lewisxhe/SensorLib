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
 * @file      HapticDriver_AW86224_Reg.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-01
 *
 */
#pragma once

#include <stdint.h>

#ifdef _BV
#undef _BV
#endif
#define _BV(bit)  (1U << (bit))

static constexpr uint8_t SOFTRESET_VAL = 0xAA;

static constexpr uint32_t EN_RAMINIT = _BV(3);
static constexpr uint32_t VBAT_GO = _BV(1);

static constexpr uint8_t PLAYCFG4_GO_ON = _BV(0);
static constexpr uint8_t PLAYCFG4_STOP_ON = _BV(1);

static constexpr uint8_t MASK_GLB_STATE = 0x0F;
static constexpr uint8_t STATE_STANDBY = 0;
static constexpr uint8_t STATE_RTP_GO = 8;

static constexpr uint8_t WAVLOOP_INIFINITELY = 0x0F;

static constexpr uint16_t AW_VBAT_MIN = 3000;
static constexpr uint16_t AW_VBAT_REFER = 4200;
static constexpr uint8_t AW_DEFAULT_GAIN = 0x80;

static constexpr uint8_t IRQ_NULL = 0;
static constexpr uint8_t IRQ_UVL = 1;
static constexpr uint8_t IRQ_OCD = 2;
static constexpr uint8_t IRQ_OT = 3;
static constexpr uint8_t IRQ_DONE = 4;
static constexpr uint8_t IRQ_ALMOST_FULL = 5;
static constexpr uint8_t IRQ_ALMOST_EMPTY = 6;

static constexpr uint8_t CONT_VBAT_SW_COMP_MODE = 0;
static constexpr uint8_t CONT_VBAT_HW_COMP_MODE = 1;

static constexpr uint16_t F0_PRE_VAL = 1700;
static constexpr uint16_t LRA_VRMS_VAL = 1000;
static constexpr uint8_t CONT_BRK_TIME_VAL = 0x06;
static constexpr uint8_t CONT_BRK_GAIN_VAL = 0x08;
static constexpr uint8_t D2S_GAIN_VAL = 0x05;
static constexpr uint8_t PWMCFG4_PRTIME_VAL = 0x32;
static constexpr uint8_t PWMCFG3_PRLVL_VAL = 0x3F;
static constexpr uint8_t CONT_DRV1_LVL_VAL = 0x7F;
static constexpr uint8_t F0_CALI_ACCURACY = 24;
static constexpr uint8_t CONT_DRV1_TIME_VAL = 0x04;
static constexpr uint8_t CONT_DRV2_TIME_VAL = 0x14;
static constexpr uint8_t CONT_TRACK_MARGIN_VAL = 0x0F;
static constexpr uint8_t GAIN_BYPASS_CHANGEABLE = _BV(6);

static constexpr uint8_t INT_MODE_EDGE = _BV(4);
static constexpr uint8_t INT_EDGE_MODE_POS = 0;


// ==============================================
// 0x00 (WO)
// ==============================================
// Register : SRST
static constexpr uint8_t REG_SRST    =  0x00;
static constexpr uint8_t MASK_RESET  = 0xFF;  // write 0xAA to reset

// ==============================================
// 0x01 (RO)
// ==============================================
// Register : SYSST
static constexpr uint8_t REG_SYSST    =  0x01;
static constexpr uint8_t MASK_DONES   = _BV(0);
static constexpr uint8_t MASK_OTS     = _BV(1);
static constexpr uint8_t MASK_OCDS    = _BV(2);
static constexpr uint8_t MASK_FF_AFS  = _BV(3);
static constexpr uint8_t MASK_FF_AES  = _BV(4);
static constexpr uint8_t MASK_UVLS    = _BV(5);

// ==============================================
// 0x02 (RC)
// ==============================================
// Register : SYSINT
static constexpr uint8_t REG_SYSINT   =  0x02;
static constexpr uint8_t MASK_DONEI   = _BV(0);
static constexpr uint8_t MASK_OTI     = _BV(1);
static constexpr uint8_t MASK_OCDI    = _BV(2);
static constexpr uint8_t MASK_FF_AFI  = _BV(3);
static constexpr uint8_t MASK_FF_AEI  = _BV(4);
static constexpr uint8_t MASK_UVLI    = _BV(5);

// ==============================================
// 0x03 (RW)
// ==============================================
// Register : SYSINTM
static constexpr uint8_t REG_SYSINTM  =  0x03;
static constexpr uint8_t MASK_DONEM   = _BV(0);
static constexpr uint8_t MASK_OTM     = _BV(1);
static constexpr uint8_t MASK_OCDM    = _BV(2);
static constexpr uint8_t MASK_FF_AFM  = _BV(3);
static constexpr uint8_t MASK_FF_AEM  = _BV(4);
static constexpr uint8_t MASK_UVLM    = _BV(5);

// ==============================================
// 0x04 (RO)
// ==============================================
// Register : SYSST2
static constexpr uint8_t REG_SYSST2   =  0x04;
static constexpr uint8_t MASK_LDO_OK  = _BV(3);

// ==============================================
// 0x07 (RW)
// ==============================================
// Register : PLAYCFG2
static constexpr uint8_t REG_PLAYCFG2 =  0x07;
static constexpr uint8_t MASK_GAIN    = 0xFF;
static constexpr uint8_t SHIFT_GAIN   = 0;

// ==============================================
// 0x08 (RW)
// ==============================================
// Register : PLAYCFG3
static constexpr uint8_t REG_PLAYCFG3 =  0x08;
static constexpr uint8_t MASK_STOP_MODE = _BV(5);
static constexpr uint8_t MASK_BRK_EN    = _BV(2);
static constexpr uint8_t MASK_PLAY_MODE = 0x03;
static constexpr uint8_t SHIFT_PLAY_MODE = 0;

// ==============================================
// 0x09 (RW)
// ==============================================
// Register : PLAYCFG4
static constexpr uint8_t REG_PLAYCFG4 =  0x09;
static constexpr uint8_t MASK_STOP     = _BV(1);
static constexpr uint8_t MASK_GO       = _BV(0);

// ==============================================
// 0x0A (RW)
// ==============================================
// Register : WAVCFG1
static constexpr uint8_t REG_WAVCFG1  =  0x0A;
static constexpr uint8_t MASK_SEQ1WAIT = _BV(7);
static constexpr uint8_t MASK_WAVSEQ1  = 0x7F;
static constexpr uint8_t SHIFT_WAVSEQ1 = 0;

// ==============================================
// 0x0B (RW)
// ==============================================
// Register : WAVCFG2
static constexpr uint8_t REG_WAVCFG2  =  0x0B;
static constexpr uint8_t MASK_SEQ2WAIT = _BV(7);
static constexpr uint8_t MASK_WAVSEQ2  = 0x7F;
static constexpr uint8_t SHIFT_WAVSEQ2 = 0;

// ==============================================
// 0x0C (RW)
// ==============================================
// Register : WAVCFG3
static constexpr uint8_t REG_WAVCFG3  =  0x0C;
static constexpr uint8_t MASK_SEQ3WAIT = _BV(7);
static constexpr uint8_t MASK_WAVSEQ3  = 0x7F;
static constexpr uint8_t SHIFT_WAVSEQ3 = 0;

// ==============================================
// 0x0D (RW)
// ==============================================
// Register : WAVCFG4
static constexpr uint8_t REG_WAVCFG4  =  0x0D;
static constexpr uint8_t MASK_SEQ4WAIT = _BV(7);
static constexpr uint8_t MASK_WAVSEQ4  = 0x7F;
static constexpr uint8_t SHIFT_WAVSEQ4 = 0;

// ==============================================
// 0x0E (RW)
// ==============================================
// Register : WAVCFG5
static constexpr uint8_t REG_WAVCFG5  =  0x0E;
static constexpr uint8_t MASK_SEQ5WAIT = _BV(7);
static constexpr uint8_t MASK_WAVSEQ5  = 0x7F;
static constexpr uint8_t SHIFT_WAVSEQ5 = 0;

// ==============================================
// 0x0F (RW)
// ==============================================
// Register : WAVCFG6
static constexpr uint8_t REG_WAVCFG6  =  0x0F;
static constexpr uint8_t MASK_SEQ6WAIT = _BV(7);
static constexpr uint8_t MASK_WAVSEQ6  = 0x7F;
static constexpr uint8_t SHIFT_WAVSEQ6 = 0;

// ==============================================
// 0x10 (RW)
// ==============================================
// Register : WAVCFG7
static constexpr uint8_t REG_WAVCFG7  =  0x10;
static constexpr uint8_t MASK_SEQ7WAIT = _BV(7);
static constexpr uint8_t MASK_WAVSEQ7  = 0x7F;
static constexpr uint8_t SHIFT_WAVSEQ7 = 0;

// ==============================================
// 0x11 (RW)
// ==============================================
// Register : WAVCFG8
static constexpr uint8_t REG_WAVCFG8  =  0x11;
static constexpr uint8_t MASK_SEQ8WAIT = _BV(7);
static constexpr uint8_t MASK_WAVSEQ8  = 0x7F;
static constexpr uint8_t SHIFT_WAVSEQ8 = 0;

// ==============================================
// 0x12 (RW)
// ==============================================
// Register : WAVCFG9
static constexpr uint8_t REG_WAVCFG9  =  0x12;
static constexpr uint8_t MASK_SEQ1LOOP = 0xF0;
static constexpr uint8_t SHIFT_SEQ1LOOP = 4;
static constexpr uint8_t MASK_SEQ2LOOP = 0x0F;
static constexpr uint8_t SHIFT_SEQ2LOOP = 0;

// ==============================================
// 0x13 (RW)
// ==============================================
// Register : WAVCFG10
static constexpr uint8_t REG_WAVCFG10 =  0x13;
static constexpr uint8_t MASK_SEQ3LOOP = 0xF0;
static constexpr uint8_t SHIFT_SEQ3LOOP = 4;
static constexpr uint8_t MASK_SEQ4LOOP = 0x0F;
static constexpr uint8_t SHIFT_SEQ4LOOP = 0;

// ==============================================
// 0x14 (RW)
// ==============================================
// Register : WAVCFG11
static constexpr uint8_t REG_WAVCFG11 =  0x14;
static constexpr uint8_t MASK_SEQ5LOOP = 0xF0;
static constexpr uint8_t SHIFT_SEQ5LOOP = 4;
static constexpr uint8_t MASK_SEQ6LOOP = 0x0F;
static constexpr uint8_t SHIFT_SEQ6LOOP = 0;

// ==============================================
// 0x15 (RW)
// ==============================================
// Register : WAVCFG12
static constexpr uint8_t REG_WAVCFG12 =  0x15;
static constexpr uint8_t MASK_SEQ7LOOP = 0xF0;
static constexpr uint8_t SHIFT_SEQ7LOOP = 4;
static constexpr uint8_t MASK_SEQ8LOOP = 0x0F;
static constexpr uint8_t SHIFT_SEQ8LOOP = 0;

// ==============================================
// 0x16 (RW)
// ==============================================
// Register : WAVCFG13
static constexpr uint8_t REG_WAVCFG13 =  0x16;
static constexpr uint8_t MASK_WAITSLOT  = 0x60;  // bits 6:5
static constexpr uint8_t SHIFT_WAITSLOT = 5;
static constexpr uint8_t MASK_MAINLOOP  = 0x0F;
static constexpr uint8_t SHIFT_MAINLOOP = 0;

// ==============================================
// 0x18 (RW)
// ==============================================
// Register : CONTCFG1
static constexpr uint8_t REG_CONTCFG1 =  0x18;
static constexpr uint8_t MASK_EDGE_FRE   = 0xF0;
static constexpr uint8_t SHIFT_EDGE_FRE  = 4;
static constexpr uint8_t MASK_EN_F0_DET  = _BV(3);
static constexpr uint8_t MASK_SIN_MODE   = _BV(0);

// ==============================================
// 0x19 (RW)
// ==============================================
// Register : CONTCFG2
static constexpr uint8_t REG_CONTCFG2 =  0x19;
static constexpr uint8_t MASK_F_PRE = 0xFF;
static constexpr uint8_t SHIFT_F_PRE = 0;

// ==============================================
// 0x1A (RW)
// ==============================================
// Register : CONTCFG3
static constexpr uint8_t REG_CONTCFG3 =  0x1A;
static constexpr uint8_t MASK_DRV_WIDTH = 0xFF;
static constexpr uint8_t SHIFT_DRV_WIDTH = 0;

// ==============================================
// 0x1C (RW)
// ==============================================
// Register : CONTCFG5
static constexpr uint8_t REG_CONTCFG5 =  0x1C;
static constexpr uint8_t MASK_BRK_GAIN = 0x0F;
static constexpr uint8_t SHIFT_BRK_GAIN = 0;

// ==============================================
// 0x1D (RW)
// ==============================================
// Register : CONTCFG6
static constexpr uint8_t REG_CONTCFG6 =  0x1D;
static constexpr uint8_t MASK_TRACK_EN  = _BV(7);
static constexpr uint8_t MASK_DRV1_LVL  = 0x7F;
static constexpr uint8_t SHIFT_DRV1_LVL = 0;

// ==============================================
// 0x1E (RW)
// ==============================================
// Register : CONTCFG7
static constexpr uint8_t REG_CONTCFG7 =  0x1E;
static constexpr uint8_t MASK_DRV2_LVL = 0x7F;
static constexpr uint8_t SHIFT_DRV2_LVL = 0;

// ==============================================
// 0x1F (RW)
// ==============================================
// Register : CONTCFG8
static constexpr uint8_t REG_CONTCFG8 =  0x1F;
static constexpr uint8_t MASK_DRV1_TIME = 0xFF;
static constexpr uint8_t SHIFT_DRV1_TIME = 0;

// ==============================================
// 0x20 (RW)
// ==============================================
// Register : CONTCFG9
static constexpr uint8_t REG_CONTCFG9 =  0x20;
static constexpr uint8_t MASK_DRV2_TIME = 0xFF;
static constexpr uint8_t SHIFT_DRV2_TIME = 0;

// ==============================================
// 0x21 (RW)
// ==============================================
// Register : CONTCFG10
static constexpr uint8_t REG_CONTCFG10 = 0x21;
static constexpr uint8_t MASK_BRK_TIME = 0xFF;
static constexpr uint8_t SHIFT_BRK_TIME = 0;

// ==============================================
// 0x22 (RW)
// ==============================================
// Register : CONTCFG11
static constexpr uint8_t REG_CONTCFG11 = 0x22;
static constexpr uint8_t MASK_TRACK_MARGIN = 0xFF;
static constexpr uint8_t SHIFT_TRACK_MARGIN = 0;

// ==============================================
// 0x25 (RO)
// ==============================================
// Register : CONTRD14
static constexpr uint8_t REG_CONTRD14 = 0x25;
static constexpr uint8_t MASK_F_LRA_F0_H = 0xFF;
static constexpr uint8_t SHIFT_F_LRA_F0_H = 0;

// ==============================================
// 0x26 (RO)
// ==============================================
// Register : CONTRD15
static constexpr uint8_t REG_CONTRD15 = 0x26;
static constexpr uint8_t MASK_F_LRA_F0_L = 0xFF;
static constexpr uint8_t SHIFT_F_LRA_F0_L = 0;

// ==============================================
// 0x27 (RO)
// ==============================================
// Register : CONTRD16
static constexpr uint8_t REG_CONTRD16 = 0x27;
static constexpr uint8_t MASK_CONT_F0_H = 0xFF;
static constexpr uint8_t SHIFT_CONT_F0_H = 0;

// ==============================================
// 0x28 (RO)
// ==============================================
// Register : CONTRD17
static constexpr uint8_t REG_CONTRD17 = 0x28;
static constexpr uint8_t MASK_CONT_F0_L = 0xFF;
static constexpr uint8_t SHIFT_CONT_F0_L = 0;

// ==============================================
// 0x2D (RW)
// ==============================================
// Register : RTPCFG1
static constexpr uint8_t REG_RTPCFG1 = 0x2D;
static constexpr uint8_t MASK_BASE_ADDR_H = 0x0F;
static constexpr uint8_t SHIFT_BASE_ADDR_H = 0;

// ==============================================
// 0x2E (RW)
// ==============================================
// Register : RTPCFG2
static constexpr uint8_t REG_RTPCFG2 = 0x2E;
static constexpr uint8_t MASK_BASE_ADDR_L = 0xFF;
static constexpr uint8_t SHIFT_BASE_ADDR_L = 0;

// ==============================================
// 0x2F (RW)
// ==============================================
// Register : RTPCFG3
static constexpr uint8_t REG_RTPCFG3 = 0x2F;
static constexpr uint8_t MASK_FIFO_AEH = 0xF0;
static constexpr uint8_t SHIFT_FIFO_AEH = 4;
static constexpr uint8_t MASK_FIFO_AFH = 0x0F;
static constexpr uint8_t SHIFT_FIFO_AFH = 0;

// ==============================================
// 0x30 (RW)
// ==============================================
// Register : RTPCFG4
static constexpr uint8_t REG_RTPCFG4 = 0x30;
static constexpr uint8_t MASK_FIFO_AEL = 0xFF;
static constexpr uint8_t SHIFT_FIFO_AEL = 0;

// ==============================================
// 0x31 (RW)
// ==============================================
// Register : RTPCFG5
static constexpr uint8_t REG_RTPCFG5 = 0x31;
static constexpr uint8_t MASK_FIFO_AFL = 0xFF;
static constexpr uint8_t SHIFT_FIFO_AFL = 0;

// ==============================================
// 0x32 (RW)
// ==============================================
// Register : RTPDATA
static constexpr uint8_t REG_RTPDATA = 0x32;
static constexpr uint8_t MASK_RTP_DATA = 0xFF;
static constexpr uint8_t SHIFT_RTP_DATA = 0;

// ==============================================
// 0x33 (RW)
// ==============================================
// Register : TRGCFG1
static constexpr uint8_t REG_TRGCFG1 = 0x33;
static constexpr uint8_t MASK_TRG1_POS   = _BV(7);
static constexpr uint8_t MASK_TRG1SEQ_P  = 0x7F;
static constexpr uint8_t SHIFT_TRG1SEQ_P = 0;

// ==============================================
// 0x36 (RW)
// ==============================================
// Register : TRGCFG4
static constexpr uint8_t REG_TRGCFG4 = 0x36;
static constexpr uint8_t MASK_TRG1_NEG   = _BV(7);
static constexpr uint8_t MASK_TRG1SEQ_N  = 0x7F;
static constexpr uint8_t SHIFT_TRG1SEQ_N = 0;

// ==============================================
// 0x39 (RW)
// ==============================================
// Register : TRGCFG7
static constexpr uint8_t REG_TRGCFG7 = 0x39;
static constexpr uint8_t MASK_TRG1_POLAR = _BV(7);
static constexpr uint8_t MASK_TRG1_LEV   = _BV(6);
static constexpr uint8_t MASK_TRG1_BRK   = _BV(5);

// ==============================================
// 0x3A (RW)
// ==============================================
// Register : TRGCFG8
static constexpr uint8_t REG_TRGCFG8 = 0x3A;
static constexpr uint8_t MASK_TRG1_MODE = 0x18;  // bits 4:3
static constexpr uint8_t SHIFT_TRG1_MODE = 3;
static constexpr uint8_t MASK_TRG1_STOP = _BV(2);

// ==============================================
// 0x3C (RW)
// ==============================================
// Register : GLBCFG2
static constexpr uint8_t REG_GLBCFG2 = 0x3C;
static constexpr uint8_t MASK_START_DLY = 0xFF;
static constexpr uint8_t SHIFT_START_DLY = 0;

// ==============================================
// 0x3E (RW)
// ==============================================
// Register : GLBCFG4
static constexpr uint8_t REG_GLBCFG4 = 0x3E;
static constexpr uint8_t MASK_GO_PRIO   = 0xC0;  // bits 7:6
static constexpr uint8_t SHIFT_GO_PRIO  = 6;
static constexpr uint8_t MASK_TRG1_PRIO = 0x03;  // bits 1:0
static constexpr uint8_t SHIFT_TRG1_PRIO = 0;

// ==============================================
// 0x3F (RO)
// ==============================================
// Register : GLBRD5
static constexpr uint8_t REG_GLBRD5 = 0x3F;
static constexpr uint8_t SHIFT_GLB_STATE = 0;

// ==============================================
// 0x40 (RW)
// ==============================================
// Register : RAMADDRH
static constexpr uint8_t REG_RAMADDRH = 0x40;
static constexpr uint8_t MASK_RAMADDRH = 0x0F;
static constexpr uint8_t SHIFT_RAMADDRH = 0;

// ==============================================
// 0x41 (RW)
// ==============================================
// Register : RAMADDRL
static constexpr uint8_t REG_RAMADDRL = 0x41;
static constexpr uint8_t MASK_RAMADDRL = 0xFF;
static constexpr uint8_t SHIFT_RAMADDRL = 0;

// ==============================================
// 0x42 (RW)
// ==============================================
// Register : RAMDATA
static constexpr uint8_t REG_RAMDATA = 0x42;
static constexpr uint8_t MASK_RAMDATA = 0xFF;
static constexpr uint8_t SHIFT_RAMDATA = 0;

// ==============================================
// 0x43 (RW)
// ==============================================
// Register : SYSCTRL1
static constexpr uint8_t REG_SYSCTRL1 = 0x43;
static constexpr uint8_t MASK_VBAT_MODE  = _BV(7);
static constexpr uint8_t MASK_EN_RAMINIT = _BV(3);
static constexpr uint8_t MASK_EN_FIR     = _BV(2);

// ==============================================
// 0x44 (RW)
// ==============================================
// Register : SYSCTRL2
static constexpr uint8_t REG_SYSCTRL2 = 0x44;
static constexpr uint8_t MASK_WAKE       = _BV(7);
static constexpr uint8_t MASK_STANDBY    = _BV(6);
static constexpr uint8_t MASK_INTN_PIN   = _BV(3);
static constexpr uint8_t MASK_WAVDAT_MODE = 0x03;
static constexpr uint8_t SHIFT_WAVDAT_MODE = 0;

// ==============================================
// 0x49 (RW)
// ==============================================
// Register : SYSCTRL7
static constexpr uint8_t REG_SYSCTRL7 = 0x49;
static constexpr uint8_t MASK_GAIN_BYPASS = _BV(6);
static constexpr uint8_t MASK_D2S_GAIN    = 0x07;
static constexpr uint8_t SHIFT_D2S_GAIN   = 0;

// ==============================================
// 0x4C (RW)
// ==============================================
// Register : PWMCFG1
static constexpr uint8_t REG_PWMCFG1 = 0x4C;
static constexpr uint8_t MASK_PRC_EN   = _BV(7);
static constexpr uint8_t MASK_PRCTIME  = 0x7F;
static constexpr uint8_t SHIFT_PRCTIME = 0;

// ==============================================
// 0x4D (RW)
// ==============================================
// Register : PWMCFG2
static constexpr uint8_t REG_PWMCFG2 = 0x4D;
static constexpr uint8_t MASK_PD_HWM   = _BV(4);  // shutdown half wave modulate

// ==============================================
// 0x4E (RW)
// ==============================================
// Register : PWMCFG3
static constexpr uint8_t REG_PWMCFG3 = 0x4E;
static constexpr uint8_t MASK_PR_EN   = _BV(7);
static constexpr uint8_t MASK_PRLVL   = 0x7F;
static constexpr uint8_t SHIFT_PRLVL  = 0;

// ==============================================
// 0x4F (RW)
// ==============================================
// Register : PWMCFG4
static constexpr uint8_t REG_PWMCFG4 = 0x4F;
static constexpr uint8_t MASK_PRTIME = 0xFF;
static constexpr uint8_t SHIFT_PRTIME = 0;

// ==============================================
// 0x51 (RW)
// ==============================================
// Register : DETCFG1
static constexpr uint8_t REG_DETCFG1 = 0x51;
static constexpr uint8_t MASK_RL_OS    = _BV(4);
static constexpr uint8_t MASK_CLK_ADC  = 0x07;
static constexpr uint8_t SHIFT_CLK_ADC = 0;

// ==============================================
// 0x52 (RW)
// ==============================================
// Register : DETCFG2
static constexpr uint8_t REG_DETCFG2 = 0x52;
static constexpr uint8_t MASK_VBAT_GO  = _BV(1);
static constexpr uint8_t MASK_DIAG_GO  = _BV(0);

// ==============================================
// 0x53 (RO)
// ==============================================
// Register : DET_RL
static constexpr uint8_t REG_DET_RL = 0x53;
static constexpr uint8_t MASK_RL = 0xFF;
static constexpr uint8_t SHIFT_RL = 0;

// ==============================================
// 0x55 (RO)
// ==============================================
// Register : DET_VBAT
static constexpr uint8_t REG_DET_VBAT = 0x55;
static constexpr uint8_t MASK_VBAT = 0xFF;
static constexpr uint8_t SHIFT_VBAT = 0;

// ==============================================
// 0x57 (RO)
// ==============================================
// Register : DET_LO
static constexpr uint8_t REG_DET_LO = 0x57;
static constexpr uint8_t MASK_VBAT_LO = 0x30;  // bits 5:4
static constexpr uint8_t SHIFT_VBAT_LO = 4;
static constexpr uint8_t MASK_RL_LO    = 0x03;  // bits 1:0
static constexpr uint8_t SHIFT_RL_LO   = 0;

// ==============================================
// 0x54 (RO)
// ==============================================
// Register : DET_OS
static constexpr uint8_t REG_DET_OS = 0x54;
static constexpr uint8_t MASK_OS = 0xFF;
static constexpr uint8_t SHIFT_OS = 0;

// ==============================================
// 0x5A (RW)
// ==============================================
// Register : TRIMCFG3
static constexpr uint8_t REG_TRIMCFG3 = 0x5A;
static constexpr uint8_t MASK_TRIM_LRA = 0x3F;
static constexpr uint8_t SHIFT_TRIM_LRA = 0;

// ==============================================
// 0x64 (RO)
// ==============================================
// Register : CHIPID
static constexpr uint8_t REG_CHIPID = 0x64;
static constexpr uint8_t MASK_CHIPID_H = _BV(6);
static constexpr uint8_t MASK_CHIPID_L = _BV(0);

// ==============================================
// 0x77 (RW)
// ==============================================
// Register : ANACFG8
static constexpr uint8_t REG_ANACFG8 = 0x77;
static constexpr uint8_t MASK_TRTF_CTRL_HDRV = 0xC0;  // bits 7:6
static constexpr uint8_t SHIFT_TRTF_CTRL_HDRV = 6;
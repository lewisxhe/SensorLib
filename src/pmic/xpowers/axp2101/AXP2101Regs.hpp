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
 * @file      AXP2101Regs.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Register Definitions
 *
 */
#pragma once
#include <stdint.h>

struct axp2101_regs {
    struct bmu {
        static constexpr uint8_t STATUS1          = 0x00;
        static constexpr uint8_t STATUS2          = 0x01;
        static constexpr uint8_t IC_TYPE          = 0x03;
        static constexpr uint8_t DATA_BUFFER1     = 0x04;
        static constexpr uint8_t DATA_BUFFER2     = 0x05;
        static constexpr uint8_t DATA_BUFFER3     = 0x06;
        static constexpr uint8_t DATA_BUFFER4     = 0x07;
    };

    struct ctrl {
        static constexpr uint8_t COMMON_CFG           = 0x10;
        static constexpr uint8_t BATFET_CTRL          = 0x12;
        static constexpr uint8_t DIE_TEMP_CFG         = 0x13;
        static constexpr uint8_t MIN_SYS_VOLTAGE      = 0x14;
        static constexpr uint8_t INPUT_VOLT_LIMIT     = 0x15;
        static constexpr uint8_t INPUT_CURR_LIMIT     = 0x16;
        static constexpr uint8_t RESET_FUEL_GAUGE     = 0x17;
        static constexpr uint8_t MODULE_EN            = 0x18;
        static constexpr uint8_t WDT_CTRL             = 0x19;
        static constexpr uint8_t LOW_BAT_WARN_SET     = 0x1A;
    };

    struct pmu {
        static constexpr uint8_t PWRON_STATUS         = 0x20;
        static constexpr uint8_t PWROFF_STATUS        = 0x21;
        static constexpr uint8_t PWROFF_EN            = 0x22;
        static constexpr uint8_t DC_OVP_UVP_CTRL      = 0x23;
        static constexpr uint8_t VOFF_SET             = 0x24;
        static constexpr uint8_t PWROK_SEQU_CTRL      = 0x25;
        static constexpr uint8_t SLEEP_WAKEUP_CTRL    = 0x26;
        static constexpr uint8_t IRQ_OFF_ON_LEVEL     = 0x27;
        static constexpr uint8_t FAST_PWRON_SET0      = 0x28;
        static constexpr uint8_t FAST_PWRON_SET1      = 0x29;
        static constexpr uint8_t FAST_PWRON_SET2      = 0x2A;
        static constexpr uint8_t FAST_PWRON_CTRL      = 0x2B;
    };

    struct adc {
        static constexpr uint8_t CHANNEL_CTRL         = 0x30;
        static constexpr uint8_t BAT_VOLT_H           = 0x34;
        static constexpr uint8_t BAT_VOLT_L           = 0x35;
        static constexpr uint8_t TS_VOLT_H            = 0x36;
        static constexpr uint8_t TS_VOLT_L            = 0x37;
        static constexpr uint8_t VBUS_VOLT_H          = 0x38;
        static constexpr uint8_t VBUS_VOLT_L          = 0x39;
        static constexpr uint8_t VSYS_VOLT_H          = 0x3A;
        static constexpr uint8_t VSYS_VOLT_L          = 0x3B;
        static constexpr uint8_t DIE_TEMP_H           = 0x3C;
        static constexpr uint8_t DIE_TEMP_L           = 0x3D;
    };

    struct irq {
        static constexpr uint8_t ENABLE1              = 0x40;
        static constexpr uint8_t ENABLE2              = 0x41;
        static constexpr uint8_t ENABLE3              = 0x42;
        static constexpr uint8_t STATUS1              = 0x48;
        static constexpr uint8_t STATUS2              = 0x49;
        static constexpr uint8_t STATUS3              = 0x4A;
    };

    struct ts {
        static constexpr uint8_t TS_PIN_CTRL          = 0x50;
        static constexpr uint8_t HYSL2H               = 0x52;
        static constexpr uint8_t HYSH2L               = 0x53;
    };

    struct jeita {
        static constexpr uint8_t VLTF_CHG             = 0x54;
        static constexpr uint8_t VHTF_CHG             = 0x55;
        static constexpr uint8_t VLTF_WORK            = 0x56;
        static constexpr uint8_t VHTF_WORK            = 0x57;
        static constexpr uint8_t EN_CTRL              = 0x58;
        static constexpr uint8_t SET0                 = 0x59;
        static constexpr uint8_t SET1                 = 0x5A;
        static constexpr uint8_t SET2                 = 0x5B;
    };

    struct chg {
        static constexpr uint8_t IPRECHG_SET          = 0x61;
        static constexpr uint8_t ICC_SETTING          = 0x62;
        static constexpr uint8_t ITERM_CTRL           = 0x63;
        static constexpr uint8_t CV_CHG_VOLTAGE       = 0x64;
        static constexpr uint8_t THERMAL_REG_THRESH   = 0x65;
        static constexpr uint8_t CHG_TIMER_CFG        = 0x67;
        static constexpr uint8_t BAT_DETECT_CTRL      = 0x68;
        static constexpr uint8_t CHGLED_CFG           = 0x69;
        static constexpr uint8_t BTN_BAT_CHG_VOL      = 0x6A;
    };

    struct dcdc {
        static constexpr uint8_t ONOFF_DVM_CTRL       = 0x80;
        static constexpr uint8_t FORCE_PWM_CTRL       = 0x81;
        static constexpr uint8_t VOL0_CTRL            = 0x82;
        static constexpr uint8_t VOL1_CTRL            = 0x83;
        static constexpr uint8_t VOL2_CTRL            = 0x84;
        static constexpr uint8_t VOL3_CTRL            = 0x85;
        static constexpr uint8_t VOL4_CTRL            = 0x86;
    };

    struct ldo {
        static constexpr uint8_t ONOFF_CTRL0          = 0x90;
        static constexpr uint8_t ONOFF_CTRL1          = 0x91;
        static constexpr uint8_t VOL0_CTRL            = 0x92;
        static constexpr uint8_t VOL1_CTRL            = 0x93;
        static constexpr uint8_t VOL2_CTRL            = 0x94;
        static constexpr uint8_t VOL3_CTRL            = 0x95;
        static constexpr uint8_t VOL4_CTRL            = 0x96;
        static constexpr uint8_t VOL5_CTRL            = 0x97;
        static constexpr uint8_t VOL6_CTRL            = 0x98;
        static constexpr uint8_t VOL7_CTRL            = 0x99;
        static constexpr uint8_t VOL8_CTRL            = 0x9A;
    };

    struct gauge {
        static constexpr uint8_t BAT_PARAMS           = 0xA1;
        static constexpr uint8_t FUEL_GAUGE_CTRL      = 0xA2;
        static constexpr uint8_t BAT_PERCENT          = 0xA4;
    };

    struct factory {
        static constexpr float VBAT_STEP    = 1.0f;
        static constexpr float VBUS_STEP    = 1.0f;
        static constexpr float VSYS_STEP    = 1.0f;
        static constexpr float TS_STEP      = 0.5f;
    };

    static constexpr uint8_t CHIP_ID              = 0x4A;
};

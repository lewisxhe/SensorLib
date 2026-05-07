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
 * @file      AXP192Regs.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP192 PMIC Register Definitions and Constants
 *
 * This header defines all register addresses and constant parameters for the
 * AXP192 PMIC. Registers are organized into functional groups: BMU (battery
 * management unit), power control, PMU (system power management), charger,
 * AC power, ADC, interrupts, GPIO, and timer. Factory-calibrated ADC step
 * sizes and the chip slave address / ID are also defined.
 *
 * @note Register addresses are grouped in nested structs for logical
 *       organization and compile-time reference safety.
 *
 */
#pragma once
#include <stdint.h>

struct axp192_regs {
    /**
     * @brief Battery Management Unit (BMU) registers
     */
    struct bmu {
        static constexpr uint8_t STATUS           = 0x00;
        static constexpr uint8_t MODE_CHGSTATUS   = 0x01;
        static constexpr uint8_t OTG_STATUS       = 0x02;
        static constexpr uint8_t IC_TYPE          = 0x03;
        static constexpr uint8_t DATA_BUFFER1     = 0x06;
        static constexpr uint8_t DATA_BUFFER2     = 0x07;
        static constexpr uint8_t DATA_BUFFER3     = 0x08;
        static constexpr uint8_t DATA_BUFFER4     = 0x09;
    };

    /**
     * @brief Power output channel control registers
     */
    struct pwr {
        static constexpr uint8_t LDO23_DC123_EXT_CTL = 0x12;
        static constexpr uint8_t DC1_VOLTAGE         = 0x26;
        static constexpr uint8_t DC2OUT_VOL          = 0x23;
        static constexpr uint8_t DC2_DVM             = 0x25;
        static constexpr uint8_t DC3OUT_VOL          = 0x27;
        static constexpr uint8_t LDO23OUT_VOL        = 0x28;
        static constexpr uint8_t LDO3OUT_VOL         = 0x29;
        static constexpr uint8_t DCDC_MODESET        = 0x80;
    };

    /**
     * @brief Power Management Unit (PMU) registers
     */
    struct pmu {
        static constexpr uint8_t IPS_SET   = 0x30;
        static constexpr uint8_t VOFF_SET  = 0x31;
        static constexpr uint8_t OFF_CTL   = 0x32;
        static constexpr uint8_t POK_SET   = 0x36;

        // REG30H[1:0] current-limit encoding (AXP192)
        // bit1: limit enable, bit0: 0=500mA, 1=100mA
        static constexpr uint8_t INPUT_CURR_MASK = 0x03;

        static inline uint8_t encodeInputCurrentLimit(uint32_t mA)
        {
            // AXP192 supports: OFF, 100mA, 500mA
            if (mA >= 500) {
                return 0x02; // enable + 500mA
            }
            if (mA >= 100) {
                return 0x03; // enable + 100mA
            }
            return 0x00;     // limit off
        }

        static inline uint32_t decodeInputCurrentLimit(uint8_t reg)
        {
            uint8_t code = reg & INPUT_CURR_MASK;
            if ((code & 0x02) == 0) {
                return 0;
            }
            return (code & 0x01) ? 100 : 500;
        }
    };

    /**
     * @brief Charger control registers
     */
    struct chg {
        static constexpr uint8_t CHARGE1    = 0x33;
        static constexpr uint8_t CHARGE2    = 0x34;
        static constexpr uint8_t BACKUP_CHG = 0x35;

        // Charger constants
        static constexpr uint16_t CHG_CUR_MIN      = 100;
        static constexpr uint16_t CHG_CUR_MAX      = 1320;
        static constexpr uint16_t CHG_CUR_STEP    = 90;
    };

    /**
     * @brief AC input measurement registers
     */
    struct ac {
        static constexpr uint8_t ACIN_VOL_H8 = 0x56;
        static constexpr uint8_t ACIN_VOL_L4 = 0x57;
        static constexpr uint8_t ACIN_CUR_H8 = 0x58;
        static constexpr uint8_t ACIN_CUR_L4 = 0x59;
    };

    /**
     * @brief ADC measurement and control registers
     */
    struct adc {
        static constexpr uint8_t VBUS_VOL_H8              = 0x5A;
        static constexpr uint8_t VBUS_VOL_L4              = 0x5B;
        static constexpr uint8_t VBUS_CUR_H8              = 0x5C;
        static constexpr uint8_t VBUS_CUR_L4              = 0x5D;
        static constexpr uint8_t INTERNAL_TEMP_H8         = 0x5E;
        static constexpr uint8_t INTERNAL_TEMP_L4         = 0x5F;
        static constexpr uint8_t TS_IN_H8                 = 0x62;
        static constexpr uint8_t TS_IN_L4                 = 0x63;
        static constexpr uint8_t GPIO0_ADC_H8             = 0x64;
        static constexpr uint8_t GPIO0_ADC_L4             = 0x65;
        static constexpr uint8_t GPIO1_ADC_H8             = 0x66;
        static constexpr uint8_t GPIO1_ADC_L4             = 0x67;
        static constexpr uint8_t GPIO2_ADC_H8             = 0x68;
        static constexpr uint8_t GPIO2_ADC_L4             = 0x69;
        static constexpr uint8_t GPIO3_ADC_H8             = 0x6A;
        static constexpr uint8_t GPIO3_ADC_L4             = 0x6B;
        static constexpr uint8_t BAT_AVERVOL_H8           = 0x78;
        static constexpr uint8_t BAT_AVERVOL_L4           = 0x79;
        static constexpr uint8_t BAT_AVERCHGCUR_H8        = 0x7A;
        static constexpr uint8_t BAT_AVERCHGCUR_L5        = 0x7B;
        static constexpr uint8_t BAT_AVERDISCHGCUR_H8     = 0x7C;
        static constexpr uint8_t BAT_AVERDISCHGCUR_L5     = 0x7D;
        static constexpr uint8_t APS_AVERVOL_H8           = 0x7E;
        static constexpr uint8_t APS_AVERVOL_L4           = 0x7F;
        static constexpr uint8_t ADC_EN1                  = 0x82;
        static constexpr uint8_t ADC_EN2                  = 0x83;
        static constexpr uint8_t ADC_SPEED                = 0x84;
        static constexpr uint8_t ADC_INPUTRANGE           = 0x85;
    };

    /**
     * @brief Interrupt control and status registers
     */
    struct irq {
        static constexpr uint8_t ENABLE1 = 0x40;
        static constexpr uint8_t ENABLE2 = 0x41;
        static constexpr uint8_t ENABLE3 = 0x42;
        static constexpr uint8_t ENABLE4 = 0x43;
        static constexpr uint8_t STATUS1 = 0x44;
        static constexpr uint8_t STATUS2 = 0x45;
        static constexpr uint8_t STATUS3 = 0x46;
        static constexpr uint8_t STATUS4 = 0x47;
        static constexpr uint8_t ENABLE5 = 0x4A;
        static constexpr uint8_t STATUS5 = 0x4D;
    };

    /**
     * @brief GPIO control and signal registers
     */
    struct gpio {
        static constexpr uint8_t GPIO0_CTL          = 0x90;
        static constexpr uint8_t GPIO0_VOL          = 0x91;
        static constexpr uint8_t GPIO1_CTL          = 0x92;
        static constexpr uint8_t GPIO2_CTL          = 0x93;
        static constexpr uint8_t GPIO012_SIGNAL     = 0x94;
        static constexpr uint8_t GPIO34_CTL         = 0x95;
        static constexpr uint8_t GPIO34_SIGNAL      = 0x96;
        static constexpr uint8_t GPIO012_PULLDOWN   = 0x97;
        static constexpr uint8_t PWM1_FREQ          = 0x98;
        static constexpr uint8_t PWM1_DUTY1         = 0x99;
        static constexpr uint8_t PWM1_DUTY2         = 0x9A;
        static constexpr uint8_t PWM2_FREQ          = 0x9B;
        static constexpr uint8_t PWM2_DUTY1         = 0x9C;
        static constexpr uint8_t PWM2_DUTY2         = 0x9D;
        static constexpr uint8_t GPIO5_CTL          = 0x9E;
    };

    /**
     * @brief Timer control registers
     */
    struct timer {
        static constexpr uint8_t TIMER_CTL = 0x8A;
    };

    /**
     * @brief Factory-calibrated ADC conversion step sizes
     *
     * These constants define the LSB weight for each ADC measurement channel.
     * Raw ADC register values are multiplied by the corresponding step to
     * obtain real-world voltage (mV), current (mA), or temperature (degC).
     */
    struct factory {
        /// @brief ACIN voltage/current steps: 1.7 mV/0.625 mA per LSB
        static constexpr float ACIN_VOLTAGE_STEP   = 1.7f;
        /// @brief ACIN current step: 0.625 mA per LSB
        static constexpr float ACIN_CURRENT_STEP   = 0.625f;
        /// @brief TS voltage step: 0.8 mV per LSB
        static constexpr float TS_VOLTAGE_STEP     = 0.8f;
        /// @brief VBUS voltage step: 1.7 mV per LSB
        static constexpr float VBUS_VOLTAGE_STEP  = 1.7f;
        /// @brief VBUS current step: 0.375 mA per LSB
        static constexpr float VBUS_CURRENT_STEP  = 0.375f;
        /// @brief BAT voltage step: 1.1 mV per LSB
        static constexpr float BAT_VOLTAGE_STEP   = 1.1f;
        /// @brief BAT charge current step: 0.5 mA per LSB
        static constexpr float BAT_CHARGE_CUR_STEP = 0.5f;
        /// @brief BAT discharge current step: 0.5 mA per LSB
        static constexpr float BAT_DISCHARGE_CUR_STEP = 0.5f;
        /// @brief APS voltage step: 1.4 mV per LSB
        static constexpr float APS_VOLTAGE_STEP   = 1.4f;
        /// @brief Internal temperature step: 0.1 degC per LSB
        static constexpr float INTERNAL_TEMP_STEP = 0.1f;
        /// @brief Internal temperature offset: 144.7 degC
        static constexpr float INTERNAL_TEMP_OFFSET = 144.7f;
        /// @brief GPIO voltage step: 0.5 V per LSB
        static constexpr float FACTORY_GPIO_VOLTAGE   = 0.5f;
    };

    static constexpr uint8_t CHIP_ID = 0x03;
};

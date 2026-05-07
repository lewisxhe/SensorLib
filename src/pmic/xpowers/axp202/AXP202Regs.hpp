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
 * @file      AXP202Regs.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP202 PMIC Register Definitions and Constants
 *
 * This header defines all register addresses, bit positions, ADC conversion
 * factors, and chip identification constants for the AXP202 PMIC.
 *
 * @section Register Groups
 * - bmu: Status and data buffer registers (0x00-0x07)
 * - pmu: Power control registers (0x12-0x80)
 * - chg: Charger configuration registers (0x33-0x35)
 * - irq: Interrupt enable and status registers (0x40-0x4C)
 * - adc: ADC enable, data output, and battery measurement registers (0x5A-0xB9)
 * - gpio: GPIO control and signal registers (0x90-0x95)
 * - timer: Watchdog/timer control register (0x8A)
 * - factory: ADC conversion scale factors for voltage, current, and temperature
 *
 * @note See AXP202 datasheet for detailed register descriptions.
 */
#pragma once
#include <stdint.h>

/**
 * @brief AXP202 register map and constants
 *
 * Nested structs group related registers by functional block.
 */
struct axp202_regs {
    // -------------------- Status --------------------
    struct bmu
    {
        static constexpr uint8_t STATUS           = 0x00;
        static constexpr uint8_t MODE_CHGSTATUS   = 0x01;
        static constexpr uint8_t OTG_STATUS       = 0x02;
        static constexpr uint8_t IC_TYPE          = 0x03;
        static constexpr uint8_t DATA_BUFFER1     = 0x04;
        static constexpr uint8_t DATA_BUFFER2     = 0x05;
        static constexpr uint8_t DATA_BUFFER3     = 0x06;
        static constexpr uint8_t DATA_BUFFER4     = 0x07;
    };

    // -------------------- Power Control --------------------
    struct pmu
    {
        static constexpr uint8_t LDO234_DC23_CTL   = 0x12;
        static constexpr uint8_t DC2OUT_VOL        = 0x23;
        static constexpr uint8_t DC3OUT_VOL        = 0x27;
        static constexpr uint8_t LDO24OUT_VOL      = 0x28;
        static constexpr uint8_t LDO3OUT_VOL       = 0x29;
        static constexpr uint8_t IPS_SET           = 0x30;
        static constexpr uint8_t VOFF_SET          = 0x31;
        static constexpr uint8_t OFF_CTL           = 0x32;
        static constexpr uint8_t POK_SET           = 0x36;
        static constexpr uint8_t LDO3_DC2_DVM    = 0x25;
        static constexpr uint8_t DCDC_FREQSET    = 0x37;
        static constexpr uint8_t APS_WARNING1    = 0x3A;
        static constexpr uint8_t APS_WARNING2    = 0x3B;
        static constexpr uint8_t DCDC_MODESET      = 0x80;
        static constexpr uint8_t VBUS_DET_SRP    = 0x8B;
        static constexpr uint8_t HOTOVER_CTL     = 0x8F;

        // REG30H[1:0] current-limit encoding (AXP202)
        // 00=900mA, 01=500mA, 10=100mA, 11=OFF
        static constexpr uint8_t INPUT_CURR_MASK = 0x03;

        static inline uint8_t encodeInputCurrentLimit(uint32_t mA)
        {
            if (mA >= 900) {
                return 0x00;
            }
            if (mA >= 500) {
                return 0x01;
            }
            if (mA >= 100) {
                return 0x02;
            }
            return 0x03;
        }

        static inline uint32_t decodeInputCurrentLimit(uint8_t reg)
        {
            switch (reg & INPUT_CURR_MASK) {
            case 0x00: return 900;
            case 0x01: return 500;
            case 0x02: return 100;
            default:   return 0;
            }
        }
    };

    // -------------------- Charger --------------------
    struct chg
    {
        static constexpr uint8_t CHARGE1           = 0x33;
        static constexpr uint8_t CHARGE2           = 0x34;
        static constexpr uint8_t BACKUP_CHG        = 0x35;
        static constexpr uint8_t VLTF_CHGSET     = 0x38;
        static constexpr uint8_t VHTF_CHGSET     = 0x39;
        static constexpr uint8_t TLTF_DISCHGSET  = 0x3C;
        static constexpr uint8_t THTF_DISCHGSET  = 0x3D;

        // Charger constants
        static constexpr uint16_t CHG_CUR_MIN      = 300;
        static constexpr uint16_t CHG_CUR_MAX      = 1800;
        static constexpr uint16_t CHG_CUR_STEP    = 100;
    };

    // -------------------- IRQ --------------------
    struct irq
    {
        static constexpr uint8_t ENABLE1           = 0x40;
        static constexpr uint8_t ENABLE2           = 0x41;
        static constexpr uint8_t ENABLE3           = 0x42;
        static constexpr uint8_t ENABLE4           = 0x43;
        static constexpr uint8_t ENABLE5           = 0x44;
        static constexpr uint8_t STATUS1           = 0x48;
        static constexpr uint8_t STATUS2           = 0x49;
        static constexpr uint8_t STATUS3           = 0x4A;
        static constexpr uint8_t STATUS4           = 0x4B;
        static constexpr uint8_t STATUS5           = 0x4C;
    };

    // -------------------- ADC --------------------
    struct adc
    {
        static constexpr uint8_t ADC_EN1           = 0x82;
        static constexpr uint8_t ADC_EN2           = 0x83;
        static constexpr uint8_t ADC_SPEED         = 0x84;
        static constexpr uint8_t ADC_INPUTRANGE  = 0x85;
        static constexpr uint8_t ADC_IRQ_RETFSET = 0x86;
        static constexpr uint8_t ADC_IRQ_FETFSET = 0x87;

        static constexpr uint8_t ACIN_VOL_H      = 0x56;
        static constexpr uint8_t ACIN_VOL_L      = 0x57;
        static constexpr uint8_t ACIN_CUR_H      = 0x58;
        static constexpr uint8_t ACIN_CUR_L      = 0x59;

        static constexpr uint8_t VBUS_VOL_H        = 0x5A;
        static constexpr uint8_t VBUS_VOL_L        = 0x5B;
        static constexpr uint8_t VBUS_CUR_H        = 0x5C;
        static constexpr uint8_t VBUS_CUR_L        = 0x5D;
        static constexpr uint8_t INTERNAL_TEMP_H   = 0x5E;
        static constexpr uint8_t INTERNAL_TEMP_L   = 0x5F;
        static constexpr uint8_t TS_IN_H           = 0x62;
        static constexpr uint8_t TS_IN_L           = 0x63;
        static constexpr uint8_t GPIO0_ADC_H       = 0x64;
        static constexpr uint8_t GPIO0_ADC_L       = 0x65;
        static constexpr uint8_t GPIO1_ADC_H       = 0x66;
        static constexpr uint8_t GPIO1_ADC_L       = 0x67;
        static constexpr uint8_t BAT_AVERVOL_H     = 0x78;
        static constexpr uint8_t BAT_AVERVOL_L     = 0x79;
        static constexpr uint8_t BAT_AVERCHGCUR_H  = 0x7A;
        static constexpr uint8_t BAT_AVERCHGCUR_L  = 0x7B;
        static constexpr uint8_t BAT_AVERDISCHGCUR_H = 0x7C;
        static constexpr uint8_t BAT_AVERDISCHGCUR_L = 0x7D;
        static constexpr uint8_t APS_AVERVOL_H     = 0x7E;
        static constexpr uint8_t APS_AVERVOL_L     = 0x7F;
        static constexpr uint8_t BAT_POWER_H       = 0x70;
        static constexpr uint8_t BAT_POWER_M       = 0x71;
        static constexpr uint8_t BAT_POWER_L       = 0x72;
        static constexpr uint8_t BATT_PERCENTAGE   = 0xB9;
    };

    // -------------------- GPIO --------------------
    struct gpio
    {
        static constexpr uint8_t GPIO0_CTL         = 0x90;
        static constexpr uint8_t GPIO0_VOL         = 0x91;
        static constexpr uint8_t GPIO1_CTL         = 0x92;
        static constexpr uint8_t GPIO2_CTL         = 0x93;
        static constexpr uint8_t GPIO012_SIGNAL    = 0x94;
        static constexpr uint8_t GPIO3_CTL         = 0x95;
    };

    // -------------------- Coulomb Counter --------------------
    struct coulomb
    {
        static constexpr uint8_t BAT_CHGCOULOMB3    = 0xB0;
        static constexpr uint8_t BAT_CHGCOULOMB2    = 0xB1;
        static constexpr uint8_t BAT_CHGCOULOMB1    = 0xB2;
        static constexpr uint8_t BAT_CHGCOULOMB0    = 0xB3;
        static constexpr uint8_t BAT_DISCHGCOULOMB3 = 0xB4;
        static constexpr uint8_t BAT_DISCHGCOULOMB2 = 0xB5;
        static constexpr uint8_t BAT_DISCHGCOULOMB1 = 0xB6;
        static constexpr uint8_t BAT_DISCHGCOULOMB0 = 0xB7;
        static constexpr uint8_t COULOMB_CTL        = 0xB8;
    };

    // -------------------- Timer / Watchdog --------------------
    struct timer
    {
        static constexpr uint8_t TIMER_CTL         = 0x8A;
    };

    // -------------------- Factory ADC conversion factors --------------------
    struct factory
    {
        static constexpr float FACTORY_VBUS_VOLTAGE   = 1.7f;
        static constexpr float FACTORY_VBUS_CURRENT   = 0.375f;
        static constexpr float FACTORY_VBAT_VOLTAGE   = 1.1f;
        static constexpr float FACTORY_BAT_CURRENT    = 0.5f;
        static constexpr float FACTORY_APS_VOLTAGE    = 1.4f;
        static constexpr float FACTORY_INTERNAL_TEMP  = 0.1f;
        static constexpr float FACTORY_INTERNAL_OFFSET = 144.7f;
        static constexpr float FACTORY_ACIN_VOLTAGE   = 1.7f;
        static constexpr float FACTORY_ACIN_CURRENT   = 0.625f;
        static constexpr float FACTORY_TS_VOLTAGE     = 0.8f;
        static constexpr float FACTORY_GPIO_VOLTAGE   = 0.5f;
    };

    // -------------------- Constants --------------------
    static constexpr uint8_t CHIP_ID              = 0x41;

    // Power enable bits in LDO234_DC23_CTL (0x12)
    static constexpr uint8_t BIT_EXTEN   = 0;
    static constexpr uint8_t BIT_DCDC3   = 1;
    static constexpr uint8_t BIT_LDO2    = 2;
    static constexpr uint8_t BIT_LDO4    = 3;
    static constexpr uint8_t BIT_DCDC2   = 4;
    static constexpr uint8_t BIT_LDO3    = 6;

    // Aliases used by some .cpp files (same as bmu/pmu)
    using status = bmu;
    using power = pmu;
};

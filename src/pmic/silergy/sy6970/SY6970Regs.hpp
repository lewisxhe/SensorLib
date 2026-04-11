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
 * @file      SY6970Regs.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 * @brief SY6970 PMIC Register Definitions and Constants
 *
 * This header file contains all register addresses, bit masks, shift values,
 * and configuration constants for the Silergy SY6970 PMIC (Power Management IC).
 *
 * The SY6970 is a highly integrated USB PD/BC1.2 compliant charger with:
 * - Buck-boost charger support (up to 5A fast charge)
 * - USB PD/HVDCP adaptive voltage detection
 * - Integrated 12-bit ADC for voltage/current monitoring
 * - OTG (On-The-Go) boost mode for power bank functionality
 * - JEITA thermal protection support
 *
 * @section Register Overview
 * - REG_PWR_INPUT (0x00): Input source control - HIZ mode, input current limit
 * - REG_PWR_VINDPM (0x01): VINDPM control - Battery HOT/COLD threshold
 * - REG_PWR_ONOFF (0x02): Power on/off control - ADC conv, boost freq, AICL/HVDCP
 * - REG_CHG_CTRL (0x03): Charge control - Bat load, OTG, charge enable, sys min
 * - REG_CHG_CURRENT (0x04): Fast charge current (0-5056mA, 64mA steps)
 * - REG_CHG_PRE_CURR (0x05): Pre-charge/termination current
 * - REG_CHG_VOLT (0x06): Charge voltage control (3840-4608mV, 16mV steps)
 * - REG_CHG_TIMER (0x07): Charge timer and watchdog
 * - REG_CHG_IR_COMP (0x08): IR compensation and thermal regulation
 * - REG_MISC_CTRL (0x09): Misc control - BATFET, pumpEX
 * - REG_BOOST_CTRL (0x0A): Boost control - boost voltage/current limit
 * - REG_STATUS (0x0B): Status register - bus/charge state, power good
 * - REG_FAULT (0x0C): Fault register - watchdog, boost, charge, bat, NTC faults
 * - REG_VINDPM (0x0D): VINDPM voltage setting
 * - REG_ADC_* (0x0E-0x12): ADC readout registers (VBAT, VSYS, NTC, VBUS, ICHG)
 * - REG_IDPM_STATUS (0x13): Input DP/DM status
 * - REG_DEVICE_REV (0x14): Device revision and configuration
 *
 * @section ADC Channels
 * - VBUS: 2600-15400mV (100mV steps)
 * - VBAT: 2304-4608mV (20mV steps)
 * - VSYS: 2304-3700mV (20mV steps)
 * - NTC: 21-100% (0.465% steps)
 * - ICHG: 0-5056mA (50mA steps via ADC)
 *
 * @section Charge Parameters
 * - Pre-charge current: 64-1024mA (64mA steps)
 * - Fast charge current: 0-5056mA (64mA steps)
 * - Termination current: 64-1024mA (64mA steps)
 * - Charge voltage: 3840-4608mV (16mV steps)
 *
 * @section Boost Mode Parameters
 * - Boost voltage: 4550-5510mV (64mV steps)
 * - Boost current limit: 500mA, 1200mA (configurable)
 *
 * @note All setter methods with step requirements will automatically quantize
 *       input values to the nearest valid step if they do not match exactly.
 *
 * @see SY6970Charger.cpp, SY6970Power.cpp, SY6970Adc.cpp, SY6970Core.cpp
 */
#pragma once

#include <stdint.h>

namespace SY6970Regs {

#ifdef _BV
#undef _BV
#endif
#define _BV(bit)  (1U << (bit))

// ==============================================
// Device Revision Register
// ==============================================
static constexpr uint8_t SY6970_DEV_REV = 0x00;

// ==============================================
// ADC Voltage Measurement Range Constants
// ==============================================
static constexpr uint16_t VBAT_MIN = 2304;
static constexpr uint16_t VBAT_MAX = 4608;
static constexpr uint16_t SYS_MIN = 3000;
static constexpr uint16_t SYS_MAX = 3700;
static constexpr uint16_t IN_CURRENT_MIN = 100;
static constexpr uint16_t IN_CURRENT_MAX = 3250;
static constexpr uint16_t CHG_CURRENT_MAX = 5056;
static constexpr uint16_t BOOST_VOL_MIN = 4550;
static constexpr uint16_t BOOST_VOL_MAX = 5510;

// ==============================================
// USB Port/Adapter Type Definitions
// ==============================================
static constexpr uint8_t BUS_STATE_NOINPUT = 0;
static constexpr uint8_t BUS_STATE_USB_SDP = 1;        ///< Standard USB Downstream Port (500mA)
static constexpr uint8_t BUS_STATE_USB_CDP = 2;     ///< Charging Downstream Port (1500mA)
static constexpr uint8_t BUS_STATE_USB_DCP = 3;     ///< Dedicated Charging Port
static constexpr uint8_t BUS_STATE_HVDCP = 4;      ///< High Voltage DCP (QC mode)
static constexpr uint8_t BUS_STATE_ADAPTER = 5;    ///< External adapter
static constexpr uint8_t BUS_STATE_NO_STANDARD_ADAPTER = 6;
static constexpr uint8_t BUS_STATE_OTG = 7;          ///< OTG mode (supplying power)

// ==============================================
// Charge State Definitions
// ==============================================
static constexpr uint8_t CHARGE_STATE_NO_CHARGE = 0;
static constexpr uint8_t CHARGE_STATE_PRE_CHARGE = 1;    ///< Pre-charge phase (weak battery)
static constexpr uint8_t CHARGE_STATE_FAST_CHARGE = 2;  ///< Fast charge phase (CC/CV)
static constexpr uint8_t CHARGE_STATE_DONE = 3;        ///< Charge complete

// ==============================================
// ADC Base Values (minimum measurable value)
// ==============================================
static constexpr uint16_t VBUS_BASE_VAL = 2600;
static constexpr uint16_t VBAT_BASE_VAL = 2304;
static constexpr uint16_t VSYS_BASE_VAL = 2304;
static constexpr float    NTC_BASE_VAL  = 21.0f;

// ==============================================
// ADC Voltage Step Values
// ==============================================
static constexpr uint16_t VBUS_VOL_STEP = 100;      ///< VBUS ADC step: 100mV
static constexpr uint16_t VBAT_VOL_STEP = 20;         ///< VBAT ADC step: 20mV
static constexpr uint16_t VSYS_VOL_STEP = 20;         ///< VSYS ADC step: 20mV
static constexpr float    NTC_VOL_STEP  = 0.465f;   ///< NTC ADC step: 0.465%

// ==============================================
// Fast Charge Current Parameters
// ==============================================
static constexpr uint16_t CHG_STEP_VAL  = 50;
static constexpr uint16_t FAST_CHG_CUR_STEP = 64;    ///< Fast charge current step: 64mA
static constexpr uint16_t FAST_CHG_CURRENT_MIN = 0;
static constexpr uint16_t FAST_CHG_CURRENT_MAX = 5056; ///< Max fast charge: 5056mA

// ==============================================
// Pre-charge Current Parameters
// ==============================================
static constexpr uint16_t PRE_CHG_CUR_BASE = 64;
static constexpr uint16_t PRE_CHG_CUR_STEP = 64;      ///< Pre-charge current step: 64mA
static constexpr uint16_t PRE_CHG_CURRENT_MIN = 64;   ///< Min pre-charge: 64mA
static constexpr uint16_t PRE_CHG_CURRENT_MAX = 1024; ///< Max pre-charge: 1024mA

// ==============================================
// Termination Current Parameters
// ==============================================
static constexpr uint16_t TERM_CHG_CUR_BASE = 64;
static constexpr uint16_t TERM_CHG_CUR_STEP = 64;    ///< Termination current step: 64mA
static constexpr uint16_t TERM_CHG_CURRENT_MIN = 64;
static constexpr uint16_t TERM_CHG_CURRENT_MAX = 1024;

// ==============================================
// Charge Voltage Parameters
// ==============================================
static constexpr uint16_t CHG_VOL_BASE = 3840;
static constexpr uint16_t CHG_VOL_STEP = 16;        ///< Charge voltage step: 16mV
static constexpr uint16_t FAST_CHG_VOL_MIN = 3840;
static constexpr uint16_t FAST_CHG_VOL_MAX = 4608;

// ==============================================
// System Voltage Parameters
// ==============================================
static constexpr uint16_t SYS_VOL_STEPS = 100;       ///< SYS voltage step: 100mV
static constexpr uint16_t SYS_VOFF_VOL_MIN = 3000;
static constexpr uint16_t SYS_VOFF_VOL_MAX = 3700;

// ==============================================
// Input Current Limit Parameters
// ==============================================
static constexpr uint16_t IN_CURRENT_STEP = 50;     ///< Input current step: 50mA
static constexpr uint16_t IN_CURRENT_MIN_VAL = 100;
static constexpr uint16_t IN_CURRENT_MAX_VAL = 3250;

// ==============================================
// Boost Mode Parameters
// ==============================================
static constexpr uint16_t BOOST_VOL_BASE = 4550;
static constexpr uint16_t BOOST_VOL_STEP = 64;       ///< Boost voltage step: 64mV
static constexpr uint16_t BOOST_VOL_MIN_VAL = 4550;
static constexpr uint16_t BOOST_VOL_MAX_VAL = 5510;

// ==============================================
// VINDPM Parameters
// ==============================================
static constexpr uint16_t VINDPM_VOL_BASE = 2600;
static constexpr uint16_t VINDPM_VOL_STEPS = 100;
static constexpr uint16_t VINDPM_VOL_MIN = 3900;
static constexpr uint16_t VINDPM_VOL_MAX = 15300;

// ==============================================
// Battery Compensation Parameters
// ==============================================
static constexpr uint16_t BAT_COMP_STEPS = 20;
static constexpr uint16_t BAT_COMP_MAX = 140;

// ==============================================
// voltage Clamp Parameters
// ==============================================
static constexpr uint16_t VCLAMP_STEP = 32;
static constexpr uint16_t VCLAMP_MAX = 224;

// ==============================================
// REG00 (RW) - Input Source Control Register
// ==============================================
static constexpr uint8_t REG_PWR_INPUT     = 0x00;
static constexpr uint8_t MASK_EN_HIZ       = _BV(7);   ///< Enable HIZ mode (high impedance)
static constexpr uint8_t MASK_EN_ILIM      = _BV(6);  ///< Enable input current limit
static constexpr uint8_t MASK_IINLIM       = 0x3F;   ///< Input current limit (6-bit)
static constexpr uint8_t SHIFT_IINLIM      = 0;

// ==============================================
// REG01 (RW) - VINDPM Control Register
// ==============================================
static constexpr uint8_t REG_PWR_VINDPM    = 0x01;
static constexpr uint8_t MASK_BHOT         = 0xC0;    ///< Battery HOT threshold (2-bit)
static constexpr uint8_t SHIFT_BHOT        = 6;
static constexpr uint8_t MASK_BCOLD        = _BV(5);  ///< Battery COLD threshold
static constexpr uint8_t MASK_VINDPM_OS    = 0x1F;   ///< VINDPM offset (5-bit)
static constexpr uint8_t SHIFT_VINDPM_OS    = 0;

// ==============================================
// REG02 (RW) - Power On/Off Control Register
// ==============================================
static constexpr uint8_t REG_PWR_ONOFF     = 0x02;
static constexpr uint8_t MASK_CONV_START   = _BV(7);   ///< ADC conversion start
static constexpr uint8_t MASK_CONV_RATE    = _BV(6);   ///< ADC conversion rate (continuous if set)
static constexpr uint8_t MASK_BOOST_FREQ   = _BV(5);  ///< Boost frequency (500kHz/1MHz)
static constexpr uint8_t MASK_AICL_EN      = _BV(4);  ///< AICL (Auto Input Current Limit) enable
static constexpr uint8_t MASK_HVDCP_EN     = _BV(3);  ///< HVDCP enable
static constexpr uint8_t MASK_HV_TYPE      = _BV(2);   ///< High voltage type
static constexpr uint8_t MASK_FORCE_DPDM   = _BV(1);   ///< Force DPDM detection
static constexpr uint8_t MASK_AUTO_DPDM_EN = _BV(0);   ///< Auto DPDM enable

// ==============================================
// REG03 (RW) - Charge Control Register
// ==============================================
static constexpr uint8_t REG_CHG_CTRL      = 0x03;
static constexpr uint8_t MASK_BAT_LOAD_EN  = _BV(7);   ///< Battery load enable
static constexpr uint8_t MASK_WD_RST       = _BV(6);   ///< Watchdog timer reset
static constexpr uint8_t MASK_OTG_CONFIG   = _BV(5);   ///< OTG configuration (boost mode)
static constexpr uint8_t MASK_CHG_CONFIG   = _BV(4);   ///< Charge configuration (enable)
static constexpr uint8_t MASK_SYS_MIN       = 0x0E;   ///< System minimum voltage (3-bit)
static constexpr uint8_t SHIFT_SYS_MIN     = 1;

// ==============================================
// REG04 (RW) - Charge Current Control Register
// ==============================================
static constexpr uint8_t REG_CHG_CURRENT   = 0x04;
static constexpr uint8_t MASK_ICHG         = 0x7F;   ///< Fast charge current (7-bit, 0-5056mA)
static constexpr uint8_t SHIFT_ICHG        = 0;

// ==============================================
// REG05 (RW) - Pre-charge/Termination Current Control
// ==============================================
static constexpr uint8_t REG_CHG_PRE_CURR  = 0x05;
static constexpr uint8_t MASK_IPRECHG      = 0xF0;   ///< Pre-charge current (4-bit)
static constexpr uint8_t SHIFT_IPRECHG     = 4;
static constexpr uint8_t MASK_ITERM        = 0x0F;   ///< Termination current (4-bit)
static constexpr uint8_t SHIFT_ITERM       = 0;

// ==============================================
// REG06 (RW) - Charge Voltage Control Register
// ==============================================
static constexpr uint8_t REG_CHG_VOLT      = 0x06;
static constexpr uint8_t MASK_VREG         = 0xFC;   ///< Charge voltage (6-bit)
static constexpr uint8_t SHIFT_VREG        = 2;
static constexpr uint8_t MASK_BATLOWV      = _BV(1);   ///< Battery low voltage threshold
static constexpr uint8_t MASK_VRECHG       = _BV(0);   ///< Recharge voltage threshold

// ==============================================
// REG07 (RW) - Charge Timer/Control Register
// ==============================================
static constexpr uint8_t REG_CHG_TIMER     = 0x07;
static constexpr uint8_t MASK_EN_TERM      = _BV(7);   ///< Enable termination
static constexpr uint8_t MASK_STAT_DIS     = _BV(6);   ///< Disable STAT pin
static constexpr uint8_t MASK_WATCHDOG     = 0x30;   ///< Watchdog timer (2-bit)
static constexpr uint8_t SHIFT_WATCHDOG    = 4;
static constexpr uint8_t MASK_EN_TIMER     = _BV(3);   ///< Enable charge timer
static constexpr uint8_t MASK_CHG_TIMER    = 0x06;   ///< Charge timer (2-bit)
static constexpr uint8_t SHIFT_CHG_TIMER   = 1;
static constexpr uint8_t MASK_JEITA_ISET   = _BV(0);   ///< JEITA ISET

// ==============================================
// REG08 (RW) - IR Compensation/Thermal Control
// ==============================================
static constexpr uint8_t REG_CHG_IR_COMP   = 0x08;
static constexpr uint8_t MASK_BAT_COMP    = 0xE0;   ///< Battery compensation (3-bit)
static constexpr uint8_t SHIFT_BAT_COMP    = 5;
static constexpr uint8_t MASK_VCLAMP       = 0x1C;   ///< Voltage clamp (3-bit)
static constexpr uint8_t SHIFT_VCLAMP      = 2;
static constexpr uint8_t MASK_TREG         = 0x03;   ///< Thermal regulation (2-bit)
static constexpr uint8_t SHIFT_TREG        = 0;

// ==============================================
// REG09 (RW) - Misc Control Register
// ==============================================
static constexpr uint8_t REG_MISC_CTRL     = 0x09;
static constexpr uint8_t MASK_FORCE_AICL   = _BV(7);   ///< Force AICL
static constexpr uint8_t MASK_TMR2X_EN    = _BV(6);   ///< Timer 2x enable
static constexpr uint8_t MASK_BATFET_DIS   = _BV(5);   ///< Battery FET disable
static constexpr uint8_t MASK_JEITA_VSET   = _BV(4);   ///< JEITA voltage selection
static constexpr uint8_t MASK_BATFET_DLY   = _BV(3);   ///< Battery FET delay
static constexpr uint8_t MASK_BATFET_RST_EN= _BV(2);   ///< Battery FET reset enable
static constexpr uint8_t MASK_PUMPX_UP     = _BV(1);   ///< PumpEX up
static constexpr uint8_t MASK_PUMPX_DN     = _BV(0);   ///< PumpEX down

// ==============================================
// REG0A (RW) - Boost Control Register
// ==============================================
static constexpr uint8_t REG_BOOST_CTRL    = 0x0A;
static constexpr uint8_t MASK_BOOSTV       = 0xF0;   ///< Boost voltage (4-bit)
static constexpr uint8_t SHIFT_BOOSTV      = 4;
static constexpr uint8_t MASK_BOOST_LIM    = 0x07;   ///< Boost current limit (3-bit)
static constexpr uint8_t SHIFT_BOOST_LIM   = 0;

// ==============================================
// REG0B (RO) - Status Register
// ==============================================
static constexpr uint8_t REG_STATUS        = 0x0B;
static constexpr uint8_t MASK_BUS_STAT     = 0xE0;   ///< Bus state (3-bit)
static constexpr uint8_t SHIFT_BUS_STAT    = 5;
static constexpr uint8_t MASK_CHRG_STAT    = 0x18;   ///< Charge state (2-bit)
static constexpr uint8_t SHIFT_CHRG_STAT   = 3;
static constexpr uint8_t MASK_PG_STAT       = _BV(2);   ///< Power good status
static constexpr uint8_t MASK_SDP_STAT     = _BV(1);   ///< SDP status
static constexpr uint8_t MASK_VSYS_STAT    = _BV(0);   ///< VSYS status

// ==============================================
// REG0C (RO) - Fault Register
// ==============================================
static constexpr uint8_t REG_FAULT         = 0x0C;
static constexpr uint8_t MASK_WD_FAULT     = _BV(7);   ///< Watchdog fault
static constexpr uint8_t MASK_BOOST_FAULT  = _BV(6);   ///< Boost fault
static constexpr uint8_t MASK_CHRG_FAULT  = 0x30;   ///< Charge fault (2-bit)
static constexpr uint8_t SHIFT_CHRG_FAULT  = 4;
static constexpr uint8_t MASK_BAT_FAULT    = _BV(3);   ///< Battery fault
static constexpr uint8_t MASK_NTC_FAULT    = 0x07;   ///< NTC fault (3-bit)
static constexpr uint8_t SHIFT_NTC_FAULT   = 0;

// ==============================================
// REG0D (RW) - VINDPM Register
// ==============================================
static constexpr uint8_t REG_VINDPM         = 0x0D;
static constexpr uint8_t MASK_VINDPM_MODE   = _BV(7);   ///< VINDPM mode
static constexpr uint8_t MASK_VINDPM        = 0x7F;   ///< VINDPM voltage (7-bit)
static constexpr uint8_t SHIFT_VINDPM       = 0;

// ==============================================
// REG0E (RO) - Battery Voltage ADC
// ==============================================
static constexpr uint8_t REG_ADC_BATV       = 0x0E;
static constexpr uint8_t MASK_THERM_STAT    = _BV(7);   ///< Thermal status
static constexpr uint8_t MASK_BATV          = 0x7F;   ///< VBAT ADC value (7-bit)
static constexpr uint8_t SHIFT_BATV         = 0;

// ==============================================
// REG0F (RO) - System Voltage ADC
// ==============================================
static constexpr uint8_t REG_ADC_SYSV       = 0x0F;
static constexpr uint8_t MASK_SYSV          = 0x7F;   ///< VSYS ADC value (7-bit)
static constexpr uint8_t SHIFT_SYSV         = 0;

// ==============================================
// REG10 (RO) - NTC Percentage ADC
// ==============================================
static constexpr uint8_t REG_ADC_NTC        = 0x10;
static constexpr uint8_t MASK_NTPCPT        = 0x7F;   ///< NTC percentage (7-bit)
static constexpr uint8_t SHIFT_NTPCPT       = 0;

// ==============================================
// REG11 (RO) - VBUS Voltage ADC
// ==============================================
static constexpr uint8_t REG_ADC_BUSV       = 0x11;
static constexpr uint8_t MASK_BUS_GD        = _BV(7);   ///< VBUS good status
static constexpr uint8_t MASK_BUSV          = 0x7F;   ///< VBUS ADC value (7-bit)
static constexpr uint8_t SHIFT_BUSV         = 0;

// ==============================================
// REG12 (RO) - Charge Current ADC
// ==============================================
static constexpr uint8_t REG_ADC_ICHGR      = 0x12;
static constexpr uint8_t MASK_ICHGR         = 0x7F;   ///< Charge current ADC (7-bit)
static constexpr uint8_t SHIFT_ICHGR        = 0;

// ==============================================
// REG13 (RO) - Input Current Limit Status
// ==============================================
static constexpr uint8_t REG_IDPM_STATUS    = 0x13;
static constexpr uint8_t MASK_VDPM_STAT     = _BV(7);   ///< VINDPM status
static constexpr uint8_t MASK_IDPM_STAT     = _BV(6);   ///< IDPM status
static constexpr uint8_t MASK_IDPM_LIM     = 0x3F;   ///< Input current limit (6-bit)
static constexpr uint8_t SHIFT_IDPM_LIM    = 0;

// ==============================================
// REG14 (RW) - Device Revision Register
// ==============================================
static constexpr uint8_t REG_DEVICE_REV     = 0x14;
static constexpr uint8_t MASK_REG_RST       = _BV(7);   ///< Register reset
static constexpr uint8_t MASK_AICL_OPTIMIZED = _BV(6);  ///< AICL optimized
static constexpr uint8_t MASK_PN            = 0x38;   ///< Part number (3-bit)
static constexpr uint8_t SHIFT_PN           = 3;
static constexpr uint8_t MASK_NTC_PROFILE   = _BV(2);   ///< NTC profile
static constexpr uint8_t MASK_DEV_REV       = 0x03;   ///< Device revision (2-bit)
static constexpr uint8_t SHIFT_DEV_REV      = 0;

}

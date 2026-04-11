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
 * @file      BQ25896Regs.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 * @brief BQ25896 PMIC Register Definitions and Constants
 *
 * This header file contains all register addresses, bit masks, shift values,
 * and configuration constants for the TI BQ25896 PMIC (Power Management IC).
 *
 * @section Device Overview
 * The BQ25896 is a high-efficiency switch-mode charger with:
 * - Buck charger (up to 3A fast charge)
 * - USB BC1.2 and HVDCP detection support
 * - Integrated 10-bit ADC for voltage/current monitoring
 * - OTG (On-The-Go) boost mode for power bank functionality
 * - JEITA battery temperature protection
 *
 * @section Register Overview
 * - REG00 (0x00): Input source control - HIZ mode, input current limit
 * - REG01 (0x01): VINDPM control - Battery HOT/COLD threshold
 * - REG02 (0x02): Power on/off control - ADC conv, boost freq, ICO
 * - REG03 (0x03): Charge control - Bat load, OTG, charge enable, sys min
 * - REG04 (0x04): Fast charge current control (0-3008mA, 64mA steps)
 * - REG05 (0x05): Pre-charge/termination current
 * - REG06 (0x06): Charge voltage control (3840-4608mV, 16mV steps)
 * - REG07 (0x07): Charge timer and watchdog
 * - REG08 (0x08): IR compensation and thermal regulation
 * - REG09 (0x09): Misc control - BATFET, pumpEX
 * - REG0A (0x0A): Boost control - boost voltage/current limit
 * - REG0B (0x0B): Status register - bus/charge state, power good
 * - REG0C (0x0C): Fault register - watchdog, boost, charge, bat, NTC faults
 * - REG0D (0x0D): VINDPM voltage setting
 * - REG0E (0x0E): ADC - Battery voltage (2304-4844mV, 20mV steps)
 * - REG0F (0x0F): ADC - System voltage (2304-4844mV, 20mV steps)
 * - REG10 (0x10): ADC - NTC temperature (21%-80%, percentage, NOT temperature)
 * - REG11 (0x11): ADC - VBUS voltage (2600-15300mV, 100mV steps)
 * - REG12 (0x12): ADC - Charge current (0-6350mA, 50mA steps)
 * - REG13 (0x13): Input current limit status
 * - REG14 (0x14): Device revision and configuration
 *
 * @section ADC Channels (Important Notes)
 * - VBAT (REG0E): 2304-4844mV, step 20mV - Battery voltage
 * - VSYS (REG0F): 2304-4844mV, step 20mV - System voltage
 * - VBUS (REG11): 2600-15300mV, step 100mV - USB input voltage
 * - ICHG (REG12): 0-6350mA, step 50mA - Charge current (ADC readout)
 * - NTC (REG10): 21%-80.055%, step 0.465% - NTC thermistor percentage
 *   @note This is NOT temperature! It's the NTC voltage as percentage of REGN.
 *         To convert to actual temperature, you need the NTC's Beta value and
 *         the pull-up resistor configuration, which varies by battery pack manufacturer.
 *         Typical conversion: ~50% ≈ 25°C, but depends on NTC specification.
 *
 * @section Charge Parameters
 * - Pre-charge current: 64-1024mA (64mA steps)
 * - Fast charge current: 0-3008mA (64mA steps, 0 disables charge)
 * - Termination current: 64-1024mA (64mA steps)
 * - Charge voltage: 3840-4608mV (16mV steps)
 *
 * @section Boost Mode Parameters
 * - Boost voltage: 4550-5510mV (64mV steps)
 * - Boost current limit: 0.5A, 0.75A, 1.2A, 1.4A, 1.65A, 1.875A, 2.15A
 *
 * @note All setter methods with step requirements will automatically quantize
 *       input values to the nearest valid step if they do not match exactly.
 *
 * @see BQ25896Charger.cpp, BQ25896Power.cpp, BQ25896Adc.cpp, BQ25896Core.cpp
 * @see BQ25896RegisterMap.md for detailed register descriptions
 */
#pragma once

#include <stdint.h>

namespace BQ25896Regs {

#ifdef _BV
#undef _BV
#endif
#define _BV(bit)  (1U << (bit))

// ==============================================
// Device Revision Register
// ==============================================
static constexpr uint8_t BQ25896_DEV_REV = 0x02;

// ==============================================
// ADC Voltage Measurement Range Constants
// ==============================================
/**
 * @brief ADC measurement range constants
 * @note VBAT_MAX uses 4608 (0x48) as practical max, register supports up to 4844mV
 */
static constexpr uint16_t VBAT_MIN = 2304;       ///< Minimum battery voltage (mV)
static constexpr uint16_t VBAT_MAX = 4608;       ///< Maximum battery voltage (mV)
static constexpr uint16_t SYS_MIN = 3000;        ///< Minimum system voltage (mV)
static constexpr uint16_t SYS_MAX = 3700;        ///< Maximum system voltage (mV)
static constexpr uint16_t IN_CURRENT_MIN = 100;   ///< Minimum input current (mA)
static constexpr uint16_t IN_CURRENT_MAX = 3250;  ///< Maximum input current (mA)
static constexpr uint16_t CHG_CURRENT_MAX = 3008;///< Maximum fast charge current (mA)
static constexpr uint16_t BOOST_VOL_MIN = 4550;   ///< Minimum boost voltage (mV)
static constexpr uint16_t BOOST_VOL_MAX = 5510;   ///< Maximum boost voltage (mV)

// ==============================================
// USB Port/Adapter Type Definitions
// ==============================================
static constexpr uint8_t BUS_STATE_NOINPUT = 0;  ///< No input detected
static constexpr uint8_t BUS_STATE_USB_SDP = 1;  ///< USB Standard Downstream Port
static constexpr uint8_t BUS_STATE_ADAPTER = 2;   ///< External adapter (3.25A)
static constexpr uint8_t BUS_STATE_OTG = 7;      ///< OTG mode (supplying power)

// ==============================================
// Charge State Definitions
// ==============================================
static constexpr uint8_t CHARGE_STATE_NO_CHARGE = 0;  ///< Not charging
static constexpr uint8_t CHARGE_STATE_PRE_CHARGE = 1;  ///< Pre-charge (< VBATLOW)
static constexpr uint8_t CHARGE_STATE_FAST_CHARGE = 2; ///< Fast charge (CC/CV)
static constexpr uint8_t CHARGE_STATE_DONE = 3;        ///< Charge termination done

// ==============================================
// ADC Base Values (minimum measurable value)
// ==============================================
static constexpr uint16_t VBUS_BASE_VAL = 2600;   ///< VBUS ADC base: 2.6V
static constexpr uint16_t VBAT_BASE_VAL = 2304;  ///< VBAT ADC base: 2.304V
static constexpr uint16_t VSYS_BASE_VAL = 2304;  ///< VSYS ADC base: 2.304V
static constexpr float    NTC_BASE_VAL  = 21.0f; ///< NTC ADC base: 21%

// ==============================================
// ADC Voltage Step Values
// ==============================================
static constexpr uint16_t VBUS_VOL_STEP = 100;      ///< VBUS step: 100mV
static constexpr uint16_t VBAT_VOL_STEP = 20;        ///< VBAT step: 20mV
static constexpr uint16_t VSYS_VOL_STEP = 20;        ///< VSYS step: 20mV
static constexpr float    NTC_VOL_STEP  = 0.465f;   ///< NTC step: 0.465% per LSB

// ==============================================
// Fast Charge Current Parameters
// ==============================================
static constexpr uint16_t CHG_STEP_VAL  = 50;       ///< ADC charge current step: 50mA
static constexpr uint16_t FAST_CHG_CUR_STEP = 64;    ///< Fast charge current step: 64mA
static constexpr uint16_t FAST_CHG_CURRENT_MIN = 0;  ///< Minimum (0 disables charge)
static constexpr uint16_t FAST_CHG_CURRENT_MAX = 3008;///< Maximum fast charge: 3008mA

// ==============================================
// Pre-charge Current Parameters
// @note Formula: IPRECHG = 64mA + [IPRECHG] * 64mA
// ==============================================
static constexpr uint16_t PRE_CHG_CUR_BASE = 64;     ///< Pre-charge base: 64mA
static constexpr uint16_t PRE_CHG_CUR_STEP = 64;     ///< Pre-charge step: 64mA
static constexpr uint16_t PRE_CHG_CURRENT_MIN = 64;  ///< Minimum: 64mA
static constexpr uint16_t PRE_CHG_CURRENT_MAX = 1024;///< Maximum: 1024mA

// ==============================================
// Termination Current Parameters
// @note Formula: ITERM = 64mA + [ITERM] * 64mA
// ==============================================
static constexpr uint16_t TERM_CHG_CUR_BASE = 64;   ///< Termination base: 64mA
static constexpr uint16_t TERM_CHG_CUR_STEP = 64;    ///< Termination step: 64mA
static constexpr uint16_t TERM_CHG_CURRENT_MIN = 64; ///< Minimum: 64mA
static constexpr uint16_t TERM_CHG_CURRENT_MAX = 1024;///< Maximum: 1024mA

// ==============================================
// Charge Voltage Parameters
// @note Formula: VREG = 3.840V + [VREG] * 16mV
// ==============================================
static constexpr uint16_t CHG_VOL_BASE = 3840;      ///< Charge voltage base: 3840mV
static constexpr uint16_t CHG_VOL_STEP = 16;         ///< Charge voltage step: 16mV
static constexpr uint16_t FAST_CHG_VOL_MIN = 3840;   ///< Minimum: 3840mV
static constexpr uint16_t FAST_CHG_VOL_MAX = 4608;   ///< Maximum: 4608mV

// ==============================================
// System Voltage Parameters
// @note Formula: SYS_MIN = 3.0V + [SYS_MIN] * 0.1V
// ==============================================
static constexpr uint16_t SYS_VOL_STEPS = 100;       ///< SYS voltage step: 100mV
static constexpr uint16_t SYS_VOFF_VOL_MIN = 3000;  ///< Minimum: 3.0V
static constexpr uint16_t SYS_VOFF_VOL_MAX = 3700;  ///< Maximum: 3.7V

// ==============================================
// Input Current Limit Parameters
// @note Formula: IINLIM = 100mA + [IINLIM] * 50mA
// ==============================================
static constexpr uint16_t IN_CURRENT_STEP = 50;      ///< Input current step: 50mA
static constexpr uint16_t IN_CURRENT_MIN_VAL = 100;  ///< Minimum: 100mA
static constexpr uint16_t IN_CURRENT_MAX_VAL = 3250; ///< Maximum: 3250mA

// ==============================================
// Boost Mode Parameters
// @note Formula: VBOOST = 4.55V + [BOOSTV] * 64mV
// ==============================================
static constexpr uint16_t BOOST_VOL_BASE = 4550;     ///< Boost voltage base: 4550mV
static constexpr uint16_t BOOST_VOL_STEP = 64;        ///< Boost voltage step: 64mV
static constexpr uint16_t BOOST_VOL_MIN_VAL = 4550;   ///< Minimum: 4550mV
static constexpr uint16_t BOOST_VOL_MAX_VAL = 5510;  ///< Maximum: 5510mV

// ==============================================
// VINDPM Parameters (Absolute Mode)
// @note Formula: VINDPM = 2.6V + [VINDPM] * 100mV
// ==============================================
static constexpr uint16_t VINDPM_VOL_BASE = 2600;    ///< VINDPM base: 2.6V
static constexpr uint16_t VINDPM_VOL_STEPS = 100;     ///< VINDPM step: 100mV
static constexpr uint16_t VINDPM_VOL_MIN = 3900;     ///< Minimum clamped: 3.9V
static constexpr uint16_t VINDPM_VOL_MAX = 15300;    ///< Maximum: 15.3V

// ==============================================
// VINDPM Offset Parameters (Relative Mode)
// @note Used when VBUS <= 6V. Formula: VINDPM_OS = [VINDPM_OS] * 100mV
// ==============================================
static constexpr uint16_t VINDPM_OS_STEP = 100;      ///< VINDPM offset step: 100mV
static constexpr uint16_t VINDPM_OS_MAX = 3100;      ///< Maximum offset: 3100mV

// ==============================================
// Battery Compensation Parameters
// @note Formula: BAT_COMP = [BAT_COMP] * 20mohm
// ==============================================
static constexpr uint16_t BAT_COMP_STEPS = 20;       ///< Battery compensation step: 20mΩ
static constexpr uint16_t BAT_COMP_MAX = 140;        ///< Maximum: 140mΩ

// ==============================================
// Voltage Clamp Parameters (IR Compensation)
// @note Formula: VCLAMP = [VCLAMP] * 32mV
// ==============================================
static constexpr uint16_t VCLAMP_STEP = 32;          ///< Voltage clamp step: 32mV
static constexpr uint16_t VCLAMP_MAX = 224;          ///< Maximum: 224mV

// ==============================================
// REG00 (RW) - Input Source Control Register
// Address: 0x00
// ==============================================
static constexpr uint8_t REG_PWR_INPUT     = 0x00;
static constexpr uint8_t MASK_EN_HIZ       = _BV(7);   ///< Enable HIZ mode (high impedance)
static constexpr uint8_t MASK_EN_ILIM      = _BV(6);   ///< Enable ILIM pin
static constexpr uint8_t MASK_IINLIM       = 0x3F;     ///< Input current limit (6-bit)
static constexpr uint8_t SHIFT_IINLIM      = 0;

// ==============================================
// REG01 (RW) - VINDPM Control Register
// Address: 0x01
// ==============================================
static constexpr uint8_t REG_PWR_VINDPM    = 0x01;
static constexpr uint8_t MASK_BHOT         = 0xC0;     ///< Boost hot temperature (2-bit)
static constexpr uint8_t SHIFT_BHOT        = 6;
static constexpr uint8_t MASK_BCOLD        = _BV(5);   ///< Boost cold temperature
static constexpr uint8_t MASK_VINDPM_OS    = 0x1F;     ///< VINDPM offset (5-bit)
static constexpr uint8_t SHIFT_VINDPM_OS    = 0;

// ==============================================
// REG02 (RW) - Power On/Off Control Register
// Address: 0x02
// ==============================================
static constexpr uint8_t REG_PWR_ONOFF     = 0x02;
static constexpr uint8_t MASK_CONV_START   = _BV(7);   ///< ADC conversion start
static constexpr uint8_t MASK_CONV_RATE    = _BV(6);   ///< ADC continuous mode
static constexpr uint8_t MASK_BOOST_FREQ   = _BV(5);   ///< Boost frequency (500kHz/1.5MHz)
static constexpr uint8_t MASK_ICO_EN       = _BV(4);   ///< Input Current Optimizer enable
static constexpr uint8_t MASK_FORCE_DPDM   = _BV(1);   ///< Force DP/DM detection
static constexpr uint8_t MASK_AUTO_DPDM_EN = _BV(0);   ///< Auto DP/DM detection enable

// ==============================================
// REG03 (RW) - Charge Control Register
// Address: 0x03
// ==============================================
static constexpr uint8_t REG_CHG_CTRL      = 0x03;
static constexpr uint8_t MASK_BAT_LOAD_EN  = _BV(7);   ///< Battery load enable
static constexpr uint8_t MASK_WD_RST       = _BV(6);   ///< Watchdog timer reset
static constexpr uint8_t MASK_OTG_CONFIG   = _BV(5);   ///< OTG (boost) mode enable
static constexpr uint8_t MASK_CHG_CONFIG   = _BV(4);   ///< Charge enable
static constexpr uint8_t MASK_SYS_MIN      = 0x0E;     ///< System minimum voltage (3-bit)
static constexpr uint8_t SHIFT_SYS_MIN     = 1;
static constexpr uint8_t MASK_MIN_VBAT_SEL  = _BV(0);  ///< Minimum battery voltage select

// ==============================================
// REG04 (RW) - Fast Charge Current Control Register
// Address: 0x04
// @note ICHG = 0 disables charging
// ==============================================
static constexpr uint8_t REG_CHG_CURRENT   = 0x04;
static constexpr uint8_t MASK_EN_PUMPX     = _BV(7);   ///< Current pulse control enable
static constexpr uint8_t MASK_ICHG         = 0x7F;     ///< Fast charge current (7-bit, 0-3008mA)
static constexpr uint8_t SHIFT_ICHG        = 0;

// ==============================================
// REG05 (RW) - Pre-charge/Termination Current Control
// Address: 0x05
// ==============================================
static constexpr uint8_t REG_CHG_PRE_CURR  = 0x05;
static constexpr uint8_t MASK_IPRECHG      = 0xF0;     ///< Pre-charge current (4-bit)
static constexpr uint8_t SHIFT_IPRECHG     = 4;
static constexpr uint8_t MASK_ITERM        = 0x0F;     ///< Termination current (4-bit)
static constexpr uint8_t SHIFT_ITERM       = 0;

// ==============================================
// REG06 (RW) - Charge Voltage Control Register
// Address: 0x06
// ==============================================
static constexpr uint8_t REG_CHG_VOLT      = 0x06;
static constexpr uint8_t MASK_VREG         = 0xFC;     ///< Charge voltage (6-bit)
static constexpr uint8_t SHIFT_VREG        = 2;
static constexpr uint8_t MASK_BATLOWV        = _BV(1); ///< Battery low voltage threshold
static constexpr uint8_t MASK_VRECHG       = _BV(0);   ///< Recharge voltage offset

// ==============================================
// REG07 (RW) - Charge Timer/Control Register
// Address: 0x07
// ==============================================
static constexpr uint8_t REG_CHG_TIMER     = 0x07;
static constexpr uint8_t MASK_EN_TERM      = _BV(7);   ///< Termination enable
static constexpr uint8_t MASK_STAT_DIS     = _BV(6);   ///< STAT pin disable
static constexpr uint8_t MASK_WATCHDOG     = 0x30;     ///< Watchdog timer (2-bit)
static constexpr uint8_t SHIFT_WATCHDOG    = 4;
static constexpr uint8_t MASK_EN_TIMER     = _BV(3);   ///< Charge timer enable
static constexpr uint8_t MASK_CHG_TIMER    = 0x06;     ///< Charge timer (2-bit)
static constexpr uint8_t SHIFT_CHG_TIMER   = 1;
static constexpr uint8_t MASK_JEITA_ISET   = _BV(0);   ///< JEITA low temp current

// ==============================================
// REG08 (RW) - IR Compensation/Thermal Control
// Address: 0x08
// ==============================================
static constexpr uint8_t REG_CHG_IR_COMP   = 0x08;
static constexpr uint8_t MASK_BAT_COMP    = 0xE0;     ///< Battery compensation (3-bit)
static constexpr uint8_t SHIFT_BAT_COMP    = 5;
static constexpr uint8_t MASK_VCLAMP       = 0x1C;     ///< Voltage clamp (3-bit)
static constexpr uint8_t SHIFT_VCLAMP       = 2;
static constexpr uint8_t MASK_TREG          = 0x03;     ///< Thermal regulation (2-bit)
static constexpr uint8_t SHIFT_TREG        = 0;

// ==============================================
// REG09 (RW) - Misc Control Register
// Address: 0x09
// ==============================================
static constexpr uint8_t REG_MISC_CTRL     = 0x09;
static constexpr uint8_t MASK_FORCE_ICO    = _BV(7);   ///< Force ICO start
static constexpr uint8_t MASK_TMR2X_EN    = _BV(6);   ///< Timer 2X during DPM/thermal
static constexpr uint8_t MASK_BATFET_DIS   = _BV(5);   ///< BATFET disable (ship mode)
static constexpr uint8_t MASK_JEITA_VSET   = _BV(4);   ///< JEITA high temp voltage
static constexpr uint8_t MASK_BATFET_DLY   = _BV(3);   ///< BATFET turn-off delay
static constexpr uint8_t MASK_BATFET_RST_EN= _BV(2);   ///< BATFET reset enable
static constexpr uint8_t MASK_PUMPX_UP     = _BV(1);   ///< Current pulse up
static constexpr uint8_t MASK_PUMPX_DN     = _BV(0);   ///< Current pulse down

// ==============================================
// REG0A (RW) - Boost Control Register
// Address: 0x0A
// ==============================================
static constexpr uint8_t REG_BOOST_CTRL    = 0x0A;
static constexpr uint8_t MASK_BOOSTV       = 0xF0;     ///< Boost voltage (4-bit)
static constexpr uint8_t SHIFT_BOOSTV      = 4;
static constexpr uint8_t MASK_PFM_OTG_DIS  = _BV(3);   ///< PFM disable in boost
static constexpr uint8_t MASK_BOOST_LIM    = 0x07;     ///< Boost current limit (3-bit)
static constexpr uint8_t SHIFT_BOOST_LIM   = 0;

// ==============================================
// REG0B (RO) - Status Register
// Address: 0x0B
// ==============================================
static constexpr uint8_t REG_STATUS        = 0x0B;
static constexpr uint8_t MASK_VBUS_STAT    = 0xE0;     ///< VBUS status (3-bit)
static constexpr uint8_t SHIFT_VBUS_STAT   = 5;
static constexpr uint8_t MASK_CHRG_STAT    = 0x18;     ///< Charge status (2-bit)
static constexpr uint8_t SHIFT_CHRG_STAT   = 3;
static constexpr uint8_t MASK_PG_STAT      = _BV(2);   ///< Power good status
static constexpr uint8_t MASK_VSYS_STAT    = _BV(0);   ///< VSYS regulation status

// ==============================================
// REG0C (RO) - Fault Register
// Address: 0x0C
// ==============================================
static constexpr uint8_t REG_FAULT         = 0x0C;
static constexpr uint8_t MASK_WD_FAULT    = _BV(7);   ///< Watchdog fault
static constexpr uint8_t MASK_BOOST_FAULT = _BV(6);   ///< Boost fault
static constexpr uint8_t MASK_CHRG_FAULT  = 0x30;     ///< Charge fault (2-bit)
static constexpr uint8_t SHIFT_CHRG_FAULT = 4;
static constexpr uint8_t MASK_BAT_FAULT   = _BV(3);   ///< Battery fault
static constexpr uint8_t MASK_NTC_FAULT   = 0x07;     ///< NTC fault (3-bit)
static constexpr uint8_t SHIFT_NTC_FAULT  = 0;

// ==============================================
// REG0D (RW) - VINDPM Register
// Address: 0x0D
// ==============================================
static constexpr uint8_t REG_VINDPM         = 0x0D;
static constexpr uint8_t MASK_FORCE_VINDPM  = _BV(7);  ///< VINDPM mode (absolute/relative)
static constexpr uint8_t MASK_VINDPM        = 0x7F;     ///< VINDPM voltage (7-bit)
static constexpr uint8_t SHIFT_VINDPM       = 0;

// ==============================================
// REG0E (RO) - Battery Voltage ADC
// Address: 0x0E
// ==============================================
static constexpr uint8_t REG_ADC_BATV       = 0x0E;
static constexpr uint8_t MASK_THERM_STAT     = _BV(7); ///< Thermal regulation status
static constexpr uint8_t MASK_BATV          = 0x7F;    ///< VBAT ADC value (7-bit)
static constexpr uint8_t SHIFT_BATV         = 0;

// ==============================================
// REG0F (RO) - System Voltage ADC
// Address: 0x0F
// ==============================================
static constexpr uint8_t REG_ADC_SYSV       = 0x0F;
static constexpr uint8_t MASK_SYSV          = 0x7F;    ///< VSYS ADC value (7-bit)
static constexpr uint8_t SHIFT_SYSV         = 0;

// ==============================================
// REG10 (RO) - NTC Percentage ADC
// Address: 0x10
// @warning IMPORTANT: This returns NTC percentage, NOT temperature!
// Formula: NTC/REGN = 21% + [TSPCT] * 0.465%
// Range: 21% - 80.055%
// To convert to actual temperature, you need the NTC's Beta value and
// pull-up resistor configuration from your battery pack specifications.
// ==============================================
static constexpr uint8_t REG_ADC_NTC        = 0x10;
static constexpr uint8_t MASK_TSPCT         = 0x7F;    ///< NTC percentage (7-bit)
static constexpr uint8_t SHIFT_TSPCT       = 0;

// ==============================================
// REG11 (RO) - VBUS Voltage ADC
// Address: 0x11
// ==============================================
static constexpr uint8_t REG_ADC_BUSV       = 0x11;
static constexpr uint8_t MASK_VBUS_GD      = _BV(7);   ///< VBUS good status
static constexpr uint8_t MASK_VBUSV         = 0x7F;     ///< VBUS ADC value (7-bit)
static constexpr uint8_t SHIFT_VBUSV        = 0;

// ==============================================
// REG12 (RO) - Charge Current ADC
// Address: 0x12
// @note Returns 0 when VBAT < VBATSHORT
// ==============================================
static constexpr uint8_t REG_ADC_ICHGR      = 0x12;
static constexpr uint8_t MASK_ICHGR         = 0x7F;     ///< Charge current ADC (7-bit)
static constexpr uint8_t SHIFT_ICHGR        = 0;

// ==============================================
// REG13 (RO) - Input Current Limit Status
// Address: 0x13
// ==============================================
static constexpr uint8_t REG_IDPM_STATUS    = 0x13;
static constexpr uint8_t MASK_VDPM_STAT     = _BV(7);   ///< VINDPM status
static constexpr uint8_t MASK_IDPM_STAT     = _BV(6);   ///< IINDPM status
static constexpr uint8_t MASK_IDPM_LIM     = 0x3F;       ///< Input current limit (6-bit)
static constexpr uint8_t SHIFT_IDPM_LIM    = 0;

// ==============================================
// REG14 (RW) - Device Revision Register
// Address: 0x14
// ==============================================
static constexpr uint8_t REG_DEVICE_REV     = 0x14;
static constexpr uint8_t MASK_REG_RST       = _BV(7);   ///< Register reset
static constexpr uint8_t MASK_ICO_OPTIMIZED  = _BV(6);   ///< ICO optimized status
static constexpr uint8_t MASK_PN             = 0x38;     ///< Part number (3-bit)
static constexpr uint8_t SHIFT_PN           = 3;
static constexpr uint8_t MASK_TS_PROFILE    = _BV(2);   ///< Temperature profile
static constexpr uint8_t MASK_DEV_REV        = 0x03;     ///< Device revision (2-bit)
static constexpr uint8_t SHIFT_DEV_REV       = 0;

}

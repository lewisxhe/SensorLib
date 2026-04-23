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
 * @file      AXP517Regs.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#pragma once
#include <stdint.h>

/**
 * @brief AXP517 PMIC registers (non-TCPC).
 *
 * Rules:
 *  - Only put PMIC/charger/ADC/IRQ/LED/BC1.2/JEITA/TS registers here.
 *  - Do NOT put TCPC / Type-C / PD (0xA6+) regs here to avoid conflicts.
 *  - Keep addresses grouped by feature.
 *
 * TCPC/PD regs are in AXP517TcpcRegs.hpp.
 */
namespace axp517::regs
{

// -------------------- BMU / Status / Fault --------------------
namespace bmu
{
static constexpr uint8_t STATUS0        = 0x00; ///< BMU status0
static constexpr uint8_t STATUS1        = 0x01; ///< BMU status1
static constexpr uint8_t DATA_BUFFER0   = 0x04; ///< Data Buffer0
static constexpr uint8_t BC_DETECT      = 0x05; ///< BC_detect (status/result)
static constexpr uint8_t FAULT0         = 0x06; ///< BMU fault0
static constexpr uint8_t FAULT1         = 0x08; ///< BMU fault1
static constexpr uint8_t MODULE_EN0     = 0x0B; ///< Module enable control0
static constexpr uint8_t DATA_BUFFER1   = 0x0C; ///< Data Buffer1
static constexpr uint8_t DATA_BUFFER2   = 0x0D; ///< Data Buffer2
} // namespace bmu

// -------------------- Common / Control / Power Path --------------------
namespace ctrl
{
static constexpr uint8_t COMMON_CFG         = 0x10; ///< Common configure
static constexpr uint8_t GPIO_CFG           = 0x11; ///< GPIO configure
static constexpr uint8_t BATFET_CTRL        = 0x12; ///< BATFET control
static constexpr uint8_t RBFET_CTRL         = 0x13; ///< RBFET control
static constexpr uint8_t DIE_TEMP_CFG       = 0x14; ///< Die temperature configure
static constexpr uint8_t MIN_SYS_VOLTAGE    = 0x15; ///< Minimum system voltage control
static constexpr uint8_t INPUT_VOLT_LIMIT   = 0x16; ///< Input voltage limit control
static constexpr uint8_t INPUT_CURR_LIMIT   = 0x17; ///< Input current limit control
static constexpr uint8_t RESET_CFG          = 0x18; ///< Reset configure
static constexpr uint8_t MODULE_EN1         = 0x19; ///< Module enable control1
static constexpr uint8_t WATCHDOG_CTRL      = 0x1A; ///< Watchdog control
static constexpr uint8_t LOW_BAT_WARN       = 0x1B; ///< Low Battery warning threshold setting
static constexpr uint8_t PWRON_CFG          = 0x1C; ///< PWRON configure
static constexpr uint8_t VBUS_OV_CHG_PWRON  = 0x1D; ///< VBUS_OV/CHG_PWRON configure
static constexpr uint8_t BOOST_CFG          = 0x1E; ///< Boost configure
static constexpr uint8_t MPPT_CFG           = 0x22; ///< MPPT configure
} // namespace ctrl

// -------------------- BC1.2 --------------------
namespace bc12
{
static constexpr uint8_t CTRL0 = 0x28; ///< BC1.2 control0
static constexpr uint8_t CTRL1 = 0x29; ///< BC1.2 control1
static constexpr uint8_t CTRL2 = 0x2A; ///< BC1.2 control2
static constexpr uint8_t CTRL3 = 0x2B; ///< BC1.2 control3
} // namespace bc12

// -------------------- LED / Breath --------------------
namespace led
{
static constexpr uint8_t CHGLED_CFG   = 0x30; ///< CHGLED configure
static constexpr uint8_t BREATH_CL    = 0x32; ///< Breath LED control0 (CL)
static constexpr uint8_t BREATH_CH    = 0x33; ///< Breath LED control1 (CH)
static constexpr uint8_t BREATH_CSR   = 0x34; ///< Breath LED control2 (CSR)
static constexpr uint8_t BREATH_NS    = 0x36; ///< Breath LED control3 (NS)
static constexpr uint8_t BREATH_NMIN  = 0x37; ///< Breath LED control4 (NMIN)
static constexpr uint8_t BREATH_M     = 0x38; ///< Breath LED control5 (M)
} // namespace led

// -------------------- IRQ --------------------
namespace irq
{
static constexpr uint8_t ENABLE0 = 0x40; ///< IRQ Enable 0
static constexpr uint8_t ENABLE1 = 0x41; ///< IRQ Enable 1
static constexpr uint8_t ENABLE2 = 0x42; ///< IRQ Enable 2
static constexpr uint8_t ENABLE3 = 0x43; ///< IRQ Enable 3
static constexpr uint8_t STATUS0 = 0x48; ///< IRQ Status 0
static constexpr uint8_t STATUS1 = 0x49; ///< IRQ Status 1
static constexpr uint8_t STATUS2 = 0x4A; ///< IRQ Status 2
static constexpr uint8_t STATUS3 = 0x4B; ///< IRQ Status 3
} // namespace irq

// -------------------- TS / JEITA --------------------
namespace ts
{
static constexpr uint8_t PIN_CFG     = 0x50; ///< TS pin configure
static constexpr uint8_t HYSL2H      = 0x52; ///< TS_HYSL2H setting
static constexpr uint8_t HYSH2L      = 0x53; ///< TS_HYSH2L setting
static constexpr uint8_t DATA_CFG0   = 0x5C; ///< TS data configure 0
static constexpr uint8_t DATA_CFG1   = 0x5D; ///< TS data configure 1
static constexpr uint8_t SOURCE_SEL  = 0x82; ///< TS source select
} // namespace ts

namespace jeita
{
static constexpr uint8_t VLTF_CHG        = 0x54; ///< VLTF_CHG setting
static constexpr uint8_t VHTF_CHG        = 0x55; ///< VHTF_CHG setting
static constexpr uint8_t VLTF_WORK       = 0x56; ///< VLTF_WORK setting
static constexpr uint8_t VHTF_WORK       = 0x57; ///< VHTF_WORK setting
static constexpr uint8_t STD_ENABLE      = 0x58; ///< JEITA standard Enable control
static constexpr uint8_t STD_SETTING0    = 0x59; ///< JEITA standard setting 0
static constexpr uint8_t STD_SETTING1    = 0x5A; ///< JEITA standard setting 1
static constexpr uint8_t STD_SETTING2    = 0x5B; ///< JEITA standard setting 2
} // namespace jeita

// -------------------- Charger parameters --------------------
namespace chg
{
static constexpr uint8_t IPRECHG_ITRICHG    = 0x61; ///< Iprechg/Itrichg setting
static constexpr uint8_t ICC_SETTING        = 0x62; ///< ICC setting
static constexpr uint8_t ITERM_CTRL         = 0x63; ///< Iterm setting and control
static constexpr uint8_t CV_CHG_VOLTAGE     = 0x64; ///< CV charger voltage setting
static constexpr uint8_t THERMAL_REG_THRESH = 0x65; ///< Thermal regulation threshold setting
static constexpr uint8_t CHG_TIMER_CFG      = 0x67; ///< Charger timer configure
static constexpr uint8_t BAT_DETECT_CTRL    = 0x68; ///< Battery detection control
} // namespace chg

// -------------------- Battery/Gauge basic --------------------
namespace gauge
{
static constexpr uint8_t BAT_PARAM       = 0x70; ///< Battery parameter
static constexpr uint8_t FUEL_GAUGE_CTRL = 0x71; ///< Fuel gauge control
static constexpr uint8_t BAT_TEMP        = 0x72; ///< Battery temperature (RO)
static constexpr uint8_t BAT_SOH         = 0x73; ///< Battery SOH (RO)
static constexpr uint8_t BAT_PERCENT     = 0x74; ///< Battery percentage (RO)
} // namespace gauge

// -------------------- ADC measurement results --------------------
namespace adc
{
static constexpr uint8_t ENABLE          = 0x90; ///< ADC channel enable control
static constexpr uint8_t BAT_VOLT_H      = 0x91; ///< BAT voltage ADC high
static constexpr uint8_t BAT_VOLT_L      = 0x92; ///< BAT voltage ADC low
static constexpr uint8_t BAT_CURR_H      = 0x93; ///< BAT current ADC high
static constexpr uint8_t BAT_CURR_L      = 0x94; ///< BAT current ADC low
static constexpr uint8_t TS_VOLT_H       = 0x95; ///< TS voltage ADC high
static constexpr uint8_t TS_VOLT_L       = 0x96; ///< TS voltage ADC low
static constexpr uint8_t VBUS_CURR_H     = 0x97; ///< VBUS current ADC high
static constexpr uint8_t VBUS_CURR_L     = 0x98; ///< VBUS current ADC low
static constexpr uint8_t VBUS_VOLT_H     = 0x99; ///< VBUS voltage ADC high
static constexpr uint8_t VBUS_VOLT_L     = 0x9A; ///< VBUS voltage ADC low
static constexpr uint8_t DATA_SEL        = 0x9B; ///< ADC data select
static constexpr uint8_t DATA_H          = 0x9C; ///< ADC data high
static constexpr uint8_t DATA_L          = 0x9D; ///< ADC data low
} // namespace adc

namespace factory
{
static constexpr float FACTORY_TS = 0.5f;                  ///< 14-Bit(including sign bit)
static constexpr float FACTORY_DIE_TEMPERATURE = 0.2f;     ///< 14-Bit(including sign bit)
static constexpr float FACTORY_VBUS_VOLTAGE = 2.0f;        ///< 14-Bit(including sign bit)
static constexpr float FACTORY_VBAT_VOLTAGE = 1.0f;        ///< 14-Bit(including sign bit)
static constexpr float FACTORY_CHARGE_CURRENT = 0.25f;     ///< 16-Bit(including sign bit)
static constexpr float FACTORY_DISCHARGE_CURRENT = 0.25f;  ///< 16-Bit(including sign bit)
static constexpr float FACTORY_VBUS_CURRENT = 1.0f;        ///< 14-Bit(including sign bit)
static constexpr float FACTORY_VSYS_VOLTAGE = 1.0f;        ///< 14-Bit(including sign bit)
static constexpr float FACTORY_BAT_CURRENT = 0.25f;        ///< 16-Bit(including sign bit)
} // namespace factory

} // namespace axp517::regs

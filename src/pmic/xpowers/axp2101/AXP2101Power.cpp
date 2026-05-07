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
 * @file      AXP2101Power.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Power Control
 *
 */
#include "AXP2101Power.hpp"
#include "AXP2101Regs.hpp"
#include <string.h>


AXP2101Power::AXP2101Power(AXP2101Core &core) : _core(core)
{
}

// ---- PmicPowerBase overrides ----

bool AXP2101Power::setMinimumSystemVoltage(uint32_t mv)
{
    if (mv < 4100 || mv > 4800) return false;
    uint8_t code = (mv - 4100) / 100;
    return _core.updateBits(axp2101_regs::ctrl::MIN_SYS_VOLTAGE, 0x70, code << 4) >= 0;
}

uint32_t AXP2101Power::getMinimumSystemVoltage() const
{
    int val = _core.readReg(axp2101_regs::ctrl::MIN_SYS_VOLTAGE);
    if (val < 0) return 0;
    return 4100 + (((val >> 4) & 0x07) * 100);
}

bool AXP2101Power::setInputVoltageLimit(uint32_t mv)
{
    if (mv < 3880) mv = 3880;
    if (mv > 5080) mv = 5080;
    uint8_t code = (mv - 3880) / 80;
    return _core.updateBits(axp2101_regs::ctrl::INPUT_VOLT_LIMIT, 0x0F, code) >= 0;
}

uint32_t AXP2101Power::getInputVoltageLimit() const
{
    int val = _core.readReg(axp2101_regs::ctrl::INPUT_VOLT_LIMIT);
    if (val < 0) return 0;
    return 3880 + ((val & 0x0F) * 80);
}

bool AXP2101Power::setInputCurrentLimit(uint32_t mA)
{
    static const uint16_t values[] = {100, 500, 900, 1000, 1500, 2000};
    static const uint8_t codes[]   = {0, 1, 2, 3, 4, 5};
    uint8_t code = 0;
    uint32_t best = 0;
    for (int i = 0; i < 6; ++i) {
        uint32_t diff = (mA > values[i]) ? (mA - values[i]) : (values[i] - mA);
        if (i == 0 || diff < best) {
            best = diff;
            code = codes[i];
        }
    }
    return _core.updateBits(axp2101_regs::ctrl::INPUT_CURR_LIMIT, 0x07, code) >= 0;
}

uint32_t AXP2101Power::getInputCurrentLimit() const
{
    static const uint16_t values[] = {100, 500, 900, 1000, 1500, 2000};
    int val = _core.readReg(axp2101_regs::ctrl::INPUT_CURR_LIMIT);
    if (val < 0) return 0;
    uint8_t code = val & 0x07;
    return (code <= 5) ? values[code] : values[5];
}

// Ship mode closes BATFET
bool AXP2101Power::enableShipMode(bool enable)
{
    if (enable) {
        return _core.clrRegBit(axp2101_regs::ctrl::BATFET_CTRL, 3);
    }
    return _core.setRegBit(axp2101_regs::ctrl::BATFET_CTRL, 3);
}

bool AXP2101Power::isShipModeEnabled() const
{
    return !_core.getRegBit(axp2101_regs::ctrl::BATFET_CTRL, 3);
}

// ---- VSYS Power Down Voltage (REG 24) ----

bool AXP2101Power::setSysPowerDownVoltage(uint16_t millivolt)
{
    if (millivolt < 2600 || millivolt > 3300 || (millivolt % 100))
        return false;
    int val = _core.readReg(axp2101_regs::pmu::VOFF_SET);
    if (val < 0) return false;
    val &= 0xF8;
    val |= (millivolt - 2600) / 100;
    return _core.writeReg(axp2101_regs::pmu::VOFF_SET, val) >= 0;
}

uint16_t AXP2101Power::getSysPowerDownVoltage()
{
    int val = _core.readReg(axp2101_regs::pmu::VOFF_SET);
    if (val < 0) return 0;
    return ((val & 0x07) * 100) + 2600;
}

// ---- Internal Discharge (REG 10 bit 5) ----

void AXP2101Power::enableInternalDischarge()
{
    _core.setRegBit(axp2101_regs::ctrl::COMMON_CFG, 5);
}

void AXP2101Power::disableInternalDischarge()
{
    _core.clrRegBit(axp2101_regs::ctrl::COMMON_CFG, 5);
}

// ---- System Reset & Shutdown (REG 10) ----

void AXP2101Power::softShutdown()
{
    _core.setRegBit(axp2101_regs::ctrl::COMMON_CFG, 0);
}

void AXP2101Power::resetSoC()
{
    _core.setRegBit(axp2101_regs::ctrl::COMMON_CFG, 1);
}

// ---- Sleep / Wakeup (REG 26) ----

bool AXP2101Power::enableSleep()
{
    return _core.setRegBit(axp2101_regs::pmu::SLEEP_WAKEUP_CTRL, 0);
}

bool AXP2101Power::disableSleep()
{
    return _core.clrRegBit(axp2101_regs::pmu::SLEEP_WAKEUP_CTRL, 0);
}

bool AXP2101Power::enableWakeup()
{
    return _core.setRegBit(axp2101_regs::pmu::SLEEP_WAKEUP_CTRL, 1);
}

bool AXP2101Power::disableWakeup()
{
    return _core.clrRegBit(axp2101_regs::pmu::SLEEP_WAKEUP_CTRL, 1);
}

// ---- PWROK & Sequence (REG 25) ----

void AXP2101Power::enablePwrOk()
{
    _core.setRegBit(axp2101_regs::pmu::PWROK_SEQU_CTRL, 4);
}

void AXP2101Power::disablePwrOk()
{
    _core.clrRegBit(axp2101_regs::pmu::PWROK_SEQU_CTRL, 4);
}

void AXP2101Power::enablePowerOffDelay()
{
    _core.setRegBit(axp2101_regs::pmu::PWROK_SEQU_CTRL, 3);
}

void AXP2101Power::disablePowerOffDelay()
{
    _core.clrRegBit(axp2101_regs::pmu::PWROK_SEQU_CTRL, 3);
}

void AXP2101Power::enablePowerSequence()
{
    _core.setRegBit(axp2101_regs::pmu::PWROK_SEQU_CTRL, 2);
}

void AXP2101Power::disablePowerSequence()
{
    _core.clrRegBit(axp2101_regs::pmu::PWROK_SEQU_CTRL, 2);
}

bool AXP2101Power::setPwrOkDelay(uint8_t opt)
{
    if (opt > 3) return false;
    int val = _core.readReg(axp2101_regs::pmu::PWROK_SEQU_CTRL);
    if (val < 0) return false;
    val &= 0xFC;
    val |= opt;
    return _core.writeReg(axp2101_regs::pmu::PWROK_SEQU_CTRL, val) >= 0;
}

uint8_t AXP2101Power::getPwrOkDelay()
{
    int val = _core.readReg(axp2101_regs::pmu::PWROK_SEQU_CTRL);
    if (val < 0) return 0;
    return val & 0x03;
}

// ---- Power Off Configuration (REG 22) ----

void AXP2101Power::enableOverTempShutdown()
{
    _core.setRegBit(axp2101_regs::pmu::PWROFF_EN, 2);
}

void AXP2101Power::disableOverTempShutdown()
{
    _core.clrRegBit(axp2101_regs::pmu::PWROFF_EN, 2);
}

void AXP2101Power::enableLongPressShutdown()
{
    _core.setRegBit(axp2101_regs::pmu::PWROFF_EN, 1);
}

void AXP2101Power::disableLongPressShutdown()
{
    _core.clrRegBit(axp2101_regs::pmu::PWROFF_EN, 1);
}

void AXP2101Power::setLongPressAction(bool restart)
{
    if (restart) {
        _core.setRegBit(axp2101_regs::pmu::PWROFF_EN, 0);
    } else {
        _core.clrRegBit(axp2101_regs::pmu::PWROFF_EN, 0);
    }
}

// ---- DCDC Protection (REG 23-24) ----

void AXP2101Power::setDCOverVoltageProtection(bool enable)
{
    if (enable) {
        _core.setRegBit(axp2101_regs::pmu::DC_OVP_UVP_CTRL, 5);
    } else {
        _core.clrRegBit(axp2101_regs::pmu::DC_OVP_UVP_CTRL, 5);
    }
}

bool AXP2101Power::isDCOverVoltageProtectionEnabled()
{
    return _core.getRegBit(axp2101_regs::pmu::DC_OVP_UVP_CTRL, 5);
}

void AXP2101Power::setDCLowVoltageProtection(uint8_t dc_id, bool enable)
{
    if (dc_id > 4) return;
    if (enable) {
        _core.setRegBit(axp2101_regs::pmu::DC_OVP_UVP_CTRL, dc_id);
    } else {
        _core.clrRegBit(axp2101_regs::pmu::DC_OVP_UVP_CTRL, dc_id);
    }
}

bool AXP2101Power::isDCLowVoltageProtectionEnabled(uint8_t dc_id)
{
    if (dc_id > 4) return false;
    return _core.getRegBit(axp2101_regs::pmu::DC_OVP_UVP_CTRL, dc_id);
}

// ---- Power On/Off Source Status (REG 20-21) ----

uint8_t AXP2101Power::getPowerOnStatus()
{
    int val = _core.readReg(axp2101_regs::pmu::PWRON_STATUS);
    return (val < 0) ? 0 : static_cast<uint8_t>(val);
}

uint8_t AXP2101Power::getPowerOffStatus()
{
    int val = _core.readReg(axp2101_regs::pmu::PWROFF_STATUS);
    return (val < 0) ? 0 : static_cast<uint8_t>(val);
}

// ---- Fuel Gauge ----

bool AXP2101Power::writeGaugeData(uint8_t *data, uint8_t len)
{
    if (len != 128 || !data) return false;
    _core.setRegBit(axp2101_regs::ctrl::RESET_FUEL_GAUGE, 2);
    _core.clrRegBit(axp2101_regs::ctrl::RESET_FUEL_GAUGE, 2);
    _core.clrRegBit(axp2101_regs::gauge::FUEL_GAUGE_CTRL, 0);
    _core.setRegBit(axp2101_regs::gauge::FUEL_GAUGE_CTRL, 0);
    for (int i = 0; i < 128; ++i) {
        _core.writeReg(axp2101_regs::gauge::BAT_PARAMS, data[i]);
    }
    _core.clrRegBit(axp2101_regs::gauge::FUEL_GAUGE_CTRL, 0);
    _core.setRegBit(axp2101_regs::gauge::FUEL_GAUGE_CTRL, 0);
    return compareGaugeData(data, len);
}

bool AXP2101Power::compareGaugeData(uint8_t *data, uint8_t len)
{
    if (len != 128 || !data) return false;
    uint8_t buffer[128];
    memset(buffer, 0, sizeof(buffer));
    _core.clrRegBit(axp2101_regs::gauge::FUEL_GAUGE_CTRL, 0);
    _core.setRegBit(axp2101_regs::gauge::FUEL_GAUGE_CTRL, 0);
    for (int i = 0; i < 128; ++i) {
        buffer[i] = _core.readReg(axp2101_regs::gauge::BAT_PARAMS);
    }
    _core.clrRegBit(axp2101_regs::gauge::FUEL_GAUGE_CTRL, 0);
    _core.setRegBit(axp2101_regs::gauge::FUEL_GAUGE_CTRL, 4);
    _core.setRegBit(axp2101_regs::ctrl::RESET_FUEL_GAUGE, 2);
    _core.clrRegBit(axp2101_regs::ctrl::RESET_FUEL_GAUGE, 2);
    return memcmp(data, buffer, 128) == 0;
}

// ---- Low Battery Thresholds (REG 1A) ----

void AXP2101Power::setLowBatWarnThreshold(uint8_t percentage)
{
    if (percentage < 5 || percentage > 20) return;
    int val = _core.readReg(axp2101_regs::ctrl::LOW_BAT_WARN_SET);
    if (val < 0) return;
    val &= 0x0F;
    val |= ((percentage - 5) << 4);
    _core.writeReg(axp2101_regs::ctrl::LOW_BAT_WARN_SET, val);
}

uint8_t AXP2101Power::getLowBatWarnThreshold()
{
    int val = _core.readReg(axp2101_regs::ctrl::LOW_BAT_WARN_SET);
    if (val < 0) return 0;
    return ((val >> 4) & 0x0F) + 5;
}

void AXP2101Power::setLowBatShutdownThreshold(uint8_t percentage)
{
    if (percentage > 15) percentage = 15;
    int val = _core.readReg(axp2101_regs::ctrl::LOW_BAT_WARN_SET);
    if (val < 0) return;
    val &= 0xF0;
    val |= percentage;
    _core.writeReg(axp2101_regs::ctrl::LOW_BAT_WARN_SET, val);
}

uint8_t AXP2101Power::getLowBatShutdownThreshold()
{
    int val = _core.readReg(axp2101_regs::ctrl::LOW_BAT_WARN_SET);
    if (val < 0) return 0;
    return val & 0x0F;
}

// ---- Die Temperature Protection (REG 13) ----

bool AXP2101Power::enableDieTempDetection(bool enable)
{
    return enable ? _core.setRegBit(axp2101_regs::ctrl::DIE_TEMP_CFG, 0)
                  : _core.clrRegBit(axp2101_regs::ctrl::DIE_TEMP_CFG, 0);
}

bool AXP2101Power::isDieTempDetectionEnabled()
{
    return _core.getRegBit(axp2101_regs::ctrl::DIE_TEMP_CFG, 0);
}

bool AXP2101Power::setDieTempLevel1Threshold(uint8_t opt)
{
    if (opt > 2) return false;
    int val = _core.readReg(axp2101_regs::ctrl::DIE_TEMP_CFG);
    if (val < 0) return false;
    val &= 0xF9;
    val |= (opt << 1);
    return _core.writeReg(axp2101_regs::ctrl::DIE_TEMP_CFG, static_cast<uint8_t>(val)) >= 0;
}

uint8_t AXP2101Power::getDieTempLevel1Threshold()
{
    int val = _core.readReg(axp2101_regs::ctrl::DIE_TEMP_CFG);
    if (val < 0) return 0;
    return (val >> 1) & 0x03;
}

// ---- DCDC PWM Mode Control (REG 81) ----

bool AXP2101Power::setDCDCForcePWM(uint8_t dc_id, bool force)
{
    // DCDC1-4 map to bits 2-5 of REG 81. DCDC5 has no PWM control.
    if (dc_id == 0 || dc_id > 4) return false;
    return force ? _core.setRegBit(axp2101_regs::dcdc::FORCE_PWM_CTRL, dc_id + 1)
                 : _core.clrRegBit(axp2101_regs::dcdc::FORCE_PWM_CTRL, dc_id + 1);
}

bool AXP2101Power::isDCDCForcePWM(uint8_t dc_id)
{
    if (dc_id == 0 || dc_id > 4) return false;
    return _core.getRegBit(axp2101_regs::dcdc::FORCE_PWM_CTRL, dc_id + 1);
}

bool AXP2101Power::setDCDCForceCCM(bool enable)
{
    return enable ? _core.setRegBit(axp2101_regs::dcdc::ONOFF_DVM_CTRL, 6)
                  : _core.clrRegBit(axp2101_regs::dcdc::ONOFF_DVM_CTRL, 6);
}

bool AXP2101Power::isDCDCForceCCM()
{
    return _core.getRegBit(axp2101_regs::dcdc::ONOFF_DVM_CTRL, 6);
}

bool AXP2101Power::setDVMRampSlow(bool slow)
{
    return slow ? _core.setRegBit(axp2101_regs::dcdc::ONOFF_DVM_CTRL, 5)
                : _core.clrRegBit(axp2101_regs::dcdc::ONOFF_DVM_CTRL, 5);
}

bool AXP2101Power::isDVMRampSlow()
{
    return _core.getRegBit(axp2101_regs::dcdc::ONOFF_DVM_CTRL, 5);
}

bool AXP2101Power::setFrequencySpreadEnable(bool enable)
{
    return enable ? _core.setRegBit(axp2101_regs::dcdc::FORCE_PWM_CTRL, 7)
                  : _core.clrRegBit(axp2101_regs::dcdc::FORCE_PWM_CTRL, 7);
}

bool AXP2101Power::isFrequencySpreadEnabled()
{
    return _core.getRegBit(axp2101_regs::dcdc::FORCE_PWM_CTRL, 7);
}

bool AXP2101Power::setFrequencySpreadRange(bool wide)
{
    return wide ? _core.setRegBit(axp2101_regs::dcdc::FORCE_PWM_CTRL, 6)
                : _core.clrRegBit(axp2101_regs::dcdc::FORCE_PWM_CTRL, 6);
}

bool AXP2101Power::getFrequencySpreadRange()
{
    return _core.getRegBit(axp2101_regs::dcdc::FORCE_PWM_CTRL, 6);
}

bool AXP2101Power::setDCDCUVPDebounce(uint8_t opt)
{
    if (opt > 3) return false;
    return _core.updateBits(axp2101_regs::dcdc::FORCE_PWM_CTRL, 0x03, opt) >= 0;
}

uint8_t AXP2101Power::getDCDCUVPDebounce()
{
    int val = _core.readReg(axp2101_regs::dcdc::FORCE_PWM_CTRL);
    if (val < 0) return 0;
    return val & 0x03;
}

// ---- Fast Power-On Sequencing (REG 28-2B) ----

bool AXP2101Power::enableFastPowerOn(bool enable)
{
    return enable ? _core.setRegBit(axp2101_regs::pmu::FAST_PWRON_CTRL, 7)
                  : _core.clrRegBit(axp2101_regs::pmu::FAST_PWRON_CTRL, 7);
}

bool AXP2101Power::isFastPowerOnEnabled()
{
    return _core.getRegBit(axp2101_regs::pmu::FAST_PWRON_CTRL, 7);
}

bool AXP2101Power::enableFastWakeup(bool enable)
{
    return enable ? _core.setRegBit(axp2101_regs::pmu::FAST_PWRON_CTRL, 6)
                  : _core.clrRegBit(axp2101_regs::pmu::FAST_PWRON_CTRL, 6);
}

bool AXP2101Power::isFastWakeupEnabled()
{
    return _core.getRegBit(axp2101_regs::pmu::FAST_PWRON_CTRL, 6);
}

bool AXP2101Power::setFastPowerOnSequence(uint8_t channel, uint8_t seq)
{
    if (channel > 13 || seq > 3) return false;

    // Register and bit mapping for each channel:
    // REG 0x28: DCDC1[1:0], DCDC2[3:2], DCDC3[5:4], DCDC4[7:6]
    // REG 0x29: DCDC5[1:0], ALDO1[3:2], ALDO2[5:4], ALDO3[7:6]
    // REG 0x2A: ALDO4[1:0], BLDO1[3:2], BLDO2[5:4], CPUSLDO[7:6]
    // REG 0x2B: DLDO1[1:0], DLDO2[3:2]
    static const uint8_t regs[] = {
        axp2101_regs::pmu::FAST_PWRON_SET0,  // DCDC1
        axp2101_regs::pmu::FAST_PWRON_SET0,  // DCDC2
        axp2101_regs::pmu::FAST_PWRON_SET0,  // DCDC3
        axp2101_regs::pmu::FAST_PWRON_SET0,  // DCDC4
        axp2101_regs::pmu::FAST_PWRON_SET1,  // DCDC5
        axp2101_regs::pmu::FAST_PWRON_SET1,  // ALDO1
        axp2101_regs::pmu::FAST_PWRON_SET1,  // ALDO2
        axp2101_regs::pmu::FAST_PWRON_SET1,  // ALDO3
        axp2101_regs::pmu::FAST_PWRON_SET2,  // ALDO4
        axp2101_regs::pmu::FAST_PWRON_SET2,  // BLDO1
        axp2101_regs::pmu::FAST_PWRON_SET2,  // BLDO2
        axp2101_regs::pmu::FAST_PWRON_SET2,  // CPUSLDO
        axp2101_regs::pmu::FAST_PWRON_CTRL,  // DLDO1
        axp2101_regs::pmu::FAST_PWRON_CTRL,  // DLDO2
    };
    static const uint8_t bits[] = {
        0, 2, 4, 6,  // DCDC1-4 in SET0
        0, 2, 4, 6,  // DCDC5, ALDO1-3 in SET1
        0, 2, 4, 6,  // ALDO4, BLDO1-2, CPUSLDO in SET2
        0, 2,          // DLDO1-2 in CTRL
    };

    return _core.updateBits(regs[channel], 0x03 << bits[channel], seq << bits[channel]) >= 0;
}

uint8_t AXP2101Power::getFastPowerOnSequence(uint8_t channel)
{
    if (channel > 13) return 3;

    static const uint8_t regs[] = {
        axp2101_regs::pmu::FAST_PWRON_SET0, axp2101_regs::pmu::FAST_PWRON_SET0,
        axp2101_regs::pmu::FAST_PWRON_SET0, axp2101_regs::pmu::FAST_PWRON_SET0,
        axp2101_regs::pmu::FAST_PWRON_SET1, axp2101_regs::pmu::FAST_PWRON_SET1,
        axp2101_regs::pmu::FAST_PWRON_SET1, axp2101_regs::pmu::FAST_PWRON_SET1,
        axp2101_regs::pmu::FAST_PWRON_SET2, axp2101_regs::pmu::FAST_PWRON_SET2,
        axp2101_regs::pmu::FAST_PWRON_SET2, axp2101_regs::pmu::FAST_PWRON_SET2,
        axp2101_regs::pmu::FAST_PWRON_CTRL, axp2101_regs::pmu::FAST_PWRON_CTRL,
    };
    static const uint8_t bits[] = {
        0, 2, 4, 6, 0, 2, 4, 6, 0, 2, 4, 6, 0, 2,
    };

    int val = _core.readReg(regs[channel]);
    if (val < 0) return 3;
    return (val >> bits[channel]) & 0x03;
}

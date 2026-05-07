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
 * @file      AXP202Adc.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-05-02
 *
 * @brief AXP202 ADC wrapper implementation
 *
 * This file contains only the non-core ADC reading methods (configuration,
 * GPIO voltage, battery power). Core ADC reading logic is in the template
 * and AXP202AdcTraits specialization.
 *
 * @note The core ADC reading methods (enableChannels, disableChannels, read)
 *       are handled by the AXP1xxAdc template via AXP202AdcTraits.
 */
#include "AXP202Adc.hpp"
#include "AXP202Regs.hpp"

// ---- ADC Configuration ----
bool AXP202Adc::setADCSampleRate(uint8_t rate)
{
    if (rate > 3) return false;
    return _core.updateBits(axp202_regs::adc::ADC_SPEED, 0xC0, rate << 6) >= 0;
}

uint16_t AXP202Adc::getADCSampleRate()
{
    int val = _core.readReg(axp202_regs::adc::ADC_SPEED);
    if (val < 0) return 0;
    static const uint16_t rates[] = {25, 50, 100, 200};
    return rates[(val >> 6) & 0x03];
}

bool AXP202Adc::setADCInputRange(uint8_t val)
{
    return _core.writeReg(axp202_regs::adc::ADC_INPUTRANGE, val) >= 0;
}

uint8_t AXP202Adc::getADCInputRange()
{
    int val = _core.readReg(axp202_regs::adc::ADC_INPUTRANGE);
    return (val < 0) ? 0 : static_cast<uint8_t>(val);
}

bool AXP202Adc::setADCIrqRisingThreshold(uint8_t val)
{
    return _core.writeReg(axp202_regs::adc::ADC_IRQ_RETFSET, val) >= 0;
}

uint8_t AXP202Adc::getADCIrqRisingThreshold()
{
    int val = _core.readReg(axp202_regs::adc::ADC_IRQ_RETFSET);
    return (val < 0) ? 0 : static_cast<uint8_t>(val);
}

bool AXP202Adc::setADCIrqFallingThreshold(uint8_t val)
{
    return _core.writeReg(axp202_regs::adc::ADC_IRQ_FETFSET, val) >= 0;
}

uint8_t AXP202Adc::getADCIrqFallingThreshold()
{
    int val = _core.readReg(axp202_regs::adc::ADC_IRQ_FETFSET);
    return (val < 0) ? 0 : static_cast<uint8_t>(val);
}

// ---- Additional ADC Channel Reads ----
float AXP202Adc::getTsVoltage()
{
    uint16_t raw = readRegisterH8L4(axp202_regs::adc::TS_IN_H, axp202_regs::adc::TS_IN_L);
    if (raw == 0xFFFF) return 0.0f;
    return static_cast<float>(raw) * axp202_regs::factory::FACTORY_TS_VOLTAGE;
}

// Datasheet REG 64H/65H (GPIO0 ADC, 12-bit H8L4, 0.5mV/step)

float AXP202Adc::getGpio0Voltage()
{
    uint16_t raw = readRegisterH8L4(axp202_regs::adc::GPIO0_ADC_H, axp202_regs::adc::GPIO0_ADC_L);
    if (raw == 0xFFFF) return 0.0f;
    return static_cast<float>(raw) * axp202_regs::factory::FACTORY_GPIO_VOLTAGE;
}

// Datasheet REG 66H/67H (GPIO1 ADC, 12-bit H8L4, 0.5mV/step)

float AXP202Adc::getGpio1Voltage()
{
    uint16_t raw = readRegisterH8L4(axp202_regs::adc::GPIO1_ADC_H, axp202_regs::adc::GPIO1_ADC_L);
    if (raw == 0xFFFF) return 0.0f;
    return static_cast<float>(raw) * axp202_regs::factory::FACTORY_GPIO_VOLTAGE;
}

//  Datasheet REG 70H-72H (Battery power, 24-bit)

float AXP202Adc::getBatteryPower()
{
    uint8_t buf[3];
    if (_core.readRegBuff(axp202_regs::adc::BAT_POWER_H, buf, 3) < 0) return 0.0f;
    uint32_t raw = (static_cast<uint32_t>(buf[0]) << 16) |
                   (static_cast<uint32_t>(buf[1]) << 8) |
                   static_cast<uint32_t>(buf[2]);
    return static_cast<float>(raw) * 2.0f * 1.1f * 0.5f / 1000.0f;
}

// ---- Helper ----

uint16_t AXP202Adc::readRegisterH8L4(uint8_t regH, uint8_t regL)
{
    int h = _core.readReg(regH);
    int l = _core.readReg(regL);
    if (h < 0 || l < 0) {
        return 0xFFFF;
    }
    return (static_cast<uint16_t>(h) << 4) | (static_cast<uint16_t>(l) & 0x0F);
}

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
 * @file      AXP202Adc.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP202 ADC Interface
 *
 * Manages the AXP202's internal 12-bit ADC, which measures battery voltage
 * and current, VBUS voltage and current, APS output voltage, die temperature,
 * TS pin resistance, and GPIO0/GPIO1 analog inputs. Channels are enabled via
 * a bitmask written to registers ADC_EN1 (0x82) and ADC_EN2 (0x83).
 *
 * @note This is a thin wrapper around the template AXP1xxAdc class,
 *       parameterized with AXP202-specific register traits.
 *
 * @see AXP202Regs.hpp, AXP1xxAdc.hpp, AXP202AdcTraits.hpp
 */
#pragma once
#include "AXP202Core.hpp"
#include "../axp1xx/AXP1xxAdc.hpp"
#include "AXP202AdcTraits.hpp"

/**
 * @brief AXP202 ADC channel control and data readout (template wrapper)
 *
 * Instantiates the generic AXP1xxAdc template with AXP202Core and AXP202 traits.
 * Provides the same interface as before while benefiting from reduced code duplication.
 */
class AXP202Adc : public axp1xx::AXP1xxAdc<AXP202Core, axp1xx::AdcTraitsBase<AXP202Core>>
{
public:
    // Hardware bitmask constants (for direct register control)
    static constexpr uint32_t ADC_BAT_VOLTAGE_HW  = 0x80;   // REG82H bit 7
    static constexpr uint32_t ADC_BAT_CURRENT_HW  = 0x40;   // REG82H bit 6
    static constexpr uint32_t ADC_ACIN_VOLTAGE_HW = 0x20;   // REG82H bit 5
    static constexpr uint32_t ADC_ACIN_CURRENT_HW = 0x10;   // REG82H bit 4
    static constexpr uint32_t ADC_VBUS_VOLTAGE_HW = 0x08;   // REG82H bit 3
    static constexpr uint32_t ADC_VBUS_CURRENT_HW = 0x04;   // REG82H bit 2
    static constexpr uint32_t ADC_APS_VOLTAGE_HW  = 0x02;   // REG82H bit 1
    static constexpr uint32_t ADC_TS_PIN_HW       = 0x01;   // REG82H bit 0
    static constexpr uint32_t ADC_TEMPERATURE_HW  = 0x8000; // REG83H bit 7
    static constexpr uint32_t ADC_GPIO0_HW        = 0x0800; // REG83H bit 3
    static constexpr uint32_t ADC_GPIO1_HW        = 0x0400; // REG83H bit 2

    /**
     * @brief Construct AXP202 ADC interface
     * @param core Reference to AXP202 core communication
     */
    explicit AXP202Adc(AXP202Core &core)
        : axp1xx::AXP1xxAdc<AXP202Core, axp1xx::AdcTraitsBase<AXP202Core>>(core) {}

    ~AXP202Adc() = default;

    // ---- ADC Configuration ----
    /**
     * @brief Set ADC sample rate (REG 84H bits 7:6)
     * @param rate 0=25Hz, 1=50Hz, 2=100Hz, 3=200Hz
     * @return true on success
     */
    bool setADCSampleRate(uint8_t rate);

    /**
     * @brief Get ADC sample rate in Hz
     * @return Sample rate: 25, 50, 100, or 200 Hz
     */
    uint16_t getADCSampleRate();

    /**
     * @brief Set ADC input range (REG 85H)
     * @param val Raw register value
     * @return true on success
     */
    bool setADCInputRange(uint8_t val);
    uint8_t getADCInputRange();

    /**
     * @brief Set ADC IRQ rising-edge threshold (REG 86H)
     * @param val Raw register value
     * @return true on success
     */
    bool setADCIrqRisingThreshold(uint8_t val);
    uint8_t getADCIrqRisingThreshold();

    /**
     * @brief Set ADC IRQ falling-edge threshold (REG 87H)
     * @param val Raw register value
     * @return true on success
     */
    bool setADCIrqFallingThreshold(uint8_t val);
    uint8_t getADCIrqFallingThreshold();

    /**
     * @brief Read TS pin voltage in mV (REG 62H/63H, 12-bit H8L4)
     * @return Voltage in mV, or 0 on error
     */
    float getTsVoltage();

    /**
     * @brief Read GPIO0 ADC voltage in mV (REG 64H/65H, 12-bit H8L4)
     * @return Voltage in mV, or 0 on error
     */
    float getGpio0Voltage();

    /**
     * @brief Read GPIO1 ADC voltage in mV (REG 66H/67H, 12-bit H8L4)
     * @return Voltage in mV, or 0 on error
     */
    float getGpio1Voltage();

    /**
     * @brief Read battery instantaneous power (REG 70H-72H, 24-bit)
     * @return Power value (raw * 0.5 * 1.1 mW), or 0 on error
     */
    float getBatteryPower();

private:
    /**
     * @brief Read a register pair where the upper 8 bits are from regH and lower 4 bits from regL
     * @param regH High-byte register address
     * @param regL Low-byte (4-bit) register address
     * @return 12-bit unsigned value
     */
    uint16_t readRegisterH8L4(uint8_t regH, uint8_t regL);
};

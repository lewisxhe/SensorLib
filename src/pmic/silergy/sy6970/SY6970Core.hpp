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
 * @file      SY6970Core.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 * @brief SY6970 Core Communication and Status Interface
 *
 * This class provides low-level I2C communication and direct access to SY6970
 * status and fault registers. It serves as the base class for charger, power,
 * and ADC modules.
 * @see PmicSY6970 for the main PMIC interface
 * @see SY6970Core.cpp for implementation
 * @see SY6970Charger for charging functions
 * @see SY6970Power for power/OTG functions
 * @see SY6970Adc for ADC functions
 * @see SY6970Regs.hpp for register definitions
 */
#pragma once

#include "../../../platform/comm/I2CDeviceNoHal.hpp"

namespace SY6970Faults {
    constexpr uint8_t WATCHDOG_TIMEOUT = 0x80;
    constexpr uint8_t BOOST_FAULT = 0x40;
    constexpr uint8_t CHARGE_FAULT_MASK = 0x30;  // Bits 5-4
    constexpr uint8_t CHARGE_FAULT_NORMAL = 0x00;
    constexpr uint8_t CHARGE_FAULT_INPUT = 0x10;  // Bit 4: BUS OVP or VBAT<BUS<3.8V
    constexpr uint8_t CHARGE_FAULT_THERMAL = 0x20;  // Bit 5: Thermal shutdown
    constexpr uint8_t CHARGE_FAULT_TIMER = 0x30;  // Bits 5+4: Safety timer expiration
    constexpr uint8_t BATTERY_FAULT = 0x08;
    constexpr uint8_t NTC_FAULT_MASK = 0x07; // Bits 2-0

    inline bool isWatchdogTimeout(uint8_t fault) { return (fault & WATCHDOG_TIMEOUT) != 0; }
    inline bool isBoostFault(uint8_t fault) { return (fault & BOOST_FAULT) != 0; }
    inline bool isChargeFault(uint8_t fault) { return (fault & CHARGE_FAULT_MASK) != 0; }
    inline uint8_t getChargeFaultType(uint8_t fault) { return fault & CHARGE_FAULT_MASK; }
    inline bool isChargeInputFault(uint8_t fault) { return (fault & CHARGE_FAULT_MASK) == CHARGE_FAULT_INPUT; }
    inline bool isChargeThermalFault(uint8_t fault) { return (fault & CHARGE_FAULT_MASK) == CHARGE_FAULT_THERMAL; }
    inline bool isChargeTimerFault(uint8_t fault) { return (fault & CHARGE_FAULT_MASK) == CHARGE_FAULT_TIMER; }
    inline bool isBatteryFault(uint8_t fault) { return (fault & BATTERY_FAULT) != 0; }
    inline uint8_t getNtcFault(uint8_t fault) { return fault & NTC_FAULT_MASK; }
}

class SY6970Core : public I2CDeviceNoHal
{
public:
    SY6970Core() = default;
    
    ~SY6970Core() = default;

    /**
     * @brief Deinitialize and release resources
     *
     * Called when shutting down the device. Currently a no-op
     * but may be used for cleanup in derived implementations.
     */
    void end();

    /**
     * @brief Check if device is valid/present
     *
     * Verifies communication by reading the device revision register
     * and comparing against expected value (0x00).
     *
     * @return true if device responds correctly
     *
     * @note This should be called after begin() to verify
     *       the SY6970 is properly connected.
     */
    bool isValid() const;

    /**
     * @brief Get device revision
     *
     * Reads the device revision from REG_DEVICE_REV (0x14).
     * The SY6970 returns 0x00 for the base revision.
     *
     * @return Device revision byte (0x00 for SY6970)
     */
    uint8_t getDeviceRevision() const;

    /**
     * @brief Reset device to default state
     *
     * Performs a register reset by setting the REG_RST bit
     * in REG_DEVICE_REV (0x14). This resets all registers
     * to their default values.
     */
    void reset();

    /**
     * @brief Check if VBUS (USB power) is present
     *
     * Reads the BUS_GD bit from the VBUS ADC register.
     * Returns true when VBUS voltage is above the valid threshold.
     *
     * @return true if VBUS is present and valid
     *
     * @see getVbusVoltage() for actual voltage reading
     */
    bool isVbusPresent();

    /**
     * @brief Get USB port type status
     *
     * Returns the detected USB port type from REG_STATUS (0x0B).
     * This is updated after BC1.2/HVDCP detection.
     *
     * @return Bus state:
     *         - 0: No input
     *         - 1: USB SDP (Standard Downstream Port)
     *         - 2: USB CDP (Charging Downstream Port)
     *         - 3: USB DCP (Dedicated Charging Port)
     *         - 4: HVDCP (High Voltage DCP)
     *         - 5: Adapter
     *         - 6: Non-standard adapter
     *         - 7: OTG mode
     */
    uint8_t getBusStatus();

    /**
     * @brief Get charging status
     *
     * Returns the current charging state from REG_STATUS (0x0B).
     *
     * @return Charge state:
     *         - 0: No charge
     *         - 1: Pre-charge (weak battery)
     *         - 2: Fast charge (CC/CV)
     *         - 3: Charge done
     */
    uint8_t getChargeStatus();

    /**
     * @brief Check if power is good
     *
     * Reads the PG_STAT bit from REG_STATUS (0x0B).
     * Indicates valid input power (VBUS > VBUS_VALID).
     *
     * @return true if power good
     */
    bool isPowerGood();

    /**
     * @brief Check if actively charging
     *
     * Returns true when charge status is not NO_CHARGE.
     * This includes both pre-charge and fast-charge phases.
     *
     * @return true if charging is in progress
     *
     * @note Use isChargeDone() to check for charge complete.
     */
    bool isCharging();

    /**
     * @brief Check if charging is complete
     *
     * Returns true when charge status is DONE (3).
     * Indicates the battery has reached full charge.
     *
     * @return true if charge cycle complete
     */
    bool isChargeDone();

    /**
     * @brief Check if in OTG (boost) mode
     *
     * Returns true when bus status is OTG (7).
     * Indicates the SY6970 is supplying power to VBUS.
     *
     * @return true if OTG mode active
     */
    bool isOtg();

    /**
     * @brief Get fault status
     *
     * Reads the fault register REG_FAULT (0x0C) and returns
     * the raw fault byte for detailed fault analysis.
     *
     * @param fault Reference to store fault byte
     * @return true on success, false on I2C error
     *
     * @section Fault Bits
     * - Bit 7: Watchdog timeout fault
     * - Bit 6: Boost fault
     * - Bits 5-4: Charge fault (00: normal, 01: input fault, 10: thermal, 11: timer fault)
     * - Bit 3: Battery fault
     * - Bits 2-0: NTC fault
     */
    bool getFaultStatus(uint8_t &fault);

private:
    bool initImpl(uint8_t param) override;
};

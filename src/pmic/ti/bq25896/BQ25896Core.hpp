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
 * @file      BQ25896Core.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 * @brief BQ25896 Core Communication and Status Interface
 *
 * This class provides low-level I2C communication and direct access to BQ25896
 * status and fault registers. It serves as the base class for charger, power,
 * and ADC modules.
 *
 * @section Device Overview
 * The BQ25896 is a USB BC1.2 compliant switch-mode charger IC featuring:
 * - High-efficiency buck charging (up to 3A)
 * - USB BC1.2 and HVDCP detection
 * - Input Current Optimizer (ICO)
 * - Integrated 10-bit ADC
 * - OTG (On-The-Go) boost mode for power bank operation
 * - JEITA battery temperature protection
 *
 * @section I2C Communication
 * - I2C Address: 0x6B (7-bit)
 * - Supports standard I2C read/write operations
 * - 8-bit register addressing
 *
 * @section Status Monitoring
 * The core provides access to:
 * - VBUS presence detection
 * - Charging state (pre-charge, fast-charge, done)
 * - USB type detection (SDP, Adapter, OTG)
 * - Power good status
 * - VINDPM/IINDPM status
 * - Fault conditions (watchdog, boost, battery, NTC)
 *
 * @see BQ25896Core.cpp for implementation
 * @see BQ25896Charger for charging functions
 * @see BQ25896Power for power/OTG functions
 * @see BQ25896Adc for ADC functions
 * @see BQ25896Regs.hpp for register definitions
 * @see BQ25896RegisterMap.md for detailed register descriptions
 */
#pragma once

#include "../../../platform/comm/I2CDeviceNoHal.hpp"
#include "BQ25896Regs.hpp"

namespace BQ25896Faults {
    constexpr uint8_t WATCHDOG_TIMEOUT = 0x80;
    constexpr uint8_t BOOST_FAULT = 0x40;
    constexpr uint8_t CHARGE_FAULT_MASK = 0x30;
    constexpr uint8_t CHARGE_FAULT_NORMAL = 0x00;
    constexpr uint8_t CHARGE_FAULT_INPUT = 0x10;  // Bit 4: BUS OVP or VBAT<BUS<3.8V
    constexpr uint8_t CHARGE_FAULT_THERMAL = 0x20;  // Bit 5: Thermal shutdown
    constexpr uint8_t CHARGE_FAULT_TIMER = 0x30;  // Bits 5+4: Safety timer expiration
    constexpr uint8_t BATTERY_FAULT = 0x08;
    constexpr uint8_t NTC_FAULT_MASK = 0x07;

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

class BQ25896Core : public I2CDeviceNoHal
{
public:
    BQ25896Core() = default;
    ~BQ25896Core() = default;

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
     * and comparing against expected value (0x02).
     *
     * @return true if device responds correctly with expected revision
     *
     * @note This should be called after begin() to verify
     *       the BQ25896 is properly connected.
     */
    bool isValid() const;

    /**
     * @brief Get device revision
     *
     * Reads the device revision from REG_DEVICE_REV (0x14).
     * The BQ25896 returns 0x02 for the revision.
     *
     * @return Device revision byte (0x02 for BQ25896)
     */
    uint8_t getDeviceRevision() const;

    /**
     * @brief Get device configuration/part number
     *
     * Reads the PN bits from REG_DEVICE_REV (0x14).
     *
     * @return Part number configuration (0x00 for BQ25896)
     */
    uint8_t getDeviceConfig();

    /**
     * @brief Reset device to default state
     *
     * Performs a register reset by setting the REG_RST bit
     * in REG_DEVICE_REV (0x14). This resets all registers
     * to their default values and resets the safety timer.
     */
    void reset();

    /**
     * @brief Check if VBUS (USB power) is present
     *
     * Reads the VBUS_GD bit from the VBUS ADC register (REG11).
     * Returns true when VBUS voltage is above the valid threshold (≥3.8V).
     *
     * @return true if VBUS is present and valid
     *
     * @note Returns false during boost (OTG) mode since VBUS is output, not input.
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
     *         - 1: USB SDP (Standard Downstream Port, 500mA)
     *         - 2: Adapter (3.25A)
     *         - 7: OTG mode
     *
     * @note CDP (Charging Downstream Port) is detected as SDP or adapter
     *       depending on DCP detection results.
     */
    uint8_t getBusStatus();

    /**
     * @brief Get charging status
     *
     * Returns the current charging state from REG_STATUS (0x0B).
     *
     * @return Charge state:
     *         - 0: Not Charging
     *         - 1: Pre-charge (battery < BATLOWV threshold)
     *         - 2: Fast Charging (CC/CV)
     *         - 3: Charge Termination Done
     */
    uint8_t getChargeStatus();

    /**
     * @brief Check if power is good
     *
     * Reads the PG_STAT bit from REG_STATUS (0x0B).
     * Indicates valid input power (VBUS > VBUS_VALID and within spec).
     *
     * @return true if power good (VBUS valid and within specifications)
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
     * Indicates the battery has reached full charge and terminated.
     *
     * @return true if charge cycle complete
     */
    bool isChargeDone();

    /**
     * @brief Check if in OTG (boost) mode
     *
     * Returns true when bus status is OTG (7).
     * Indicates the BQ25896 is supplying power to VBUS (5V/1A-2A).
     *
     * @return true if OTG mode active (VBUS is output)
     */
    bool isOtg();

    /**
     * @brief Check if VSYS is in regulation
     *
     * Reads the VSYS_STAT bit from REG_STATUS (0x0B).
     * Returns true when system voltage is being regulated to SYS_MIN.
     * This happens when battery voltage drops below the minimum system voltage.
     *
     * @return true if VSYS is in minimum voltage regulation
     *
     * @note This typically occurs during high load or low battery conditions.
     */
    bool isVsysInRegulation();

    /**
     * @brief Get fault status
     *
     * Reads the VDPM_STAT bit from REG13 (0x13).
     * Returns true when input voltage has dropped to the VINDPM threshold.
     *
     * @return true if VINDPM is active (input voltage limited)
     * @return false if input voltage is above VINDPM threshold
     *
     * @note This indicates the adapter is being loaded heavily and the charger
     *       is limiting input current to prevent the adapter from collapsing
     * @see setInputVoltageLimit() in BQ25896Power to configure VINDPM threshold
     */
    bool isVindpmActive();

    /**
     * @brief Check if IINDPM is active (input current limiting)
     *
     * Reads the IDPM_STAT bit from REG13 (0x13).
     * Returns true when input current has reached the IINDPM limit.
     *
     * @return true if IINDPM is active (input current limited)
     * @return false if input current is below IINDPM threshold
     *
     * @note This indicates the charger is drawing maximum allowed current from adapter
     * @see setInputCurrentLimit() in BQ25896Power to configure IINDPM threshold
     */
    bool isIindpmActive();

    /**
     * @brief Get fault status
     *
     * Reads the fault register REG0C (0x0C) and returns
     * the raw fault byte for detailed fault analysis.
     *
     * @param fault Reference to store fault byte
     * @return true on success, false on I2C error
     *
     * @section Fault Bits
     * - Bit 7 (WATCHDOG_FAULT): Watchdog timer expired
     * - Bit 6 (BOOST_FAULT): Boost mode fault - VBUS overloaded, OVP, or battery too low
     * - Bits 5-4 (CHRG_FAULT): Charge fault
     *   - 00: Normal
     *   - 01: Input fault (VBUS OVP or VBAT < VBUS < 3.8V)
     *   - 10: Thermal shutdown
     *   - 11: Charge safety timer expiration
     * - Bit 3 (BAT_FAULT): Battery over-voltage protection triggered
     * - Bits 2-0 (NTC_FAULT): NTC temperature fault
     *   - Buck Mode: 000=Normal, 010=Warm, 011=Cool, 101=Cold, 110=Hot
     *   - Boost Mode: 000=Normal, 101=Cold, 110=Hot
     */
    bool getFaultStatus(uint8_t &fault);

    /**
     * @brief Check if Input Current Optimizer has optimized
     *
     * Reads the ICO_OPTIMIZED bit from REG_DEVICE_REV (0x14).
     * Returns true when ICO has determined the maximum input current.
     *
     * @return true if ICO has completed optimization (maximum input current detected)
     * @return false if ICO is still in progress or disabled
     *
     * @note ICO (Input Current Optimizer) automatically finds the maximum
     *       input current the adapter can provide without collapsing.
     * @note This is a status flag, not a control. ICO runs automatically if enabled.
     * @see ICO_EN bit in REG02 to enable/disable ICO
     */
    bool isInputCurrentOptimizerOptimized();


    bool disableWatchdog();

private:
    bool initImpl(uint8_t param) override;
};

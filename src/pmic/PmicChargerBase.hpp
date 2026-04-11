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
 * @file      PmicChargerBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 * @brief PMIC Charger Base Interface
 *
 * This abstract class defines the standard interface for PMIC battery charger control.
 * It provides a unified API for common charging operations across different PMIC chips.
 *
 * @section Charger Overview
 * Battery chargers typically operate in several phases:
 * - Pre-charge: Low current charging for deeply discharged batteries
 * - Fast Charge (CC/CV): High current charging until voltage threshold
 * - Termination: Detection of full charge and charging stop
 *
 * @section Units
 * All voltage and current parameters use standard units:
 * - Voltage: millivolts (mV)
 * - Current: milliamps (mA)
 *
 * @section Implementation Notes
 * - Some PMICs only support discrete voltage/current steps; implementations
 *   should clamp/round as needed but keep the API in real units.
 * - If a feature is not supported, implementations should return false.
 *
 */
#pragma once

#include <stdint.h>

/**
 * @brief PMIC charger capability interface.
 */
class PmicChargerBase
{
public:
    /**
     * @brief Charging status enumeration
     *
     * Represents the current charging state of the battery charger.
     */
    enum class ChargingStatus : uint8_t {
        NO_CHARGING = 0,   ///< Not charging, battery idle or disconnected
        PRE_CHARGE = 1,    ///< Pre-charge phase (low current for weak battery)
        FAST_CHARGE = 2,   ///< Fast charge phase (constant current)
        TERMINATION = 3,  ///< Charging terminated (battery full)
        UNKNOWN = 0xFF    ///< Unable to determine status
    };

    /**
     * @brief Virtual destructor
     */
    virtual ~PmicChargerBase() = default;

    /**
     * @brief Enable or disable charging
     * @param enable true to enable charging, false to disable
     * @return true on success, false if not supported or I2C error
     *
     * @note When disabled, charging is stopped immediately
     * @see isCharging()
     */
    virtual bool enableCharging(bool enable) = 0;

    /**
     * @brief Check whether charger is currently charging
     * @return true if charging is in progress, false otherwise
     *
     * @note Returns false in all states except PRE_CHARGE and FAST_CHARGE
     * @see getStatus()
     */
    virtual bool isCharging() = 0;

    /**
     * @brief Set pre-charge current
     *
     * Pre-charge is the initial charging phase for deeply discharged batteries.
     * The battery is charged at a lower current until it reaches the pre-charge
     * voltage threshold, then transitions to fast charge.
     *
     * @param mA Pre-charge current in milliamps (chip-specific range)
     * @return true on success, false if not supported or I2C error
     *
     * @see getPreChargeCurrent()
     */
    virtual bool setPreChargeCurrent(uint16_t mA) = 0;

    /**
     * @brief Get pre-charge current setting
     * @return Pre-charge current in milliamps
     *
     * @see setPreChargeCurrent()
     */
    virtual uint16_t getPreChargeCurrent() = 0;

    /**
     * @brief Set fast charge (constant-current) current
     *
     * Fast charge is the main charging phase where the battery is charged
     * at high constant current until it reaches the charge voltage threshold.
     *
     * @param mA Fast charge current in milliamps (chip-specific range)
     * @return true on success, false if not supported or I2C error
     *
     * @note Typical range: 0-3008mA or higher depending on PMIC
     * @see getFastChargeCurrent()
     */
    virtual bool setFastChargeCurrent(uint16_t mA) = 0;

    /**
     * @brief Get fast charge current setting
     * @return Fast charge current in milliamps
     *
     * @see setFastChargeCurrent()
     */
    virtual uint16_t getFastChargeCurrent() = 0;

    /**
     * @brief Set termination current
     *
     * When the charge current drops below this threshold during the constant-voltage
     * phase, charging is terminated (battery considered full).
     *
     * @param mA Termination current in milliamps (chip-specific range)
     * @return true on success, false if not supported or I2C error
     */
    virtual bool setTerminationCurrent(uint16_t mA) = 0;

    /**
     * @brief Get termination current setting
     * @return Termination current in milliamps
     *
     * @see setTerminationCurrent()
     */
    virtual uint16_t getTerminationCurrent() = 0;

    /**
     * @brief Set charge voltage (CV mode voltage)
     *
     * Sets the constant voltage level during the CV (constant-voltage) phase
     * of charging. Once the battery reaches this voltage, current decreases
     * until termination current is reached.
     *
     * @param mV Charge voltage in millivolts (chip-specific range)
     * @return true on success, false if not supported or I2C error
     */
    virtual bool setChargeVoltage(uint16_t mV) = 0;

    /**
     * @brief Get charge voltage setting
     * @return Charge voltage in millivolts
     *
     * @see setChargeVoltage()
     */
    virtual uint16_t getChargeVoltage() = 0;

    /**
     * @brief Charger status structure
     *
     * This is intentionally minimal; different PMICs expose richer status sets.
     * Implementations can fill what they can and leave others at default.
     */
    struct Status {
        bool online = false;                      ///< PMIC reachable / initialized
        bool vbusPresent = false;                 ///< VBUS detected / good (USB power present)
        bool batteryPresent = false;              ///< Battery present and detected
        bool charging = false;                    ///< Charging state (coarse: true if in precharge/fastcharge)
        bool chargeDone = false;                  ///< Charge termination/done (battery full)
        bool fault = false;                       ///< Any fault latched/active
        uint32_t faultCode = 0;                   ///< Optional raw fault code for diagnostics
        ChargingStatus chargingStatus = ChargingStatus::NO_CHARGING; ///< Charging status enumeration
    };

    /**
     * @brief Get current charger status
     * @return Status structure with current charger state
     *
     * @note Different PMICs may not support all fields; unsupported fields will be default values
     */
    virtual Status getStatus() = 0;
};

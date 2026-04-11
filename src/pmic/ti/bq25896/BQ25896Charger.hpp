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
 * @file      BQ25896Charger.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 * @brief BQ25896 Charger Control Interface
 *
 * This class provides control over charging functions including:
 * - Enable/disable charging
 * - Pre-charge current setting (for weak battery recovery)
 * - Fast charge current setting (main charging phase)
 * - Termination current setting (charge done detection)
 * - Charge voltage setting (constant voltage phase)
 *
 * @section Charge Phases
 * The BQ25896 follows a multi-phase charging algorithm:
 * 1. \b Pre-charge: When battery voltage is below BATLOWV (default 3.0V),
 *    charger applies pre-charge current (default 128mA) to safely
 *    recover a deeply discharged battery.
 * 2. \b Fast Charge (CC/CV): Once battery reaches BATLOWV threshold,
 *    constant current is applied (configurable 0-3008mA). When battery
 *    voltage reaches the set charge voltage, it switches to constant
 *    voltage mode to prevent overcharging.
 * 3. \b Termination: When charge current drops below the termination
 *    threshold, charging is complete.
 *
 * @section Quantization
 * @note All setter methods will automatically quantize input values
 *       to the nearest valid step if they do not match exactly.
 *       - Pre-charge current: 64mA steps (64-1024mA)
 *       - Fast charge current: 64mA steps (0-3008mA, 0 disables charge)
 *       - Termination current: 64mA steps (64-1024mA)
 *       - Charge voltage: 16mV steps (3840-4608mV)
 *
 * @see BQ25896Charger.cpp for implementation
 * @see BQ25896Regs.hpp for register definitions and parameter ranges
 * @see BQ25896RegisterMap.md for detailed register descriptions
 */
#pragma once

#include "../../PmicChargerBase.hpp"
#include "BQ25896Core.hpp"

class BQ25896Charger : public PmicChargerBase
{
public:
    /**
     * @brief Construct charger interface
     * @param core Reference to BQ25896 core communication
     */
    explicit BQ25896Charger(BQ25896Core &core);

    ~BQ25896Charger() = default;

    /**
     * @brief Enable or disable charging
     *
     * Enables or disables the charging function. When disabled,
     * the charger stops all charging operations immediately.
     *
     * @param enable true to enable charging, false to disable
     * @return true on success, false on I2C error
     *
     * @note Even when enabled, charging may not start if:
     *       - No valid input power (VBUS not present)
     *       - Battery voltage is out of range
     *       - A fault condition exists (thermal, NTC, etc.)
     *
     * @see isCharging() to check if charging is actually happening
     * @see getStatus() for detailed charger state
     */
    bool enableCharging(bool enable) override;

    /**
     * @brief Check if charging is in progress
     *
     * Returns the current charging state. This is a best-effort
     * indication and may not always reflect the exact charger state.
     *
     * @return true if charging is in progress (pre-charge or fast charge)
     * @return false if not charging (no charge, done, or disabled)
     *
     * @note Use getStatus() for more detailed state information
     */
    bool isCharging() override;

    /**
     * @brief Set pre-charge current
     *
     * Sets the current used during the pre-charge phase when battery
     * voltage is below the BATLOWV threshold (default 3.0V).
     *
     * @param mA Pre-charge current in milliamps (64-1024mA)
     *        Value will be quantized to nearest valid step (64mA).
     *        - Input 0-32mA → 64mA
     *        - Input 33-96mA → 64mA
     *        - Input 97-160mA → 128mA
     *        - etc.
     * @return true on success, false on I2C error
     *
     * @note Formula: IPRECHG = 64mA + [IPRECHG] * 64mA
     * @note Default: 128mA (IPRECHG = 0001)
     *
     * @see getPreChargeCurrent()
     * @see setFastChargeCurrent()
     */
    bool setPreChargeCurrent(uint16_t mA) override;

    /**
     * @brief Get pre-charge current
     *
     * Reads the configured pre-charge current from hardware.
     *
     * @return Pre-charge current in milliamps (64-1024mA)
     *
     * @see setPreChargeCurrent()
     */
    uint16_t getPreChargeCurrent() override;

    /**
     * @brief Set fast charge (constant-current) current
     *
     * Sets the maximum charging current during the fast charge phase.
     * This is the main charging current applied when battery voltage
     * is above the BATLOWV threshold.
     *
     * @param mA Fast charge current in milliamps (0-3008mA)
     *        Value will be quantized to nearest valid step (64mA).
     *        - Input 0-32mA → 0 (charge disabled)
     *        - Input 33-96mA → 64mA
     *        - Input 1984-2016mA → 1984mA
     *        - Input >3008mA → 3008mA (clamped)
     * @return true on success, false on I2C error
     *
     * @note Formula: ICHG = [ICHG] * 64mA
     * @note Setting 0 disables charging
     * @note Default: 2048mA (ICHG = 0010_0000)
     *
     * @warning Setting high charge current requires adequate thermal
     *          management. The device will reduce current if thermal
     *          regulation activates.
     *
     * @see getFastChargeCurrent()
     * @see setChargeVoltage()
     */
    bool setFastChargeCurrent(uint16_t mA) override;

    /**
     * @brief Get fast charge current
     *
     * Reads the configured fast charge current from hardware.
     *
     * @return Fast charge current in milliamps (0-3008mA)
     * @return 0 if charging is disabled
     *
     * @see setFastChargeCurrent()
     */
    uint16_t getFastChargeCurrent() override;

    /**
     * @brief Set termination current
     *
     * Sets the current threshold used to detect charge termination.
     * When charge current drops below this value, charging is considered
     * complete.
     *
     * @param mA Termination current in milliamps (64-1024mA)
     *        Value will be quantized to nearest valid step (64mA).
     * @return true on success, false on I2C error
     *
     * @note Formula: ITERM = 64mA + [ITERM] * 64mA
     * @note Default: 256mA (ITERM = 0011)
     * @note Termination detection requires EN_TERM to be enabled (default)
     *
     * @see getTerminationCurrent()
     * @see isChargeDone()
     */
    bool setTerminationCurrent(uint16_t mA) override;

    /**
     * @brief Get termination current
     *
     * Reads the configured termination current from hardware.
     *
     * @return Termination current in milliamps (64-1024mA)
     *
     * @see setTerminationCurrent()
     */
    uint16_t getTerminationCurrent() override;

    /**
     * @brief Set charge voltage (constant voltage phase)
     *
     * Sets the target voltage during the constant voltage (CV) phase
     * of charging. The charger maintains this voltage while the current
     * gradually decreases.
     *
     * @param mV Charge voltage in millivolts (3840-4608mV)
     *        Value will be quantized to nearest valid step (16mV).
     *        - Input 3840-3848mV → 3840mV
     *        - Input 4200-4216mV → 4192mV
     *        - Input >4600mV → 4608mV (clamped)
     * @return true on success, false on I2C error
     *
     * @note Formula: VREG = 3.840V + [VREG] * 16mV
     * @note Default: 4.208V (VREG = 010111)
     * @note Common values: 4.2V (4200mV), 4.35V (4352mV), 4.4V (4416mV)
     *
     * @warning Do not exceed battery manufacturer's maximum voltage rating!
     *
     * @see getChargeVoltage()
     */
    bool setChargeVoltage(uint16_t mV) override;

    /**
     * @brief Get charge voltage
     *
     * Reads the configured charge voltage from hardware.
     *
     * @return Charge voltage in millivolts (3840-4608mV)
     *
     * @see setChargeVoltage()
     */
    uint16_t getChargeVoltage() override;

    /**
     * @brief Get charger status
     *
     * Returns a comprehensive status structure containing all
     * relevant charger state information.
     *
     * @return Status structure containing:
     *         - online: Device is responding
     *         - vbusPresent: USB power connected
     *         - charging: Currently charging
     *         - chargeDone: Charge cycle complete
     *         - chargingStatus: NO_CHARGING/PRE_CHARGE/FAST_CHARGE/TERMINATION
     *         - fault: Any fault condition present
     *         - faultCode: Raw fault register value for detailed analysis
     *
     * @note The faultCode can be decoded using IRQ_* macros in BQ25896Regs.hpp
     */
    Status getStatus() override;

private:
    BQ25896Core &_core;
    bool _userDisableCharge = false;
};

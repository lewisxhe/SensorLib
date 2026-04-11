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
 * @file      SY6970Power.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 * @brief SY6970 Power Management Interface
 *
 * This class provides control over power management functions including:
 * - System minimum voltage
 * - Input voltage/current limits (VINDPM)
 * - OTG (On-The-Go) boost mode
 * - Boost output voltage
 *
 * @section Power Management Features
 * - VINDPM (Voltage Input Dynamic Power Management): Protects input source
 *   by limiting power draw based on adapter capability
 * - AICL (Auto Input Current Limit): Automatically adjusts input current
 *   based on detected adapter limits
 * - OTG/Boost Mode: Converts battery power to VBUS for powering USB devices
 *
 * @section Voltage Parameters
 * - System minimum: 3000-3700mV (100mV steps)
 * - Input voltage limit (VINDPM): 3900-15300mV (100mV steps)
 * - Input current limit: 100-3250mA (50mA steps)
 * - Boost voltage: 4550-5510mV (64mV steps)
 *
 * @note All setter methods with step requirements will automatically quantize
 *       input values to the nearest valid step if they do not match exactly.
 *
 * @see SY6970Power.cpp for implementation details
 * @see SY6970Regs.hpp for register and step value definitions
 */
#pragma once

#include "SY6970Core.hpp"
#include "../../PmicPowerBase.hpp"
#include "../../PmicWatchdogBase.hpp"

class SY6970Power : public PmicPowerBase, public PmicWatchdogBase
{
public:
    /**
     * @brief Construct power management interface
     * @param core Reference to SY6970 core communication
     */
    explicit SY6970Power(SY6970Core &core);

    ~SY6970Power() = default;

    /**
     * @brief Set minimum system voltage
     *
     * Sets the minimum system voltage threshold. The system voltage (VSYS)
     * is prevented from dropping below this value during load transients.
     * Value is quantized to nearest 100mV step.
     *
     * @param mv Minimum system voltage in millivolts (3000-3700mV)
     * @return true on success
     *
     * @note Values not matching 100mV step will be quantized to nearest valid value.
     * @see getMinimumSystemVoltage()
     */
    bool setMinimumSystemVoltage(uint32_t mv) override;

    /**
     * @brief Get minimum system voltage
     * @return Minimum system voltage in millivolts
     */
    uint32_t getMinimumSystemVoltage() const override;

    /**
     * @brief Set input voltage limit (VINDPM)
     *
     * Sets the VINDPM threshold - the minimum voltage the adapter/input
     * is allowed to be pulled down to. This protects weak adapters
     * from being overloaded.
     * Value is quantized to nearest 100mV step.
     *
     * @param mv Input voltage limit in millivolts (3900-15300mV)
     * @return true on success
     *
     * @note Values not matching 100mV step will be quantized to nearest valid value.
     * @see getInputVoltageLimit()
     */
    bool setInputVoltageLimit(uint32_t mv) override;

    /**
     * @brief Get input voltage limit
     * @return Input voltage limit in millivolts
     */
    uint32_t getInputVoltageLimit() const override;

    /**
     * @brief Set input current limit
     *
     * Sets the maximum input current drawn from VBUS.
     * Value is quantized to nearest 50mA step.
     *
     * @param mA Input current limit in milliamps (100-3250mA)
     * @return true on success
     *
     * @note Values not matching 50mA step will be quantized to nearest valid value.
     * @see getInputCurrentLimit()
     */
    bool setInputCurrentLimit(uint32_t mA) override;

    /**
     * @brief Get input current limit
     * @return Input current limit in milliamps
     */
    uint32_t getInputCurrentLimit() const override;

    /**
     * @brief Enable/disable OTG (boost) mode
     *
     * Enables or disables OTG (On-The-Go) boost mode. When enabled,
     * the SY6970 converts battery power to 5V/2A on VBUS to power
     * external USB devices.
     *
     * @param enable true to enable OTG, false to disable
     * @return true on success
     *
     * @note OTG cannot be enabled while VBUS is present.
     * @see isBoostEnabled()
     * @see setBoostVoltage()
     */
    bool enableBoost(bool enable) override;

    /**
     * @brief Check if boost mode is enabled
     * @return true if OTG mode is active
     */
    bool isBoostEnabled() const override;

    /**
     * @brief Set boost output voltage
     *
     * Sets the output voltage when in OTG boost mode.
     * Value is quantized to nearest 64mV step.
     *
     * @param mv Boost voltage in millivolts (4550-5510mV)
     * @return true on success
     *
     * @note Values not matching 64mV step will be quantized to nearest valid value.
     * @note Must have boost enabled first.
     * @see getBoostVoltage()
     */
    bool setBoostVoltage(uint16_t mv) override;

    /**
     * @brief Get boost output voltage
     * @return Boost voltage in millivolts
     */
    uint16_t getBoostVoltage() const override;

    /**
     * @brief Enable or disable ship mode
     *
     * Ship mode disconnects the battery from the system by turning off
     * the BATFET. This is used for:
     * - Shipping mode (device stored for long periods)
     * - Complete power off when no external power is available
     *
     * When ship mode is enabled and no VBUS is present, the system is
     * completely powered down (only the charger IC remains in low-power mode).
     *
     * @param enable true to enter ship mode (battery disconnected), false to exit
     * @return true on success, false on I2C error
     *
     * @note To exit ship mode, connect VBUS. The BATFET will automatically
     *       turn on when valid VBUS is detected.
     * @note This function writes to REG09 BATFET_DIS bit.
     *
     * @see isShipModeEnabled()
     */
    bool enableShipMode(bool enable) override;

    /**
     * @brief Check if ship mode is enabled
     *
     * Reads the BATFET_DIS bit from REG09 to determine if ship mode is active.
     *
     * @return true if ship mode is active (battery disconnected)
     * @return false if ship mode is disabled (battery connected)
     *
     * @see enableShipMode()
     */
    bool isShipModeEnabled() const override;

    /**
     * @brief Enable or disable the I2C watchdog timer
     *
     * When enabled, the watchdog monitors I2C communication. If no I2C access
     * occurs within the timeout period, the PMIC will reset and restore
     * default register values.
     *
     * @param enable true to enable watchdog (default 40s timeout), false to disable
     * @return true on success, false on I2C error
     *
     * @note When watchdog expires, register settings are reset to defaults.
     *       Call resetWatchdog() regularly to prevent this.
     * @note This function controls REG07 WATCHDOG[1:0] bits.
     *
     * @see setWatchdogTimeout()
     * @see resetWatchdog()
     * @see isWatchdogEnabled()
     */
    bool enableWatchdog(bool enable) override;

    /**
     * @brief Check if watchdog is enabled
     * @return true if watchdog is enabled, false if disabled
     * @see enableWatchdog()
     */
    bool isWatchdogEnabled() const override;

    /**
     * @brief Set the watchdog timeout duration
     *
     * Sets how long the watchdog will wait before triggering a fault.
     * Must be called after enableWatchdog(true).
     *
     * @param timeout_s Timeout in seconds (0=disabled, values >0 enable watchdog)
     *        Valid chip values: 0, 40, 80, 160 seconds. Other values will be clamped.
     * @return true on success, false on I2C error
     *
     * @note Default timeout after enableWatchdog(true) is 40 seconds.
     * @see enableWatchdog()
     * @see getWatchdogTimeout()
     */
    bool setWatchdogTimeout(uint16_t timeout_s) override;

    /**
     * @brief Get the current watchdog timeout setting
     * @return Timeout in seconds (0 if disabled, or 40/80/160)
     * @see setWatchdogTimeout()
     */
    uint16_t getWatchdogTimeout() const override;

    /**
     * @brief Reset the watchdog timer (feed the watchdog)
     *
     * This must be called regularly before the watchdog timeout expires
     * to prevent the PMIC from resetting due to I2C communication timeout.
     *
     * @return true on success, false on I2C error
     *
     * @note Should be called at least once before the watchdog timeout.
     *       Recommended to call every 20-30 seconds for 40s timeout.
     * @note This function writes to REG03 WD_RST bit (self-clearing).
     *
     * @see enableWatchdog()
     * @see setWatchdogTimeout()
     */
    bool resetWatchdog() override;

private:
    SY6970Core &_core;
};

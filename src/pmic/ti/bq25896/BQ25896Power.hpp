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
 * @file      BQ25896Power.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 * @brief BQ25896 Power Management Interface
 *
 * This class provides control over power management functions including:
 * - System minimum voltage (VSYS minimum during load transients)
 * - Input voltage limit (VINDPM) - protects weak adapters
 * - Input current limit (IINDPM) - prevents adapter overloading
 * - OTG/Boost mode - supplies power to external USB devices
 *
 * @section Power Management Features
 *
 * \b VINDPM (Voltage Input Dynamic Power Management)
 * - Protects the input source (USB adapter) from being overloaded
 * - When VBUS voltage drops to VINDPM threshold due to high load,
 *   the charger reduces input current to prevent adapter collapse
 * - Can operate in relative mode (offset from VBUS no-load) or
 *   absolute mode (fixed voltage threshold)
 *
 * \b IINDPM (Input Current Dynamic Power Management)
 * - Limits the current drawn from the input source
 * - Prevents drawing more current than the adapter can provide
 * - The ICO (Input Current Optimizer) feature can automatically
 *   determine the maximum safe input current
 *
 * \b OTG/Boost Mode
 * - Converts battery power to 5V on VBUS to power external devices
 * - Useful for power bank applications
 * - Requires no VBUS input (must be battery powered)
 * - Can provide up to 2.15A depending on configuration
 *
 * @section Voltage Parameters
 * - System minimum (VSYS): 3000-3700mV (100mV steps)
 * - Input voltage limit (VINDPM): 3900-15300mV (100mV steps)
 * - Input current limit: 100-3250mA (50mA steps)
 * - Boost voltage: 4550-5510mV (64mV steps)
 *
 * @section Quantization
 * @note All setter methods will automatically quantize input values
 *       to the nearest valid step if they do not match exactly.
 *
 * @section Boost Mode Usage Warning
 * @warning When enabling boost mode, ensure:
 *          1. No USB cable is connected to VBUS (input)
 *          2. Battery voltage is above minimum threshold (2.5V default)
 *          3. Charger is disabled or in proper state
 *          Enabling boost while VBUS is present will fail and return false.
 *
 * @see BQ25896Power.cpp for implementation
 * @see BQ25896Regs.hpp for register definitions and parameter ranges
 * @see BQ25896RegisterMap.md for detailed register descriptions
 */
#pragma once

#include "BQ25896Core.hpp"
#include "../../PmicPowerBase.hpp"
#include "../../PmicWatchdogBase.hpp"

class BQ25896Power : public PmicPowerBase, public PmicWatchdogBase
{
public:
    /**
     * @brief Construct power management interface
     * @param core Reference to BQ25896 core communication
     */
    explicit BQ25896Power(BQ25896Core &core);

    ~BQ25896Power() = default;

    /**
     * @brief Set minimum system voltage
     *
     * Sets the minimum VSYS voltage threshold. VSYS is the charger
     * output that powers the system load. During high load or low
     * battery conditions, VSYS may drop but will not go below this value.
     *
     * @param mv Minimum system voltage in millivolts (3000-3700mV)
     *        Value will be quantized to nearest valid step (100mV).
     * @return true on success, false on I2C error
     *
     * @note Formula: SYS_MIN = 3.0V + [SYS_MIN] * 0.1V
     * @note Default: 3.5V (SYS_MIN = 101)
     * @note Range: 3000mV (000) to 3700mV (111)
     *
     * @see getMinimumSystemVoltage()
     */
    bool setMinimumSystemVoltage(uint32_t mv) override;

    /**
     * @brief Get minimum system voltage
     *
     * Reads the configured minimum system voltage.
     *
     * @return Minimum system voltage in millivolts
     */
    uint32_t getMinimumSystemVoltage() const override;

    /**
     * @brief Set input voltage limit (VINDPM)
     *
     * Sets the VINDPM threshold - the minimum voltage the input source
     * is allowed to be pulled down to. This protects weak or low-quality
     * adapters from being overloaded.
     *
     * When VBUS voltage drops to this threshold, the charger automatically
     * reduces input current to prevent the adapter from collapsing.
     *
     * @param mv Input voltage limit in millivolts (3900-15300mV)
     *        Value will be quantized to nearest valid step (100mV).
     * @return true on success, false on I2C error
     *
     * @note Formula: VINDPM = 2.6V + [VINDPM] * 100mV (absolute mode)
     * @note Minimum clamped at 3.9V
     * @note Default: 4.4V (VINDPM = 0010010)
     *
     * @note The BQ25896 supports both relative and absolute VINDPM modes:
     *       - Relative mode: VINDPM offset from VBUS no-load voltage
     *       - Absolute mode: Fixed voltage threshold (this method)
     *
     * @see getInputVoltageLimit()
     * @see isVindpmActive() to check if VINDPM is active
     */
    bool setInputVoltageLimit(uint32_t mv) override;

    /**
     * @brief Get input voltage limit
     *
     * Reads the configured VINDPM threshold.
     *
     * @return Input voltage limit in millivolts
     */
    uint32_t getInputVoltageLimit() const override;

    /**
     * @brief Set input current limit
     *
     * Sets the maximum input current drawn from VBUS. This prevents
     * the charger from drawing more current than the adapter can provide.
     *
     * The actual limit is the lower of this setting and the ILIM pin
     * (if EN_ILIM is enabled).
     *
     * @param mA Input current limit in milliamps (100-3250mA)
     *        Value will be quantized to nearest valid step (50mA).
     * @return true on success, false on I2C error
     *
     * @note Formula: IINLIM = 100mA + [IINLIM] * 50mA
     * @note Default: 500mA (IINLIM = 001000)
     * @note Range: 100mA (000000) to 3250mA (111111)
     *
     * @note The Input Current Optimizer (ICO) can automatically detect
     *       the maximum safe input current. Enable via REG02 ICO_EN.
     *
     * @see getInputCurrentLimit()
     * @see isIindpmActive() to check if current limiting is active
     */
    bool setInputCurrentLimit(uint32_t mA) override;

    /**
     * @brief Get input current limit
     *
     * Reads the configured input current limit.
     *
     * @return Input current limit in milliamps
     */
    uint32_t getInputCurrentLimit() const override;

    /**
     * @brief Enable or disable OTG (boost) mode
     *
     * Enables or disables the boost (OTG) mode. When enabled, the BQ25896
     * operates as a power source, converting battery voltage to 5V on VBUS
     * to power external USB devices (phones, tablets, accessories).
     *
     * @param enable true to enable boost/OTG mode, false to disable
     * @return true on success, false on I2C error or VBUS present
     *
     * @warning Boost mode cannot be enabled when VBUS is connected!
     *          The method will return false if VBUS is detected.
     *
     * @note Before enabling boost:
     *       - Ensure no USB cable is connected to VBUS input
     *       - Battery voltage must be above minimum threshold
     *       - Battery must have sufficient charge
     *
     * @note Default boost voltage: 4.998V (BOOSTV = 0111)
     * @note Default boost current limit: 1.4A (BOOST_LIM = 011)
     *
     * @see isBoostEnabled()
     * @see setBoostVoltage()
     * @see getBoostVoltage()
     */
    bool enableBoost(bool enable) override;

    /**
     * @brief Check if boost mode is enabled
     *
     * Returns the current boost/OTG mode status.
     *
     * @return true if boost mode is active (supplying power to VBUS)
     * @return false if boost mode is disabled
     */
    bool isBoostEnabled() const override;

    /**
     * @brief Set boost output voltage
     *
     * Sets the output voltage when in OTG boost mode. This is the voltage
     * that appears on VBUS when operating as a power source.
     *
     * @param mv Boost voltage in millivolts (4550-5510mV)
     *        Value will be quantized to nearest valid step (64mV).
     * @return true on success, false on I2C error
     *
     * @note Formula: VBOOST = 4.55V + [BOOSTV] * 64mV
     * @note Default: 4.998V (BOOSTV = 0111)
     * @note Common values: 5V (5000mV), 5.1V (5184mV), 5.2V (5248mV)
     *
     * @note Cannot change boost voltage while boost is enabled.
     *       Disable boost first, then change voltage.
     *
     * @see getBoostVoltage()
     * @see enableBoost()
     */
    bool setBoostVoltage(uint16_t mv) override;

    /**
     * @brief Get boost output voltage
     *
     * Reads the configured boost voltage.
     *
     * @return Boost voltage in millivolts (4550-5510mV)
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
    BQ25896Core &_core;
};

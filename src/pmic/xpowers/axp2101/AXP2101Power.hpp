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
 * @file      AXP2101Power.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Power Control
 *
 */
#pragma once
#include "AXP2101Core.hpp"
#include "../../PmicPowerBase.hpp"

class AXP2101Power : public PmicPowerBase
{
public:
    // DCDC identifiers for protection control
    static constexpr uint8_t DCDC1 = 0;
    static constexpr uint8_t DCDC2 = 1;
    static constexpr uint8_t DCDC3 = 2;
    static constexpr uint8_t DCDC4 = 3;
    static constexpr uint8_t DCDC5 = 4;

    /**
     * @brief Construct an AXP2101Power instance.
     * @param core Reference to the AXP2101Core bus driver.
     */
    explicit AXP2101Power(AXP2101Core &core);
    ~AXP2101Power() = default;

    // ---- PmicPowerBase overrides ----

    /**
     * @brief Set the minimum VSYS system voltage (REG 0x14 bits 6:4).
     * @param mv Voltage in millivolts (4100-4800, 100mV steps).
     * @return true on success.
     */
    bool setMinimumSystemVoltage(uint32_t mv) override;

    /**
     * @brief Get the minimum VSYS system voltage (REG 0x14 bits 6:4).
     * @return Voltage in millivolts (4100-4800).
     */
    uint32_t getMinimumSystemVoltage() const override;

    /**
     * @brief Set the VBUS input voltage limit (REG 0x15 bits 3:0).
     * @param mv Voltage in millivolts (3880-5080, 80mV steps).
     * @return true on success.
     */
    bool setInputVoltageLimit(uint32_t mv) override;

    /**
     * @brief Get the VBUS input voltage limit (REG 0x15 bits 3:0).
     * @return Voltage in millivolts (3880-5080).
     */
    uint32_t getInputVoltageLimit() const override;

    /**
     * @brief Set the VBUS input current limit (REG 0x16 bits 2:0).
     * @param mA Current in milliamps. Valid values: 100, 500, 900, 1000, 1500, 2000.
     * @return true on success.
     */
    bool setInputCurrentLimit(uint32_t mA) override;

    /**
     * @brief Get the VBUS input current limit (REG 0x16 bits 2:0).
     * @return Current in milliamps.
     */
    uint32_t getInputCurrentLimit() const override;

    /**
     * @brief Enable or disable ship mode (REG 0x12 bit 3).
     *
     * When ship mode is enabled the BATFET is opened, disconnecting
     * the battery from the system for ultra-low-power storage.
     *
     * @param enable true to enter ship mode (BATFET off), false for normal operation.
     * @return true on success.
     */
    bool enableShipMode(bool enable) override;

    /**
     * @brief Check whether ship mode is currently active (REG 0x12 bit 3).
     * @return true if ship mode is enabled (BATFET off).
     */
    bool isShipModeEnabled() const override;

    // ---- VSYS Power Down Voltage (REG 24) ----

    /**
     * @brief Set the VSYS power-off voltage threshold (REG 0x24 bits 2:0).
     * @param millivolt Threshold in millivolts (2600-3300, 100mV steps).
     * @return true on success, false if millivolt is out of range.
     */
    bool setSysPowerDownVoltage(uint16_t millivolt);

    /**
     * @brief Get the VSYS power-off voltage threshold (REG 0x24 bits 2:0).
     * @return Threshold in millivolts (2600-3300).
     */
    uint16_t getSysPowerDownVoltage();

    // ---- Internal Discharge (REG 10 bit 5) ----

    /**
     * @brief Enable internal battery discharge resistor (REG 0x10 bit 5).
     */
    void enableInternalDischarge();

    /**
     * @brief Disable internal battery discharge resistor (REG 0x10 bit 5).
     */
    void disableInternalDischarge();

    // ---- System Reset & Shutdown (REG 10) ----

    /**
     * @brief Initiate a soft shutdown (REG 0x10 bit 0).
     */
    void softShutdown();

    /**
     * @brief Trigger a system-on-chip reset (REG 0x10 bit 1).
     */
    void resetSoC();

    // ---- Sleep / Wakeup (REG 26) ----

    /**
     * @brief Enable sleep mode (REG 0x26 bit 0).
     * @return true on success.
     */
    bool enableSleep();

    /**
     * @brief Disable sleep mode (REG 0x26 bit 0).
     * @return true on success.
     */
    bool disableSleep();

    /**
     * @brief Enable wakeup function (REG 0x26 bit 1).
     * @return true on success.
     */
    bool enableWakeup();

    /**
     * @brief Disable wakeup function (REG 0x26 bit 1).
     * @return true on success.
     */
    bool disableWakeup();

    // ---- PWROK & Sequence (REG 25) ----

    /**
     * @brief Enable PWROK output (REG 0x25 bit 4).
     */
    void enablePwrOk();

    /**
     * @brief Disable PWROK output (REG 0x25 bit 4).
     */
    void disablePwrOk();

    /**
     * @brief Enable power-off delay (REG 0x25 bit 3).
     */
    void enablePowerOffDelay();

    /**
     * @brief Disable power-off delay (REG 0x25 bit 3).
     */
    void disablePowerOffDelay();

    /**
     * @brief Enable power-on sequencing (REG 0x25 bit 2).
     */
    void enablePowerSequence();

    /**
     * @brief Disable power-on sequencing (REG 0x25 bit 2).
     */
    void disablePowerSequence();

    /**
     * @brief Set the PWROK delay time (REG 0x25 bits 1:0).
     * @param opt 0=8ms, 1=16ms, 2=32ms, 3=64ms.
     * @return true on success, false if opt is out of range.
     */
    bool setPwrOkDelay(uint8_t opt);

    /**
     * @brief Get the PWROK delay time (REG 0x25 bits 1:0).
     * @return Delay option (0=8ms, 1=16ms, 2=32ms, 3=64ms).
     */
    uint8_t getPwrOkDelay();

    // ---- Power Off Configuration (REG 22) ----

    /**
     * @brief Enable automatic shutdown on over-temperature (REG 0x22 bit 2).
     */
    void enableOverTempShutdown();

    /**
     * @brief Disable automatic shutdown on over-temperature (REG 0x22 bit 2).
     */
    void disableOverTempShutdown();

    /**
     * @brief Enable long-press power-off (REG 0x22 bit 1).
     */
    void enableLongPressShutdown();

    /**
     * @brief Disable long-press power-off (REG 0x22 bit 1).
     */
    void disableLongPressShutdown();

    /**
     * @brief Set the long-press power key action (REG 0x22 bit 0).
     * @param restart true for restart on long press, false for shutdown.
     */
    void setLongPressAction(bool restart);

    // ---- DCDC Protection (REG 23-24) ----

    /**
     * @brief Enable or disable DC over-voltage protection (REG 0x23 bit 5).
     * @param enable true to enable, false to disable.
     */
    void setDCOverVoltageProtection(bool enable);

    /**
     * @brief Check if DC over-voltage protection is enabled (REG 0x23 bit 5).
     * @return true if enabled.
     */
    bool isDCOverVoltageProtectionEnabled();

    /**
     * @brief Enable or disable under-voltage protection for a DCDC (REG 0x23 bits 0-4).
     * @param dc_id DCDC index (0-4 for DCDC1-DCDC5).
     * @param enable true to enable, false to disable.
     */
    void setDCLowVoltageProtection(uint8_t dc_id, bool enable);

    /**
     * @brief Check if under-voltage protection is enabled for a DCDC (REG 0x23 bits 0-4).
     * @param dc_id DCDC index (0-4 for DCDC1-DCDC5).
     * @return true if enabled.
     */
    bool isDCLowVoltageProtectionEnabled(uint8_t dc_id);

    // ---- Power On/Off Source Status (REG 20-21) ----

    /**
     * @brief Read the power-on source status register (REG 0x20, read-only).
     * @return Bitmask indicating the source that powered the device on.
     */
    uint8_t getPowerOnStatus();

    /**
     * @brief Read the power-off source status register (REG 0x21, read-only).
     * @return Bitmask indicating the source that powered the device off.
     */
    uint8_t getPowerOffStatus();

    // ---- Fuel Gauge ----

    /**
     * @brief Write user data to the fuel-gauge backup registers.
     * @param data Pointer to the data buffer to write.
     * @param len  Number of bytes to write.
     * @return true on success.
     */
    bool writeGaugeData(uint8_t *data, uint8_t len);

    /**
     * @brief Compare data against the fuel-gauge backup registers.
     * @param data Pointer to the expected data buffer.
     * @param len  Number of bytes to compare.
     * @return true if the stored data matches, false otherwise.
     */
    bool compareGaugeData(uint8_t *data, uint8_t len);

    // ---- Low Battery Thresholds (REG 1A) ----

    /**
     * @brief Set the low-battery warning threshold (REG 0x1A bits 7:4).
     * @param percentage Warning level (5-20%).
     */
    void setLowBatWarnThreshold(uint8_t percentage);

    /**
     * @brief Get the low-battery warning threshold (REG 0x1A bits 7:4).
     * @return Warning level percentage (5-20%).
     */
    uint8_t getLowBatWarnThreshold();

    /**
     * @brief Set the low-battery shutdown threshold (REG 0x1A bits 3:0).
     * @param percentage Shutdown level (0-15%).
     */
    void setLowBatShutdownThreshold(uint8_t percentage);

    /**
     * @brief Get the low-battery shutdown threshold (REG 0x1A bits 3:0).
     * @return Shutdown level percentage (0-15%).
     */
    uint8_t getLowBatShutdownThreshold();

    // ---- Die Temperature Protection (REG 13) ----

    /**
     * @brief Enable or disable die temperature detection.
     * @param enable true to enable (default), false to disable.
     * @return true on success.
     */
    bool enableDieTempDetection(bool enable);

    /**
     * @brief Check if die temperature detection is enabled.
     * @return true if enabled.
     */
    bool isDieTempDetectionEnabled();

    /**
     * @brief Set die over-temperature Level1 threshold (REG 13 bits 2:1).
     * @param opt 0=115°C, 1=125°C (default), 2=135°C. 3=reserved.
     * @return true on success.
     */
    bool setDieTempLevel1Threshold(uint8_t opt);

    /**
     * @brief Get die over-temperature Level1 threshold (REG 13 bits 2:1).
     * @return Threshold option (0=115°C, 1=125°C, 2=135°C).
     */
    uint8_t getDieTempLevel1Threshold();

    // ---- DCDC PWM Mode Control (REG 81) ----

    /**
     * @brief Force a DCDC converter to always-PWM mode (REG 81 bits 2-5).
     * @param dc_id DCDC index (1-4 for DCDC1-DCDC4). DCDC5 has no PWM control.
     * @param force true for always-PWM, false for auto PWM/PFM.
     * @return true on success. Returns false if dc_id is 0 or >4.
     */
    bool setDCDCForcePWM(uint8_t dc_id, bool force);

    /**
     * @brief Check if a DCDC converter is in forced-PWM mode (REG 81 bits 2-5).
     * @param dc_id DCDC index (1-4 for DCDC1-DCDC4). DCDC5 has no PWM control.
     * @return true if forced-PWM is active, false for auto PWM/PFM.
     */
    bool isDCDCForcePWM(uint8_t dc_id);

    /**
     * @brief Force all DCDC1-4 to CCM (continuous conduction mode) (REG 80 bit 6).
     * @param enable true to force CCM, false for auto.
     * @return true on success.
     */
    bool setDCDCForceCCM(bool enable);

    /**
     * @brief Check if all DCDC1-4 are in forced CCM mode (REG 80 bit 6).
     * @return true if forced CCM is active.
     */
    bool isDCDCForceCCM();

    /**
     * @brief Set DVM voltage ramp rate (REG 80 bit 5).
     * @param slow true for 31.250us/step, false for 15.625us/step (default).
     * @return true on success.
     */
    bool setDVMRampSlow(bool slow);

    /**
     * @brief Check if DVM ramp is set to slow (REG 80 bit 5).
     * @return true if slow ramp is active.
     */
    bool isDVMRampSlow();

    /**
     * @brief Enable DCDC frequency spread spectrum (REG 81 bit 7).
     * @param enable true to enable, false to disable (default).
     * @return true on success.
     */
    bool setFrequencySpreadEnable(bool enable);

    /**
     * @brief Check if frequency spread spectrum is enabled (REG 81 bit 7).
     * @return true if enabled.
     */
    bool isFrequencySpreadEnabled();

    /**
     * @brief Set frequency spread range (REG 81 bit 6).
     * @param wide true for 100kHz, false for 50kHz (default).
     * @return true on success.
     */
    bool setFrequencySpreadRange(bool wide);

    /**
     * @brief Get the frequency spread range (REG 81 bit 6).
     * @return true if wide (100kHz), false if narrow (50kHz).
     */
    bool getFrequencySpreadRange();

    /**
     * @brief Set DCDC UVP debounce time (REG 81 bits 1:0).
     * @param opt 0=60us, 1=120us, 2=180us, 3=240us.
     * @return true on success.
     */
    bool setDCDCUVPDebounce(uint8_t opt);

    /**
     * @brief Get DCDC UVP debounce time (REG 81 bits 1:0).
     * @return Debounce option (0=60us, 1=120us, 2=180us, 3=240us).
     */
    uint8_t getDCDCUVPDebounce();

    // ---- Fast Power-On Sequencing (REG 28-2B) ----

    /**
     * @brief Enable fast power-on sequence (REG 2B bit 7).
     * @param enable true to enable, false to disable (default).
     * @return true on success.
     */
    bool enableFastPowerOn(bool enable);

    /**
     * @brief Check if fast power-on sequence is enabled (REG 2B bit 7).
     * @return true if enabled.
     */
    bool isFastPowerOnEnabled();

    /**
     * @brief Enable fast wakeup (REG 2B bit 6).
     * @param enable true to enable, false to disable (default).
     * @return true on success.
     */
    bool enableFastWakeup(bool enable);

    /**
     * @brief Check if fast wakeup is enabled (REG 2B bit 6).
     * @return true if enabled.
     */
    bool isFastWakeupEnabled();

    /**
     * @brief Set fast power-on start sequence for a channel (REG 28-2B).
     *
     * @param channel Channel index (0-13 for DCDC1-DLDO2).
     *                Use AXP2101Channel::CH_xxx constants.
     * @param seq Sequence level (0-2 for start order, 3=disable).
     * @return true on success.
     */
    bool setFastPowerOnSequence(uint8_t channel, uint8_t seq);

    /**
     * @brief Get fast power-on start sequence for a channel (REG 28-2B).
     * @param channel Channel index (0-13 for DCDC1-DLDO2).
     * @return Sequence level (0-2 for start order, 3=disabled).
     */
    uint8_t getFastPowerOnSequence(uint8_t channel);

private:
    AXP2101Core &_core;
};

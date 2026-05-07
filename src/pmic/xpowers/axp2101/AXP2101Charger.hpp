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
 * @file      AXP2101Charger.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Charger Control
 *
 */
#pragma once
#include "../../PmicChargerBase.hpp"
#include "AXP2101Core.hpp"

class AXP2101Charger : public PmicChargerBase
{
public:
    /**
     * @brief Construct AXP2101Charger with a reference to the AXP2101Core.
     * @param core Reference to the AXP2101Core providing register access.
     */
    explicit AXP2101Charger(AXP2101Core &core);
    ~AXP2101Charger() = default;

    // ---- PmicChargerBase overrides ----

    /**
     * @brief Enable or disable battery charging.
     *        Controls the charge enable bit in REG 0x18.
     * @param enable true to enable charging, false to disable.
     * @return true on success.
     */
    bool enableCharging(bool enable) override;

    /**
     * @brief Check if the charger is actively charging.
     *        Reads STATUS2 (REG 0x01) bits 2:0 to determine charge phase.
     * @return true if charging is in progress (precharge, CC, or CV phase).
     */
    bool isCharging() override;

    /**
     * @brief Set the precharge current limit (REG 0x61, bits 3:0).
     *        Current = 25 * N mA, where N is clamped to 0-8 (0-200 mA).
     * @param mA Desired precharge current in milliamps (0-200).
     * @return true on success.
     */
    bool setPreChargeCurrent(uint16_t mA) override;

    /**
     * @brief Get the configured precharge current (REG 0x61, bits 3:0).
     *        Current = 25 * N mA.
     * @return Precharge current in milliamps (0-200).
     */
    uint16_t getPreChargeCurrent() override;

    /**
     * @brief Set the fast-charge current limit (REG 0x62, bits 4:0).
     *        For N <= 8: current = 25 * N mA. For N > 8: current = 200 + 100 * (N - 8) mA.
     *        Maximum 1000 mA.
     * @param mA Desired fast-charge current in milliamps (0-1000).
     * @return true on success.
     */
    bool setFastChargeCurrent(uint16_t mA) override;

    /**
     * @brief Get the configured fast-charge current (REG 0x62, bits 4:0).
     *        For N <= 8: current = 25 * N mA. For N > 8: current = 200 + 100 * (N - 8) mA.
     * @return Fast-charge current in milliamps (0-1000).
     */
    uint16_t getFastChargeCurrent() override;

    /**
     * @brief Set the charge termination current (REG 0x63, bits 3:0).
     *        Current = 25 * N mA, where N is clamped to 0-8 (0-200 mA).
     * @param mA Desired termination current in milliamps (0-200).
     * @return true on success.
     */
    bool setTerminationCurrent(uint16_t mA) override;

    /**
     * @brief Get the configured termination current (REG 0x63, bits 3:0).
     *        Current = 25 * N mA.
     * @return Termination current in milliamps (0-200).
     */
    uint16_t getTerminationCurrent() override;

    /**
     * @brief Set the charge voltage target (REG 0x64, bits 2:0).
     *        Valid values: 0=5.0V, 1=4.0V, 2=4.1V, 3=4.2V, 4=4.35V, 5=4.4V.
     * @param mV Desired charge voltage in millivolts.
     * @return true on success.
     */
    bool setChargeVoltage(uint16_t mV) override;

    /**
     * @brief Get the configured charge voltage target (REG 0x64, bits 2:0).
     * @return Charge voltage in millivolts (e.g. 4200, 4350, etc.).
     */
    uint16_t getChargeVoltage() override;

    /**
     * @brief Get the current charger status.
     *        Reads STATUS1 (REG 0x00) and STATUS2 (REG 0x01).
     *        STATUS1 bit 5 = VBUS present, bit 3 = battery present.
     *        STATUS2 bits 2:0 = charge phase (0=no charging, 1=precharge, 2/3=CC, 4=done, 5+=no charging).
     * @return Status struct with vbusPresent, batteryPresent, and chargePhase fields.
     */
    Status getStatus() override;

    // ---- Termination Control (REG 63 bit 4) ----

    /**
     * @brief Enable charge termination detection (REG 0x63, bit 4).
     *        When enabled, charging stops when current drops below the termination current threshold.
     */
    void enableChargeTermination();

    /**
     * @brief Disable charge termination detection (REG 0x63, bit 4).
     *        Charging will not automatically stop based on termination current.
     */
    void disableChargeTermination();

    /**
     * @brief Check if charge termination detection is enabled (REG 0x63, bit 4).
     * @return true if charge termination is enabled.
     */
    bool isChargeTerminationEnabled();

    // ---- Button Battery Charge (REG 18 bit 2, REG 6A) ----

    /**
     * @brief Enable button/coin-cell battery charging (REG 0x18, bit 2).
     */
    void enableButtonBatteryCharge();

    /**
     * @brief Disable button/coin-cell battery charging (REG 0x18, bit 2).
     */
    void disableButtonBatteryCharge();

    /**
     * @brief Check if button battery charging is enabled (REG 0x18, bit 2).
     * @return true if button battery charge is enabled.
     */
    bool isButtonBatteryChargeEnabled();

    /**
     * @brief Set the button battery charge voltage (REG 0x6A, bits 2:0).
     *        Valid range: 2600-3300 mV in 100 mV steps.
     * @param millivolt Desired voltage in millivolts (2600-3300).
     * @return true on success.
     */
    bool setButtonBatteryChargeVoltage(uint16_t millivolt);

    /**
     * @brief Get the configured button battery charge voltage (REG 0x6A, bits 2:0).
     * @return Charge voltage in millivolts (2600-3300).
     */
    uint16_t getButtonBatteryChargeVoltage();

    // ---- Battery Detection (REG 68 bit 0) ----

    /**
     * @brief Enable battery detection (REG 0x68, bit 0).
     *        When enabled, the charger detects whether a battery is connected.
     */
    void enableBatteryDetection();

    /**
     * @brief Disable battery detection (REG 0x68, bit 0).
     */
    void disableBatteryDetection();

    /**
     * @brief Check if battery detection is enabled (REG 0x68, bit 0).
     * @return true if battery detection is enabled.
     */
    bool isBatteryDetectionEnabled();

    // ---- Charger Safety Timer (REG 67) ----

    /**
     * @brief Set the precharge safety timer (REG 0x67, bits 2:0).
     *        bit 2 = timer enable. bits 1:0 select timeout: 0=40min, 1=50min, 2=60min, 3=70min.
     * @param opt Timer option value (bits 2:0). Bit 2 enables, bits 1:0 select duration.
     */
    void setPreChargeSafetyTimer(uint8_t opt);

    /**
     * @brief Set the charge-done safety timer (REG 0x67, bits 6:4).
     *        bit 6 = timer enable. bits 5:4 select timeout: 0=5h, 1=8h, 2=12h, 3=20h.
     * @param opt Timer option value (bits 6:4). Bit 6 enables, bits 5:4 select duration.
     */
    void setChargeDoneSafetyTimer(uint8_t opt);

    // ---- Thermal Regulation (REG 65) ----

    /**
     * @brief Set the thermal regulation threshold (REG 0x65, bits 1:0).
     *        Valid values: 0=60°C, 1=80°C, 2=100°C, 3=120°C.
     * @param opt Threshold option (0-3).
     */
    void setThermalRegulationThreshold(uint8_t opt);

    /**
     * @brief Get the thermal regulation threshold (REG 0x65, bits 1:0).
     * @return Threshold option (0=60°C, 1=80°C, 2=100°C, 3=120°C).
     */
    uint8_t getThermalRegulationThreshold();

    // ---- JEITA Temperature Protection (REG 54-5B) ----

    /**
     * @brief Enable or disable JEITA temperature standard.
     * @param enable true to enable, false to disable.
     * @return true on success.
     */
    bool enableJeita(bool enable);

    /**
     * @brief Check if JEITA temperature standard is enabled.
     * @return true if enabled.
     */
    bool isJeitaEnabled();

    // -- Temperature Fault Thresholds --

    /**
     * @brief Set charge low-temperature fault threshold (REG 54, VLTF_CHG).
     * @param val Raw register value. VLTF = val * 32 mV. Default 0x29 (~0°C).
     * @return true on success.
     */
    bool setChargeLowTempFaultThreshold(uint8_t val);

    /**
     * @brief Get charge low-temperature fault threshold (REG 54, VLTF_CHG).
     * @return Raw register value. VLTF = val * 32 mV.
     */
    uint8_t getChargeLowTempFaultThreshold();

    /**
     * @brief Set charge high-temperature fault threshold (REG 55, VHTF_CHG).
     * @param val Raw register value. VHTF = val * 2 mV. Default 0x58 (~45°C).
     * @return true on success.
     */
    bool setChargeHighTempFaultThreshold(uint8_t val);

    /**
     * @brief Get charge high-temperature fault threshold (REG 55, VHTF_CHG).
     * @return Raw register value. VHTF = val * 2 mV.
     */
    uint8_t getChargeHighTempFaultThreshold();

    /**
     * @brief Set work low-temperature fault threshold (REG 56, VLTF_WORK).
     * @param val Raw register value. VLTF = val * 32 mV. Default 0x3E (~-10°C).
     * @return true on success.
     */
    bool setWorkLowTempFaultThreshold(uint8_t val);

    /**
     * @brief Get work low-temperature fault threshold (REG 56, VLTF_WORK).
     * @return Raw register value. VLTF = val * 32 mV.
     */
    uint8_t getWorkLowTempFaultThreshold();

    /**
     * @brief Set work high-temperature fault threshold (REG 57, VHTF_WORK).
     * @param val Raw register value. VHTF = val * 2 mV. Default 0x4C (~55°C).
     * @return true on success.
     */
    bool setWorkHighTempFaultThreshold(uint8_t val);

    /**
     * @brief Get work high-temperature fault threshold (REG 57, VHTF_WORK).
     * @return Raw register value. VHTF = val * 2 mV.
     */
    uint8_t getWorkHighTempFaultThreshold();

    // -- Temperature Hysteresis --

    /**
     * @brief Set low-temperature hysteresis (REG 52, HYSL2H).
     * @param val Raw register value. Hysteresis = val * 16 mV. Default 0x02 (32mV).
     * @return true on success.
     */
    bool setLowTempHysteresis(uint8_t val);

    /**
     * @brief Get low-temperature hysteresis (REG 52, HYSL2H).
     * @return Raw register value. Hysteresis = val * 16 mV.
     */
    uint8_t getLowTempHysteresis();

    /**
     * @brief Set high-temperature hysteresis (REG 53, HYSH2L).
     * @param val Raw register value. Hysteresis = val * 4 mV. Default 0x01 (4mV).
     * @return true on success.
     */
    bool setHighTempHysteresis(uint8_t val);

    /**
     * @brief Get high-temperature hysteresis (REG 53, HYSH2L).
     * @return Raw register value. Hysteresis = val * 4 mV.
     */
    uint8_t getHighTempHysteresis();

    // -- JEITA CV Configuration (REG 59) --

    /**
     * @brief Set warm-zone current reduction in JEITA.
     * @param reduce true for 50%, false for 100%. Default false.
     * @return true on success.
     */
    bool setJeitaWarmCurrentReduction(bool reduce);

    /**
     * @brief Get warm-zone current reduction setting in JEITA (REG 0x59).
     * @return true if current is reduced to 50% in warm zone, false for 100%.
     */
    bool getJeitaWarmCurrentReduction();

    /**
     * @brief Set cool-zone current reduction in JEITA.
     * @param reduce true for 50%, false for 100%. Default true.
     * @return true on success.
     */
    bool setJeitaCoolCurrentReduction(bool reduce);

    /**
     * @brief Get cool-zone current reduction setting in JEITA (REG 0x59).
     * @return true if current is reduced to 50% in cool zone, false for 100%.
     */
    bool getJeitaCoolCurrentReduction();

    /**
     * @brief Set warm-zone voltage drop levels in JEITA.
     * @param levels 0=0mV, 1=1 level, 2=2 levels. Default 1.
     * @return true on success.
     */
    bool setJeitaWarmVoltageDrop(uint8_t levels);

    /**
     * @brief Get warm-zone voltage drop levels in JEITA (REG 0x59).
     * @return Voltage drop level (0=0mV, 1=1 level, 2=2 levels).
     */
    uint8_t getJeitaWarmVoltageDrop();

    /**
     * @brief Set cool-zone voltage drop levels in JEITA.
     * @param levels 0=0mV, 1=1 level, 2=2 levels. Default 0.
     * @return true on success.
     */
    bool setJeitaCoolVoltageDrop(uint8_t levels);

    /**
     * @brief Get cool-zone voltage drop levels in JEITA (REG 0x59).
     * @return Voltage drop level (0=0mV, 1=1 level, 2=2 levels).
     */
    uint8_t getJeitaCoolVoltageDrop();

    // -- JEITA Temperature Thresholds (REG 5B) --

    /**
     * @brief Set JEITA cool temperature threshold T2 (REG 5B low nibble).
     * @param val Raw register value. VHTF = val * 16 mV. Default 0x37 (~10°C).
     * @return true on success.
     */
    bool setJeitaCoolThreshold(uint8_t val);

    /**
     * @brief Get JEITA cool temperature threshold T2 (REG 5B low nibble).
     * @return Raw register value. VHTF = val * 16 mV.
     */
    uint8_t getJeitaCoolThreshold();

    /**
     * @brief Set JEITA warm temperature threshold T3 (REG 5B high nibble).
     * @param val Raw register value. VHTF = val * 8 mV. Default 0x1E (~45°C).
     * @return true on success.
     */
    bool setJeitaWarmThreshold(uint8_t val);

    /**
     * @brief Get JEITA warm temperature threshold T3 (REG 5B high nibble).
     * @return Raw register value. VHTF = val * 8 mV.
     */
    uint8_t getJeitaWarmThreshold();

private:
    AXP2101Core &_core;
};

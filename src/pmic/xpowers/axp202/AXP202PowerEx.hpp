/**
 * @file      AXP202PowerEx.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-05-02
 * @brief     AXP202-specific power management extensions
 *
 * Features not covered by AXP1xxPower template: power-key shutdown control,
 * LDO3 and DCDC operating modes, APS warning thresholds, VBUS detection/SRP,
 * over-temperature shutdown, and DCDC frequency setting.
 *
 */
#pragma once
#include "AXP202Core.hpp"
#include "AXP202Regs.hpp"

class AXP202PowerEx
{
public:
    enum class LDO3Mode : uint8_t {
        LDO = 0,
        DCIN = 1,
    };

    explicit AXP202PowerEx(AXP202Core &core) : _core(core) {}

    // ---- Power Key Configuration (REG 36H) ----

    /**
     * @brief Enable automatic power-off when the power key is held 
     */
    void enableLongPressShutdown()
    {
        _core.setRegBit(axp202_regs::pmu::POK_SET, 3);
    }

    /**
     * @brief Disable automatic power-off when the power key is held 
     */
    void disableLongPressShutdown()
    {
        _core.clrRegBit(axp202_regs::pmu::POK_SET, 3);
    }

    // ---- LDO3 Operating Mode (REG 29H) ----

    /**
     * @brief Select the LDO3 regulated output or DCIN pass-through path
     * @param mode LDO for regulated output, DCIN for pass-through
     * @return true on success
     */
    bool setLDO3Mode(LDO3Mode mode)
    {
        switch (mode) {
        case LDO3Mode::LDO:
            return _core.clrRegBit(axp202_regs::pmu::LDO3OUT_VOL, 7);
        case LDO3Mode::DCIN:
            return _core.setRegBit(axp202_regs::pmu::LDO3OUT_VOL, 7);
        default:
            return false;
        }
    }

    // ---- DCDC Operating Mode (REG 80H) ----

    /**
     * @brief Force DCDC2 to PWM mode (REG 80H bit 2)
     * @param pwm true=forced PWM, false=auto PWM/PFM
     * @return true on success
     */
    bool setDCDC2PWMMode(bool pwm)
    {
        return pwm ? _core.setRegBit(axp202_regs::pmu::DCDC_MODESET, 2)
                   : _core.clrRegBit(axp202_regs::pmu::DCDC_MODESET, 2);
    }

    bool isDCDC2PWMMode()
    {
        return _core.getRegBit(axp202_regs::pmu::DCDC_MODESET, 2);
    }

    /**
     * @brief Force DCDC3 to PWM mode (REG 80H bit 1)
     * @param pwm true=forced PWM, false=auto PWM/PFM
     * @return true on success
     */
    bool setDCDC3PWMMode(bool pwm)
    {
        return pwm ? _core.setRegBit(axp202_regs::pmu::DCDC_MODESET, 1)
                   : _core.clrRegBit(axp202_regs::pmu::DCDC_MODESET, 1);
    }

    bool isDCDC3PWMMode()
    {
        return _core.getRegBit(axp202_regs::pmu::DCDC_MODESET, 1);
    }

    // ---- APS Warning Thresholds (REG 3AH, 3BH) ----

    /**
     * @brief Set APS warning level 1 voltage (REG 3AH)
     * @param opt Raw value. Voltage = 2.8672V + opt * 5.6mV. Range 0-255.
     * @return true on success
     */
    bool setAPSWarningLevel1(uint8_t opt)
    {
        return _core.updateBits(axp202_regs::pmu::APS_WARNING1, 0xFF, opt) >= 0;
    }

    uint8_t getAPSWarningLevel1()
    {
        int val = _core.readReg(axp202_regs::pmu::APS_WARNING1);
        return (val < 0) ? 0 : static_cast<uint8_t>(val & 0xFF);
    }

    /**
     * @brief Set APS warning level 2 voltage (REG 3BH)
     * @param opt Raw value. Same formula as level 1.
     * @return true on success
     */
    bool setAPSWarningLevel2(uint8_t opt)
    {
        return _core.updateBits(axp202_regs::pmu::APS_WARNING2, 0xFF, opt) >= 0;
    }

    uint8_t getAPSWarningLevel2()
    {
        int val = _core.readReg(axp202_regs::pmu::APS_WARNING2);
        return (val < 0) ? 0 : static_cast<uint8_t>(val & 0xFF);
    }

    // ---- Over-Temperature Shutdown (REG 8FH) ----

    /**
     * @brief Enable over-temperature shutdown (REG 8FH bit 2)
     * @param enable true to enable shutdown on over-temp
     * @return true on success
     */
    bool enableOverTemperatureShutdown(bool enable)
    {
        return enable ? _core.setRegBit(axp202_regs::pmu::HOTOVER_CTL, 2)
                      : _core.clrRegBit(axp202_regs::pmu::HOTOVER_CTL, 2);
    }

    bool isOverTemperatureShutdownEnabled()
    {
        return _core.getRegBit(axp202_regs::pmu::HOTOVER_CTL, 2);
    }

    // ---- DCDC Frequency (REG 37H) ----
    /**
     * @brief Set DCDC switching frequency (REG 37H)
     * @param freq Frequency code. Consult datasheet for valid values.
     * @return true on success
     */
    bool setDCDCFrequency(uint8_t freq)
    {
        return _core.writeReg(axp202_regs::pmu::DCDC_FREQSET, freq) >= 0;
    }

    uint8_t getDCDCFrequency()
    {
        int val = _core.readReg(axp202_regs::pmu::DCDC_FREQSET);
        return (val < 0) ? 0 : static_cast<uint8_t>(val);
    }

    // ---- VBUS Detection/SRP (REG 8BH) ----

    /**
     * @brief Enable VBUS Valid detection (REG 8BH bit 3)
     * @param enable true to enable
     * @return true on success
     */
    bool enableVBUSDetection(bool enable)
    {
        return enable ? _core.setRegBit(axp202_regs::pmu::VBUS_DET_SRP, 3)
                      : _core.clrRegBit(axp202_regs::pmu::VBUS_DET_SRP, 3);
    }

    bool isVBUSDetectionEnabled()
    {
        return _core.getRegBit(axp202_regs::pmu::VBUS_DET_SRP, 3);
    }

private:
    AXP202Core &_core;
};

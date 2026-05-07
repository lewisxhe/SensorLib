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
 * @file      AXP2101Core.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Core I2C Communication and Module Control
 *
 */
#pragma once
#include "../../../platform/comm/I2CDeviceWithHal.hpp"

/**
 * @brief Core I2C communication layer for the AXP2101 PMIC.
 *
 * Provides low-level I2C register access (inherited from I2CDeviceWithHal),
 * chip ID verification, and module enable/disable control for the AXP2101's
 * internal functional blocks.
 */
class AXP2101Core : public I2CDeviceWithHal
{
public:
    AXP2101Core() = default;
    ~AXP2101Core() = default;

    /**
     * @brief Identifiers for AXP2101 internal functional modules.
     *
     * Each module maps to a specific enable bit in REG 0x18 (MODULE_EN)
     * or REG 0x30 (ADC_CHANNEL_CTRL) or REG 0x68 (BAT_DETECT_CTRL).
     */
    enum class Module : uint8_t {
        CELL_CHARGE,        ///< Cell battery charger (REG 0x18 bit 1)
        BTN_CHARGE,         ///< Button/backup battery charger (REG 0x18 bit 2)
        WATCHDOG,           ///< Watchdog timer (REG 0x18 bit 0)
        GAUGE,              ///< Fuel gauge (REG 0x18 bit 3)
        BAT_DETECT,         ///< Battery detection (REG 0x68 bit 0)
        GENERAL_ADC,        ///< General-purpose ADC (REG 0x30 bit 5)
        TEMP_MEASURE,       ///< Die temperature ADC (REG 0x30 bit 4)
        VSYS_MEASURE,       ///< VSYS voltage ADC (REG 0x30 bit 3)
        VBUS_MEASURE,       ///< VBUS voltage ADC (REG 0x30 bit 2)
        TS_MEASURE,         ///< TS pin ADC (REG 0x30 bit 1)
        BAT_VOLT_MEASURE,   ///< Battery voltage ADC (REG 0x30 bit 0)
    };

    /** Watchdog timer configuration options. */
    enum class WatchdogConfig : uint8_t {
        TIMEOUT_IRQ_ONLY = 0,                       ///< Trigger IRQ only
        TIMEOUT_IRQ_AND_SYS_RESET,                  ///< Trigger IRQ and system reset
        TIMEOUT_IRQ_AND_SYS_RESET_PULLDOWN_PWROK_1S,///< Trigger IRQ, system reset, and pull down PWROK after 1s
        TIMEOUT_IRQ_AND_SYS_RESET_AFTER_POWEROFF_THEN_POWERON ///< Trigger IRQ, system reset after power off, then power on
    };

    /**
     * @brief Enable or disable an internal functional module.
     * @param module Module identifier.
     * @param enable true to enable, false to disable.
     * @return true on success.
     */
    bool enableModule(Module module, bool enable);

    /**
     * @brief Check whether an internal module is enabled.
     * @param module Module identifier.
     * @return true if the module is enabled.
     */
    bool isModuleEnabled(Module module);

    /**
     * @brief Set watchdog timer configuration.
     * @param config Watchdog configuration.
     * @return true on success or false on failure.
     */
    bool setWatchdogConfig(WatchdogConfig config);

private:
    bool initImpl(uint8_t param) override;
};

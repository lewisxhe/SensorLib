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
 * @file      PmicAXP2101.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 PMIC Unified Interface
 *
 */
#pragma once

#include "../../PmicBase.hpp"
#include "../../PmicAdcBase.hpp"
#include "../../PmicIrqBase.hpp"
#include "../../PmicPowerBase.hpp"

#include "AXP2101Regs.hpp"
#include "AXP2101Core.hpp"
#include "AXP2101Adc.hpp"
#include "AXP2101Irq.hpp"
#include "AXP2101Charger.hpp"
#include "AXP2101Led.hpp"
#include "AXP2101Power.hpp"
#include "AXP2101Pwron.hpp"
#include "AXP2101Watchdog.hpp"
#include "AXP2101Channel.hpp"

static constexpr uint8_t AXP2101_SLAVE_ADDRESS = 0x34;

/**
 * @brief Top-level unified interface for the AXP2101 PMIC.
 *
 * Aggregates all AXP2101 sub-modules (ADC, IRQ, charger, LED, power,
 * power button, watchdog, and power channels) into a single
 * convenient class. Provides I2C initialisation across multiple
 * platforms (Arduino, ESP-IDF, custom HAL) and exposes both the
 * polymorphic base-class accessors and chip-specific module references.
 *
 * Capabilities:
 * - Charger (PmicSupportCharger)
 * - Power management (PmicSupportPower)
 * - ADC (PmicSupportAdc)
 * - Interrupt controller (PmicSupportIrq)
 * - Power button (PmicSupportPowerBtn)
 * - Power output channels (PmicSupportChannel)
 */
class PmicAXP2101 : public PmicBase
{
public:
    using Module = AXP2101Core::Module;

    /**
     * @brief Construct the AXP2101 PMIC interface, initialising all sub-modules.
     */
    PmicAXP2101() : _core(), _adc(_core)
        , _irq(_core), _charger(_core)
        , _led(_core), _power(_core)
        , _pwron(_core), _watchdog(_core), _channel(_core)
    {}

#if defined(ARDUINO)
    /**
     * @brief Initialise the AXP2101 using Arduino Wire (TwoWire) API.
     * @param wire Reference to TwoWire bus object.
     * @param addr I2C slave address.
     * @param sda SDA pin number (-1 for default).
     * @param scl SCL pin number (-1 for default).
     * @return true on success.
     */
    bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(wire, addr, sda, scl);
    }
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialise the AXP2101 using ESP-IDF legacy I2C driver.
     * @param port_num I2C port number.
     * @param addr I2C slave address.
     * @param sda SDA pin number (-1 for default).
     * @param scl SCL pin number (-1 for default).
     * @return true on success.
     */
    bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(port_num, addr, sda, scl);
    }
#else
    /**
     * @brief Initialise the AXP2101 using ESP-IDF new I2C master driver.
     * @param handle I2C master bus handle.
     * @param addr I2C slave address.
     * @return true on success.
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr)
    {
        return _core.begin(handle, addr);
    }
#endif
#endif

    /**
     * @brief Initialise the AXP2101 with custom communication callbacks.
     * @param callback Custom I2C callback.
     * @param hal_cb Custom HAL callback.
     * @param addr I2C slave address.
     * @return true on success.
     */
    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_cb,
               uint8_t addr)
    {
        return _core.begin(callback, hal_cb, addr);
    }

    /**
     * @brief De-initialise the PMIC (no-op for AXP2101).
     */
    void end()
    {
    }

    /**
     * @brief Get the supported PMIC capabilities bitmask.
     * @return Capability flags.
     */
    PmicCapability::Capability getCapabilities() const override
    {
        return PmicCapability::Capability::PmicSupportCharger |
               PmicCapability::Capability::PmicSupportPower |
               PmicCapability::Capability::PmicSupportAdc |
               PmicCapability::Capability::PmicSupportIrq |
               PmicCapability::Capability::PmicSupportLed |
               PmicCapability::Capability::PmicSupportPowerBtn |
               PmicCapability::Capability::PmicSupportChannel;
    }

    /**
     * @brief Get the PMIC configuration descriptor.
     * @return const reference to static PmicConfig.
     */
    const PmicConfig &getConfig() const override
    {
        static const PmicConfig config = {
            .chipName = "AXP2101",
            .i2cAddress = 0x34,
            .chipIdReg = 0x03,
            .chipIdValue = 0x4A,
            .channelCount = 14,
            .capabilities = PmicCapability::Capability::PmicSupportCharger |
            PmicCapability::Capability::PmicSupportPower |
            PmicCapability::Capability::PmicSupportAdc |
            PmicCapability::Capability::PmicSupportIrq |
            PmicCapability::Capability::PmicSupportLed |
            PmicCapability::Capability::PmicSupportPowerBtn |
            PmicCapability::Capability::PmicSupportChannel,
        };
        return config;
    }


    /**
     * @brief Get the power management interface.
     * @return Reference to AXP2101Power.
     */
    PmicPowerBase &power() override
    {
        return _power;
    }

    /**
     * @brief Get the charger interface (pointer for optional support).
     * @return Pointer to AXP2101Charger, or nullptr if not available.
     */
    PmicChargerBase *getCharger() override
    {
        return &_charger;
    }

    /**
     * @brief Get the ADC interface.
     * @return Reference to AXP2101Adc.
     */
    PmicAdcBase &adc() override
    {
        return _adc;
    }

    /**
     * @brief Get the LED interface.
     * @return Reference to AXP2101Led.
     */
    PmicLedBase &led() override
    {
        return _led;
    }

    /**
     * @brief Get the power output channel interface.
     * @return Pointer to AXP2101Channel, nullptr if not supported.
     */
    PmicChannelBase *getChannel() override
    {
        return &_channel;
    }

    PmicIrqBase *getIrq() override
    {
        return &_irq;
    }

    /**
     * @brief Get the human-readable chip name.
     * @return String "AXP2101".
     */
    const char *getChipName() const override
    {
        return "AXP2101";
    }

    // ---- Chip-specific module accessors ----

    /**
     * @brief Get the chip-specific IRQ controller.
     * @return Reference to AXP2101Irq.
     */
    AXP2101Irq &irq()
    {
        return _irq;
    }

    /**
     * @brief Get the chip-specific charger interface.
     * @return Reference to AXP2101Charger.
     */
    AXP2101Charger &charger()
    {
        return _charger;
    }

    /**
     * @brief Get the chip-specific LED module.
     * @return Reference to AXP2101Led.
     */
    AXP2101Led &ledModule()
    {
        return _led;
    }

    /**
     * @brief Get the chip-specific power button interface.
     * @return Reference to AXP2101Pwron.
     */
    AXP2101Pwron &pwron()
    {
        return _pwron;
    }

    /**
     * @brief Get the chip-specific watchdog interface.
     * @return Reference to AXP2101Watchdog.
     */
    AXP2101Watchdog &watchdog()
    {
        return _watchdog;
    }

    /**
     * @brief Get the chip-specific ADC interface.
     * @return Reference to AXP2101Adc.
     */
    AXP2101Adc &chipAdc()
    {
        return _adc;
    }

    /**
     * @brief Get the core I2C communication interface.
     * @return Reference to AXP2101Core.
     */
    AXP2101Core &core()
    {
        return _core;
    }

    /**
     * @brief Get the chip-specific power management interface.
     * @return Reference to AXP2101Power.
     */
    AXP2101Power &powerEx()
    {
        return _power;
    }

    // ---- Module enable/disable ----

    /**
     * @brief Enable or disable an internal functional module.
     *
     * Convenience wrapper for AXP2101Core::enableModule().
     *
     * @param module Module identifier.
     * @param enable true to enable, false to disable.
     * @return true on success.
     */
    bool enableModule(Module module, bool enable)
    {
        return _core.enableModule(module, enable);
    }

    /**
     * @brief Check whether an internal module is enabled.
     *
     * Convenience wrapper for AXP2101Core::isModuleEnabled().
     *
     * @param module Module identifier.
     * @return true if the module is enabled.
     */
    bool isModuleEnabled(Module module)
    {
        return _core.isModuleEnabled(module);
    }

    // ---- Status & basic info ----

    /**
     * @brief Read the combined STATUS1 and STATUS2 registers.
     * @return 16-bit status word (high byte = STATUS1, low byte = STATUS2).
     *         Returns 0 on I2C error.
     */
    uint16_t getStatus()
    {
        int s1 = _core.readReg(axp2101_regs::bmu::STATUS1);
        int s2 = _core.readReg(axp2101_regs::bmu::STATUS2);
        if (s1 < 0 || s2 < 0) return 0;
        return (static_cast<uint16_t>(s1) << 8) | static_cast<uint16_t>(s2);
    }

    /**
     * @brief Check if a valid VBUS power source is present.
     * @return true if VBUS is good.
     */
    bool isVbusGood()
    {
        return _core.getRegBit(axp2101_regs::bmu::STATUS1, 5);
    }

    /**
     * @brief Check if a battery is connected.
     * @return true if battery is detected.
     */
    bool isBatteryConnect()
    {
        return _core.getRegBit(axp2101_regs::bmu::STATUS1, 3);
    }

    /**
     * @brief Check if the battery is currently charging.
     * @return true if charging.
     */
    bool isCharging()
    {
        return _charger.isCharging();
    }

    /**
     * @brief Read the AXP2101 chip ID / version register.
     * @return Chip ID byte, or 0 on error.
     */
    uint8_t getChipID()
    {
        int id = _core.readReg(axp2101_regs::bmu::IC_TYPE);
        if (id < 0) return 0;
        return static_cast<uint8_t>(id);
    }

    /**
     * @brief Soft power-off the PMIC.
     */
    void shutdown()
    {
        _power.softShutdown();
    }

    /**
     * @brief Reset (restart) the SoC system.
     */
    void reset()
    {
        _power.resetSoC();
    }

private:
    /**
     * @brief Const accessor to the core (used internally).
     * @return const reference to AXP2101Core.
     */
    const AXP2101Core &core() const
    {
        return _core;
    }

    AXP2101Core _core;
    AXP2101Adc _adc;
    AXP2101Irq _irq;
    AXP2101Charger _charger;
    AXP2101Led _led;
    AXP2101Power _power;
    AXP2101Pwron _pwron;
    AXP2101Watchdog _watchdog;
    AXP2101Channel _channel;
};

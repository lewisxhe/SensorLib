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
 * @file      PmicAXP202.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP202 PMIC Unified Interface
 *
 * Top-level class that aggregates all AXP202 functional blocks (ADC,
 * charger, power, GPIO, channel, IRQ, LED, timer, power-on) into
 * a single PmicBase-compatible interface. Provides platform-specific
 * begin() overloads for Arduino (TwoWire), ESP-IDF (I2C legacy and
 * new driver), and custom communication callbacks.
 *
 * @section Capabilities
 * - Charger (via AXP1xxCharger<axp202_regs>)
 * - Power management (via AXP1xxPower<axp202_regs>)
 * - ADC measurements (via AXP202Adc)
 * - GPIO control (via AXP202Gpio)
 * - Power output channels (via AXP202Channel)
 * - Interrupt handling (via AXP1xxIrq<axp202_regs>)
 * - LED control (via AXP1xxLed<axp202_regs>)
 * - Power-on button (via AXP1xxPwron<axp202_regs>)
 * - Countdown timer (via AXP1xxTimer<axp202_regs>)
 *
 * @note All shared AXP1xx template classes are parameterised with axp202_regs
 *       to use AXP202-specific register addresses and constants.
 */
#pragma once

#include "../../PmicBase.hpp"
#include "../../PmicAdcBase.hpp"
#include "../../PmicIrqBase.hpp"

#include "AXP202Regs.hpp"
#include "AXP202Core.hpp"
#include "AXP202Adc.hpp"
#include "AXP202PowerEx.hpp"
#include "AXP202ChargerEx.hpp"
#include "AXP202Coulomb.hpp"
#include "../axp1xx/AXP1xxIrq.hpp"
#include "../axp1xx/AXP1xxCharger.hpp"
#include "AXP202Gpio.hpp"
#include "AXP202Channel.hpp"

#include "../axp1xx/AXP1xxPower.hpp"
#include "../axp1xx/AXP1xxPwron.hpp"
#include "../axp1xx/AXP1xxTimer.hpp"
#include "../axp1xx/AXP1xxLed.hpp"

static constexpr uint8_t AXP202_SLAVE_ADDRESS = 0x35;

/**
 * @brief Unified AXP202 PMIC driver
 *
 * Wraps all AXP202 sub-modules and provides a single entry point for
 * initialisation, status query, and system control (shutdown, sleep).
 */
class PmicAXP202 : public PmicBase
{
public:
    /**
     * @brief Construct the AXP202 driver with all sub-modules
     */
    PmicAXP202() : _core(), _adc(_core)
        , _irq(_core), _charger(_core)
        , _power(_core)
        , _pwron(_core)
        , _timer(_core)
        , _led(_core)
        , _gpio(_core), _channel(_core)
        , _powerEx(_core), _chargerEx(_core), _coulomb(_core)
    {}

#if defined(ARDUINO)
    /**
     * @brief Initialise the PMIC using Arduino TwoWire
     * @param wire TwoWire bus reference
     * @param addr I2C slave address (default 0x35)
     * @param sda SDA pin (-1 for default)
     * @param scl SCL pin (-1 for default)
     * @return true on success
     */
    bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(wire, addr, sda, scl);
    }
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialise the PMIC using ESP-IDF legacy I2C driver
     * @param port_num I2C port number
     * @param addr I2C slave address
     * @param sda SDA pin (-1 for default)
     * @param scl SCL pin (-1 for default)
     * @return true on success
     */
    bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(port_num, addr, sda, scl);
    }
#else
    /**
     * @brief Initialise the PMIC using ESP-IDF new I2C driver
     * @param handle I2C master bus handle
     * @param addr I2C slave address
     * @return true on success
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr)
    {
        return _core.begin(handle, addr);
    }
#endif
#endif

    /**
     * @brief Initialise the PMIC using custom communication callbacks
     * @param callback Custom I2C callback function
     * @param hal_cb Custom HAL-level callback
     * @param addr I2C slave address
     * @return true on success
     */
    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_cb,
               uint8_t addr)
    {
        return _core.begin(callback, hal_cb, addr);
    }

    /**
     * @brief De-initialise the PMIC (no-op)
     */
    void end()
    {
    }

    /**
     * @brief Get the PMIC capability flags
     * @return Bitmask of PmicCapability::Capability values
     */
    PmicCapability::Capability getCapabilities() const override
    {
        return PmicCapability::Capability::PmicSupportCharger |
               PmicCapability::Capability::PmicSupportPower |
               PmicCapability::Capability::PmicSupportAdc |
               PmicCapability::Capability::PmicSupportIrq |
               PmicCapability::Capability::PmicSupportGpio |
               PmicCapability::Capability::PmicSupportLed |
               PmicCapability::Capability::PmicSupportPowerBtn |
               PmicCapability::Capability::PmicSupportChannel;
    }

    /**
     * @brief Get the PMIC configuration descriptor
     * @return Reference to a static PmicConfig with chip name, address,
     *         chip ID register/value, channel count, and capabilities
     */
    const PmicConfig &getConfig() const override
    {
        static const PmicConfig config = {
            .chipName = "AXP202",
            .i2cAddress = 0x35,
            .chipIdReg = 0x03,
            .chipIdValue = 0x41,
            .channelCount = 7,
            .capabilities = PmicCapability::Capability::PmicSupportCharger |
            PmicCapability::Capability::PmicSupportPower |
            PmicCapability::Capability::PmicSupportAdc |
            PmicCapability::Capability::PmicSupportIrq |
            PmicCapability::Capability::PmicSupportGpio |
            PmicCapability::Capability::PmicSupportLed |
            PmicCapability::Capability::PmicSupportPowerBtn |
            PmicCapability::Capability::PmicSupportChannel,
        };
        return config;
    }

    // ---- Interface accessors (PmicBase contract) ----

    /**
     * @brief Access the power management interface
     * @return Reference to AXP1xxPower
     */
    PmicPowerBase &power() override
    {
        return _power;
    }

    /**
     * @brief Access the charger interface
     * @return Pointer to AXP1xxCharger, or nullptr if unsupported
     */
    PmicChargerBase *getCharger() override
    {
        return &_charger;
    }

    /**
     * @brief Access the ADC interface
     * @return Reference to AXP202Adc
     */
    PmicAdcBase &adc() override
    {
        return _adc;
    }

    PmicGpioBase *getGpio() override
    {
        return &_gpio;
    }

    /**
     * @brief Access the LED control interface
     * @return Reference to AXP1xxLed
     */
    PmicLedBase &led() override
    {
        return _led;
    }

    /**
     * @brief Access the power channel control interface
     * @return Pointer to AXP202Channel instance
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
     * @brief Get the chip name string
     * @return "AXP202"
     */
    const char *getChipName() const override
    {
        return "AXP202";
    }

    // ---- Chip-specific module accessors ----

    /**
     * @brief Access the IRQ (interrupt) interface
     * @return Reference to AXP1xxIrq<axp202_regs>
     */
    AXP1xxIrq<axp202_regs> &irq()
    {
        return _irq;
    }

    /**
     * @brief Access the countdown timer subsystem
     * @return Reference to AXP1xxTimer<axp202_regs>
     */
    AXP1xxTimer<axp202_regs> &timer()
    {
        return _timer;
    }

    /**
     * @brief Access the charger interface (chip-specific)
     * @return Reference to AXP1xxCharger<axp202_regs>
     */
    AXP1xxCharger<axp202_regs> &charger()
    {
        return _charger;
    }

    /**
     * @brief Access the GPIO interface
     * @return Reference to AXP202Gpio
     */
    AXP202Gpio &gpio()
    {
        return _gpio;
    }

    /**
     * @brief Access the ADC interface (chip-specific)
     * @return Reference to AXP202Adc
     */
    AXP202Adc &chipAdc()
    {
        return _adc;
    }

    /**
     * @brief Access the core communication interface
     * @return Reference to AXP202Core
     */
    AXP202Core &core()
    {
        return _core;
    }

    /**
     * @brief Access AXP202-specific power extensions (DCDC mode, APS warning)
     * @return Reference to AXP202PowerEx
     */
    AXP202PowerEx &powerEx()
    {
        return _powerEx;
    }

    /**
     * @brief Access AXP202-specific charger extensions (thermal, backup, timeout)
     * @return Reference to AXP202ChargerEx
     */
    AXP202ChargerEx &chargerEx()
    {
        return _chargerEx;
    }

    /**
     * @brief Access coulomb counter for charge/discharge energy measurement
     * @return Reference to AXP202Coulomb
     */
    AXP202Coulomb &coulomb()
    {
        return _coulomb;
    }

    /**
     * @brief Access the power-button timing/control interface
     * @return Reference to AXP1xxPwron<axp202_regs>
     */
    AXP1xxPwron<axp202_regs> &pwron()
    {
        return _pwron;
    }

    // ---- Status & basic info ----

    /**
     * @brief Read combined STATUS and MODE_CHGSTATUS registers
     * @return 16-bit value: high byte = STATUS (0x00), low byte = MODE_CHGSTATUS (0x01)
     */
    uint16_t getStatus()
    {
        int s0 = _core.readReg(axp202_regs::bmu::STATUS);
        int s1 = _core.readReg(axp202_regs::bmu::MODE_CHGSTATUS);
        if (s0 < 0 || s1 < 0) return 0;
        return (static_cast<uint16_t>(s0) << 8) | static_cast<uint16_t>(s1);
    }

    /**
     * @brief Check if VBUS is present and valid
     * @return true if VBUS is good (STATUS register bit 5)
     */
    bool isVbusGood()
    {
        return _core.getRegBit(axp202_regs::bmu::STATUS, 5);
    }

    /**
     * @brief Check if a battery is connected
     * @return true if battery is detected (MODE_CHGSTATUS register bit 5)
     */
    bool isBatteryConnect()
    {
        return _core.getRegBit(axp202_regs::bmu::MODE_CHGSTATUS, 5);
    }

    /**
     * @brief Check if the battery is currently charging
     * @return true if charging
     */
    bool isCharging()
    {
        return _charger.isCharging();
    }

    /**
     * @brief Read the AXP202 chip ID register
     * @return Chip ID byte (expected 0x41), or 0 on error
     */
    uint8_t getChipID()
    {
        int id = _core.readReg(axp202_regs::bmu::IC_TYPE);
        if (id < 0) return 0;
        return static_cast<uint8_t>(id);
    }

    // ---- System control ----

    /**
     * @brief Power off the PMIC (shutdown)
     *
     * Sets the OFF_CTL register bit 7 to initiate a system power-down.
     * The PMIC will remain off until the power key is pressed or VBUS
     * is applied.
     */
    void shutdown()
    {
        _core.setRegBit(axp202_regs::pmu::OFF_CTL, 7);
    }

    /**
     * @brief Enable or disable sleep mode
     * @param en true to enter sleep, false to wake
     * @return true on success
     *
     * @note Sleep mode is controlled via VOFF_SET register bit 3.
     */
    bool enableSleep(bool en)
    {
        return en ? _core.setRegBit(axp202_regs::pmu::VOFF_SET, 3)
               : _core.clrRegBit(axp202_regs::pmu::VOFF_SET, 3);
    }

private:
    /**
     * @brief Const access to core (internal use)
     */
    const AXP202Core &core() const
    {
        return _core;
    }

    AXP202Core _core;
    AXP202Adc _adc;
    AXP1xxIrq<axp202_regs> _irq;
    AXP1xxCharger<axp202_regs> _charger;
    AXP1xxPower<axp202_regs> _power;
    AXP1xxPwron<axp202_regs> _pwron;
    AXP1xxTimer<axp202_regs> _timer;
    AXP1xxLed<axp202_regs> _led;
    AXP202Gpio _gpio;
    AXP202Channel _channel;
    AXP202PowerEx _powerEx;
    AXP202ChargerEx _chargerEx;
    AXP202Coulomb _coulomb;
};

/** @brief Convenience alias: AXP202Irq::IRQ_ACIN_OV etc. */
using AXP202Irq = AXP1xxIrq<axp202_regs>;

/** @brief Convenience alias: AXP202Timer::setTimer() etc. */
using AXP202Timer = AXP1xxTimer<axp202_regs>;

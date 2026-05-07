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
 * @file      PmicAXP192.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP192 PMIC Unified Interface
 *
 * Top-level driver class that aggregates all AXP192 subsystems into a single
 * interface: core communication, ADC, GPIO, power channels, charging, IRQ,
 * power-on control, timer, LED, and system power management. Uses shared
 * AXP1xx template classes (AXP1xxIrq, AXP1xxCharger, AXP1xxPower,
 * AXP1xxPwron, AXP1xxTimer, AXP1xxLed) for common PMIC functionality.
 *
 * @section Features
 * - 3x DC-DC converters + 2x LDO + LDOio + EXTEN
 * - Battery charging with configurable current/voltage
 * - 12-bit ADC with multiple measurement channels
 * - GPIO control (pins 0-5)
 * - Configurable power-on/power-off behavior
 * - Countdown timer
 * - Interrupt support (5 groups)
 * - Ship mode and soft shutdown
 *
 * @see AXP192Core.hpp
 * @see AXP192Adc.hpp
 * @see AXP192Gpio.hpp
 * @see AXP192Channel.hpp
 * @see AXP1xxCharger
 * @see AXP1xxPower
 * @see AXP1xxIrq
 */
#pragma once

#include "../../PmicBase.hpp"
#include "../../PmicAdcBase.hpp"
#include "../../PmicIrqBase.hpp"
#include "../../PmicPowerBase.hpp"

#include "AXP192Regs.hpp"
#include "AXP192Core.hpp"
#include "AXP192Adc.hpp"
#include "../axp1xx/AXP1xxIrq.hpp"
#include "../axp1xx/AXP1xxCharger.hpp"
#include "AXP192Gpio.hpp"
#include "AXP192Channel.hpp"

#include "../axp1xx/AXP1xxPower.hpp"
#include "../axp1xx/AXP1xxPwron.hpp"
#include "../axp1xx/AXP1xxTimer.hpp"
#include "../axp1xx/AXP1xxLed.hpp"

static constexpr uint8_t AXP192_SLAVE_ADDRESS = 0x34;

/**
 * @brief AXP192 PMIC Unified Interface
 *
 * Top-level driver that composes all AXP192 functional blocks and exposes
 * them through a consistent API. Supports begin() with Arduino Wire,
 * ESP-IDF I2C legacy, ESP-IDF I2C new driver, or custom communication
 * wrappers.
 *
 * @note Charger, power, IRQ, power-on, timer, and LED implementations
 *       are shared via the axp1xx template family parameterised on the
 *       axp192_regs register map.
 */
class PmicAXP192 : public PmicBase
{
public:
    PmicAXP192() : _core(), _adc(_core)
        , _irq(_core), _charger(_core)
        , _led(_core)
        , _power(_core)
        , _gpio(_core), _pwron(_core)
        , _timer(_core)
        , _channel(_core)
    {}

#if defined(ARDUINO)
    /**
     * @brief Initialise the AXP192 with Arduino Wire interface
     * @param wire Reference to TwoWire bus object
     * @param addr I2C slave address (default 0x34)
     * @param sda SDA pin number (-1 for default)
     * @param scl SCL pin number (-1 for default)
     * @return true on success, false on failure
     */
    bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(wire, addr, sda, scl);
    }
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialise the AXP192 with ESP-IDF legacy I2C driver
     * @param port_num I2C port number
     * @param addr I2C slave address (default 0x34)
     * @param sda SDA pin number (-1 for default)
     * @param scl SCL pin number (-1 for default)
     * @return true on success, false on failure
     */
    bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(port_num, addr, sda, scl);
    }
#else
    /**
     * @brief Initialise the AXP192 with ESP-IDF new I2C driver
     * @param handle I2C master bus handle
     * @param addr I2C slave address (default 0x34)
     * @return true on success, false on failure
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr)
    {
        return _core.begin(handle, addr);
    }
#endif
#endif

    /**
     * @brief Initialise the AXP192 with custom communication callbacks
     * @param callback Custom I2C read/write callback
     * @param hal_cb Custom HAL callback
     * @param addr I2C slave address
     * @return true on success, false on failure
     */
    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_cb,
               uint8_t addr)
    {
        return _core.begin(callback, hal_cb, addr);
    }

    void end()
    {
    }

    /**
     * @brief Get supported capabilities bitmask
     * @return Capability flags indicating supported features
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
     * @brief Get PMIC configuration structure
     * @return Const reference to a static PmicConfig with AXP192 parameters
     */
    const PmicConfig &getConfig() const override
    {
        static const PmicConfig config = {
            .chipName = "AXP192",
            .i2cAddress = 0x34,
            .chipIdReg = 0x03,
            .chipIdValue = 0x03,
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

    /**
     * @brief Access the power management interface
     * @return Reference to AXP1xxPower instance
     */
    PmicPowerBase &power() override
    {
        return _power;
    }

    /**
     * @brief Access the charger interface
     * @return Pointer to AXP1xxCharger instance
     */
    PmicChargerBase *getCharger() override
    {
        return &_charger;
    }

    /**
     * @brief Access the ADC interface
     * @return Reference to AXP192Adc instance
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
     * @brief Access the LED interface
     * @return Reference to AXP1xxLed instance
     */
    PmicLedBase &led() override
    {
        return _led;
    }

    /**
     * @brief Access the power channel control interface
     * @return Pointer to AXP192Channel instance
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
     * @return Pointer to "AXP192"
     */
    const char *getChipName() const override
    {
        return "AXP192";
    }

    /**
     * @brief Access the IRQ subsystem
     * @return Reference to AXP1xxIrq instance
     */
    AXP1xxIrq<axp192_regs> &irq()
    {
        return _irq;
    }

    /**
     * @brief Access the charger subsystem
     * @return Reference to AXP1xxCharger instance
     */
    AXP1xxCharger<axp192_regs> &charger()
    {
        return _charger;
    }

    /**
     * @brief Access the LED module
     * @return Reference to AXP1xxLed instance
     */
    AXP1xxLed<axp192_regs> &ledModule()
    {
        return _led;
    }

    /**
     * @brief Access the GPIO interface
     * @return Reference to AXP192Gpio instance
     */
    AXP192Gpio &gpio()
    {
        return _gpio;
    }

    /**
     * @brief Access the power-on control subsystem
     * @return Reference to AXP1xxPwron instance
     */
    AXP1xxPwron<axp192_regs> &pwron()
    {
        return _pwron;
    }

    /**
     * @brief Access the countdown timer subsystem
     * @return Reference to AXP1xxTimer instance
     */
    AXP1xxTimer<axp192_regs> &timer()
    {
        return _timer;
    }

    /**
     * @brief Access the chip-specific ADC interface
     * @return Reference to AXP192Adc instance
     */
    AXP192Adc &chipAdc()
    {
        return _adc;
    }

    /**
     * @brief Access the core communication object
     * @return Reference to AXP192Core instance
     */
    AXP192Core &core()
    {
        return _core;
    }

    /**
     * @brief Read combined status registers (STATUS + MODE_CHGSTATUS)
     * @return 16-bit value: high byte = STATUS (0x00), low byte = MODE_CHGSTATUS (0x01)
     */
    uint16_t getStatus()
    {
        int s0 = _core.readReg(axp192_regs::bmu::STATUS);
        int s1 = _core.readReg(axp192_regs::bmu::MODE_CHGSTATUS);
        if (s0 < 0 || s1 < 0) return 0;
        return (static_cast<uint16_t>(s0) << 8) | static_cast<uint16_t>(s1);
    }

    /**
     * @brief Check if VBUS power is present and good
     * @return true if VBUS is good
     */
    bool isVbusGood()
    {
        return _core.getRegBit(axp192_regs::bmu::STATUS, 5);
    }

    /**
     * @brief Check if a battery is connected
     * @return true if battery is detected
     */
    bool isBatteryConnect()
    {
        return _core.getRegBit(axp192_regs::bmu::MODE_CHGSTATUS, 5);
    }

    /**
     * @brief Check if the battery is currently charging
     * @return true if charging, false otherwise
     */
    bool isCharging()
    {
        return _charger.isCharging();
    }

    /**
     * @brief Read the AXP192 chip identification register
     * @return Chip ID byte (expected 0x03), or 0 on read error
     */
    uint8_t getChipID()
    {
        int id = _core.readReg(axp192_regs::bmu::IC_TYPE);
        if (id < 0) return 0;
        return static_cast<uint8_t>(id);
    }

    /**
     * @brief Shut down the AXP192 (soft power-off)
     *
     * Sets the OFF bit to put the PMIC into shutdown mode. The system
     * will remain off until the power key is pressed or VBUS is reapplied.
     */
    void shutdown()
    {
        _core.setRegBit(axp192_regs::pmu::OFF_CTL, 7);
    }

    /**
     * @brief Reset the AXP192
     *
     * Triggers a system reset by setting the PEK reset bit in OFF_CTL.
     */
    void reset()
    {
        _core.setRegBit(axp192_regs::pmu::OFF_CTL, 3);
    }

private:
    /**
     * @brief Const accessor for core (private)
     */
    const AXP192Core &core() const
    {
        return _core;
    }

    AXP192Core _core;
    AXP192Adc _adc;
    AXP1xxIrq<axp192_regs> _irq;
    AXP1xxCharger<axp192_regs> _charger;
    AXP1xxLed<axp192_regs> _led;
    AXP1xxPower<axp192_regs> _power;
    AXP192Gpio _gpio;
    AXP1xxPwron<axp192_regs> _pwron;
    AXP1xxTimer<axp192_regs> _timer;
    AXP192Channel _channel;
};

/** @brief Convenience alias: AXP192Irq::IRQ_ACIN_OV etc. */
using AXP192Irq = AXP1xxIrq<axp192_regs>;

/** @brief Convenience alias: AXP192Timer::setTimer() etc. */
using AXP192Timer = AXP1xxTimer<axp192_regs>;

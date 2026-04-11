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
 * @file      PmicSY6970.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 * @brief SY6970 PMIC Unified Interface
 *
 * This is the main facade class for the Silergy SY6970 PMIC, providing
 * access to all PMIC subsystems through a unified interface.
 *
 * @see SY6970Charger for charging functions
 * @see SY6970Power for power management
 * @see SY6970Adc for ADC functions
 * @see SY6970Core for low-level access
 * @see SY6970Regs.hpp for register definitions
 */
#pragma once

#include "../../PmicBase.hpp"
#include "../../PmicPowerBase.hpp"
#include "../../PmicAdcBase.hpp"

#include "SY6970Core.hpp"
#include "SY6970Charger.hpp"
#include "SY6970Power.hpp"
#include "SY6970Adc.hpp"
#include "SY6970Led.hpp"

/**
 * @brief SY6970 I2C slave address
 *
 * The 7-bit I2C address of the SY6970 PMIC.
 */
static constexpr uint8_t SY6970_SLAVE_ADDRESS = 0x6A;

class PmicSY6970 : public PmicBase
{
public:
    /**
     * @brief Construct SY6970 PMIC interface
     *
     * Initializes the internal core and sub-modules.
     * Sub-modules (charger, power, adc) are constructed with
     * reference to the shared core.
     */
    PmicSY6970() : _core(), _charger(_core), _power(_core), _adc(_core), _led(_core) {}

#if defined(ARDUINO)
    /**
     * @brief Initialize using Arduino Wire interface
     * @param wire Reference to TwoWire (Wire, Wire1, etc.)
     * @param addr I2C address (default: SY6970_SLAVE_ADDRESS)
     * @param sda SDA pin (-1 = default)
     * @param scl SCL pin (-1 = default)
     * @return true on success
     */
    bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(wire, addr, sda, scl);
    }
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialize using ESP-IDF legacy I2C API
     * @param port_num I2C port number
     * @param addr I2C address (default: SY6970_SLAVE_ADDRESS)
     * @param sda SDA GPIO number (-1 = default)
     * @param scl SCL GPIO number (-1 = default)
     * @return true on success
     */
    bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(port_num, addr, sda, scl);
    }
#else
    /**
     * @brief Initialize using ESP-IDF new I2C master API
     * @param handle I2C master bus handle
     * @param addr I2C address (default: SY6970_SLAVE_ADDRESS)
     * @return true on success
     *
     * @note Requires idf version > 5.0.0 with esp_system I2C master
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr)
    {
        return _core.begin(handle, addr);
    }
#endif
#endif

    /**
     * @brief Initialize using custom I2C callback
     *
     * For custom I2C implementations (bit-banging, SPI-I2C bridge, etc.)
     *
     * @param callback Custom read/write callback function
     * @param addr I2C address (default: SY6970_SLAVE_ADDRESS)
     * @return true on success
     */
    bool begin(SensorCommCustom::CustomCallback callback, uint8_t addr)
    {
        return _core.begin(callback, addr);
    }

    /**
     * @brief Deinitialize and release resources
     *
     * Calls end() on the core to perform cleanup.
     */
    void end()
    {
        _core.end();
    }

    /**
     * @brief Get supported PMIC capabilities
     * @return Bitmask of PmicCapability::Capability flags
     *
     * @note SY6970 supports: CHARGER, POWER, ADC
     */
    PmicCapability::Capability getCapabilities() const override
    {
        return PmicCapability::Capability::PMIC_CHARGER | PmicCapability::Capability::PMIC_POWER | PmicCapability::Capability::PMIC_ADC;
    }

    /**
     * @brief Get charger interface
     * @return Reference to SY6970Charger
     *
     * Use for:
     * - enableCharging(bool)
     * - setPreChargeCurrent(uint16_t mA) [quantized to 64mA step]
     * - setFastChargeCurrent(uint16_t mA) [quantized to 64mA step]
     * - setTerminationCurrent(uint16_t mA) [quantized to 64mA step]
     * - setChargeVoltage(uint16_t mV) [quantized to 16mV step]
     * - getStatus()
     */
    SY6970Charger &charger()
    {
        return _charger;
    }

    /**
     * @brief Get charger interface (base class pointer)
     * @return Pointer to PmicChargerBase
     */
    PmicChargerBase *getCharger() override
    {
        return &_charger;
    }

    /**
     * @brief Get power management interface
     * @return Reference to SY6970Power
     *
     * Use for:
     * - setMinimumSystemVoltage(uint32_t mv) [quantized to 100mV step]
     * - setInputVoltageLimit(uint32_t mv) [quantized to 100mV step]
     * - setInputCurrentLimit(uint32_t mA) [quantized to 50mA step]
     * - enableBoost(bool)
     * - setBoostVoltage(uint16_t mv) [quantized to 64mV step]
     */
    PmicPowerBase &power() override
    {
        return _power;
    }

    /**
     * @brief Get ADC interface
     * @return Reference to SY6970Adc
     *
     * Use for:
     * - read(Channel::VBAT_VOLTAGE, float &)
     * - read(Channel::VSYS_VOLTAGE, float &)
     * - read(Channel::VBUS_VOLTAGE, float &)
     * - read(Channel::BAT_CURRENT, float &)
     * - read(Channel::BAT_TEMPERATURE, float &)
     * - setContinuousMode(bool)
     */
    PmicAdcBase &adc() override
    {
        return _adc;
    }

    /**
     * @brief Get LED interface
     *
     * Controls the STAT pin for charging status indication.
     *
     * @code
     * // Disable STAT LED
     * pmic.led().setMode(PmicLedBase::Mode::DISABLE);
     *
     * // Enable STAT LED (hardware-controlled)
     * pmic.led().setMode(PmicLedBase::Mode::AUTO);
     * @endcode
     *
     * @see PmicLedBase for available methods
     * @see SY6970Led for chip-specific implementation
     */
    PmicLedBase &led() override
    {
        return _led;
    }

    /**
     * @brief Get device name
     * @return "SY6970"
     */
    const char *getChipName() const override
    {
        return "SY6970";
    }

    /**
     * @brief Get core interface (mutable)
     * @return Reference to SY6970Core
     *
     * For direct register access and status reading.
     */
    SY6970Core &core()
    {
        return _core;
    }

    /**
     * @brief Get core interface (const)
     * @return Const reference to SY6970Core
     */
    const SY6970Core &core() const
    {
        return _core;
    }

private:
    SY6970Core _core;
    SY6970Charger _charger;
    SY6970Power _power;
    SY6970Adc _adc;
    SY6970Led _led;
};

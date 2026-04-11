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
 * @file      PmicBQ25896.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-09
 *
 * @brief BQ25896 PMIC Unified Interface
 *
 * This is the main facade class for the TI BQ25896 PMIC, providing
 * access to all PMIC subsystems through a unified interface.
 *
 * @section Device Overview
 * The BQ25896 is a highly integrated USB BC1.2 compliant charger with:
 * - High-efficiency buck charger (up to 3A fast charge)
 * - USB BC1.2 and HVDCP detection support
 * - Input Current Optimizer (ICO) for adapter detection
 * - Integrated 10-bit ADC for voltage/current monitoring
 * - OTG (On-The-Go) boost mode for power bank operation
 * - JEITA battery temperature protection
 *
 * @section Usage
 * The PmicBQ25896 class aggregates four sub-modules:
 * - Charger: Pre-charge, fast charge, termination, voltage control
 * - Power: System voltage, input limits, OTG/boost mode
 * - ADC: Battery, system, VBUS, current, NTC readings
 * - Core: Low-level register access and status
 *
 * @note All setter methods with step requirements will automatically quantize
 *       input values to the nearest valid step if they do not match exactly.
 *       Use getter methods to read back the actual configured values.
 *
 * @see BQ25896Charger for charging functions
 * @see BQ25896Power for power management
 * @see BQ25896Adc for ADC functions
 * @see BQ25896Core for low-level access
 * @see BQ25896Regs.hpp for register definitions
 * @see BQ25896RegisterMap.md for detailed register descriptions
 */
#pragma once

#include "../../PmicBase.hpp"
#include "../../PmicPowerBase.hpp"
#include "../../PmicAdcBase.hpp"

#include "BQ25896Core.hpp"
#include "BQ25896Charger.hpp"
#include "BQ25896Power.hpp"
#include "BQ25896Adc.hpp"
#include "BQ25896Led.hpp"

/**
 * @brief BQ25896 I2C slave address
 *
 * The 7-bit I2C address of the BQ25896 PMIC.
 * In 8-bit form (for Arduino), use (BQ25896_SLAVE_ADDRESS << 1).
 */
static constexpr uint8_t BQ25896_SLAVE_ADDRESS = 0x6B;

class PmicBQ25896 : public PmicBase
{
public:
    /**
     * @brief Construct BQ25896 PMIC interface
     *
     * Initializes the internal core and sub-modules.
     * Sub-modules (charger, power, adc) are constructed with
     * reference to the shared core.
     */
    PmicBQ25896() : _core(), _charger(_core), _power(_core), _adc(_core), _led(_core) {}

#if defined(ARDUINO)
    /**
     * @brief Initialize using Arduino Wire interface
     * @param wire Reference to TwoWire (Wire, Wire1, etc.)
     * @param addr I2C address (default: BQ25896_SLAVE_ADDRESS = 0x6B)
     * @param sda SDA pin (-1 = use default)
     * @param scl SCL pin (-1 = use default)
     * @return true on success
     *
     * @note If sda/scl are -1, uses default pins for the board
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
     * @param addr I2C address (default: BQ25896_SLAVE_ADDRESS = 0x6B)
     * @param sda SDA GPIO number (-1 = use default)
     * @param scl SCL GPIO number (-1 = use default)
     * @return true on success
     *
     * @note If sda/scl are -1, uses default pins for the port
     */
    bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(port_num, addr, sda, scl);
    }
#else
    /**
     * @brief Initialize using ESP-IDF new I2C master API
     * @param handle I2C master bus handle
     * @param addr I2C address (default: BQ25896_SLAVE_ADDRESS = 0x6B)
     * @return true on success
     *
     * @note Requires ESP-IDF version > 5.0.0 with esp_system I2C master
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
     * @param addr I2C address (default: BQ25896_SLAVE_ADDRESS = 0x6B)
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
     * @note BQ25896 supports: CHARGER, POWER, ADC
     */
    PmicCapability::Capability getCapabilities() const override
    {
        return PmicCapability::Capability::PMIC_CHARGER | PmicCapability::Capability::PMIC_POWER | PmicCapability::Capability::PMIC_ADC;
    }

    /**
     * @brief Get charger interface
     * @return Reference to BQ25896Charger
     *
     * Use for:
     * - enableCharging(bool)
     * - setPreChargeCurrent(uint16_t mA) [quantized to 64mA step, 64-1024mA]
     * - setFastChargeCurrent(uint16_t mA) [quantized to 64mA step, 0-3008mA]
     * - setTerminationCurrent(uint16_t mA) [quantized to 64mA step, 64-1024mA]
     * - setChargeVoltage(uint16_t mV) [quantized to 16mV step, 3840-4608mV]
     * - getStatus()
     */
    BQ25896Charger &charger()
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
     * @return Reference to BQ25896Power
     *
     * Use for:
     * - setMinimumSystemVoltage(uint32_t mv) [quantized to 100mV step, 3000-3700mV]
     * - setInputVoltageLimit(uint32_t mv) [quantized to 100mV step, 3900-15300mV]
     * - setInputCurrentLimit(uint32_t mA) [quantized to 50mA step, 100-3250mA]
     * - enableBoost(bool) [requires no VBUS connected]
     * - setBoostVoltage(uint16_t mv) [quantized to 64mV step, 4550-5510mV]
     */
    PmicPowerBase &power() override
    {
        return _power;
    }

    /**
     * @brief Get ADC interface
     * @return Reference to BQ25896Adc
     *
     * Use for:
     * - read(Channel::VBUS_VOLTAGE, float &) - millivolts
     * - read(Channel::BAT_VOLTAGE, float &) - millivolts
     * - read(Channel::VSYS_VOLTAGE, float &) - millivolts
     * - read(Channel::BAT_CURRENT, float &) - milliamps
     * - read(Channel::BAT_TEMPERATURE, float &) - NTC percentage (NOT temperature!)
     * - read(Channel::DIE_TEMPERATURE, float &) - thermal status flag
     * - setContinuousMode(bool)
     *
     * @warning BAT_TEMPERATURE returns percentage (21-80%), NOT temperature!
     *          See BQ25896Adc.hpp documentation for details.
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
     * @see BQ25896Led for chip-specific implementation
     */
    PmicLedBase &led() override
    {
        return _led;
    }

    /**
     * @brief Get device name
     * @return "BQ25896"
     */
    const char *getChipName() const override
    {
        return "BQ25896";
    }

    /**
     * @brief Get core interface (mutable)
     * @return Reference to BQ25896Core
     *
     * For direct register access, status reading, and advanced control.
     * Use this for:
     * - isVbusPresent() - check if USB power is connected
     * - getBusStatus() - get detected input type (SDP/Adapter/OTG)
     * - isVindpmActive() / isIindpmActive() - check dynamic power management
     * - getNtcPercentage() - get NTC percentage (see ADC warning above)
     * - getFaultStatus() - read detailed fault information
     */
    BQ25896Core &core()
    {
        return _core;
    }

    /**
     * @brief Get core interface (const)
     * @return Const reference to BQ25896Core
     */
    const BQ25896Core &core() const
    {
        return _core;
    }

private:
    BQ25896Core _core;
    BQ25896Charger _charger;
    BQ25896Power _power;
    BQ25896Adc _adc;
    BQ25896Led _led;
};

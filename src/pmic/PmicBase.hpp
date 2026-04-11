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
 * @file      PmicBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 * @brief PMIC (Power Management IC) Base Interface
 *
 * This is the base abstract class for all PMIC driver implementations.
 * It provides a unified interface for accessing various PMIC sub-modules
 * such as charger, power management, ADC, GPIO, LED, IRQ, BC12, Type-C, and power-on functionality.
 *
 * @section PMIC Overview
 * PMICs (Power Management ICs) are integrated circuits that manage power requirements
 * for host systems. They typically include:
 * - Battery charging control
 * - Power path management
 * - Voltage regulation
 * - System monitoring via ADC
 * - Protection features
 *
 */
#pragma once

#include "PmicPowerBase.hpp"
#include "PmicChargerBase.hpp"
#include "PmicAdcBase.hpp"
#include "PmicLedBase.hpp"
#include "../platform/SensorCommCustom.hpp"

/**
 * @brief PMIC Capability Flags
 *
 * Used to indicate which features are available in a specific PMIC implementation.
 * These flags help determine which sub-modules can be accessed.
 */
namespace PmicCapability {
enum class Capability : uint32_t {
    PMIC_NONE        = 0,    ///< No capabilities (should not occur)
    PMIC_CHARGER     = (1 << 0),  ///< Battery charger support
    PMIC_POWER       = (1 << 1),  ///< Power management (input limits, boost, etc.)
    PMIC_ADC         = (1 << 2),  ///< Analog-to-Digital converter for monitoring
    PMIC_GPIO        = (1 << 3),  ///< General purpose I/O support
    PMIC_LED         = (1 << 4),  ///< LED driver support
    PMIC_IRQ         = (1 << 5),  ///< Interrupt handling support
    PMIC_BC12        = (1 << 6),  ///< BC1.2 (USB Battery Charging) support
    PMIC_TYPEC       = (1 << 7),  ///< USB Type-C support
    PMIC_PWRON       = (1 << 8),  ///< Power-on/Power-off control support
};

/**
 * @brief Bitwise OR operator for PmicCapability::Capability
 * @param a First capability
 * @param b Second capability
 * @return Combined capability flags
 */
constexpr Capability operator|(Capability a, Capability b)
{
    return static_cast<Capability>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}

/**
 * @brief Bitwise AND operator for PmicCapability::Capability
 * @param a First capability
 * @param b Second capability
 * @return True if both capabilities are set
 */
constexpr bool operator&(Capability a, Capability b)
{
    return (static_cast<uint32_t>(a) & static_cast<uint32_t>(b)) != 0;
}
}

/**
 * @brief PMIC Base Abstract Class
 *
 * This class defines the interface that all PMIC implementations must provide.
 * It handles initialization, provides access to sub-modules, and reports capabilities.
 */
class PmicBase
{
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~PmicBase() = default;

#if defined(ARDUINO)
    /**
     * @brief Initialize PMIC with Arduino TwoWire interface
     * @param wire Reference to TwoWire (I2C) object
     * @param addr I2C slave address of the PMIC
     * @param sda GPIO pin for SDA (default: -1 use board default)
     * @param scl GPIO pin for SCL (default: -1 use board default)
     * @return true on success, false on failure
     */
    virtual bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1) = 0;
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialize PMIC with ESP-IDF legacy I2C API
     * @param port_num I2C port number
     * @param addr I2C slave address of the PMIC
     * @param sda GPIO pin for SDA (default: -1 use board default)
     * @param scl GPIO pin for SCL (default: -1 use board default)
     * @return true on success, false on failure
     */
    virtual bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1) = 0;
#else
    /**
     * @brief Initialize PMIC with ESP-IDF I2C master bus handle
     * @param handle I2C master bus handle
     * @param addr I2C slave address of the PMIC
     * @return true on success, false on failure
     */
    virtual bool begin(i2c_master_bus_handle_t handle, uint8_t addr) = 0;
#endif
#endif

    /**
     * @brief Initialize PMIC with custom communication callback
     * @param callback Custom communication callback for read/write operations
     * @param addr Device address
     * @return true on success, false on failure
     */
    virtual bool begin(SensorCommCustom::CustomCallback callback, uint8_t addr) = 0;

    /**
     * @brief Deinitialize PMIC and release resources
     */
    virtual void end() = 0;

    /**
     * @brief Get PMIC capabilities
     * @return Bitmask of supported capabilities
     *
     * @see PmicCapability::Capability enum for available capability flags
     */
    virtual PmicCapability::Capability getCapabilities() const = 0;

    /**
     * @brief Get power management interface
     * @return Reference to power interface
     */
    virtual PmicPowerBase &power() = 0;

    /**
     * @brief Get charger interface
     * @return Pointer to charger interface, nullptr if not supported
     *
     * @note Returns nullptr if CHARGER capability is not present
     * @see getCapabilities()
     */
    virtual PmicChargerBase *getCharger() = 0;

    /**
     * @brief Get ADC interface
     * @return Reference to ADC interface
     *
     * @note Returns reference to ADC implementation
     * @see PmicAdcBase
     */
    virtual PmicAdcBase &adc() = 0;

    /**
     * @brief Get LED interface
     * @return Reference to LED interface
     *
     * @note Returns reference to LED implementation
     * @see PmicLedBase
     */
    virtual PmicLedBase &led() = 0;

    /**
     * @brief Get PMIC device name
     * @return C-string containing device name (e.g., "BQ25896", "SY6970")
     */
    virtual const char *getChipName() const = 0;
};

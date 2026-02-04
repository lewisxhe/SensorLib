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
 * @file      SensorBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-22
 *
 * @brief Abstract base class template for all sensors
 * @note This class cannot be instantiated directly. Use derived classes such as:
 *       - AccelerometerBase for accelerometers
 *       - GyroscopeBase for gyroscopes
 *       - MagnetometerBase for magnetometers
 */
#pragma once

#include "../SensorPlatform.hpp"
#include "SensorDefs.hpp"
#include <functional>
#include <vector>
#include <cstring>

/**
 * @brief Structure containing sensor configuration parameters
 */
struct SensorConfig {
    float data_rate_hz;      ///< Data output rate in Hertz
    float full_scale_range;  ///< Full-scale range in appropriate units
    float sensitivity;       ///< Sensitivity (units per LSB)
    int16_t x_offset;        ///< X-axis calibration offset
    int16_t y_offset;        ///< Y-axis calibration offset
    int16_t z_offset;        ///< Z-axis calibration offset
    uint16_t fifo_size;      ///< FIFO buffer size (if available)
    uint8_t mode;            ///< Operation mode
    bool interrupt_enabled;  ///< Interrupt enable flag
    bool fifo_enabled;       ///< FIFO enable flag
};

/**
 * @brief Structure containing sensor identification information
 */
struct SensorInfo {
    const char *manufacturer;       ///< Manufacturer name
    const char *model;              ///< Model name/identifier
    uint8_t i2c_address;            ///< Default I2C address
    uint8_t version;                ///< Hardware/firmware version
    uint32_t uid;                   ///< Unique identifier
    SensorType type;                ///< Sensor type
    uint8_t address_count;          ///< Number of supported I2C addresses
    uint8_t *alternate_addresses;   ///< List of alternate I2C addresses
};

/**
 * @brief Utility class providing static sensor-related helper functions
 */
class SensorUtils
{
public:
    /**
     * @brief Convert sensor type enumeration to string representation
     *
     * @param type Sensor type enumeration value
     * @return const char* String representation of the sensor type
     */
    static const char *typeToString(SensorType type)
    {
        switch (type) {
        case SensorType::ACCELEROMETER: return "ACCELEROMETER";
        case SensorType::GYROSCOPE:     return "GYROSCOPE";
        case SensorType::MAGNETOMETER:  return "MAGNETOMETER";
        case SensorType::PRESSURE:      return "PRESSURE";
        case SensorType::TEMPERATURE:   return "TEMPERATURE";
        case SensorType::HUMIDITY:      return "HUMIDITY";
        case SensorType::MULTI_AXIS:    return "MULTI_AXIS";
        default:                        return "UNKNOWN";
        }
    }
};

/**
 * @brief Template base class for all sensor implementations
 *
 * @tparam T Sensor data type (e.g., AccelerometerData, GyroscopeData)
 *
 * @note This is an abstract base class and cannot be instantiated directly.
 *       Derived classes must implement the pure virtual functions.
 *
 * @warning Constructor is protected to prevent direct instantiation
 */
template<typename T>
class SensorBase
{
protected:
    /**
     * @brief Construct a new SensorBase object
     *
     * @param sensor_type Type of sensor (default: UNKNOWN)
     *
     * @note Constructor is protected to prevent direct instantiation of abstract base class
     */
    SensorBase(SensorType sensor_type = SensorType::UNKNOWN)
        : comm(nullptr),
          hal(nullptr)
    {
        memset(&_config, 0, sizeof(SensorConfig));
        memset(&_info, 0, sizeof(SensorInfo));
        _info.type = sensor_type;
    }

    /**
     * @brief Virtual destructor for proper polymorphic cleanup
     */
    virtual ~SensorBase() = default;

public:
    // Delete copy operations to prevent slicing
    SensorBase(const SensorBase &) = delete;
    SensorBase &operator=(const SensorBase &) = delete;

#if defined(ARDUINO)
    /**
     * @brief Initialize sensor for Arduino platform using I2C
     *
     * @param wire    TwoWire I2C interface instance
     * @param addr    I2C device address
     * @param sda     SDA pin number (-1 for default)
     * @param scl     SCL pin number (-1 for default)
     * @return true   Initialization successful
     * @return false  Initialization failed
     */
    virtual bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
            return false;
        }
        return initImpl(addr);
    }

#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialize sensor for ESP-IDF platform (legacy I2C driver)
     *
     * @param port_num I2C port number
     * @param addr     I2C device address
     * @param sda      SDA pin number (-1 for default)
     * @param scl      SCL pin number (-1 for default)
     * @return true    Initialization successful
     * @return false   Initialization failed
     */
    virtual bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
            return false;
        }
        return initImpl(addr);
    }
#else
    /**
     * @brief Initialize sensor for ESP-IDF platform (I2C master driver)
     *
     * @param handle   I2C master bus handle
     * @param addr     I2C device address
     * @return true    Initialization successful
     * @return false   Initialization failed
     */
    virtual bool begin(i2c_master_bus_handle_t handle, uint8_t addr)
    {
        // Note: sda and scl parameters removed as they're configured in the handle
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr, -1, -1)) {
            return false;
        }
        return initImpl(addr);
    }
#endif
#endif

    /**
     * @brief Initialize sensor using custom communication callbacks
     *
     * @param callback     Custom read/write callback function
     * @param hal_callback Custom hardware abstraction callback
     * @param addr         I2C device address
     * @return true        Initialization successful
     * @return false       Initialization failed
     */
    virtual bool begin(SensorCommCustom::CustomCallback callback,
                       SensorCommCustomHal::CustomHalCallback hal_callback,
                       uint8_t addr)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(
                    COMM_CUSTOM, callback, hal_callback, addr, comm, hal)) {
            return false;
        }
        return initImpl(addr);
    }

    /**
     * @brief Set sensor calibration offsets
     *
     * @param x X-axis offset
     * @param y Y-axis offset
     * @param z Z-axis offset
     */
    void setOffset(int16_t x, int16_t y, int16_t z)
    {
        _config.x_offset = x;
        _config.y_offset = y;
        _config.z_offset = z;
    }

    /**
     * @brief Get sensor sensitivity
     *
     * @return float Sensitivity value
     */
    float getSensitivity() const
    {
        return _config.sensitivity;
    }

    /**
     * @brief Get sensor information
     *
     * @return SensorInfo Sensor information structure
     */
    SensorInfo getSensorInfo() const
    {
        return _info;
    }

    // Pure virtual interface functions

    /**
     * @brief Read data from sensor
     *
     * @param data Reference to store read data
     * @return true  Read successful
     * @return false Read failed
     */
    virtual bool readData(T &data) = 0;

    /**
     * @brief Check if new data is available
     *
     * @return true  New data available
     * @return false No new data
     */
    virtual bool isDataReady() = 0;

    /**
     * @brief Reset sensor to default state
     *
     * @return true  Reset successful
     * @return false Reset failed
     */
    virtual bool reset() = 0;

    /**
     * @brief Perform sensor self-test
     *
     * @return true  Self-test passed
     * @return false Self-test failed
     */
    virtual bool selfTest() = 0;

    /**
     * @brief Get current sensor configuration
     *
     * @return SensorConfig Current configuration
     */
    virtual SensorConfig getConfig() const
    {
        return _config;
    }

private:
    /**
     * @brief Sensor-specific initialization implementation
     *
     * @param addr I2C address
     * @return true  Initialization successful
     * @return false Initialization failed
     */
    virtual bool initImpl(uint8_t addr) = 0;

protected:
    std::unique_ptr<SensorCommBase> comm;   ///< Communication interface
    std::unique_ptr<SensorHal> hal;         ///< Hardware abstraction layer
    SensorConfig _config;                   ///< Current configuration
    SensorInfo _info;                       ///< Sensor information
};

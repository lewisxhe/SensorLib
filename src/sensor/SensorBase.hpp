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
 */
#pragma once

#include "../SensorPlatform.hpp"
#include <functional>
#include <vector>

struct SensorVector {
    float x;
    float y;
    float z;
};

struct RawVector {
    int16_t x;
    int16_t y;
    int16_t z;
};

enum class SensorType {
    UNKNOWN = 0,
    ACCELEROMETER,
    GYROSCOPE,
    MAGNETOMETER,
    PRESSURE,
    TEMPERATURE,
    HUMIDITY,
    MULTI_AXIS
};

struct SensorConfig {
    uint8_t mode;                   // Operation mode
    float data_rate_hz;             // Data output rate (Hz)
    uint8_t full_scale_range;       // Full-scale range
    int16_t x_offset;               // X-axis offset
    int16_t y_offset;               // Y-axis offset
    int16_t z_offset;               // Z-axis offset
    float sensitivity;              // Sensitivity
    bool interrupt_enabled;         // Interrupt enabled
    bool fifo_enabled;              // FIFO enabled
    uint16_t fifo_size;             // FIFO size
};

struct SensorInfo {
    const char *manufacturer;       // Manufacturer
    const char *model;              // Model
    uint8_t i2c_address;            // I2C address
    uint8_t version;                // Version
    uint32_t uid;                   // Unique ID
    SensorType type;                // Sensor type
    uint8_t address_count;          // Supported address count
    uint8_t *alternate_addresses;   // Alternate address list
};

// *INDENT-OFF*
class SensorUtils
{
public:
    /**
     * @brief  Convert sensor type to string
     * @note   This function converts the sensor type enumeration to a string representation.
     * @param  type: The sensor type enumeration value.
     * @retval A string representation of the sensor type.
     */
    static const char *typeToString(SensorType type)
    {
        switch (type) {
        case SensorType::ACCELEROMETER: return "ACCELEROMETER";
        case SensorType::GYROSCOPE: return "GYROSCOPE";
        case SensorType::MAGNETOMETER: return "MAGNETOMETER";
        case SensorType::PRESSURE: return "PRESSURE";
        case SensorType::TEMPERATURE: return "TEMPERATURE";
        case SensorType::HUMIDITY: return "HUMIDITY";
        case SensorType::MULTI_AXIS: return "MULTI_AXIS";
        default: return "UNKNOWN";
        }
    }
};
// *INDENT-ON*


template<typename T>
class SensorBase
{
public:

    using DataReadyCallback = std::function<void(const T &)>;

    /**
     * @brief  SensorBase constructor.
     * @note   This function initializes the sensor base class.
     * @param  sensor_type: The type of the sensor.
     * @retval None
     */
    SensorBase(SensorType sensor_type = SensorType::UNKNOWN) :  comm(nullptr), hal(nullptr),
        _data_ready_callback(nullptr)
    {
        memset(&_config, 0, sizeof(SensorConfig));
        memset(&_info, 0, sizeof(SensorInfo));
        _info.type = sensor_type;
    }

    /**
     * @brief  Virtual destructor for sensor base class.
     * @note   This function is called when the sensor object is destroyed.
     * @retval None
     */
    virtual ~SensorBase()
    {
        if (comm) {
            comm->deinit();
        }
    }

    /**
     * @brief  Virtual function for sensor-specific initialization.
     * @note   This function is called during the initialization process to perform
     *         any sensor-specific setup.
     * @param  addr: The I2C address of the sensor.
     * @retval True if initialization is successful, false otherwise.
     */
    virtual bool initImpl(uint8_t addr) = 0;

#if defined(ARDUINO)
    /**
      * @brief Initialization function for the Arduino platform. Sets up the sensor over I2C.
      *
      * @param wire Reference to a TwoWire object (typically Wire) for I2C communication.
      * @param addr I2C slave address of the sensor.
      * @param sda I2C SDA pin number. Defaults to -1 (use board's default pin).
      * @param scl I2C SCL pin number. Defaults to -1 (use board's default pin).
      * @return true if initialization is successful, false otherwise.
      */
    virtual bool begin(TwoWire &wire, uint8_t addr, int sda, int scl)
    {
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
            return false;
        }
        return initImpl(addr);
    }
#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialization function for ESP32 platform (legacy I2C mode).
     *
     * @param port_num I2C port number to use for communication.
     * @param addr I2C slave address of the sensor.
     * @param sda I2C SDA pin number. Defaults to -1 (use default pin).
     * @param scl I2C SCL pin number. Defaults to -1 (use default pin).
     * @return true if initialization is successful, false otherwise.
     */
    virtual bool begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
            return false;
        }
        return initImpl(addr);
    }
#else
    /**
     * @brief Initialization function for ESP32 platform (new I2C master mode).
     *
     * @param handle I2C master bus handle for communication.
     * @param addr I2C slave address of the sensor.
     * @return true if initialization is successful, false otherwise.
     *
     * @note Assumes `sda` and `scl` are handled via the bus handle configuration.
     */
    virtual bool begin(i2c_master_bus_handle_t handle, uint8_t addr)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr, sda, scl)) {
            return false;
        }
        return initImpl(addr);
    }
#endif
#endif

    /**
     * @brief Initialization function for custom communication with callback support.
     *
     * @param callback Custom communication callback function for read/write operations.
     * @param hal_callback Custom hardware abstraction callback function (e.g., delays).
     * @param addr I2C slave address of the sensor.
     * @return true if initialization is successful, false otherwise.
     */
    virtual bool begin(SensorCommCustom::CustomCallback callback,
                       SensorCommCustomHal::CustomHalCallback hal_callback,
                       uint8_t addr)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
                callback, hal_callback, addr, comm, hal)) {
            return false;
        }
        return initImpl(addr);
    }

    /**
     * @brief  Set the sensor offset values.
     * @note   This function sets the offset values for the X, Y, and Z axes.
     * @param  x: Offset value for the X axis.
     * @param  y: Offset value for the Y axis.
     * @param  z: Offset value for the Z axis.
     * @retval None
     */
    void setOffset(int16_t  x, int16_t  y, int16_t  z)
    {
        _config.x_offset = x; _config.y_offset = y; _config.z_offset = z;
    }

    /**
     * @brief  Get the sensor offset values.
     * @note   This function retrieves the offset values for the X, Y, and Z axes.
     * @retval None
     */
    float getSensitivity()
    {
        return _config.sensitivity;
    }

    /**
     * @brief  Get the sensor sensitivity.
     * @note   This function retrieves the sensitivity value for the sensor.
     * @retval Sensor sensitivity in Gauss/LSB.
     */
    SensorInfo getSensorInfo()
    {
        return _info;
    }

    /**
     * @brief  Read data from the sensor.
     * @note   This function reads data from the sensor and stores it in the provided reference.
     * @param  &data: Reference to the variable where the read data will be stored.
     * @retval True if the read operation is successful, false otherwise.
     */
    virtual bool readData(T &data) = 0;

    /**
     * @brief  Virtual function to check if new data is available.
     * @note   This function checks the sensor's data ready status.
     * @retval True if new data is available, false otherwise.
     */
    virtual bool isDataReady() = 0;

    /**
     * @brief  Virtual function to reset the sensor.
     * @note   This function resets the sensor to its initial state.
     * @retval True if reset is successful, false otherwise.
     */
    virtual bool reset() = 0;

    /**
     * @brief  Virtual function to perform a self-test on the sensor.
     * @note   This function checks the sensor's functionality and returns the test result.
     * @retval True if self-test is successful, false otherwise.
     */
    virtual bool selfTest() = 0;

    /**
     * @brief  Virtual function to get the sensor configuration.
     * @note   This function retrieves the current configuration of the sensor.
     * @retval The current sensor configuration.
     */
    virtual SensorConfig getConfig() const
    {
        return _config;
    }

    /**
    * @brief  Sets a callback function to be called when new data is available.
    * @note   This function allows the user to register a callback that will be
    *         invoked whenever the magnetometer has new data ready for processing.
    * @param  callback: A callback function to be called when data is ready.
    * @retval None
    */
    void setDataReadyCallback(DataReadyCallback callback)
    {
        _data_ready_callback = callback;
    }

    /**
     * @brief  Enable or disable data ready interrupt.
     * @note   This function enables or disables the data ready interrupt for the sensor.
     * @param  enable: True to enable, false to disable.
     * @retval True if the operation was successful, false otherwise.
     */
    virtual bool enableDataReadyInterrupt(bool enable = true)
    {
        // TODO: Implement enabling/disabling data ready interrupt
        return false;
    }

    /**
     * @brief  Enable or disable FIFO.
     * @note   This function enables or disables the FIFO (First In First Out) feature for the sensor.
     * @param  enable: True to enable, false to disable.
     * @retval True if the operation was successful, false otherwise.
     */
    virtual bool enableFifo(bool enable = true)
    {
        // TODO: Implement enabling/disabling FIFO
        return true;
    }

    /**
     * @brief  Read FIFO data.
     * @note   This function reads data from the FIFO buffer.
     * @param  &data_list: Output list of sensor data.
     * @param  max_count: Maximum number of samples to read.
     * @retval The actual number of samples read.
     */
    virtual uint16_t readFifoData(std::vector<T> &data_list, uint16_t max_count)
    {
        // TODO: Implement reading FIFO data
        return 0;
    }

    /**
     * @brief  Clear FIFO buffer.
     * @note   This function clears the FIFO buffer.
     * @retval True if the operation was successful, false otherwise.
     */
    virtual bool clearFifo()
    {
        return true;
    }

protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
    DataReadyCallback _data_ready_callback;
    SensorConfig _config;
    SensorInfo _info;

    /**
    * @brief  Triggers the data ready callback.
    * @note   This function is called when new magnetometer data is available.
    * @param  data: The magnetometer data.
    * @retval None
    */
    virtual void triggerDataReady(const T& data)
    {
        if (_data_ready_callback) {
            _data_ready_callback(data);
        }
    }

};

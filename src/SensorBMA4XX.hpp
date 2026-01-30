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
 * @file      SensorBMA4XX.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-28
 * @note      BMA4XX class based on [BMA456 Sensor API](https://github.com/boschsensortec/BMA456_SensorAPI)
 *            Simplification for Arduino
 */
#pragma once

#include "SensorPlatform.hpp"
#include "sensor/AccelerometerBase.hpp"
#include "bosch/bma4xx/bma4.h"

/**
 * @note   This address is used when the SDO pin is connected to GND.
 */
static constexpr uint8_t BMA4XX_I2C_ADDR_SDO_LOW = (0x18);
/**
 * @note   This address is used when the SDO pin is connected to VDD.
 */
static constexpr uint8_t BMA4XX_I2C_ADDR_SDO_HIGH = (0x19);


/**
   * @brief BMA4 Accelerometer averaging filter configuration
   * These values configure the internal averaging filter to reduce noise.
   * Higher averaging provides better noise reduction but increases power consumption.
   */
enum class AccelBandwidth : uint8_t {
    OSR4_AVG1    = 0,  ///< OSR4 mode with 1 sample averaging (lowest power, highest noise)
    OSR2_AVG2    = 1,  ///< OSR2 mode with 2 samples averaged
    NORMAL_AVG4  = 2,  ///< Normal mode with 4 samples averaged
    CIC_AVG8     = 3,  ///< CIC filter with 8 samples averaged
    RES_AVG16    = 4,  ///< Resolution mode with 16 samples averaged
    RES_AVG32    = 5,  ///< Resolution mode with 32 samples averaged
    RES_AVG64    = 6,  ///< Resolution mode with 64 samples averaged
    RES_AVG128   = 7   ///< Resolution mode with 128 samples averaged (best noise reduction, highest power)
};

/**
* @brief BMA4 Accelerometer performance mode configuration
* Configures the internal filter behavior for sample averaging.
*/
enum class AccelPerfMode : uint8_t {
    CIC_AVG_MODE     = 0,  ///< Averaging filter mode: samples are averaged based on configured bandwidth and ODR
    CONTINUOUS_MODE  = 1   ///< Continuous mode: no averaging, raw samples with lower latency
};

/**
 * @brief  Tap detection type
 * @note   This enum defines the different types of tap events that can be detected by the sensor.
 * @note   Whether or not such attributes exist depends on the specific sensor or the different feature configurations.
 */
enum class TapType {
    NO_TAP = 0,
    SINGLE_TAP = 1,
    DOUBLE_TAP = 2,
    TRIPLE_TAP = 3,
};

/**
 * @brief  Activity detection type
 * @note   This enum defines the different types of activity events that can be detected by the sensor.
 * @note   Whether or not such attributes exist depends on the specific sensor or the different feature configurations.
 */
enum class ActivityType {
    STATIONARY = 0,
    WALKING = 1,
    RUNNING = 2,
    UNKNOWN = 3,
};

/**
 * @brief Abstract base class for BMA4XX series accelerometer sensors.
 *
 * This class defines the common interface for BMA4XX family accelerometers (BMA422, BMA423, BMA456).
 * It provides core functionality for sensor initialization, data reading, and configuration.
 *
 * @warning This is an abstract base class and cannot be instantiated directly.
 *          You must use one of the derived classes such as:
 *          - SensorBMA422 for BMA422 sensors
 *          - SensorBMA423 for BMA423 sensors
 *          - SensorBMA456 for BMA456 sensors
 *
 * @note Implementation details and sensor-specific features are handled by derived classes.
 *       This class only provides the common interface and shared functionality.
 *
 * @example
 * // Correct usage (through derived class):
 * SensorBMA456 sensor;
 * if (sensor.begin(Wire, BMA4XX_I2C_ADDR_SDO_HIGH)) {
 *     AccelerometerData data;
 *     if (sensor.readData(data)) {
 *         // Process data
 *     }
 * }
 *
 * Incorrect usage (will not compile):
 * SensorBMA4XX sensor;  // Error: Cannot instantiate abstract class
 *
 * @see SensorBMA422, SensorBMA423, SensorBMA456
 */
class SensorBMA4XX : public AccelerometerBase
{
public:

    /**
     * @brief  typedef for BMA4 device structure
     */
    using bma4_dev_t = struct bma4_dev ;

    /**
        * @enum SensorRemap
        * @brief Enumeration representing different remapping options for the sensor's orientation.
        *
        * This enum defines various positions and orientations of the sensor chip. Each value corresponds
        * to a specific corner or location of the chip, which can be used to remap the axes of the sensor
        * according to its physical placement.
        *
        * Top view of the chip, where 'T' stands for top,
        * 'B' stands for bottom,
        * 'L' stands for left, and 'R' stands for right
        *  -------------
        * | TL         TR |
        * |               |
        * |               |
        * |               |
        * | BL         BR |
        *  -------------
        *
        * There is also a bottom view of the chip：
        *
        *  -------------
        * | BT         BB |
        * |               |
        * |               |
        * |               |
        * | LT         RT |
        *  -------------
        */
    enum SensorRemap {
        // Chip top view, upper left corner
        //  -------------
        // | *             |
        // |               |
        // |               |
        // |               |
        // |               |
        //  -------------
        TOP_LAYER_LEFT_CORNER,
        // Chip top view, upper right corner
        //  -------------
        // |             * |
        // |               |
        // |               |
        // |               |
        // |               |
        //  -------------
        TOP_LAYER_RIGHT_CORNER,
        // Chip top view, bottom right corner of the top
        //  -------------
        // |               |
        // |               |
        // |               |
        // |               |
        // |             * |
        //  -------------
        TOP_LAYER_BOTTOM_RIGHT_CORNER,
        // The top view of the chip, the lower left corner of the front bottom
        //  -------------
        // |               |
        // |               |
        // |               |
        // |               |
        // | *             |
        //  -------------
        TOP_LAYER_BOTTOM_LEFT_CORNER,
        // The bottom view of the chip, the upper left corner of the top
        //  -------------
        // | *             |
        // |               |
        // |               |
        // |               |
        // |               |
        //  -------------
        BOTTOM_LAYER_TOP_LEFT_CORNER,
        // The bottom view of the chip, the upper right corner of the top
        //  -------------
        // |             * |
        // |               |
        // |               |
        // |               |
        // |               |
        //  -------------
        BOTTOM_LAYER_TOP_RIGHT_CORNER,
        // The bottom view of the chip, the lower right corner of the bottom
        //  -------------
        // |               |
        // |               |
        // |               |
        // |               |
        // |             * |
        //  -------------
        BOTTOM_LAYER_BOTTOM_RIGHT_CORNER,
        // Chip bottom view, bottom left corner
        //  -------------
        // |               |
        // |               |
        // |               |
        // |               |
        // | *             |
        //  -------------
        BOTTOM_LAYER_BOTTOM_LEFT_CORNER,
    };


    /**
     * @brief  Enumeration of BMA4XX models
     */
    enum  BMA4XXModel {
        BMA422 = 0x12,
        BMA423 = 0x13,
        BMA456 = 0x16,
        BMA456_AN_SEC = 0x1A
    };

protected:
    /**
     * @brief  Constructor for the SensorBMA4XX class
     * @note   Initializes the communication interfaces and hardware abstraction layer.
     */
    SensorBMA4XX(): comm(nullptr), hal(nullptr), staticComm(nullptr),
        dev(nullptr),  _remap_reg_offset(0),  _half_scale(0)
    {
    }

    /**
     * @brief  Destructor for the SensorBMA4XX class
     * @note   Cleans up the resources used by the class.
     */
    ~SensorBMA4XX() = default;


public:

#if defined(ARDUINO)
    /**
     * @brief  Initialization using the Arduino Wire Interface
     * @note   This function sets up the I2C communication parameters for the sensor.
     * @param  &wire: Reference to the TwoWire I2C interface.
     * @param  addr: I2C address of the sensor.
     * @param  sda: SDA pin number, default is -1 (use board default)
     * @param  scl: SCL pin number, default is -1 (use board default)
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1) override
    {
        if (!beginCommonStatic<SensorCommI2C, HalArduino>(comm, staticComm, hal, wire, addr, sda, scl)) {
            return false;
        }
        return initImpl(static_cast<uint8_t>(COMM_I2C));
    }


    /**
     * @brief  Initialization using the Arduino SPI Interface
     * @note   This function sets up the SPI communication parameters for the sensor.
     * @param  &spi: Reference to the SPIClass instance.
     * @param  csPin: Chip select pin number.
     * @param  mosi: Master Out Slave In pin number, default is -1 (use board default)
     * @param  miso: Master In Slave Out pin number, default is -1 (use board default)
     * @param  sck: Serial Clock pin number, default is -1 (use board default)
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(SPIClass &spi, uint8_t csPin, int mosi = -1, int miso = -1, int sck = -1)
    {
        if (!beginCommonStatic<SensorCommSPI, HalArduino>(comm,
                staticComm, hal,
                spi, csPin, mosi, miso, sck)) {
            return false;
        }
        return initImpl(static_cast<uint8_t>(COMM_SPI));
    }

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)

    /**
     * @brief  Initialization using the ESP-IDF I2C Legacy Interface
     * @note   This function sets up the I2C communication parameters for the sensor.
     * @param  port_num: I2C port number.
     * @param  addr: I2C address of the sensor.
     * @param  sda: SDA pin number, default is -1 (use board default)
     * @param  scl: SCL pin number, default is -1 (use board default)
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1) override
    {
        if (!beginCommonStatic<SensorCommI2C, HalEspIDF>(comm, staticComm, hal, port_num, addr, sda, scl)) {
            return false;
        }
        return initImpl(static_cast<uint8_t>(COMM_I2C));
    }
#else

    /**
     * @brief  Initialization using the ESP-IDF I2C LL Interface idf version > 5.0.0
     * @note   This function sets up the I2C communication parameters for the sensor.
     * @param  handle: I2C master bus handle.
     * @param  addr: I2C address of the sensor.
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr) override
    {
        if (!beginCommonStatic<SensorCommI2C, HalEspIDF>(comm, staticComm, hal, handle, addr)) {
            return false;
        }
        return initImpl(static_cast<uint8_t>(COMM_I2C));
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    /**
     * @brief  Initialization using a custom communication interface.
     * @note   This function sets up the communication parameters for the sensor.
     * @param  interface: Communication mode, COMM_SPI or COMM_I2C
     * @param  callback: Custom callback function for communication.
     * @param  hal_callback: Hardware abstraction layer callback function.
     * @param  addr: I2C address of the sensor.
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(CommInterface interface,
               SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(interface,
                callback, hal_callback, addr, comm, hal)) {
            return false;
        }
        return initImpl(static_cast<uint8_t>(interface));
    }

    /**
     * @brief  Get the accelerometer data.
     * @note   This function retrieves the current accelerometer data from the sensor.
     * @param  &data: Reference to the data structure to be filled.
     * @retval True if data retrieval is successful, false otherwise.
     */
    bool readData(AccelerometerData &data) override
    {
        if (!dev) {
            log_e("Device not initialized");
            return false;
        }
        struct bma4_accel sens_data = { 0, 0, 0 };
        if (bma4_read_accel_xyz(&sens_data, dev.get()) != 0) {
            log_e("Failed to read acceleration data");
            return false;
        }
        data.raw.x = sens_data.x;
        data.raw.y = sens_data.y;
        data.raw.z = sens_data.z;

        data.temperature = getTemperature();

        data.mps2.x = sens_data.x * _config.sensitivity;
        data.mps2.y = sens_data.y * _config.sensitivity;
        data.mps2.z = sens_data.z * _config.sensitivity;

        data.mps2.x = AccelerometerUtils::gToMps2(data.mps2.x);
        data.mps2.y = AccelerometerUtils::gToMps2(data.mps2.y);
        data.mps2.z = AccelerometerUtils::gToMps2(data.mps2.z);

        return true;
    }

    /**
     * @brief  Check if new accelerometer data is available.
     * @note   This function checks the interrupt status to determine if new data is ready to be read.
     * @retval True if new data is available, false otherwise.
     */
    bool isDataReady() override
    {
        uint16_t int_status = 0;
        bma4_read_int_status(&int_status, dev.get());
        return (int_status & BMA4_ACCEL_DATA_RDY_INT) != 0;
    }

    /**
     * @brief  Reset the sensor.
     * @note   This function performs a software reset on the sensor.
     * @retval True if reset is successful, false otherwise.
     */
    bool reset() override
    {
        if (!dev) {
            log_e("Device not initialized");
            return false;
        }
        return bma4_soft_reset(dev.get()) == 0;
    }

    /**
     * @brief  Perform a self-test on the sensor.
     * @note   This function checks the functionality of the sensor by performing a self-test.
     * @retval True if self-test is successful, false otherwise.
     */
    bool selfTest() override
    {
        if (!dev) {
            log_e("Device not initialized");
            return false;
        }
        int8_t selftest_rslt = 0;
        if (bma4_perform_accel_selftest(&selftest_rslt, dev.get()) != 0) {
            log_i("Self test failed");
            return false;
        }
        return selftest_rslt == 0;
    }


    /**
     * @brief  Set the full-scale range of the accelerometer.
     * @note   This function configures the accelerometer to operate within a specific range.
     * @param  range: FS_2G, FS_4G, FS_8G, FS_16G
     * @retval True if the configuration is successful, false otherwise.
     */
    bool setFullScaleRange(AccelFullScaleRange range) override
    {
        uint8_t regValue = 0;
        switch (range) {
        case AccelFullScaleRange::FS_2G:  // ±2g
            regValue = BMA4_ACCEL_RANGE_2G;
            break;
        case AccelFullScaleRange::FS_4G:  // ±4g
            regValue = BMA4_ACCEL_RANGE_4G;
            break;
        case AccelFullScaleRange::FS_8G:  // ±8g
            regValue = BMA4_ACCEL_RANGE_8G;
            break;
        case AccelFullScaleRange::FS_16G: // ±16g
            regValue = BMA4_ACCEL_RANGE_16G;
            break;
        default:
            log_e("Error: Invalid range %d\n", static_cast<int>(range));
            return false;
        }
        _accel_conf.range = regValue;
        if (bma4_set_accel_config(&_accel_conf, dev.get()) == 0) {
            float tmp_range = AccelerometerUtils::rangeToG(range);
            _config.full_scale_range = (int)tmp_range;
            _config.sensitivity = tmp_range / _half_scale;
            return true;
        }
        return false;
    }

    /**
     * @brief  Set the output data rate of the accelerometer.
     * @note   This function configures the accelerometer to output data at a specific rate.
     * @param  data_rate_hz: The desired output data rate in Hz. Allowed values are 0.78, 1.56,
     *                          3.12, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600.
     * @retval True if the configuration is successful, false otherwise.
     */
    bool setOutputDataRate(float data_rate_hz) override
    {
        int rangeInt = static_cast<int>(data_rate_hz * 100 + 0.5);
        log_d("Input: %.2f, Integer: %d\n", data_rate_hz, rangeInt);
        uint8_t regValue = 0;
        switch (rangeInt) {
        case 78: // 0.78HZ
            regValue = BMA4_OUTPUT_DATA_RATE_0_78HZ;
            break;
        case 156: // 1.56HZ
            regValue = BMA4_OUTPUT_DATA_RATE_1_56HZ;
            break;
        case 312: // 3.12HZ
            regValue = BMA4_OUTPUT_DATA_RATE_3_12HZ;
            break;
        case 625: // 6.25HZ
            regValue = BMA4_OUTPUT_DATA_RATE_6_25HZ;
            break;
        case 1250: // 12.5HZ
            regValue = BMA4_OUTPUT_DATA_RATE_12_5HZ;
            break;
        case 2500: // 25.0HZ
            regValue = BMA4_OUTPUT_DATA_RATE_25HZ;
            break;
        case 5000: // 50.0HZ
            regValue = BMA4_OUTPUT_DATA_RATE_50HZ;
            break;
        case 10000: // 100.0HZ
            regValue = BMA4_OUTPUT_DATA_RATE_100HZ;
            break;
        case 20000: // 200.0HZ
            regValue = BMA4_OUTPUT_DATA_RATE_200HZ;
            break;
        case 40000: // 400.0HZ
            regValue = BMA4_OUTPUT_DATA_RATE_400HZ;
            break;
        case 80000: // 800.0HZ
            regValue = BMA4_OUTPUT_DATA_RATE_800HZ;
            break;
        case 160000: // 1600.0HZ
            regValue = BMA4_OUTPUT_DATA_RATE_1600HZ;
            break;
        default:
            log_e("Error: Invalid range %.2f\n", data_rate_hz);
            break;
        }
        _accel_conf.odr = regValue;
        if (bma4_set_accel_config(&_accel_conf, dev.get()) == 0) {
            _config.data_rate_hz = data_rate_hz;
            return true;
        }
        return false;
    }

    /**
     * @brief  Set the bandwidth of the accelerometer.
     * @note   This function configures the accelerometer to operate with a specific bandwidth.
     * @param  bandwidth: The desired bandwidth setting. Allowed values are OSR4_AVG1, OSR2_AVG2, NORMAL_AVG4, etc.
     * @retval True if the configuration is successful, false otherwise.
     */
    bool setBandwidth(AccelBandwidth bandwidth)
    {
        uint8_t regValue = 0;
        switch (bandwidth) {
        case AccelBandwidth::OSR4_AVG1:
            regValue = BMA4_ACCEL_OSR4_AVG1;
            break;
        case AccelBandwidth::OSR2_AVG2:
            regValue = BMA4_ACCEL_OSR2_AVG2;
            break;
        case AccelBandwidth::NORMAL_AVG4:
            regValue = BMA4_ACCEL_NORMAL_AVG4;
            break;
        case AccelBandwidth::CIC_AVG8:
            regValue = BMA4_ACCEL_CIC_AVG8;
            break;
        case AccelBandwidth::RES_AVG16:
            regValue = BMA4_ACCEL_RES_AVG16;
            break;
        case AccelBandwidth::RES_AVG32:
            regValue = BMA4_ACCEL_RES_AVG32;
            break;
        case AccelBandwidth::RES_AVG64:
            regValue = BMA4_ACCEL_RES_AVG64;
            break;
        case AccelBandwidth::RES_AVG128:
            regValue = BMA4_ACCEL_RES_AVG128;
            break;
        default:
            log_e("Error: Invalid bandwidth %d\n", static_cast<int>(bandwidth));
            return false;
        }
        _accel_conf.bandwidth = regValue;
        if (bma4_set_accel_config(&_accel_conf, dev.get()) == 0) {
            return true;
        }
        return false;
    }

    /**
     * @brief  Set the performance mode of the accelerometer.
     * @note   This function configures the accelerometer to operate in a specific performance mode.
     * @param  mode: The desired performance mode. Allowed values are CIC_AVG_MODE, CONTINUOUS_MODE.
     *         CIC_AVG_MODE: Averaging samples (Default)
     *         CONTINUOUS_MODE: No averaging
     * @retval True if the configuration is successful, false otherwise.
     */
    bool setPerformanceMode(AccelPerfMode mode)
    {
        uint8_t regValue = 0;
        switch (mode) {
        case AccelPerfMode::CIC_AVG_MODE:
            regValue = BMA4_CIC_AVG_MODE;
            break;
        case AccelPerfMode::CONTINUOUS_MODE:
            regValue = BMA4_CONTINUOUS_MODE;
            break;
        default:
            log_e("Error: Invalid performance mode %d\n", static_cast<int>(mode));
            return false;
        }
        _accel_conf.perf_mode = regValue;
        if (bma4_set_accel_config(&_accel_conf, dev.get()) == 0) {
            return true;
        }
        return false;
    }

    /**
    * @brief  Set the operation mode of the accelerometer.
    * @note   This function configures the accelerometer to operate in a specific mode.
    * @param  mode: The desired operation mode. Allowed values are SUSPEND, NORMAL.
    * @retval True if the configuration is successful, false otherwise.
    */
    bool setOperationMode(AccelOperationMode mode) override
    {
        bool enable = false;
        switch (mode) {
        case AccelOperationMode::SUSPEND:
            enable = false;
            break;
        case AccelOperationMode::NORMAL:
            enable = true;
            break;
        default:
            log_e("Error: Invalid operation mode %d\n", static_cast<int>(mode));
            break;
        }
        return bma4_set_accel_enable(enable, dev.get()) == 0;
    }

    /**
     * @brief  Configure the accelerometer.
     * @note   This function sets various parameters of the accelerometer in a single call.
     * @param  mode: The desired operation mode. Allowed values are SUSPEND, NORMAL.
     * @param  range: The desired full-scale range. Allowed values are FS_2G, FS_4G, FS_8G, FS_16G.
     * @param  data_rate_hz: The desired output data rate in Hz. Allowed values are 0.78, 1.56, 3.12, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600.
     * @param  bandwidth: The desired bandwidth. Allowed values are OSR4_AVG1, OSR2_AVG2, NORMAL_AVG4, etc.
     * @param  perf_mode: The desired performance mode. Allowed values are CIC_AVG_MODE, CONTINUOUS_MODE.
     * @retval True if the configuration is successful, false otherwise.
     */
    bool configAccelerometer(AccelOperationMode mode, AccelFullScaleRange range, float data_rate_hz,
                             AccelBandwidth bandwidth, AccelPerfMode perf_mode)

    {
        bool rslt;

        rslt = setFullScaleRange(range);
        if (!rslt) {
            return false;
        }

        rslt = setOutputDataRate(data_rate_hz);
        if (!rslt) {
            return false;
        }

        rslt = setBandwidth(bandwidth);
        if (!rslt) {
            return false;
        }

        rslt = setPerformanceMode(perf_mode);
        if (!rslt) {
            return false;
        }

        rslt = setOperationMode(mode);
        if (!rslt) {
            return false;
        }
        return true;
    }

    /**
     * @brief  Get the temperature reading from the sensor.
     * @note   This function retrieves the current temperature reading from the sensor.
     * @retval The temperature in degrees Celsius, or NaN if the reading fails.
     */
    float getTemperature() const
    {
        float temperature = NAN;
        int32_t raw_temperature = 0;
        if (bma4_get_temperature(&raw_temperature, BMA4_DEG, dev.get()) == 0) {
            if (((raw_temperature - 23) / BMA4_SCALE_TEMP) != 0x80) {
                temperature = static_cast<float>(raw_temperature) / BMA4_SCALE_TEMP;
            }
        }
        return temperature;
    }

    /**
     * @brief Get the current interrupt status flags.
     *
     * Reads the interrupt status register which indicates which interrupts have occurred.
     * Each bit in the returned 16-bit value represents a different interrupt source.
     *
     * @note Use with interrupt masks (e.g., BMA4_ACCEL_DATA_RDY_INT, BMA4_FIFO_FULL_INT)
     *       to check specific interrupts.
     * @return uint16_t Bitmask of active interrupt flags, or 0 on error.
     */
    uint16_t getInterruptStatus()
    {
        uint16_t int_status = 0;
        if (bma4_read_int_status(&int_status, dev.get()) != 0) {
            return 0;
        }
        return int_status;
    }

    /**
    * @brief Check if a specific interrupt is active.
    *
    * Convenience function to check if a particular interrupt flag is set.
    *
    * @param interrupt_mask Bitmask of the interrupt(s) to check.
    * @return True if any of the specified interrupts are active, false otherwise.
    */
    bool isInterruptActive(uint16_t interrupt_mask)
    {
        uint16_t int_status = getInterruptStatus();
        return (int_status & interrupt_mask) != 0;
    }

    /**
     * @brief Configure the electrical properties of an interrupt pin.
     *
     * Sets up the physical characteristics of the interrupt pin, including:
     * - Trigger type (edge or level)
     * - Active level (active low or active high)
     * - Output type (open-drain or push-pull)
     * - Input/output enables
     *
     * @param pin_map Which interrupt pin to configure (PIN1 or PIN2).
     * @param edge_trigger True for edge-triggered, false for level-triggered.
     * @param active_low True for active-low, false for active-high.
     * @param output_en True to enable output driver.
     * @param input_en True to enable input buffer
     * @return True if configuration succeeded, false otherwise.
     */
    bool setInterruptPinConfig(InterruptPinMap pin_map,
                               bool edge_trigger = false,
                               bool active_low = true,
                               bool output_en = true,
                               bool input_en = false)
    {
        struct bma4_int_pin_config config ;
        config.edge_ctrl = edge_trigger ? BMA4_EDGE_TRIGGER : BMA4_LEVEL_TRIGGER;
        config.lvl = active_low ? BMA4_ACTIVE_LOW : BMA4_ACTIVE_HIGH;
        config.od = active_low ? BMA4_OPEN_DRAIN : BMA4_PUSH_PULL;
        config.output_en = output_en ? BMA4_OUTPUT_ENABLE : BMA4_OUTPUT_DISABLE;
        config.input_en = input_en ? BMA4_INPUT_ENABLE : BMA4_INPUT_DISABLE;
        uint8_t int_line = pin_map == InterruptPinMap::PIN1 ? BMA4_INTR1_MAP : BMA4_INTR2_MAP;
        return bma4_set_int_pin_config(&config, int_line, dev.get()) == 0;
    }

    /**
    * @brief Enable or disable specific interrupt sources on a pin.
    *
    * Maps interrupt sources (e.g., data ready, FIFO full) to a physical interrupt pin.
    * Multiple interrupt sources can be combined using bitwise OR.
    *
    * Example: enableInterrupt(PIN1, BMA4_DATA_RDY_INT | BMA4_FIFO_FULL_INT, true)
    *
    * @param pin_map Which interrupt pin to map to (PIN1 or PIN2).
    * @param interrupt_sources Bitmask of interrupt sources to enable/disable.
    * @param enable True to enable, false to disable.
    * @return True if operation succeeded, false otherwise.
    */
    bool enableInterrupt(InterruptPinMap pin_map, uint16_t interrupt_sources, bool enable)
    {
        uint8_t int_line = 0;
        switch (pin_map) {
        case InterruptPinMap::PIN1:
            int_line = 0x00;
            break;
        case InterruptPinMap::PIN2:
            int_line = 0x01;
            break;
        default:
            log_e("Invalid interrupt pin specified");
            return false;
        }

        int8_t result = bma4_map_interrupt(int_line, interrupt_sources, enable, dev.get());
        if (result != 0) {
            log_e("Failed to %s interrupt 0x%04X on pin %d: %d",
                  enable ? "enable" : "disable",
                  interrupt_sources,
                  (pin_map == InterruptPinMap::PIN1) ? 1 : 2,
                  result);
            return false;
        }

        return true;
    }

    /**
     * @brief  Set the remapping of axes for the BMA4XX sensor.
     * @note   This function configures the sensor to use a specific axis remapping.
     * @param  remap: The desired axis remapping configuration, See SensorRemap enum for options.
     * @retval True if the remapping is successful, false otherwise.
     */
    virtual bool setRemapAxes(SensorRemap remap)
    {
        if (_remap_reg_offset == 0) {
            log_e("Remap feature not supported for this model");
            return false;
        }
        struct bma4_remap remap_data;
        switch (remap) {
        case SensorRemap::TOP_LAYER_LEFT_CORNER:
            remap_data.x = BMA4_NEG_Y;
            remap_data.y = BMA4_X;
            remap_data.z = BMA4_Z;
            break;
        case SensorRemap::TOP_LAYER_RIGHT_CORNER:
            remap_data.x = BMA4_X;
            remap_data.y = BMA4_Y;
            remap_data.z = BMA4_Z;
            break;
        case SensorRemap::TOP_LAYER_BOTTOM_RIGHT_CORNER:
            remap_data.x = BMA4_Y;
            remap_data.y = BMA4_NEG_Y;
            remap_data.z = BMA4_Z;
            break;
        case SensorRemap::TOP_LAYER_BOTTOM_LEFT_CORNER:
            remap_data.x = BMA4_NEG_X;
            remap_data.y = BMA4_NEG_Y;
            remap_data.z = BMA4_Z;
            break;
        case SensorRemap::BOTTOM_LAYER_TOP_LEFT_CORNER:
            remap_data.x = BMA4_NEG_X;
            remap_data.y = BMA4_Y;
            remap_data.z = BMA4_NEG_Z;
            break;
        case SensorRemap::BOTTOM_LAYER_TOP_RIGHT_CORNER:
            remap_data.x = BMA4_Y;
            remap_data.y = BMA4_X;
            remap_data.z = BMA4_NEG_Z;
            break;
        case SensorRemap::BOTTOM_LAYER_BOTTOM_RIGHT_CORNER:
            remap_data.x = BMA4_X;
            remap_data.y = BMA4_NEG_Y;
            remap_data.z = BMA4_NEG_Z;
            break;
        case SensorRemap::BOTTOM_LAYER_BOTTOM_LEFT_CORNER:
            remap_data.x = BMA4_NEG_Y;
            remap_data.y = BMA4_NEG_X;
            remap_data.z = BMA4_NEG_Z;
            break;
        default:
            log_e("Invalid remap option");
            return false;
        }
        log_d("Remap register offset: 0x%X", _remap_reg_offset);
        auto feature_config = std::unique_ptr<uint8_t[]>(new uint8_t[dev->feature_len]());
        return bma4_set_remap_axes(&remap_data, feature_config.get(),
                                   _remap_reg_offset, dev->feature_len, dev.get()) == 0;
    }

    /**
     * @brief  Get the model name of the BMA4XX sensor.
     * @note   This function returns a string representing the model name based on the chip ID.
     * @retval The model name as a string.
     */
    const char *getModelName() const
    {
        if (!dev) {
            log_e("Device not initialized");
            return "Unknown";
        }
        switch (dev->chip_id) {
        case BMA4XXModel::BMA422:
            return "BMA422";
        case BMA4XXModel::BMA423:
            return "BMA423";
        case BMA4XXModel::BMA456:
            return "BMA456";
        case BMA4XXModel::BMA456_AN_SEC:
            return "BMA456_AN_SEC";
        default:
            return "Unknown";
        }
    }

    /**
     * @brief  Get the device structure for the BMA4XX sensor.
     * @note   This function returns a pointer to the internal device structure.
     * @retval Pointer to the bma4_dev_t structure.
     */
    bma4_dev_t *getDev()
    {
        return dev.get();
    }

    /**
    * @brief Get the current sensor time in milliseconds.
    *
    * Reads the 24-bit free-running sensor time counter and converts it to milliseconds.
    * The counter increments at 39.0625 μs per tick (0.0390625 ms per tick) and overflows
    * after approximately 10.9 minutes.
    *
    * @return uint32_t Sensor time in milliseconds, or 0 on error.
    */
    uint32_t getTimeSampleMs() const
    {
        uint32_t sensor_time_ticks = 0;
        bma4_get_sensor_time(&sensor_time_ticks, dev.get());
        // Convert ticks to milliseconds (1 tick = 39.0625 μs = 0.0390625 ms)
        float time_ms = sensor_time_ticks * 0.0390625f;
        return static_cast<uint32_t>(time_ms);
    }

    /**
     * @brief  Update the sensor data, this method is implemented by a subclass.
     * @note   This function is called periodically to update the sensor data.
     * @retval None
     */
    virtual void update() = 0;

private:
    /**
     * @brief  Bosch sensor initialization implementation.
     * @note   This function is called during the initialization process to perform
     *         any Bosch-specific setup.
     * @retval True if initialization is successful, false otherwise.
     */
    virtual bool boschInitImpl() = 0;

    /**
     * @brief  Bosch sensor initialization implementation.
     * @note   This function is called during the initialization process to perform
     *         any Bosch-specific setup.
     * @param  addr: The I2C address of the sensor.
     * @retval True if initialization is successful, false otherwise.
     */
    bool initImpl(uint8_t addr)
    {
        CommInterface interface = static_cast<CommInterface>(addr);

        dev = std::make_unique<struct bma4_dev>();
        if (!dev) {
            log_e(" Device handler malloc failed!");
            return false;
        }
        switch (interface) {
        case COMM_I2C:
            dev->intf = BMA4_I2C_INTF;
            break;
        case COMM_SPI:
            dev->intf = BMA4_SPI_INTF;
            break;
        default:
            return false;
        }
        dev->bus_read = SensorCommStatic::sensor_static_read_data;
        dev->bus_write = SensorCommStatic::sensor_static_write_data;
        dev->intf_ptr = staticComm.get();
        dev->delay_us = SensorCommStatic::sensor_static_delay_us;
        dev->read_write_len = 32;
        dev->perf_mode_status = BMA4_DISABLE;

        if (bma4_soft_reset(dev.get()) != 0) {
            log_e("BMA4xx soft reset failed");
            return false;
        }
        hal->delay(20);


        if (!boschInitImpl()) {
            log_e("BMA4xx sensor initialization failed");
            return false;
        }

        // Set default axis remapping
        struct bma4_axes_remap axes_remap;
        axes_remap.x_axis = BMA4_MAP_X_AXIS;
        axes_remap.x_axis_sign = BMA4_MAP_POSITIVE;
        axes_remap.y_axis = BMA4_MAP_Y_AXIS;
        axes_remap.y_axis_sign = BMA4_MAP_POSITIVE;
        axes_remap.z_axis = BMA4_MAP_Z_AXIS;
        axes_remap.z_axis_sign = BMA4_MAP_POSITIVE;
        dev->remap = axes_remap;

        // Set accelerometer configuration values
        _accel_conf.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
        _accel_conf.odr = BMA4_OUTPUT_DATA_RATE_50HZ;
        _accel_conf.perf_mode = BMA4_CIC_AVG_MODE;
        _accel_conf.range = BMA4_ACCEL_RANGE_2G;

        // Calculate half scale
        _half_scale = powf(2.0f, (float)dev->resolution) / 2.0f;

        log_d("Half scale calculated: %.0f (for %d-bit sensor)",
              _half_scale, dev->resolution);

        // Set default full scale range to 2g
        if (!setFullScaleRange(AccelFullScaleRange::FS_2G)) {
            log_e("Failed to set default 2g range");
            return false;
        }

        // Set default output data rate to 50Hz
        if (!setOutputDataRate(50.0f)) {
            log_e("Failed to set default ODR 50Hz");
            return false;
        }

        return true;
    }

protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
    std::unique_ptr<SensorCommStatic> staticComm;
    std::unique_ptr<struct bma4_dev> dev;
    struct bma4_accel_config _accel_conf;
    uint8_t  _remap_reg_offset;
    float    _half_scale;
};

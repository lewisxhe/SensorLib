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
 * @file      SensorQMC5883L.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-20
 *
 */
#pragma once
#include "sensor/MagnetometerBase.hpp"

static constexpr uint8_t QMC5883L_SLAVE_ADDRESS = 0x0D;

class SensorQMC5883L : public MagnetometerBase
{
public:

    /**
     * @brief Default constructor. Initializes communication and hardware abstraction pointers to nullptr.
     *
     * Creates a SensorQMC5883L object with no active communication or hardware
     * abstraction instances.
     */
    SensorQMC5883L() {}

    /**
     * @brief Destructor. Deinitializes the communication object if it exists.
     *
     * Ensures proper cleanup by deinitializing the communication interface
     * when the sensor object is destroyed.
     */
    ~SensorQMC5883L() = default;

    /**
     * @brief  Reads the magnetic field data from the sensor.
     * @note   This function retrieves the latest magnetic field measurements
     *         from the sensor and stores them in the provided data structure.
     * @param  data: A reference to a MagnetometerData structure to store the
     *               retrieved magnetic field data.
     * @retval True if the data was read successfully, false otherwise.
     */
    bool readData(MagnetometerData &data) override
    {
        uint8_t buffer[6] = {0};
        int16_t x = 0, y = 0, z = 0;

        int status = comm->readRegister(REG_0x06_STATUS);
        if (status < 0) {
            log_e("Failed to read status register");
            return false;
        }

        // OVL (Overflow)
        if (isBitSet(status, 1)) {
            data.overflow = true;
            log_e("Data overflow detected");
        } else {
            data.overflow = false;
        }

        // DOR (Data Skip)
        if (isBitSet(status, 2)) {
            log_e("Data skip detected");
            data.skip_data = true;
        } else {
            data.skip_data = false;
        }

        // DRDY (Data Ready)
        if (!isBitSet(status, 0)) {
            // log_w("Data not ready");
            return false;
        }

        if (comm->readRegister(REG_0x00_LSB_DX, buffer, 6) < 0) {
            log_e("Failed to read magnetic field data");
            return false;
        }

        x = (int16_t)(buffer[1] << 8) | (buffer[0]);  // Combine X LSB and MSB
        y = (int16_t)(buffer[3] << 8) | (buffer[2]);  // Combine Y LSB and MSB
        z = (int16_t)(buffer[5] << 8) | (buffer[4]);  // Combine Z LSB and MSB

        data.raw.x = x - _x_offset;
        data.raw.y = y - _y_offset;
        data.raw.z = z - _z_offset;

        // Convert raw values to Gauss using sensitivity (depends on selected magnetic range)
        data.magnetic_field.x = (float)(data.raw.x) * _sensitivity;
        data.magnetic_field.y = (float)(data.raw.y) * _sensitivity;
        data.magnetic_field.z = (float)(data.raw.z) * _sensitivity;

        // Calculate heading
        data.heading = MagnetometerUtils::calculateHeading(data, _declination_rad);

        // Convert heading to degrees
        data.heading_degrees = data.heading * (180.0 / M_PI);

        return true;
    }

    /**
     * @brief  Checks if new data is available from the sensor.
     * @note   This function reads the status register to determine if new
     *         magnetic field data is ready to be processed.
     * @retval True if new data is available, false otherwise.
     */
    bool isDataReady() override
    {
        return comm->getRegisterBit(REG_0x06_STATUS, 0);
    }

    /**
     * @brief  Checks if data overflow has occurred.
     * @note   This function reads the status register to determine if
     *         the magnetic field data has overflowed.
     * @retval True if data overflow has occurred, false otherwise.
     */
    bool isDataOverflow()
    {
        return comm->getRegisterBit(REG_0x06_STATUS, 1);
    }

    /**
     * @brief  Checks if data has been skipped.
     * @note   This function reads the status register to determine if
     *         the magnetic field data has been skipped.
     * @retval True if data has been skipped, false otherwise.
     */
    bool isDataSkipped()
    {
        return comm->getRegisterBit(REG_0x06_STATUS, 2);
    }

    /**
     * @brief  Resets the sensor to its default state.
     * @note   This function sends a reset command to the sensor and waits
     *         for a short period to allow the reset to take effect.
     * @retval True if the reset command was successful, false otherwise.
     */
    bool reset() override
    {
        comm->writeRegister(REG_0x0A_CMD2, (uint8_t)0x80);
        hal->delay(10);
        comm->writeRegister(REG_0x0A_CMD2, (uint8_t)0x00);
        return true;
    }

    /**
     * @brief  QMC5883L does not support self-test checks.
     * @note   This function is a placeholder and does not perform any actual self-test.
     * @retval Always return True.
     */
    bool selfTest() override
    {
        log_e("QMC5883L not self-test function");
        return true;
    }

    /**
     * @brief  Sets the full-scale range of the magnetometer.
     * @note   This function configures the magnetometer to operate within a
     *         specific range, affecting the sensitivity and maximum measurable
     *         magnetic field strength.
     * @param  range: FS_2G, FS_8G
     * @retval True if the range was set successfully, false otherwise.
     */
    bool setFullScaleRange(MagFullScaleRange range) override
    {
        float full_scale = 0;
        float sensitivity = 0.0f;
        uint8_t range_value = 0;
        switch (range) {
        case MagFullScaleRange::FS_8G:
            sensitivity = 0.00033333f; // 3000 LSB/Gauss
            range_value = 0x01 << 4;
            full_scale = 8.0f;
            break;
        case MagFullScaleRange::FS_2G:
            sensitivity = 0.00008333f; // 12000 LSB/Gauss
            range_value = 0x00 << 4;
            full_scale = 2.0f;
            break;
        default:
            log_e("Invalid magnetometer range");
            return false;
        }
        if (comm->writeRegister(REG_0x09_CMD1, 0xCF, range_value) > 0) {
            log_e("Failed to set full scale range");
            return false;
        }
        _config.range = full_scale;
        _sensitivity = sensitivity;
        return true;
    }

    /**
     * @brief  Sets the bandwidth of the magnetometer.
     * @note   This function configures the magnetometer's bandwidth, affecting
     *         the trade-off between noise and response time.
     * @param  data_rate_hz: The desired output data rate in Hz.  Allowed values are 10.0, 50.0, 100.0 and 200.0HZ.
     * @retval True if the bandwidth was set successfully, false otherwise.
     */
    bool setOutputDataRate(float data_rate_hz) override
    {
        int rangeInt = static_cast<int>(data_rate_hz * 100 + 0.5);
        log_d("Input: %.2f, Integer: %d\n", data_rate_hz, rangeInt);
        uint8_t regValue = 0;
        switch (rangeInt) {
        case 1000:         // 10.0
            regValue = 0x00 << 2;
            break;
        case 5000:         // 50.0
            regValue = 0x01 << 2;
            break;
        case 10000:         // 100.0
            regValue = 0x02 << 2;
            break;
        case 20000:         // 200.0
            regValue = 0x03 << 2;
            break;
        default:
            log_e("Invalid output data rate");
            return false;
        }
        if (comm->writeRegister(REG_0x09_CMD1, 0xF3, regValue) < 0) {
            log_e("Failed to set bandwidth");
            return false;
        }
        _config.sample_rate = data_rate_hz;
        return true;
    }

    /**
     * @brief  Sets the operation mode of the magnetometer.
     * @note   This function configures the magnetometer to operate in a specific
     *         mode, affecting its power consumption and measurement behavior.
     * @param  mode: SUSPEND, NORMAL, CONTINUOUS_MEASUREMENT
     * @retval True if the mode was set successfully, false otherwise.
     */
    bool setOperationMode(OperationMode mode) override
    {
        uint8_t mode_val = 0;
        switch (mode) {
        case OperationMode::SUSPEND:
            mode_val = 0x00;
            break;
        case OperationMode::NORMAL:
        case OperationMode::CONTINUOUS_MEASUREMENT:
            mode_val = 0x01;
            break;
        default:
            log_e("Invalid operation mode");
            return false;
        }
        if (comm->writeRegister(REG_0x09_CMD1, 0xFC, mode_val) < 0) {
            log_e("Failed to set operation mode");
            return false;
        }
        _config.mode = mode;
        return true;
    }

    /**
     * @brief  Sets the oversampling rate of the magnetometer.
     * @note   This function configures the magnetometer's oversampling rate,
     *         affecting the trade-off between noise and measurement speed.
     * @param  osr: OSR_8, OSR_4, OSR_2, OSR_1
     * @retval True if the oversampling rate was set successfully, false otherwise.
     */
    bool setOversamplingRate(MagOverSampleRatio osr) override
    {
        uint8_t osr_val = 0;
        switch (osr) {
        // 512
        case MagOverSampleRatio::OSR_8:
            osr_val = 0x00 << 6;
            _oversampling_rate = 512;
            break;
        // 256
        case MagOverSampleRatio::OSR_4:
            osr_val = 0x01 << 6;
            _oversampling_rate = 256;
            break;
        // 128
        case MagOverSampleRatio::OSR_2:
            osr_val = 0x02 << 6;
            _oversampling_rate = 128;
            break;
        // 64
        case MagOverSampleRatio::OSR_1:
            osr_val = 0x03 << 6;
            _oversampling_rate = 64;
            break;
        default:
            log_e("Invalid oversampling rate");
            return false;
        }
        return comm->writeRegister(REG_0x09_CMD1, 0x3F, osr_val) == 0;
    }

    /**
     * @brief  QM5883L does not support downsampling.
     * @note   This function is a placeholder and does not perform any action.
     * @retval always false.
     */
    bool setDownsamplingRate(MagDownSampleRatio dsr) override
    {
        log_e("QSTMagnetic does not support downsampling");
        return false;
    }

    /**
     * @todo   The actual test result was 0, which may be due to a chip problem.
     * @brief  Gets the temperature reading from the sensor.
     * @note   This function retrieves the temperature data from the sensor.
     * @param  &temperature: The temperature value in degrees Celsius.
     * @param  &raw_temp: The raw temperature value from the sensor.
     * @retval True if the temperature was read successfully, false otherwise.
     */
    bool getTemperature(float &temperature, int16_t &raw_temp)
    {
        uint8_t buffer[2] = {0};

        if (comm->readRegister(REG_0x07_TOUT_LOW, buffer, 2) < 0) {
            log_e("Failed to read temperature registers");
            return false;
        }
        raw_temp = (int16_t)((buffer[1] << 8) | buffer[0]);
        // 100 LSB/°C
        temperature = (float)raw_temp / 100.0f;
        //TODO: The actual test result was 0, which may be due to a chip problem.
        return true;
    }

    /**
     * @brief  Gets the chip ID of the magnetometer.
     * @note   This function retrieves the chip ID from the sensor.
     * @retval The chip ID.
     */
    uint8_t getChipID() const
    {
        return _info.uid;
    }

    /**
     * @brief  Configures the magnetometer with multiple parameters.
     * @note   This function sets the operation mode, full-scale range, output data rate,
     *         oversampling ratio, and downsampling ratio of the magnetometer.
     * @param  mode: SUSPEND, NORMAL, CONTINUOUS_MEASUREMENT
     * @param  range: FS_2G, FS_8G
     * @param  data_rate_hz: The desired output data rate in Hz.  Allowed values are 10.0, 50.0, 100.0 and 200.0HZ.
     * @param  osr: OSR_8, OSR_4, OSR_2, OSR_1
     * @param  dsr: Placeholder parameters, meaningless
     * @retval True if the configuration was successful, false otherwise.
     */
    bool configMagnetometer(OperationMode mode, MagFullScaleRange range, float data_rate_hz,
                            MagOverSampleRatio osr, MagDownSampleRatio dsr = MagDownSampleRatio::DSR_1)
    {
        if (!setOperationMode(mode)) {
            log_e("Failed to set operation mode");
            return false;
        }
        if (!setFullScaleRange(range)) {
            log_e("Failed to set full scale range");
            return false;
        }
        if (!setOutputDataRate(data_rate_hz)) {
            log_e("Failed to set bandwidth");
            return false;
        }
        if (!setOversamplingRate(osr)) {
            log_e("Failed to set oversampling rate");
            return false;
        }
        return true;
    }

private:

    static constexpr uint8_t REG_0x00_LSB_DX = 0x00;
    static constexpr uint8_t REG_0x01_MSB_DX = 0x01;
    static constexpr uint8_t REG_0x02_LSB_DY = 0x02;
    static constexpr uint8_t REG_0x03_MSB_DY = 0x03;
    static constexpr uint8_t REG_0x04_LSB_DZ = 0x04;
    static constexpr uint8_t REG_0x05_MSB_DZ = 0x05;
    static constexpr uint8_t REG_0x06_STATUS = 0x06;
    static constexpr uint8_t REG_0x07_TOUT_LOW = 0x07;
    static constexpr uint8_t REG_0x08_TOUT_HIGH = 0x08;
    static constexpr uint8_t REG_0x09_CMD1 = 0x09;
    static constexpr uint8_t REG_0x0A_CMD2 = 0x0A;
    static constexpr uint8_t REG_0x0B_CMD3 = 0x0B;
    static constexpr uint8_t REG_0x0D_CHIP_ID = 0x0D;
    static constexpr uint8_t QMC5883L_CHIP_ID = 0xFF;

    bool initImpl(uint8_t addr)
    {
        reset();

        hal->delay(20);

        // SET/RESET Period is controlled by FBR [7:0].
        // It is recommended that the register 0BH is written by 0x01.
        uint8_t set_reset_period = 0x01;
        if (comm->writeRegister(REG_0x0B_CMD3, set_reset_period) < 0) {
            log_e("Failed to set SET/RESET period");
            return false;
        }
        _info.uid = comm->readRegister(REG_0x0D_CHIP_ID);
        _info.manufacturer = "QSTMagnetic";
        _info.model = "QMC5883L";
        _info.type = SensorType::MAGNETOMETER;
        _info.i2c_address = addr;
        _info.version = 1;  // Set a default version


        // Set default configuration
        configMagnetometer(OperationMode::SUSPEND,
                           MagFullScaleRange::FS_8G,
                           50.0f,
                           MagOverSampleRatio::OSR_8);

        _config.mode = OperationMode::SUSPEND;
        _config.range = 2.0f;
        _config.sample_rate = 50.0f;
        _config.latency = 0;
        _config.type = SensorType::MAGNETOMETER;

        return _info.uid == QMC5883L_CHIP_ID;
    }

};

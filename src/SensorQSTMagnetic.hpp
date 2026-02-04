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
 * @file      SensorQSTMagnetic.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-20
 *
 */
#pragma once
#include "sensor/MagnetometerBase.hpp"

static constexpr uint8_t QMC6310U_SLAVE_ADDRESS = 0x1C;
static constexpr uint8_t QMC6310N_SLAVE_ADDRESS = 0x3C;
static constexpr uint8_t QMC5883P_SLAVE_ADDRESS = 0x2C;

class SensorQSTMagnetic : public MagnetometerBase
{
public:
    /**
     * @brief Enumeration defining the types of supported magnetic sensor chips.
     */
    enum ChipType {
        CHIP_QMC6310U,   ///< Represents the QMC6310U chip type
        CHIP_QMC6310N,   ///< Represents the QMC6310N chip type
        CHIP_QMC5883P,   ///< Represents the QMC5883P chip type
        CHIP_UNKNOWN     ///< Represents an unknown or unsupported chip type
    };

    /**
     * @brief Default constructor. Initializes communication and hardware abstraction pointers to nullptr.
     *
     * Creates a SensorQSTMagnetic object with no active communication or hardware
     * abstraction instances.
     */
    SensorQSTMagnetic() {}

    /**
     * @brief Destructor. Deinitializes the communication object if it exists.
     *
     * Ensures proper cleanup by deinitializing the communication interface
     * when the sensor object is destroyed.
     */
    ~SensorQSTMagnetic() = default;

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

        // QMC has no data skipping bit
        data.skip_data = false;

        int status = comm->readRegister(REG_0x09_STAT);
        if (status < 0) {
            log_e("Failed to read status register");
            return false;
        }

        // OVL (Overflow)
        if (isBitSet(status, 1)) {
            data.overflow = true;
            log_w("Data overflow detected");
        } else {
            data.overflow = false;
        }

        // DRDY (Data Ready)
        if (!isBitSet(status, 0)) {
            // log_e("Data not ready");
            return false;
        }

        if (comm->readRegister(REG_0x01_LSB_DX, buffer, 6) < 0) {
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
        return comm->getRegisterBit(REG_0x09_STAT, 0);
    }

    /**
    * @brief  Checks if data overflow has occurred.
    * @note   This function reads the status register to determine if
    *         the magnetic field data has overflowed.
    * @retval True if data overflow has occurred, false otherwise.
    */
    bool isDataOverflow()
    {
        return comm->getRegisterBit(REG_0x09_STAT, 1);
    }

    /**
    * @brief  Resets the sensor to its default state.
    * @note   This function sends a reset command to the sensor and waits
    *         for a short period to allow the reset to take effect.
    * @retval True if the reset command was successful, false otherwise.
    */
    bool reset() override
    {
        comm->writeRegister(REG_0x0B_CMD2, (uint8_t)0x80);
        hal->delay(10);
        comm->writeRegister(REG_0x0B_CMD2, (uint8_t)0x00);
        return true;
    }

    /**
     * @brief  Checks if the sensor is functioning correctly.
     * @note   This function performs a self-test by comparing the
     *         magnetic field readings before and after enabling self-test mode.
     * @retval True if the self-test passed, false otherwise.
     */
    bool selfTest() override
    {
        int16_t x_result = 0, y_result = 0, z_result = 0;
        return selfTest(x_result, y_result, z_result);
    }

    /**
     * @brief  Performs a self-test on the sensor.
     * @note   This function compares the magnetic field readings before and after
     *         enabling self-test mode to determine if the sensor is functioning
     *         correctly.
     * @param  &x_result: Return the difference between before and after self-check.
     * @param  &y_result: Return the difference between before and after self-check.
     * @param  &z_result: Return the difference between before and after self-check.
     * @retval True if the self-test passed, false otherwise.
     */
    bool selfTest(int16_t &x_result, int16_t &y_result, int16_t &z_result)
    {
        if (!setOperationMode(OperationMode::CONTINUOUS_MEASUREMENT)) {
            log_e("Failed to set CONTINUOUS_MEASUREMENT for selfTest");
            return false;
        }
        hal->delay(20);

        MagnetometerData data;

        if (!readData(data)) {
            log_e("Failed to read data before selfTest");
            return false;
        }
        int16_t x_pre = data.raw.x;
        int16_t y_pre = data.raw.y;
        int16_t z_pre = data.raw.z;

        comm->setRegisterBit(REG_0x0B_CMD2, 6);
        hal->delay(5);

        if (!readData(data)) {
            log_e("Failed to read data before selfTest");
            return false;
        }
        x_result = data.raw.x - x_pre;
        y_result = data.raw.y - y_pre;
        z_result = data.raw.z - z_pre;

        comm->clrRegisterBit(REG_0x0B_CMD2, 6);

        return setOperationMode(OperationMode::SUSPEND);
    }

    /**
    * @brief  Sets the full-scale range of the magnetometer.
    * @note   This function configures the magnetometer to operate within a
    *         specific range, affecting the sensitivity and maximum measurable
    *         magnetic field strength.
    * @param  range: FS_2G, FS_8G, FS_12G, FS_30G
    * @retval True if the range was set successfully, false otherwise.
    */
    bool setFullScaleRange(MagFullScaleRange range) override
    {
        float full_scale = 0;
        float sensitivity = 0.0f;
        uint8_t range_value = 0;
        switch (range) {
        case MagFullScaleRange::FS_30G:
            sensitivity = 0.001f;      // 1000 LSB/Gauss
            range_value = 0x00 << 2;
            full_scale = 30.0f;
            break;
        case MagFullScaleRange::FS_12G:
            sensitivity = 0.0004f;     // 2500 LSB/Gauss
            range_value = 0x01 << 2;
            full_scale = 12.0f;
            break;
        case MagFullScaleRange::FS_8G:
            sensitivity = 0.00026667f; // 3750 LSB/Gauss
            range_value = 0x02 << 2;
            full_scale = 8.0f;
            break;
        case MagFullScaleRange::FS_2G:
            sensitivity = 0.00006667f; // 15000 LSB/Gauss
            range_value = 0x03 << 2;
            full_scale = 2.0f;
            break;
        default:
            log_e("Invalid magnetometer range");
            return false;
        }
        if (comm->writeRegister(REG_0x0B_CMD2, 0xF3, range_value) < 0) {
            log_e("Failed to set full-scale range");
            return false;
        }
        _sensitivity = sensitivity;
        _config.range = full_scale;
        return true;
    }

    /**
     * @brief  Sets the output data rate for the magnetometer.
     * @note   This function should be called to configure the sensor's output data rate.
     * @param  odr: The desired output data rate in Hz.  Allowed values are 10.0, 50.0, 100.0 and 200.0HZ.
     * @retval True if the output data rate was set successfully, false otherwise.
     */
    bool setOutputDataRate(float odr) override
    {
        int rangeInt = static_cast<int>(odr * 100 + 0.5);
        log_d("Input: %.2f, Integer: %d\n", odr, rangeInt);
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
        if (comm->writeRegister(REG_0x0A_CMD1, 0xF3, regValue) < 0) {
            log_e("Failed to set bandwidth");
            return false;
        }
        _config.sample_rate = odr;
        return true;
    }

    /**
    * @brief  Sets the operation mode of the magnetometer.
    * @note   This function configures the magnetometer to operate in a specific
    *         mode, affecting its power consumption and measurement behavior.
    * @param  mode: SUSPEND, NORMAL, SINGLE_MEASUREMENT, CONTINUOUS_MEASUREMENT
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
            mode_val = 0x01;
            break;
        case OperationMode::SINGLE_MEASUREMENT:
            mode_val = 0x02;
            break;
        case OperationMode::CONTINUOUS_MEASUREMENT:
            mode_val = 0x03;
            break;
        default:
            log_e("Invalid operation mode");
            return false;
        }
        if (comm->writeRegister(REG_0x0A_CMD1, 0xFC, mode_val) < 0) {
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
        case MagOverSampleRatio::OSR_8:
            osr_val = 0x00 << 4;
            _oversampling_rate = 8;
            break;
        case MagOverSampleRatio::OSR_4:
            osr_val = 0x01 << 4;
            _oversampling_rate = 4;
            break;
        case MagOverSampleRatio::OSR_2:
            osr_val = 0x02 << 4;
            _oversampling_rate = 2;
            break;
        case MagOverSampleRatio::OSR_1:
            osr_val = 0x03 << 4;
            _oversampling_rate = 1;
            break;
        default:
            log_e("Invalid oversampling rate");
            return false;
        }
        return comm->writeRegister(REG_0x0A_CMD1, 0xCF, osr_val) == 0;
    }

    /**
     * @brief  Sets the downsampling rate of the magnetometer.
     * @note   This function configures the magnetometer's downsampling rate,
     *         affecting the trade-off between noise and measurement speed.
     * @param  dsr: DSR_1, DSR_2, DSR_4, DSR_8
     * @retval True if the downsampling rate was set successfully, false otherwise.
     */
    bool setDownsamplingRate(MagDownSampleRatio dsr) override
    {
        uint8_t dsr_val = 0;
        switch (dsr) {
        case MagDownSampleRatio::DSR_1:
            dsr_val = 0x00 << 6;
            break;
        case MagDownSampleRatio::DSR_2:
            dsr_val = 0x01 << 6;
            break;
        case MagDownSampleRatio::DSR_4:
            dsr_val = 0x02 << 6;
            break;
        case MagDownSampleRatio::DSR_8:
            dsr_val = 0x03 << 6;
            break;
        default:
            log_e("Invalid downsampling rate");
            return false;
        }
        return comm->writeRegister(REG_0x0A_CMD1, 0x3F, dsr_val) == 0;
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
     * @param  mode: SUSPEND, NORMAL, SINGLE_MEASUREMENT, CONTINUOUS_MEASUREMENT
     * @param  range: FS_2G, FS_8G, FS_12G, FS_30G
     * @param  odr: Allowed values are 10.0, 50.0, 100.0 and 200.0HZ.
     * @param  osr: OSR_8, OSR_4, OSR_2, OSR_1
     * @param  dsr: DSR_1, DSR_2, DSR_4, DSR_8
     * @retval True if the configuration was successful, false otherwise.
     */
    bool configMagnetometer(OperationMode mode, MagFullScaleRange range, float odr,
                            MagOverSampleRatio osr, MagDownSampleRatio dsr)
    {
        if (!setOperationMode(mode)) {
            return false;
        }
        if (!setFullScaleRange(range)) {
            return false;
        }
        if (!setOutputDataRate(odr)) {
            return false;
        }
        if (!setOversamplingRate(osr)) {
            return false;
        }
        if (!setDownsamplingRate(dsr)) {
            return false;
        }
        return true;
    }

private:
    static constexpr uint8_t REG_0x00_CHIP_ID = 0x00;
    static constexpr uint8_t REG_0x01_LSB_DX = 0x01;
    static constexpr uint8_t REG_0x02_MSB_DX = 0x02;
    static constexpr uint8_t REG_0x03_LSB_DY = 0x03;
    static constexpr uint8_t REG_0x04_MSB_DY = 0x04;
    static constexpr uint8_t REG_0x05_LSB_DZ = 0x05;
    static constexpr uint8_t REG_0x06_MSB_DZ = 0x06;
    static constexpr uint8_t REG_0x09_STAT = 0x09;
    static constexpr uint8_t REG_0x0A_CMD1 = 0x0A;
    static constexpr uint8_t REG_0x0B_CMD2 = 0x0B;
    static constexpr uint8_t REG_0x29_SIGN = 0x29;

    static constexpr uint8_t QMC6310_CHIP_ID = 0x80;
    static constexpr uint8_t QMC5883P_CHIP_ID = 0x80;
    static constexpr uint8_t QMC6309_CHIP_ID = 0x90;

    ChipType _type;

    bool initImpl(uint8_t addr)
    {
        reset();

        hal->delay(20);

        _info.uid = comm->readRegister(REG_0x00_CHIP_ID);
        _info.manufacturer = "QSTMagnetic";
        _info.type = SensorType::MAGNETOMETER;
        _info.i2c_address = addr;
        _info.version = 1;  // Set a default version

        switch (addr) {
        case QMC6310U_SLAVE_ADDRESS:
            _type = CHIP_QMC6310U;
            _info.model = "QMC6310U";
            break;
        case QMC6310N_SLAVE_ADDRESS:
            _type = CHIP_QMC6310N;
            _info.model = "QMC6310N";
            break;
        case QMC5883P_SLAVE_ADDRESS:
            _type = CHIP_QMC5883P;
            _info.model = "QMC5883P";
            break;
        default:
            _type = CHIP_UNKNOWN;
            _info.model = "UNKNOWN";
            return false;
        }

        // Set default configuration
        configMagnetometer(OperationMode::SUSPEND,
                           MagFullScaleRange::FS_8G,
                           50.0f,
                           MagOverSampleRatio::OSR_8,
                           MagDownSampleRatio::DSR_1);

        _config.mode = OperationMode::SUSPEND;
        _config.range = 8.0f;
        _config.sample_rate = 50.0f;
        _config.latency = 0;
        _config.type = SensorType::MAGNETOMETER;

        return true;
    }

};

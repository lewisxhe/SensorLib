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
 * @file      ImuBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-08
 */
#pragma once

#include "SensorDefs.hpp"

/**
 * @brief Abstract IMU device interface for polymorphic IMU sensor implementations.
 *
 * This interface defines the common contract that all IMU drivers must implement.
 * It enables polymorphic usage of different IMU sensors through a unified interface.
 *
 * A concrete IMU driver (e.g., QMI8658...) should implement this.
 * Then adapters can expose AccelerometerBase / GyroscopeBase views without multiple inheritance.
 */
class ImuBase
{
public:
    enum class DataReadyMask : uint8_t {
        ACCEL = 0x01,
        GYRO  = 0x02,
        BOTH  = 0x03,
    };

    virtual ~ImuBase() = default;

    // ==================== Common / Device-level Operations ====================

    /**
     * @brief Reset the IMU sensor to its default state.
     * @return true if reset successful, false otherwise.
     */
    virtual bool reset() = 0;

    /**
     * @brief Perform self-test on the IMU sensor.
     * @return true if self-test passed, false otherwise.
     */
    virtual bool selfTest() = 0;

    /**
     * @brief Check if the IMU sensor is responding.
     * @return true if sensor is present, false otherwise.
     */
    virtual bool isPresent() = 0;

    /**
     * @brief Get the chip ID / WHO_AM_I value.
     * @return Chip ID value.
     */
    virtual uint8_t getChipId() = 0;

    /**
     * @brief Get the firmware version.
     * @return Firmware version value.
     */
    virtual uint32_t getFirmwareVersion() = 0;

    /**
     * @brief Set IMU operation mode.
     * @param mode Target operation mode.
     * @return true if mode switch successful, false otherwise.
     */
    virtual bool setOperationMode(OperationMode mode) = 0;

    /**
     * @brief Compatibility wrapper: switch IMU to NORMAL mode.
     */
    bool powerOn()
    {
        return setOperationMode(OperationMode::NORMAL);
    }

    /**
     * @brief Compatibility wrapper: switch IMU to SUSPEND mode.
     */
    bool powerDown()
    {
        return setOperationMode(OperationMode::SUSPEND);
    }

    /**
     * @brief Get the temperature reading from the sensor.
     * @return Temperature in degrees Celsius, or NaN on error.
     */
    virtual float getTemperature() = 0;

    /**
     * @brief Get the timestamp from the sensor.
     * @return Timestamp value.
     */
    virtual uint32_t getTimestamp() = 0;

    // ==================== Accelerometer Operations ====================

    /**
     * @brief Check if requested IMU data is available.
     * @param mask Data-ready target mask (ACCEL / GYRO / BOTH).
     * @return true if requested data is ready, false otherwise.
     */
    virtual bool isDataReady(uint8_t mask = static_cast<uint8_t>(DataReadyMask::BOTH)) = 0;

    /**
     * @brief Read accelerometer data.
     * @param[out] out Reference to store the read data.
     * @return true if read successful, false otherwise.
     */
    virtual bool readAccel(AccelerometerData &out) = 0;

    /**
     * @brief Enable the accelerometer.
     * @return true if enabled successfully, false otherwise.
     */
    virtual bool enableAccel() = 0;

    /**
     * @brief Disable the accelerometer.
     * @return true if disabled successfully, false otherwise.
     */
    virtual bool disableAccel() = 0;

    /**
     * @brief Check if accelerometer is enabled.
     * @return true if enabled, false otherwise.
     */
    virtual bool isAccelEnabled() = 0;

    /**
     * @brief Set accelerometer full scale range.
     * @param range Full scale range setting.
     * @return true if set successfully, false otherwise.
     */
    virtual bool setAccelFullScaleRange(AccelFullScaleRange range) = 0;

    /**
     * @brief Set accelerometer output data rate.
     * @param data_rate_hz Output data rate in Hz.
     * @return true if set successfully, false otherwise.
     */
    virtual bool setAccelOutputDataRate(float data_rate_hz) = 0;

    /**
     * @brief Read raw accelerometer data without scaling.
     * @param[out] x X-axis raw value.
     * @param[out] y Y-axis raw value.
     * @param[out] z Z-axis raw value.
     * @return true if read successful, false otherwise.
     */
    virtual bool readAccelRaw(int16_t &x, int16_t &y, int16_t &z) = 0;

    // ==================== Gyroscope Operations ====================

    /**
     * @brief Read gyroscope data.
     * @param[out] out Reference to store the read data.
     * @return true if read successful, false otherwise.
     */
    virtual bool readGyro(GyroscopeData &out) = 0;

    /**
     * @brief Enable the gyroscope.
     * @return true if enabled successfully, false otherwise.
     */
    virtual bool enableGyro() = 0;

    /**
     * @brief Disable the gyroscope.
     * @return true if disabled successfully, false otherwise.
     */
    virtual bool disableGyro() = 0;

    /**
     * @brief Check if gyroscope is enabled.
     * @return true if enabled, false otherwise.
     */
    virtual bool isGyroEnabled() = 0;

    /**
     * @brief Set gyroscope full scale range.
     * @param range Full scale range setting.
     * @return true if set successfully, false otherwise.
     */
    virtual bool setGyroFullScaleRange(GyroFullScaleRange range) = 0;

    /**
     * @brief Set gyroscope output data rate.
     * @param data_rate_hz Output data rate in Hz.
     * @return true if set successfully, false otherwise.
     */
    virtual bool setGyroOutputDataRate(float data_rate_hz) = 0;

    /**
     * @brief Read raw gyroscope data without scaling.
     * @param[out] x X-axis raw value.
     * @param[out] y Y-axis raw value.
     * @param[out] z Z-axis raw value.
     * @return true if read successful, false otherwise.
     */
    virtual bool readGyroRaw(int16_t &x, int16_t &y, int16_t &z) = 0;

    // ==================== Synchronous Sample Mode ====================

    /**
     * @brief Enable synchronous sampling mode.
     * @return true if enabled successfully, false otherwise.
     */
    virtual bool enableSyncMode() = 0;

    /**
     * @brief Disable synchronous sampling mode.
     * @return true if disabled successfully, false otherwise.
     */
    virtual bool disableSyncMode() = 0;

    // ==================== Advanced Features (Optional) ====================

    /**
     * @brief Configure the FIFO buffer.
     * @param enable true to enable FIFO, false to disable.
     * @param watermark_samples Number of samples for watermark interrupt.
     * @return true if configured successfully, false otherwise.
     */
    virtual bool configureFifo(bool enable, uint8_t watermark_samples = 16) = 0;

    /**
     * @brief Read data from FIFO buffer.
     * @param[out] accel_data Array to store accelerometer samples.
     * @param accel_count Maximum number of accelerometer samples.
     * @param[out] gyro_data Array to store gyroscope samples.
     * @param gyro_count Maximum number of gyroscope samples.
     * @return Number of samples read, or 0 on error.
     */
    virtual uint16_t readFromFifo(AccelerometerData *accel_data, uint16_t accel_count,
                                   GyroscopeData *gyro_data, uint16_t gyro_count) = 0;
};

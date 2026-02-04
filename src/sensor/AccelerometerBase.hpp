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
 * @file      AccelerometerBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-22
 *
 * @brief Abstract base class for accelerometer sensors
 * @note This class cannot be instantiated directly. Use derived classes such as:
 *       - SensorBMA4XX for BMA4XX series accelerometers
 */
#pragma once

#include "SensorBase.hpp"
#include "AccelerometerUtils.hpp"

/**
 * @brief Abstract base class for accelerometer implementations
 *
 * @note This is an abstract base class and cannot be instantiated directly.
 *       Use derived classes such as SensorBMA4XX, SensorMPU6050, etc.
 */
class AccelerometerBase : public SensorBase<AccelerometerData>
{
protected:
    /**
     * @brief Construct a new AccelerometerBase object
     *
     * @note Constructor is protected to prevent direct instantiation
     */
    explicit AccelerometerBase() : SensorBase(SensorType::ACCELEROMETER) {}

    /**
     * @brief Destroy the AccelerometerBase object
     */
    virtual ~AccelerometerBase() = default;

    // Delete copy operations
    AccelerometerBase(const AccelerometerBase &) = delete;
    AccelerometerBase &operator=(const AccelerometerBase &) = delete;

public:
    // Pure virtual interface functions

    /**
     * @brief Read accelerometer data
     *
     * @param data Reference to store accelerometer data
     * @return true  Read successful
     * @return false Read failed
     */
    virtual bool readData(AccelerometerData &data) override = 0;

    /**
     * @brief Check if new accelerometer data is available
     *
     * @return true  New data available
     * @return false No new data
     */
    virtual bool isDataReady() override = 0;

    /**
     * @brief Reset accelerometer to default state
     *
     * @return true  Reset successful
     * @return false Reset failed
     */
    virtual bool reset() override = 0;

    /**
     * @brief Perform accelerometer self-test
     *
     * @return true  Self-test passed
     * @return false Self-test failed
     */
    virtual bool selfTest() override = 0;

    /**
     * @brief Set accelerometer full-scale range
     *
     * @param range Desired full-scale range
     * @return true  Configuration successful
     * @return false Configuration failed
     */
    virtual bool setFullScaleRange(AccelFullScaleRange range) = 0;

    /**
     * @brief Set accelerometer output data rate
     *
     * @param data_rate_hz Desired output data rate in Hertz
     * @return true        Configuration successful
     * @return false       Configuration failed
     */
    virtual bool setOutputDataRate(float data_rate_hz) = 0;

    /**
     * @brief Set accelerometer operation mode
     *
     * @param mode Desired operation mode
     * @return true  Configuration successful
     * @return false Configuration failed
     */
    virtual bool setOperationMode(OperationMode mode) = 0;

    // Virtual getter functions with default implementations

    /**
     * @brief Get current full-scale range setting
     *
     * @return float Full-scale range in g-force
     */
    virtual float getFullScaleRange() const
    {
        return _config.range;
    }

    /**
     * @brief Get current output data rate setting
     *
     * @return float Output data rate in Hertz
     */
    virtual float getOutputDataRate() const
    {
        return _config.sample_rate;
    }

    /**
     * @brief Get current operation mode setting
     *
     * @return OperationMode Operation mode enumeration
     */
    virtual OperationMode getOperationMode() const
    {
        return _config.mode;
    }
};

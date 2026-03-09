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
 * @file      GyroscopeBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-08
 *
 * @brief Abstract base class for gyroscope sensors
 * @note This class cannot be instantiated directly. Use derived classes such as:
 *       - SensorQMI8658, etc.
 */
#pragma once

#include "SensorBase.hpp"
#include "GyroscopeUtils.hpp"

/**
 * @brief Abstract base class for gyroscope implementations
 *
 * @note This is an abstract base class and cannot be instantiated directly.
 *       Use derived classes for specific gyroscope sensor ICs.
 */
class GyroscopeBase : public SensorBase<GyroscopeData>
{
protected:
    /**
     * @brief Construct a new GyroscopeBase object
     *
     * @note Constructor is protected to prevent direct instantiation.
     */
    explicit GyroscopeBase() : SensorBase(SensorType::GYROSCOPE) {}

    /**
     * @brief Destroy the GyroscopeBase object
     */
    virtual ~GyroscopeBase() = default;

    // Delete copy operations
    GyroscopeBase(const GyroscopeBase &) = delete;
    GyroscopeBase &operator=(const GyroscopeBase &) = delete;

public:
    /**
     * @brief Read gyroscope data
     *
     * @param data Reference to store gyroscope data
     * @return true  Read successful
     * @return false Read failed
     */
    virtual bool readData(GyroscopeData &data) override = 0;

    /**
     * @brief Check if new gyroscope data is available
     *
     * @return true  New data available
     * @return false No new data
     */
    virtual bool isDataReady() override = 0;

    /**
     * @brief Reset gyroscope to default state
     *
     * @return true  Reset successful
     * @return false Reset failed
     */
    virtual bool reset() override = 0;

    /**
     * @brief Perform gyroscope self-test
     *
     * @return true  Self-test passed
     * @return false Self-test failed
     */
    virtual bool selfTest() override = 0;

    /**
     * @brief Set gyroscope full-scale range
     *
     * @param range Desired full-scale range
     * @return true  Configuration successful
     * @return false Configuration failed
     */
    virtual bool setFullScaleRange(GyroFullScaleRange range) = 0;

    /**
     * @brief Set gyroscope output data rate
     *
     * @param data_rate_hz Desired output data rate in Hertz
     * @return true        Configuration successful
     * @return false       Configuration failed
     */
    virtual bool setOutputDataRate(float data_rate_hz) = 0;
};

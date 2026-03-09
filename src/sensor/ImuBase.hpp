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
 * 
 */
#pragma once

#include "SensorDefs.hpp"
/**
 * @brief Abstract IMU device interface (composition target).
 *
 * A concrete IMU driver (e.g., QMI8658...) should implement this.
 * Then adapters can expose AccelerometerBase / GyroscopeBase views without multiple inheritance.
 */
class ImuBase
{
public:
    virtual ~ImuBase() = default;

    // Common / device-level operations
    virtual bool reset() = 0;
    virtual bool selfTest() = 0;

    // Accel capabilities
    virtual bool accelIsDataReady() = 0;
    virtual bool readAccel(AccelerometerData &out) = 0;
    virtual bool setAccelFullScaleRange(AccelFullScaleRange range) = 0;
    virtual bool setAccelOutputDataRate(float data_rate_hz) = 0;

    // Gyro capabilities
    virtual bool gyroIsDataReady() = 0;
    virtual bool readGyro(GyroscopeData &out) = 0;
    virtual bool setGyroFullScaleRange(GyroFullScaleRange range) = 0;
    virtual bool setGyroOutputDataRate(float data_rate_hz) = 0;
};

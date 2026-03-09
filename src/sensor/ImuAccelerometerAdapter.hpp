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
 * @file      ImuAccelerometerAdapter.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-08
 * 
 */
#pragma once

#include "AccelerometerBase.hpp"
#include "ImuBase.hpp"

/**
 * @brief Adapter: expose an IMU device as an AccelerometerBase.
 */
class ImuAccelerometerAdapter : public AccelerometerBase
{
public:
    explicit ImuAccelerometerAdapter(ImuBase &dev) : _dev(dev) {}

    bool readData(AccelerometerData &data) override
    {
        return _dev.readAccel(data);
    }

    bool isDataReady() override
    {
        return _dev.accelIsDataReady();
    }

    bool reset() override
    {
        return _dev.reset();
    }

    bool selfTest() override
    {
        // device-level self-test; if you want accel-only selftest you can split later
        return _dev.selfTest();
    }

    bool setFullScaleRange(AccelFullScaleRange range) override
    {
        return _dev.setAccelFullScaleRange(range);
    }

    bool setOutputDataRate(float data_rate_hz) override
    {
        return _dev.setAccelOutputDataRate(data_rate_hz);
    }

private:
    ImuBase &_dev;
};
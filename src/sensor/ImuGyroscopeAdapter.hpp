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
 * @file      ImuGyroscopeAdapter.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-08
 * 
 */
#pragma once

#include "GyroscopeBase.hpp"
#include "ImuBase.hpp"

/**
 * @brief Adapter: expose an IMU device as a GyroscopeBase.
 */
class ImuGyroscopeAdapter : public GyroscopeBase
{
public:
    explicit ImuGyroscopeAdapter(ImuBase &dev) : _dev(dev) {}

    bool readData(GyroscopeData &data) override
    {
        return _dev.readGyro(data);
    }

    bool isDataReady() override
    {
        return _dev.gyroIsDataReady();
    }

    bool reset() override
    {
        return _dev.reset();
    }

    bool selfTest() override
    {
        return _dev.selfTest();
    }

    bool setFullScaleRange(GyroFullScaleRange range) override
    {
        return _dev.setGyroFullScaleRange(range);
    }

    bool setOutputDataRate(float data_rate_hz) override
    {
        return _dev.setGyroOutputDataRate(data_rate_hz);
    }

private:
    ImuBase &_dev;
};
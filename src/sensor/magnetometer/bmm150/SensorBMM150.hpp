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
 * @file      SensorBMM150.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-17
 *
 */

#pragma once
#include "../../../bosch/BMM150/bmm150.h"
#include "../../../platform/comm/ComplexStaticDeviceWithHal.hpp"
#include "../../MagnetometerBase.hpp"
#include <memory>

static constexpr uint8_t BMM150_DEFAULT_I2C_ADDRESS = 0x10;
static constexpr uint8_t BMM150_I2C_ADDRESS_CSB_LOW_SDO_HIGH = 0x11;
static constexpr uint8_t BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW = 0x12;
static constexpr uint8_t BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH = 0x13;

/**
 * @brief Driver for Bosch BMM150 magnetometer.
 *
 * This class adapts the Bosch BMM150 C driver to the unified MagnetometerBase
 * interface used by this library.
 */
class SensorBMM150 : public MagnetometerBase, public ComplexStaticDeviceWithHal
{
public:
    /**
     * @brief Construct a BMM150 driver instance.
     */
    SensorBMM150();

    /**
     * @brief Destroy the BMM150 driver instance.
     */
    ~SensorBMM150() = default;

    /**
     * @brief Read one magnetometer sample.
     * @param data Output data container.
     * @return true on success.
     */
    bool readData(MagnetometerData &data) override;

    /**
     * @brief Check if fresh data is ready.
     * @return true if data-ready interrupt flag is asserted.
     */
    bool isDataReady() override;

    /**
     * @brief Reset the sensor using external pin and software reset.
     * @return true on success.
     */
    bool reset() override;

    /**
     * @brief Execute BMM150 normal self-test.
     * @return true if self-test passes.
     */
    bool selfTest() override;

    /**
     * @brief Set full-scale range.
     * @note BMM150 range is fixed by hardware model; only FS_16G is accepted.
     * @param range Requested full-scale range.
     * @return true when accepted.
     */
    bool setFullScaleRange(MagFullScaleRange range) override;

    /**
     * @brief Set output data rate.
     * @param odr Output data rate in Hz.
     * @return true on success.
     */
    bool setOutputDataRate(float odr) override;

    /**
     * @brief Set operation mode.
     * @param mode Operation mode from MagnetometerBase.
     * @return true on success.
     */
    bool setOperationMode(OperationMode mode) override;

    /**
     * @brief Set oversampling level by mapping to BMM150 XY/Z repetition values.
     * @param osr Oversampling ratio level.
     * @return true on success.
     */
    bool setOversamplingRate(MagOverSampleRatio osr) override;

    /**
     * @brief Set downsampling rate.
     * @note BMM150 does not support configurable downsampling.
     * @param dsr Downsampling ratio.
     * @return Always false.
     */
    bool setDownsamplingRate(MagDownSampleRatio dsr) override;

    /**
     * @brief Apply combined magnetometer configuration.
     * @param mode Operation mode.
     * @param range Full-scale range.
     * @param odr Output data rate in Hz.
     * @param osr Oversampling ratio.
     * @param dsr Downsampling ratio (ignored).
     * @return true on success.
     */
    bool configMagnetometer(OperationMode mode, MagFullScaleRange range, float odr,
                            MagOverSampleRatio osr, MagDownSampleRatio dsr = MagDownSampleRatio::DSR_1) override;

private:
    bool initImpl(uint8_t param) override;
    bool updateInterruptStatus();

private:
    std::unique_ptr<struct bmm150_dev> _dev;
    int8_t _error_code;
    struct bmm150_settings _settings;
};

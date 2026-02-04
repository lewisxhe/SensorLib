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
 * @file      MagnetometerBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-22
 */
#pragma once

#include "SensorBase.hpp"
#include "MagnetometerUtils.hpp"

class MagnetometerBase : public SensorBase<MagnetometerData>
{
public:

    /**
     * @brief  Constructor for the MagnetometerBase class.
     * @note   This constructor initializes the magnetometer with default settings.
     * @retval None
     */
    explicit MagnetometerBase() : SensorBase(SensorType::MAGNETOMETER),
        _oversampling_rate(128), _declination_rad(0.0) {}

    // *INDENT-OFF*
    /**
     * @brief  Destructor for the MagnetometerBase class.
     * @note   This virtual destructor ensures proper cleanup of derived classes.
     * @retval None
     */
    virtual ~MagnetometerBase() = default;

    /**
     * @brief  Checks if new data is available from the magnetometer.
     * @note   This function should be called periodically to determine if new
     *         sensor data is ready to be read.
     * @retval True if new data is available, false otherwise.
     */
    bool readData(MagnetometerData &data) override = 0;

    /**
     * @brief  Checks if new data is available from the magnetometer.
     * @note   This function should be called periodically to determine if new
     *         sensor data is ready to be read.
     * @retval True if new data is available, false otherwise.
     */
    bool isDataReady() override = 0;
    /**
     * @brief  Resets the magnetometer to its default settings.
     * @note   This function should be called to reset the sensor configuration.
     * @retval True if the reset was successful, false otherwise.
     */
    bool reset() override = 0;

    /**
     * @brief  Performs a self-test on the magnetometer.
     * @note   This function should be called to verify the sensor's functionality.
     * @retval True if the self-test passed, false otherwise.
     */
    bool selfTest() override = 0;

    /**
     * @brief  Sets the full-scale range for the magnetometer.
     * @note   This function should be called to configure the sensor's measurement range.
     * @param  range: The desired full-scale range.
     * @retval True if the range was set successfully, false otherwise.
     */
    virtual bool setFullScaleRange(MagFullScaleRange range) = 0;

    /**
     * @brief  Sets the output data rate for the magnetometer.
     * @note   This function should be called to configure the sensor's output data rate.
     * @param  odr: The desired output data rate in Hz.
     * @retval True if the output data rate was set successfully, false otherwise.
     */
    virtual bool setOutputDataRate(float odr) = 0;

    /**
     * @brief  Sets the operation mode for the magnetometer.
     * @note   This function should be called to configure the sensor's operation mode.
     * @param  mode: The desired operation mode.
     * @retval True if the mode was set successfully, false otherwise.
     */
    virtual bool setOperationMode(OperationMode mode) = 0;

    /**
     * @brief  Sets the oversampling rate for the magnetometer.
     * @note   This function should be called to configure the sensor's oversampling rate.
     * @param  osr: The desired oversampling rate.
     * @retval True if the oversampling rate was set successfully, false otherwise.
     */
    virtual bool setOversamplingRate(MagOverSampleRatio osr) = 0;

    /**
     * @brief  Sets the downsampling rate for the magnetometer.
     * @note   This function should be called to configure the sensor's downsampling rate.
     * @param  dsr: The desired downsampling rate.
     * @retval True if the downsampling rate was set successfully, false otherwise.
     */
    virtual bool setDownsamplingRate(MagDownSampleRatio dsr) = 0;

    /**
     * @brief  Gets the full-scale range for the magnetometer.
     * @note   This function should be called to retrieve the sensor's measurement range.
     * @retval The current full-scale range (e.g., 2.0, 4.0, 8.0, 16.0).
     */
    virtual float getFullScaleRange() const { 
        return _config.range;
    }

    /**
     * @brief  Gets the operation mode for the magnetometer.
     * @note   This function should be called to retrieve the sensor's current operation mode.
     * @retval The current operation mode.
     */
    virtual OperationMode getOperationMode() const { 
        return _config.mode; 
    }

    /**
     * @brief  Gets the oversampling rate for the magnetometer.
     * @note   This function should be called to retrieve the sensor's current oversampling rate.
     * @retval The current oversampling rate.
     */
    virtual uint16_t getOversamplingRate() const { return _oversampling_rate; }

    /**
     * @brief Set the magnetic declination correction value.
     * 
     * Magnetic declination is the angle between true north (geographic north) and
     * magnetic north. This correction is essential for accurate compass headings.
     * 
     * @note Positive values indicate east declination, negative values indicate west declination.
     *       Example: +7.5° for Boulder, Colorado; -3.33° for London, UK.
     * 
     * @param declination_deg Magnetic declination in degrees
     */
    virtual void setDeclination(float declination_deg) {
        _declination_rad = declination_deg * M_PI / 180.0f;
    }

    /**
     * @brief Get the currently set magnetic declination in degrees.
     * 
     * @return Magnetic declination in degrees (positive east, negative west)
     */
    virtual float getDeclinationDeg() const {
        return _declination_rad * 180.0f / M_PI;
    }
    
    /**
     * @brief Get the currently set magnetic declination in radians.
     * 
     * This is the internal representation used for calculations.
     * 
     * @return Magnetic declination in radians (positive east, negative west)
     */
    virtual float getDeclinationRad() const {
        return _declination_rad;
    }

    // *INDENT-ON*

protected:
    uint16_t _oversampling_rate;
    float _declination_rad;
};

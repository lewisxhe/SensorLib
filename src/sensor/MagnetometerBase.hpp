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

/**
* @brief Enumeration defining full-scale range settings for the sensor.
*/
enum class MagFullScaleRange {
    FS_2G = 0,      // ±2 Gauss
    FS_4G,          // ±4 Gauss
    FS_8G,          // ±8 Gauss
    FS_12G,         // ±12 Gauss
    FS_16G,         // ±16 Gauss
    FS_20G,         // ±20 Gauss
    FS_30G,         // ±30 Gauss
    FS_32G          // ±32 Gauss
};

/**
* @brief Enumeration defining over-sample ratios for the sensor.
*/
enum class MagOverSampleRatio {
    OSR_8,   ///< 8x over-sample ratio
    OSR_4,   ///< 4x over-sample ratio
    OSR_2,   ///< 2x over-sample ratio
    OSR_1    ///< 1x over-sample ratio
};

/**
 * @brief Enumeration defining down-sample ratios for the sensor.
 */
enum class MagDownSampleRatio {
    DSR_1,   ///< 1x down-sample ratio
    DSR_2,   ///< 2x down-sample ratio
    DSR_4,   ///< 4x down-sample ratio
    DSR_8,   ///< 8x down-sample ratio
};

enum class MagLowPassFilter {
    LPF_1,  ///< 1 Hz low-pass filter
    LPF_2,  ///< 2 Hz low-pass filter
    LPF_4,  ///< 4 Hz low-pass filter
    LPF_8,  ///< 8 Hz low-pass filter
    LPF_16  ///< 16 Hz low-pass filter
};

enum class MagOperationMode {
    SUSPEND = 0,
    NORMAL,
    SINGLE_MEASUREMENT,
    CONTINUOUS_MEASUREMENT,
    EXTERNAL_TRIGGER_MEASUREMENT
};

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
    virtual bool setOperationMode(MagOperationMode mode) = 0;

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
        return _config.full_scale_range;
    }

    /**
     * @brief  Gets the operation mode for the magnetometer.
     * @note   This function should be called to retrieve the sensor's current operation mode.
     * @retval The current operation mode.
     */
    virtual MagOperationMode getOperationMode() const { 
        return static_cast<MagOperationMode>(_config.mode); 
    }

    /**
     * @brief  Gets the oversampling rate for the magnetometer.
     * @note   This function should be called to retrieve the sensor's current oversampling rate.
     * @retval The current oversampling rate.
     */
    virtual uint16_t getOversamplingRate() const { return _oversampling_rate; }


    /**
     * @brief  Sets the interrupt threshold for the magnetometer.
     * @note   This function should be called to configure the sensor's interrupt threshold.
     * @param  enable: Whether to enable the interrupt.
     * @param  threshold: The interrupt threshold value (in uT).
     * @param  axis: The axis mask (0x01=X, 0x02=Y, 0x04=Z, 0x07=all axes).
     * @retval True if the interrupt threshold was set successfully, false otherwise.
     */
    virtual bool setInterruptThreshold(bool enable, float threshold, uint8_t axis = 0x07) {
        // TODO: Implement interrupt threshold setting
        return false;
    }


    /**
     * @brief Convert degrees-minutes-seconds to decimal degrees
     * @param degrees Whole degrees (can be negative)
     * @param minutes Arc minutes (0-59)
     * @param seconds Arc seconds (0-59.999...)
     * @return Decimal degrees
     */
    float dmsToDecimalDegrees(int degrees, int minutes, float seconds = 0.0f) {
        float sign = (degrees < 0 || (degrees == 0 && (minutes < 0 || seconds < 0))) ? -1.0f : 1.0f;
        return sign * (abs(degrees) + minutes / 60.0f + seconds / 3600.0f);
    }

    /**
     * @brief Convert degrees-minutes-seconds to radians
     */
    float dmsToRadians(int degrees, int minutes, float seconds = 0.0f) {
        return dmsToDecimalDegrees(degrees, minutes, seconds) * M_PI / 180.0f;
    }

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

    /**
     * @brief Calculate heading (yaw) angle from magnetometer data in radians.
     *
     * This function computes the magnetic heading in the horizontal plane using
     * the x and y components of the magnetometer. The result is corrected for
     * magnetic declination (difference between true north and magnetic north).
     *
     * @note This calculation assumes the device is level (no tilt compensation).
     * For accurate results when device is tilted, use with accelerometer data.
     *
     * @param data Magnetometer data structure containing magnetic field measurements
     * @param declination Magnetic declination correction in radians (positive east)
     * @return Heading angle in radians, range [0, 2π) (0 = magnetic north)
     */
    virtual float calculateHeading(const MagnetometerData& data, float declination = 0.0f) const
    {
        const auto& mag = data.magnetic_field;

        // Handle edge case when magnetic field is zero or aligned vertically
        if (mag.x == 0.0f && mag.y == 0.0f) {
            return 0.0f;
        }

        // Calculate angle from x-axis (east) using arctangent of y/x
        // atan2 returns angle in range [-π, π]
        float heading = atan2f(mag.y, mag.x);

        // Convert to [0, 2π) range (0 to 2π radians)
        if (heading < 0) {
            heading += 2 * M_PI;
        }

        // Apply magnetic declination correction
        // Positive declination adds to heading (east declination)
        heading += declination;

        // Normalize to [0, 2π) range after declination adjustment
        if (heading > 2 * M_PI) {
            heading -= 2 * M_PI;
        } else if (heading < 0) {
            heading += 2 * M_PI;
        }

        return heading;
    }

    /**
     * @brief Calculate heading (yaw) angle from magnetometer data in degrees.
     *
     * This is a convenience wrapper that returns the heading in degrees (0-360).
     *
     * @param data Magnetometer data structure containing magnetic field measurements
     * @param declination_deg Magnetic declination correction in degrees (positive east)
     * @return Heading angle in degrees, range [0, 360) (0 = magnetic north)
     */
    virtual float calculateHeadingDegrees(const MagnetometerData& data, float declination_deg = 0.0f) const
    {
        const auto& mag = data.magnetic_field;
        if (mag.x == 0.0f && mag.y == 0.0f) {
            return 0.0f;
        }
        float heading_deg = atan2f(mag.y, mag.x) * 180.0f / M_PI;
        if (heading_deg < 0) {
            heading_deg += 360.0f;
        }
        heading_deg += declination_deg;
        if (heading_deg >= 360.0f) {
            heading_deg -= 360.0f;
        } else if (heading_deg < 0) {
            heading_deg += 360.0f;
        }
        return heading_deg;
    }

    /**
     * @brief Calculate total magnetic field strength.
     *
     * Computes the magnitude of the magnetic field vector from all three axes.
     * This represents the total strength of the magnetic field at the sensor.
     *
     * @param data Magnetometer data structure containing magnetic field measurements
     * @return Magnetic field strength in the same units as input (typically microtesla, μT)
     */
    virtual float calculateMagneticStrength(const MagnetometerData& data) const
    {
        const auto& mag = data.magnetic_field;

        // Compute Euclidean norm of the magnetic field vector
        return sqrtf(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
    }

    /**
     * @brief  Converts Gauss to microTesla.
     * @note   This function is used to convert magnetic field strength from Gauss to microTesla.
     * @param  gauss: The magnetic field strength in Gauss.
     * @retval The magnetic field strength in microTesla.
     */
    float gaussToMicroTesla(float gauss) const
    {
        return gauss * 100.0f; // 1 Gauss = 100 μT
    }

    /**
     * @brief  Converts microTesla to Gauss.
     * @note   This function is used to convert magnetic field strength from microTesla to Gauss.
     * @param  ut: The magnetic field strength in microTesla.
     * @retval The magnetic field strength in Gauss.
     */
    float microTeslaToGauss(float ut) const
    {
        return ut * 0.01f; // 1 μT = 0.01 Gauss
    }
    // *INDENT-ON*

protected:
    uint16_t _oversampling_rate;
    float _declination_rad;
};

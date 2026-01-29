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
 */
#pragma once

#include "SensorBase.hpp"

/**
 * @brief  Structure representing the raw and processed data from the accelerometer.
 * @note   This structure contains the acceleration data in both raw and processed formats,
 *         as well as temperature information. Whether a temperature value is valid depends
 *         on the specific sensor used.
 */
struct AccelerometerData  {
    SensorVector mps2;                  // Accel (m/s²)
    RawVector raw;                      // Raw data
    float temperature;                  // Temperature (°C)
};

/**
 * @brief  Enumeration defining the full-scale range settings for the accelerometer.
 */
enum class AccelFullScaleRange {
    FS_2G = 0,      // ±2g
    FS_4G,          // ±4g
    FS_8G,          // ±8g
    FS_16G,         // ±16g
    FS_32G,         // ±32g
    FS_64G,         // ±64g
    FS_128G         // ±128g
};

/**
 * @brief  Enumeration defining the operation modes for the accelerometer.
 */
enum class AccelOperationMode {
    SUSPEND = 0,
    LOW_POWER,
    NORMAL,
    HIGH_RESOLUTION,
    ULTRA_HIGH_RESOLUTION
};

/**
 * @brief  Enumeration defining the interrupt pin mapping for the accelerometer.
 */
enum class InterruptPinMap  {
    PIN1 = 1,
    PIN2,
};


class AccelerometerUtils
{
public:
    /**
     * @brief  Convert g-force to m/s²
     * @note   1g = 9.80665 m/s²
     * @param  g_value: The g-force value to convert
     * @retval The converted value in m/s²
     */
    static float gToMps2(float g_value)
    {
        return g_value * 9.80665f; // 1g = 9.80665 m/s²
    }

    /**
     * @brief  Convert m/s² to g-force
     * @param  mps2_value: The value in m/s² to convert
     * @retval The converted value in g-force
     */
    static float mps2ToG(float mps2_value)
    {
        return mps2_value / 9.80665f;
    }

    /**
     * @brief  Convert full-scale range to g-force
     * @param  range: The full-scale range to convert
     * @retval The converted value in g-force
     */
    static float rangeToG(AccelFullScaleRange range)
    {
        switch (range) {
        case AccelFullScaleRange::FS_2G: return 2.0f;
        case AccelFullScaleRange::FS_4G: return 4.0f;
        case AccelFullScaleRange::FS_8G: return 8.0f;
        case AccelFullScaleRange::FS_16G: return 16.0f;
        case AccelFullScaleRange::FS_32G: return 32.0f;
        case AccelFullScaleRange::FS_64G: return 64.0f;
        case AccelFullScaleRange::FS_128G: return 128.0f;
        default: return 8.0f;
        }
    }
    /**
     * @brief Calculate the magnitude of the acceleration vector.
     *
     * This function computes the Euclidean norm (magnitude) of the acceleration vector
     * using the formula: sqrt(x² + y² + z²).
     *
     * @param data Reference to the AccelerometerData structure containing acceleration values in m/s².
     * @return float The magnitude of the acceleration vector in m/s².
     */
    static float calculateMagnitude(const AccelerometerData &data)
    {
        const auto &acc = data.mps2;
        return sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    }

    /**
     * @brief Calculate the inclination angle between a specific axis and the gravity vector (in radians).
     *
     * This function computes the angle between the specified axis (0=X, 1=Y, 2=Z) and the
     * total acceleration vector. The result is in radians and represents the tilt angle
     * of that axis relative to the current gravity direction.
     *
     * @note Returns NAN if the acceleration magnitude is too small or axis parameter is invalid.
     *
     * @param data Reference to the AccelerometerData structure containing acceleration values.
     * @param axis The axis to calculate inclination for (0=X, 1=Y, 2=Z).
     * @return float Inclination angle in radians, or NAN if calculation is invalid.
     */
    static float calculateInclination(const AccelerometerData &data, uint8_t axis)
    {
        if (axis > 2) return NAN;

        float magnitude = calculateMagnitude(data);
        if (magnitude == 0.0f) return NAN;

        switch (axis) {
        case 0: // X-axis
            return acosf(data.mps2.x / magnitude);
        case 1: // Y-axis
            return acosf(data.mps2.y / magnitude);
        case 2: // Z-axis
            return acosf(data.mps2.z / magnitude);
        default:
            break;
        }
        return NAN;
    }

    /**
     * @brief Calculate the inclination angle between a specific axis and the gravity vector (in degrees).
     *
     * This is a convenience wrapper for calculateInclination that converts the result from radians to degrees.
     *
     * @param data Reference to the AccelerometerData structure containing acceleration values.
     * @param axis The axis to calculate inclination for (0=X, 1=Y, 2=Z).
     * @return float Inclination angle in degrees, or NAN if calculation is invalid.
     */
    static float calculateInclinationDegrees(const AccelerometerData &data, uint8_t axis)
    {
        float rad = calculateInclination(data, axis);
        return rad * 180.0f / M_PI;
    }
};

class AccelerometerBase : public SensorBase<AccelerometerData>
{
public:
    /**
     * @brief  Constructor for the AccelerometerBase class.
     * @note   Initializes the sensor type to ACCELEROMETER.
     * @retval None
     */
    explicit AccelerometerBase() : SensorBase(SensorType::ACCELEROMETER) {}

    /**
     * @brief  Destructor for the AccelerometerBase class.
     * @note   Cleans up any resources used by the class.
     * @retval None
     */
    virtual ~AccelerometerBase() = default;

    /**
     * @brief  Get the accelerometer data.
     * @note   This function retrieves the current data representation of the accelerometer.
     * @param  &data: Reference to the structure to store the accelerometer data.
     * @retval True if data was successfully read, false otherwise.
     */
    bool readData(AccelerometerData &data) override = 0;

    /**
     * @brief  Check if new accelerometer data is available.
     * @note   This function checks the status of the accelerometer to determine if new data
     *         has been acquired since the last read.
     * @retval True if new data is available, false otherwise.
     */
    bool isDataReady() override = 0;

    /**
     * @brief  Reset the accelerometer.
     * @note   This function resets the internal state of the accelerometer.
     * @retval True if the reset was successful, false otherwise.
     */
    bool reset() override = 0;

    /**
     * @brief  Perform a self-test on the accelerometer.
     * @note   This function initiates a self-test procedure to verify the functionality
     *         of the accelerometer.
     * @retval True if the self-test passed, false otherwise.
     */
    bool selfTest() override = 0;

    /**
     * @brief  Set the full-scale range of the accelerometer.
     * @note   This function configures the accelerometer to operate within a specific
     *         range of acceleration values.
     * @param  range: The desired full-scale range (e.g., ±2g, ±4g, ±8g, ±16g).
     * @retval True if the configuration was successful, false otherwise.
     */
    virtual bool setFullScaleRange(AccelFullScaleRange range) = 0;

    /**
     * @brief  Set the output data rate of the accelerometer.
     * @note   This function configures the accelerometer to output data at a specific
     *         rate.
     * @param  bandwidth: The desired output data rate in Hz.
     * @retval True if the configuration was successful, false otherwise.
     */
    virtual bool setOutputDataRate(float bandwidth) = 0;

    /**
     * @brief  Set the operation mode of the accelerometer.
     * @note   This function configures the accelerometer to operate in a specific mode.
     * @param  mode: The desired operation mode.
     * @retval True if the configuration was successful, false otherwise.
     */
    virtual bool setOperationMode(AccelOperationMode mode) = 0;

    /**
     * @brief  Get the full-scale range of the accelerometer.
     * @note   This function retrieves the current full-scale range setting of the
     *         accelerometer.
     * @retval The current full-scale range (e.g., 2.0, 4.0, 8.0, 16.0).
     */
    virtual float getFullScaleRange() const
    {
        return _config.full_scale_range;
    }

    /**
     * @brief  Get the output data rate of the accelerometer.
     * @note   This function retrieves the current output data rate setting of the
     *         accelerometer.
     * @retval The current output data rate in Hz.
     */
    virtual float getOutputDataRate() const
    {
        return _config.data_rate_hz;
    }

    /**
     * @brief  Get the operation mode of the accelerometer.
     * @note   This function retrieves the current operation mode setting of the
     *         accelerometer.
     * @retval The current operation mode.
     */
    virtual AccelOperationMode getOperationMode() const
    {
        return static_cast<AccelOperationMode>(_config.mode);
    }
};

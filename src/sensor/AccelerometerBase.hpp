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
#include <cmath>

/**
 * @brief Enumeration of accelerometer full-scale range settings
 */
enum class AccelFullScaleRange {
    FS_2G = 0,   ///< ±2g range
    FS_4G,       ///< ±4g range
    FS_8G,       ///< ±8g range
    FS_16G,      ///< ±16g range
    FS_32G,      ///< ±32g range
    FS_64G,      ///< ±64g range
    FS_128G      ///< ±128g range
};

/**
 * @brief Enumeration of accelerometer operation modes
 */
enum class AccelOperationMode {
    SUSPEND = 0,               ///< Power-down/suspend mode
    LOW_POWER,                 ///< Low-power mode
    NORMAL,                    ///< Normal operation mode
    HIGH_RESOLUTION,           ///< High-resolution mode
    ULTRA_HIGH_RESOLUTION      ///< Ultra-high-resolution mode
};

/**
 * @brief Enumeration of interrupt pin mapping options
 */
enum class InterruptPinMap {
    PIN1 = 1,   ///< Interrupt pin 1
    PIN2,       ///< Interrupt pin 2
};

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
 * @brief Utility class for accelerometer-specific calculations
 */
class AccelerometerUtils
{
public:
    /**
     * @brief Convert acceleration from g-force to m/s²
     *
     * @param g_value Acceleration in g-force (1g = 9.80665 m/s²)
     * @return float Acceleration in m/s²
     */
    static float gToMps2(float g_value)
    {
        return g_value * 9.80665f; // Standard gravity constant
    }

    /**
     * @brief Convert acceleration from m/s² to g-force
     *
     * @param mps2_value Acceleration in m/s²
     * @return float Acceleration in g-force
     */
    static float mps2ToG(float mps2_value)
    {
        return mps2_value / 9.80665f;
    }

    /**
     * @brief Convert full-scale range enumeration to g-force value
     *
     * @param range Full-scale range enumeration
     * @return float Corresponding g-force value
     */
    static float rangeToG(AccelFullScaleRange range)
    {
        switch (range) {
        case AccelFullScaleRange::FS_2G:   return 2.0f;
        case AccelFullScaleRange::FS_4G:   return 4.0f;
        case AccelFullScaleRange::FS_8G:   return 8.0f;
        case AccelFullScaleRange::FS_16G:  return 16.0f;
        case AccelFullScaleRange::FS_32G:  return 32.0f;
        case AccelFullScaleRange::FS_64G:  return 64.0f;
        case AccelFullScaleRange::FS_128G: return 128.0f;
        default:                           return 8.0f; // Default fallback
        }
    }

    /**
     * @brief Calculate magnitude of acceleration vector
     *
     * Computes the Euclidean norm: √(x² + y² + z²)
     *
     * @param data Accelerometer data structure
     * @return float Magnitude of acceleration in m/s²
     */
    static float calculateMagnitude(const AccelerometerData &data)
    {
        const auto &acc = data.mps2;
        return sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    }

    /**
     * @brief Calculate inclination angle of specified axis relative to gravity
     *
     * Computes the angle between the specified axis and the total acceleration vector.
     * Useful for tilt sensing and orientation detection.
     *
     * @param data Accelerometer data structure
     * @param axis Axis to calculate inclination for (0=X, 1=Y, 2=Z)
     * @return float Inclination angle in radians, or NAN if calculation invalid
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
            return NAN;
        }
    }

    /**
     * @brief Calculate inclination angle in degrees
     *
     * Convenience wrapper that converts radians to degrees.
     *
     * @param data Accelerometer data structure
     * @param axis Axis to calculate inclination for (0=X, 1=Y, 2=Z)
     * @return float Inclination angle in degrees, or NAN if calculation invalid
     */
    static float calculateInclinationDegrees(const AccelerometerData &data, uint8_t axis)
    {
        float rad = calculateInclination(data, axis);
        return isnan(rad) ? NAN : rad * 180.0f / M_PI;
    }
};

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
    virtual bool setOperationMode(AccelOperationMode mode) = 0;

    // Virtual getter functions with default implementations

    /**
     * @brief Get current full-scale range setting
     *
     * @return float Full-scale range in g-force
     */
    virtual float getFullScaleRange() const
    {
        return _config.full_scale_range;
    }

    /**
     * @brief Get current output data rate setting
     *
     * @return float Output data rate in Hertz
     */
    virtual float getOutputDataRate() const
    {
        return _config.data_rate_hz;
    }

    /**
     * @brief Get current operation mode setting
     *
     * @return AccelOperationMode Operation mode enumeration
     */
    virtual AccelOperationMode getOperationMode() const
    {
        return static_cast<AccelOperationMode>(_config.mode);
    }
};

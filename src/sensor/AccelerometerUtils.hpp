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
 * @file      AccelerometerUtils.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-04
 *
 * @brief Utility class for accelerometer-specific calculations
 */
#pragma once

#include "SensorDefs.hpp"

// *INDENT-OFF*
/**
 * @brief Utility class for accelerometer-specific calculations
 */
namespace AccelerometerUtils
{
    /**
     * @brief Convert acceleration from g-force to m/s²
     *
     * @param g_value Acceleration in g-force (1g = 9.80665 m/s²)
     * @return float Acceleration in m/s²
     */
    float gToMps2(float g_value)
    {
        return g_value * 9.80665f; // Standard gravity constant
    }

    /**
     * @brief Convert acceleration from m/s² to g-force
     *
     * @param mps2_value Acceleration in m/s²
     * @return float Acceleration in g-force
     */
    float mps2ToG(float mps2_value)
    {
        return mps2_value / 9.80665f;
    }

    /**
     * @brief Convert full-scale range enumeration to g-force value
     *
     * @param range Full-scale range enumeration
     * @return float Corresponding g-force value
     */
    float rangeToG(AccelFullScaleRange range)
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
    float calculateMagnitude(const AccelerometerData &data)
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
    float calculateInclination(const AccelerometerData &data, uint8_t axis)
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
    float calculateInclinationDegrees(const AccelerometerData &data, uint8_t axis)
    {
        float rad = calculateInclination(data, axis);
        return isnan(rad) ? NAN : rad * 180.0f / M_PI;
    }
};
// *INDENT-ON*

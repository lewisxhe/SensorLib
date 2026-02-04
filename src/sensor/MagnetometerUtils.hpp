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
namespace MagnetometerUtils
{
    /**
     * @brief Convert degrees-minutes-seconds to decimal degrees
     * @param degrees Whole degrees (can be negative)
     * @param minutes Arc minutes (0-59)
     * @param seconds Arc seconds (0-59.999...)
     * @return Decimal degrees
     */
    float dmsToDecimalDegrees(int degrees, int minutes, float seconds = 0.0f)
    {
        float sign = (degrees < 0 || (degrees == 0 && (minutes < 0 || seconds < 0))) ? -1.0f : 1.0f;
        return sign * (abs(degrees) + minutes / 60.0f + seconds / 3600.0f);
    }

    /**
     * @brief Convert degrees-minutes-seconds to radians
     */
    float dmsToRadians(int degrees, int minutes, float seconds = 0.0f)
    {
        return dmsToDecimalDegrees(degrees, minutes, seconds) * M_PI / 180.0f;
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
    float calculateHeading(const MagnetometerData& data, float declination = 0.0f)
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
    float calculateHeadingDegrees(const MagnetometerData& data, float declination_deg = 0.0f)
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
    float calculateMagneticStrength(const MagnetometerData& data)
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
    float gaussToMicroTesla(float gauss)
    {
        return gauss * 100.0f; // 1 Gauss = 100 μT
    }

    /**
     * @brief  Converts microTesla to Gauss.
     * @note   This function is used to convert magnetic field strength from microTesla to Gauss.
     * @param  ut: The magnetic field strength in microTesla.
     * @retval The magnetic field strength in Gauss.
     */
    float microTeslaToGauss(float ut)
    {
        return ut * 0.01f; // 1 μT = 0.01 Gauss
    }


    float rangeToGauss(MagFullScaleRange range)
    {
        switch (range) {
        case MagFullScaleRange::FS_4G:
            return 4.0f;
        case MagFullScaleRange::FS_8G:
            return 8.0f;
        case MagFullScaleRange::FS_12G:
            return 12.0f;
        case MagFullScaleRange::FS_16G:
            return 16.0f;
        case MagFullScaleRange::FS_25G:
            return 25.0f;
        case MagFullScaleRange::FS_30G:
            return 30.0f;
        case MagFullScaleRange::FS_32G:
            return 32.0f;
        default:
            return 0.0f;
        }
        return 0.0f;
    }
}
// *INDENT-ON*
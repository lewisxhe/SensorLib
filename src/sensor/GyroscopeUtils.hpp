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
 * @file      GyroscopeUtils.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-04
 *
 * @brief Utility class for gyroscope-specific calculations
 */
#pragma once

#include "SensorDefs.hpp"
#include <cmath>

// *INDENT-OFF*
namespace GyroscopeUtils
{
    /**
     * @brief Convert full-scale range enumeration to degrees per second value
     *
     * @param range Full-scale range enumeration
     * @return float Corresponding degrees per second value
     */
    inline float rangeToDegreesPerSecond(GyroFullScaleRange range)
    {
        switch (range) {
        case GyroFullScaleRange::FS_125_DPS:   return 125.0f;
        case GyroFullScaleRange::FS_250_DPS:   return 250.0f;
        case GyroFullScaleRange::FS_500_DPS:   return 500.0f;
        case GyroFullScaleRange::FS_1000_DPS:   return 1000.0f;
        case GyroFullScaleRange::FS_2000_DPS:   return 2000.0f;
        case GyroFullScaleRange::FS_4000_DPS:   return 4000.0f;
        default: break;
        }
        return 500.0f; // Default fallback
    }
}
// *INDENT-ON*

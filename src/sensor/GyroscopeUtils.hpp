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
    float rangeToDegreesPerSecond(GyroFullScaleRange range)
    {
        switch (range) {
        case GyroFullScaleRange::FS_125_DPS:   return 125.0f;
        case GyroFullScaleRange::FS_250_DPS:   return 250.0f;
        case GyroFullScaleRange::FS_500_DPS:   return 500.0f;
        case GyroFullScaleRange::FS_1000_DPS:   return 1000.0f;
        case GyroFullScaleRange::FS_2000_DPS:   return 2000.0f;
        case GyroFullScaleRange::FS_4000_DPS:   return 4000.0f;
        default:                           return 500.0f; // Default fallback
        }
        return 0.0f;
    }
}
// *INDENT-ON*

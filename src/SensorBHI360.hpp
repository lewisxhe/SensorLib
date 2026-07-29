#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorBHI360.hpp is deprecated. Include ImuDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_BHI360
#include "sensor/imu/bhi360/SensorBHI360.hpp"
#endif

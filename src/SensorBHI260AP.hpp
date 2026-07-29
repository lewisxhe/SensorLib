#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorBHI260AP.hpp is deprecated. Include ImuDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_BHI260
#include "sensor/imu/bhi260/SensorBHI260AP.hpp"
#endif

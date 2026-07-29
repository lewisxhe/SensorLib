#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorBHI260AP_Klio.hpp is deprecated. Include ImuDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_BHI260_KLIO
#include "sensor/imu/bhi260/SensorBHI260AP_Klio.hpp"
#endif

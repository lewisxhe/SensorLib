#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorDRV2605.hpp is deprecated. Include HapticDrivers.hpp instead.")

#if !SENSORLIB_EXCLUDE_HAPTIC_DRV2605
#include "haptic/HapticDriver_DRV2605.hpp"


using SensorDRV2605 = HapticDriver_DRV2605;
#endif

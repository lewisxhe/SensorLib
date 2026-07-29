#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorBMA4XX.hpp is deprecated. Include AccelerometerDrv.hpp instead for all BMA4XX sensors.")

#if !SENSORLIB_EXCLUDE_BMA4XX_COMMON
#include "AccelerometerDrv.hpp"
#endif

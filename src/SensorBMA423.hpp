#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorBMA423.hpp is deprecated. Include AccelerometerDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_BMA423
#include "sensor/accelerometer/bma/SensorBMA423.hpp"
#endif

#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorBMA422.hpp is deprecated. Include AccelerometerDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_BMA422
#include "sensor/accelerometer/bma/SensorBMA422.hpp"
#endif

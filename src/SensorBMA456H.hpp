#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorBMA456H.hpp is deprecated. Include AccelerometerDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_BMA456H
#include "sensor/accelerometer/bma/SensorBMA456H.hpp"
#endif

#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorQMC5883L.hpp is deprecated. Include MagnetometerDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_QMC5883L
#include "sensor/magnetometer/qmc/SensorQMC5883L.hpp"
#endif

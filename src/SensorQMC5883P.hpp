#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorQMC5883P.hpp is deprecated. Include MagnetometerDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_QMC5883P
#include "sensor/magnetometer/qmc/SensorQMC5883P.hpp"
#endif

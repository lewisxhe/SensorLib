#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorQMC6310.hpp is deprecated. Include MagnetometerDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_QMC6310
#include "sensor/magnetometer/qmc/SensorQMC6310.hpp"
#endif

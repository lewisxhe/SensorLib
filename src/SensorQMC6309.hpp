#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorQMC6309.hpp is deprecated. Include MagnetometerDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_QMC6309
#include "sensor/magnetometer/qmc/SensorQMC6309.hpp"
#endif

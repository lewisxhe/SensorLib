#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorLTR553.hpp is deprecated. Include LightSensorDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_LTR553
#include "sensor/light_sensor/SensorLTR553.hpp"
#endif

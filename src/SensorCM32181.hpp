#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorCM32181.hpp is deprecated. Include LightSensorDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_CM32181
#include "sensor/light_sensor/SensorCM32181.hpp"
#endif

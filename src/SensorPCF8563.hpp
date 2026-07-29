#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorPCF8563.hpp is deprecated. Include RtcDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_PCF8563
#include "time/pcf8563/SensorPCF8563.hpp"
#endif

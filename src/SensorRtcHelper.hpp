#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorRtcHelper.hpp is deprecated. Include RtcDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_RTC
#include "time/SensorRtcHelper.hpp"
#endif

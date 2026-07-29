#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorRTC_POSIX.hpp is deprecated. Include RtcDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_RTC
#include "time/SensorRTC_POSIX.hpp"
#endif

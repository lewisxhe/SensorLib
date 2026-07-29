#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: GaugeBQ27220.hpp is deprecated. Include GaugeDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_GAUGE_BQ27220
#include "gauge/ti/GaugeBQ27220.hpp"
#endif

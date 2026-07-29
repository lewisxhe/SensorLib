#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: TouchDrvInterface.hpp is deprecated. Include TouchDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_TOUCH_COMMON
#include "touch/TouchDrvInterface.hpp"
#endif

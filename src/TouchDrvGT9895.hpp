#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: TouchDrvGT9895.hpp is deprecated. Include TouchDrvGoodix.hpp or TouchDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_TOUCH_GT9895
#include "touch/TouchDrvGT9895.hpp"
#endif

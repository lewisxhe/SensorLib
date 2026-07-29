#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: TouchDrvHI8561.hpp is deprecated. Include TouchDrvJadard.hpp or TouchDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_TOUCH_HI8561
#include "touch/TouchDrvHI8561.hpp"
#endif

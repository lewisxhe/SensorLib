#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: TouchDrvGT911.hpp is deprecated. Include TouchDrvGoodix.hpp or TouchDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_TOUCH_GT911
#include "touch/TouchDrvGT911.hpp"
#endif

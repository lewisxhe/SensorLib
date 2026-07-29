#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: TouchDrvCHSC5816.hpp is deprecated. Include TouchDrvChipshine.hpp or TouchDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_TOUCH_CHSC5816
#include "touch/TouchDrvCHSC5816.hpp"
#endif

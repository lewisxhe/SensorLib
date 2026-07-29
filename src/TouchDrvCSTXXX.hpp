#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: TouchDrvCSTXXX.hpp is deprecated. Include TouchDrvCST.hpp or TouchDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_TOUCH_CSTXXX
#include "touch/TouchDrvCSTXXX.hpp"
#endif

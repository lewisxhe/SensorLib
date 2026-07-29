#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: TouchDrvFT6X36.hpp is deprecated. Include TouchDrvFocalTech.hpp or TouchDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_TOUCH_FT6X36
#include "touch/TouchDrvFT6X36.hpp"
#endif

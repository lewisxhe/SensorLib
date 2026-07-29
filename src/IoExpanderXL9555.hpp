#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: IoExpanderXL9555.hpp is deprecated. Include IoExpanderDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_IO_EXPANDER_XL9555
#include "expander/IoExpanderXL9555.hpp"
#endif

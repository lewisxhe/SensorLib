#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: GaugeAXP2602.hpp is deprecated. Include GaugeDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_GAUGE_AXP2602
#include "gauge/xpowers/GaugeAXP2602.hpp"
#endif

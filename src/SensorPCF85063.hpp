#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: SensorPCF85063.hpp is deprecated. Include RtcDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_PCF85063
#include "time/pcf85063/SensorPCF85063.hpp"
#endif

#pragma once

#include "SensorBuildOpt.h"

#pragma message("WARNING: ExtensionIOXL9555.hpp is deprecated. Include IoExpanderDrv.hpp instead.")

#if !SENSORLIB_EXCLUDE_IO_EXPANDER_XL9555
#include "expander/IoExpanderXL9555.hpp"

using ExtensionIOXL9555 = IoExpanderXL9555;
#endif

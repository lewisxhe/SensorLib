#pragma once

#warning "CRITICAL: File 'SensorDRV2605.hpp' has been renamed to 'HapticDrivers.hpp'."

/*
* Action Required:
* 1. Search your codebase for: #include "SensorDRV2605.hpp" replace #include "HapticDrivers.hpp"
* 2. Replace all instances with: SensorDRV2605 with HapticDriver_DRV2605"
 */
#include "HapticDrivers.hpp"

using SensorDRV2605 = HapticDriver_DRV2605;

#pragma once

#pragma message("WARNING: ExtensionIOXL9555.hpp is deprecated. Include IoExpanderDrv.hpp instead for all XL9555.")

/*
* Action Required:
* 1. Search your codebase for: #include "ExtensionIOXL9555.hpp" replace #include "IoExpanderDrv.hpp"
* 2. Replace all instances with: ExtensionIOXL9555 with IoExpanderXL9555"
 */
#include "IoExpanderDrv.hpp"

using ExtensionIOXL9555 = IoExpanderXL9555;

/**
 *
 * @license MIT License
 *
 * Copyright (c) 2026 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      AXP517Core.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#pragma once

#include "../../../platform/comm/I2CDeviceWithHal.hpp"

class AXP517Core : public I2CDeviceWithHal
{
public:
    AXP517Core() = default;

    ~AXP517Core() = default;

    enum class Module : uint8_t {

        // Module enable control0
        BC12,                 ///< REG 0BH bit 4 BC1.2
        TYPEC,                ///< REG 0BH bit 3 Type-C
        GAUGE,                ///< REG 0BH bit 2 Fuel gauge
        // RESERVED,          ///< REG 0BH bit 1 Reserved
        WATCHDOG,             ///< REG 0BH bit 0 Watchdog CLK and REG 19H bit 0 Watchdog Module enable

        // Module enable control1
        GAUGE_LOW_POWER,      ///< REG 19H bit 6 Low power mode
        // RESERVED,          ///< REG 19H bit 5 Reserved
        BOOST,                ///< REG 19H bit 4 Boost converter
        BUCK,                 ///< REG 19H bit 3 Buck converter
        CHGLED,               ///< REG 19H bit 2 Charge LED
        CHARGE,               ///< REG 19H bit 1 Charger

        MPPT,                  ///< REG 2H bit 0 MPPT configure
    };

    /**
     * @brief  Enable or disable a module.
     * @note   The master switch for each functional module (e.g. boost, buck, charger,
     *         LED, BC1.2, TCPC, gauge, watchdog, etc.).
     * @param  module: The module to enable/disable.
     * @param  enable: True to enable, false to disable.
     * @retval True on success, false on failure.
     */
    bool enableModule(Module module, bool enable);

    /**
     * @brief  Check if a module is enabled.
     * @note   This function checks the status of a specific functional module.
     * @param  module: The module to check.
     * @retval True if the module is enabled, false otherwise.
     */
    bool isModuleEnabled(Module module);

private:
    /**
     * @brief  Initialize the I2C device.
     * @param  param: Initialization parameter.
     * @retval Zero is returned on success, negative error code on failure.
     */
    bool initImpl(uint8_t param) override;
};

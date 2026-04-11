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
 * @file      PmicButtonBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 * @brief PMIC Button/Power-On Base Interface
 *
 * This abstract class defines the interface for PMIC button or power-on functionality.
 * Some PMICs integrate power button monitoring with debounce and IRQ capabilities.
 *
 * @section Button Overview
 * PMICs with button support typically provide:
 * - Configurable on-level timing (button press duration to trigger on)
 * - Configurable off-level timing (button hold to trigger off/power down)
 * - Configurable IRQ timing for button events
 *
 */
#pragma once
#include <stdint.h>

class PmicButtonBase
{
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~PmicButtonBase() = default;

    /**
     * @brief Set button on-level timing
     *
     * Configures the minimum duration the button must be pressed (held low/active)
     * to trigger a power-on event.
     *
     * @param ms Duration in milliseconds (chip-specific valid range)
     * @return true on success, false on failure
     *
     * @note Valid range is chip-specific
     */
    virtual bool setOnLevel(uint16_t ms) = 0;

    /**
     * @brief Set button off-level timing
     *
     * Configures the minimum duration the button must be pressed (held low/active)
     * to trigger a power-off event.
     *
     * @param ms Duration in milliseconds (chip-specific valid range)
     * @return true on success, false on failure
     *
     * @note Valid range is chip-specific
     */
    virtual bool setOffLevel(uint16_t ms) = 0;

    /**
     * @brief Set button IRQ level timing
     *
     * Configures the duration for button-related interrupt generation.
     * Some PMICs generate IRQs on button press, release, or both.
     *
     * @param ms Duration in milliseconds (chip-specific valid range)
     * @return true on success, false on failure
     *
     * @note Valid range is chip-specific
     */
    virtual bool setIrqLevel(uint16_t ms) = 0;
};

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
 * @file      PmicLedBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
* @brief PMIC LED capability interface.
 *
 * This interface targets "PMIC-controlled indicator LED" use cases:
 *  - charge/status LED pin (open-drain / push-pull)
 *  - hardware-controlled modes (charging blink)
 *  - optional manual override (on/off/Hi-Z)
 *
 * Notes:
 *  - Many PMICs only support a subset; unsupported operations should return false.
 */
#pragma once

#include <stdint.h>

class PmicLedBase
{
public:
    virtual ~PmicLedBase() = default;

    enum class OutputType : uint8_t {
        OpenDrain = 0,
        PushPull  = 1,
    };

    /**
     * @brief High-level LED operating mode (portable subset).
     */
    enum class Mode : uint8_t {
        /**
         * @brief PMIC controls LED automatically (e.g. charge indicator).
         * The actual behavior depends on the chip.
         */
        AUTO = 0,

        /**
         * @brief Manual control through registers.
         */
        MANUAL = 1,

        /**
         * @brief Breath/blink mode controlled by PMIC/hardware (if supported).
         */
        BREATH = 2,

        /**
         * @brief LED is disabled (if supported).
         */
        DISABLE = 3,

        /**
         * @brief Unknown/implementation-specific mode.
         */
        VENDOR = 255,
    };

    /**
     * @brief Manual LED state (portable subset).
     */
    enum class ManualState : uint8_t {
        HiZ = 0,
        LEVEL_LOW = 1,
        LEVEL_HIGH = 2,
        BLINK_1HZ = 3,
        BLINK_4HZ = 4,
        UNDEFINED = 255,
    };

    /**
     * @brief  Set LED output type.
     * @note   This controls the LED driver configuration (open-drain vs push-pull).
     * @param  type: Desired output type.
     * @retval True on success, false on failure.
     */
    virtual bool setOutputType(OutputType type) = 0;

    /**
     * @brief  Set LED operating mode.
     * @note   This controls the LED behavior (e.g. auto/manual).
     * @param  mode: Desired operating mode.
     * @retval True on success, false on failure.
     */
    virtual bool setMode(Mode mode) = 0;

    /**
     * @brief  Get LED operating mode.
     * @note   This retrieves the current LED behavior (e.g. auto/manual).
     * @retval Current operating mode.
     */
    virtual Mode getMode() = 0;

    /**
     * @brief  Set manual LED state (only meaningful when Mode::Manual is active).
     * @note   This controls the LED behavior (e.g. on/off/Hi-Z).
     * @param  state: Desired manual state.
     * @retval True on success, false on failure.
     */
    virtual bool setManualState(ManualState state) = 0;

    /**
     * @brief  Get manual LED state (only meaningful when Mode::Manual is active).
     * @note   This retrieves the current LED behavior (e.g. on/off/Hi-Z).
     * @retval Current manual state.
     */
    virtual ManualState getManualState() = 0;
};

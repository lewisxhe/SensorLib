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
 * @file      HapticBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-10
 * 
 */
#pragma once

#include "HapticDefs.hpp"

/**
 * @brief Abstract base for haptic drivers (actuators like DRV2605/AW862xx/...).
 *
 * Design goals:
 * - Provide a common minimal API used by applications
 * - Allow feature probing by returning false for unsupported operations
 * - Keep "effect" concept generic (ROM waveform / bank / RAM program)
 */
class HapticBase
{
public:
    virtual ~HapticBase() = default;

    /**
     * @brief Set the actuator type.
     *
     * @param type The actuator type to set.
     * @return true if successful, false if unsupported.
     */
    virtual bool setActuatorType(HapticActuatorType type) = 0;

    /**
     * @brief Get the current actuator type.
     *
     * @return The current actuator type.
     */
    virtual HapticActuatorType getActuatorType() const = 0;

    /**
     * @brief Set the operating mode.
     *
     * @param mode The mode to set.
     * @return true if successful, false if unsupported.
     */
    virtual bool setMode(HapticMode mode) = 0;

    /**
     * @brief Get the current operating mode.
     *
     * @return The current operating mode.
     */
    virtual HapticMode getMode() const = 0;

    /**
     * @brief Play a haptic effect.
     *
     * @param effect The effect to play.
     * @return true if successful, false if unsupported.
     */
    virtual bool playEffect(HapticEffectId effect) = 0;

    /**
     * @brief Start haptic playback.
     *
     * @return true if successful, false if unsupported.
     */
    virtual bool run() = 0;

    /**
     * @brief Stop haptic playback.
     *
     * @return true if successful, false if unsupported.
     */
    virtual bool stop() = 0;
    
    /**
     * @brief Set the real-time value.
     * @note  RTP amplitude; return false if unsupported
     * @param value The value to set.
     * @return true if successful, false if unsupported.
     */
    virtual bool setRealtimeValue(uint8_t value) = 0;
};

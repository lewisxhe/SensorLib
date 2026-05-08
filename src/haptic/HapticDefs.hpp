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
 * @file      HapticDefs.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-10
 *
 */
#pragma once

#include <stdint.h>

/**
 * @brief Actuator type.
 */
enum class HapticActuatorType : uint8_t {
    ERM = 0,  ///< Eccentric Rotating Mass motor
    LRA = 1,  ///< Linear Resonant Actuator
};

/**
 * @brief Generic haptic operating mode.
 */
enum class HapticMode : uint8_t {
    INTERNAL_TRIGGER,     ///< Play by selecting waveforms/effects and calling run()
    EXT_TRIGGER_EDGE,     ///< External trigger on edge (pin interrupt)
    EXT_TRIGGER_LEVEL,   ///< External trigger on level (pin state)
    PWM_ANALOG,          ///< PWM or analog input drive
    AUDIO_TO_VIBE,       ///< Audio input to vibration
    REAL_TIME_PLAYBACK,  ///< RTP mode (direct drive value)
    DIAGNOSTICS,         ///< Diagnostics mode
    AUTO_CALIBRATE,      ///< Auto-calibration mode
    STANDBY,             ///< Standby mode
};

/**
 * @brief Haptic effect ID type.
 * For chips like DRV2605, this maps to ROM waveform index.
 * For AW862xx, this maps to RAM waveform slot.
 */
using HapticEffectId = uint16_t;

/**
 * @brief Vibration playback status.
 */
enum class HapticStatus : uint8_t {
    IDLE        = 0,  ///< Not playing
    PLAYING     = 1,  ///< Currently playing
    PAUSED      = 2, ///< Playback paused
    CALIBRATING = 3, ///< Calibrating
    ERROR       = 4, ///< Error state
    STANDBY     = 5, ///< Standby mode
};

/**
 * @brief Device capability flags.
 *
 * Use with getCapabilities() to query what features
 * a specific haptic driver supports.
 */
struct HapticCapabilities {
    uint8_t hasF0Calibration    : 1;  ///< Supports F0 (resonant frequency) calibration
    uint8_t hasVbatCompensation : 1;  ///< Supports automatic voltage compensation
    uint8_t hasRTP              : 1;  ///< Supports Real-Time Playback mode
    uint8_t hasSequence         : 1;  ///< Supports waveform sequencing
    uint8_t hasAutoCalibration  : 1;  ///< Supports auto-calibration
    uint8_t hasBreakEffect      : 1;  ///< Supports brake/break effects
    uint8_t hasOverdrive        : 1;  ///< Supports overdrive effects
    uint8_t hasContinuousMode   : 1;  ///< Supports continuous vibration mode

    uint8_t maxEffectCount;      ///< Maximum number of effect IDs supported
    uint8_t maxSequenceLength;  ///< Maximum waveform sequence length
    uint16_t maxDurationMs;     ///< Maximum vibration duration in ms (0 = unlimited)

    constexpr HapticCapabilities()
        : hasF0Calibration(0), hasVbatCompensation(0), hasRTP(0)
        , hasSequence(0), hasAutoCalibration(0), hasBreakEffect(0)
        , hasOverdrive(0), hasContinuousMode(0)
        , maxEffectCount(0), maxSequenceLength(0), maxDurationMs(0) {}
};

/**
 * @brief Preset haptic effect types for common use cases.
 *
 * These provide a unified way to play common effects across
 * different haptic driver chips.
 */
enum class HapticPreset : uint8_t {
    CLICK,           ///< Short click/tick
    DOUBLE_CLICK,    ///< Double click
    TRIPLE_CLICK,    ///< Triple click
    LONG_VIBRATION,  ///< Continuous vibration
    BUZZ,            ///< Medium buzz
    STRONG_BUZZ,     ///< Strong buzz
    TAP,             ///< Light tap
    HEAVY_TAP,       ///< Heavy tap
    PULSE,           ///< Pulsed vibration
    RAMP_UP,         ///< Gradually increasing intensity
    RAMP_DOWN,       ///< Gradually decreasing intensity
    WAVE,            ///< Wave-like pattern
    RAINBOW,         ///< Multi-frequency effect
};


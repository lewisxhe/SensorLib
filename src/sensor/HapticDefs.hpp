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
    ERM = 0,
    LRA = 1,
};

/**
 * @brief Generic haptic operating mode.
 *
 * Not all chips support all modes; implementations should return false if unsupported.
 */
enum class HapticMode : uint8_t {
    INTERNAL_TRIGGER,     ///< play by selecting waveforms/effects and calling run()
    EXT_TRIGGER_EDGE,   ///< external trigger on edge (e.g. pin interrupt)
    EXT_TRIGGER_LEVEL,  ///< external trigger on level (e.g. pin state)
    PWM_ANALOG,         ///< PWM or analog input drive (e.g. for direct control or audio-to-vibe)
    AUDIO_TO_VIBE,        ///< audio input to vibration
    REAL_TIME_PLAYBACK,   ///< RTP mode (direct drive value / amplitude)
    DIAGNOSTICS,          ///< diagnostics mode
    AUTO_CALIBRATE,       ///< auto-calibration mode

    MODE_INTTRIG __attribute__((deprecated("Use INTERNAL_TRIGGER instead."))) = INTERNAL_TRIGGER ,   ///< alias for DRV2605 compatibility
    MODE_EXTTRIGEDGE __attribute__((deprecated("Use EXT_TRIGGER_EDGE instead."))) = EXT_TRIGGER_EDGE, ///< alias for DRV2605 compatibility
    MODE_EXTTRIGLVL __attribute__((deprecated("Use EXT_TRIGGER_LEVEL instead."))) = EXT_TRIGGER_LEVEL,     ///< alias for DRV2605 compatibility
    MODE_PWMANALOG __attribute__((deprecated("Use PWM_ANALOG instead."))) = PWM_ANALOG, ///< alias for DRV2605 compatibility
    MODE_AUDIOVIBE __attribute__((deprecated("Use AUDIO_TO_VIBE instead."))) = AUDIO_TO_VIBE,     ///< alias for DRV2605 compatibility
    MODE_REALTIME __attribute__((deprecated("Use REAL_TIME_PLAYBACK instead."))) = REAL_TIME_PLAYBACK,      ///< alias for DRV2605 compatibility
    MODE_DIAGNOS __attribute__((deprecated("Use DIAGNOSTICS instead."))) = DIAGNOSTICS,          ///< alias for DRV2605 compatibility
    MODE_AUTOCAL __attribute__((deprecated("Use AUTO_CALIBRATE instead."))) = AUTO_CALIBRATE,       ///< alias for DRV2605 compatibility

};

/**
 * @brief Generic haptic effect ID.
 *
 * For chips like DRV2605, effect usually maps to ROM waveform index.
 * For other chips, it could map to a built-in effect bank or RAM program.
 */
using HapticEffectId = uint16_t;

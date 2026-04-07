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
 * @brief Abstract base class for haptic drivers.
 *
 * This class provides a unified interface for different haptic motor drivers
 * (e.g., DRV2605, AW86224, AW86225). It allows users to write platform-independent
 * haptic control code while each driver handles chip-specific details.
 *
 * @section usage Usage Example
 * @code
 * // Works with any HapticBase-compatible driver
 * HapticBase* haptic;
 *
 * // Query capabilities
 * auto cap = haptic->getCapabilities();
 * if (cap.hasContinuousMode) {
 *     haptic->continuousVibration(1000);  // 1 second
 * }
 * @endcode
 *
 * @section design Design Principles
 * - **Feature probing**: Return false or default values for unsupported operations
 * - **Unified API**: Same method names across different chips
 * - **Optional features**: Subclasses can override default implementations
 */
class HapticBase
{
public:
    virtual ~HapticBase() = default;

    // ==================== Lifecycle ====================

    /**
     * @brief Deinitialize the haptic driver and release resources.
     */
    virtual void end() {}

    /**
     * @brief Check if the driver is initialized and ready.
     * @return true if ready, false otherwise.
     */
    virtual bool isReady() const = 0;

    // ==================== Playback Control ====================

    /**
     * @brief Start haptic playback.
     * @return true if successful, false otherwise.
     */
    virtual bool run() = 0;

    /**
     * @brief Stop haptic playback immediately.
     * @return true if successful, false otherwise.
     */
    virtual bool stop() = 0;

    /**
     * @brief Check if haptic is currently playing.
     * @return true if playing, false otherwise.
     */
    virtual bool isPlaying() const = 0;

    /**
     * @brief Get current playback status.
     * @return Current HapticStatus.
     */
    virtual HapticStatus getStatus() const = 0;

    /**
     * @brief Emergency stop - stops all vibration immediately.
     *
     * Default implementation calls stop(). Subclasses can override
     * for chip-specific emergency handling.
     */
    virtual void emergencyStop()
    {
        stop();
    }

    // ==================== Gain Control ====================

    /**
     * @brief Set the gain/amplitude (0x00 to 0xFF).
     * @param gain Gain value.
     * @return true if successful, false otherwise.
     */
    virtual bool setGain(uint8_t gain) = 0;

    /**
     * @brief Get the current gain value.
     * @return Current gain (0x00 to 0xFF), or 0 if unsupported.
     */
    virtual uint8_t getGain() const = 0;

    // ==================== Effect Playback ====================

    /**
     * @brief Play a haptic effect.
     *
     * This is a blocking call - it returns only after the effect completes.
     * For chips that don't support blocking, use playEffectAsync() instead.
     *
     * @param effect Effect ID to play (chip-specific mapping).
     * @return true if successful, false otherwise.
     */
    virtual bool playEffect(HapticEffectId effect) = 0;

    /**
     * @brief Play a haptic effect (non-blocking).
     *
     * Returns immediately; use isPlaying() or vibrationUpdate() to check status.
     *
     * @param effect Effect ID to play.
     * @return true if started successfully, false otherwise.
     */
    virtual bool playEffectAsync(HapticEffectId effect)
    {
        return playEffect(effect);
    }

    /**
     * @brief Play a continuous vibration for specified duration.
     *
     * Default implementation uses playEffect() in a loop.
     * Subclasses should override for optimized continuous mode.
     *
     * @param duration_ms Duration in milliseconds.
     * @param blocking If true, blocks until vibration completes.
     */
    virtual void continuousVibration(uint32_t duration_ms, bool blocking = true);

    /**
     * @brief Update vibration state (for non-blocking mode).
     *
     * Call this periodically in your main loop when using non-blocking
     * vibration functions. Default implementation does nothing.
     */
    virtual void vibrationUpdate() {}

    // ==================== Waveform Sequence ====================

    /**
     * @brief Set a waveform in the sequence.
     * @param slot Sequence slot index (0 to maxSequenceLength-1).
     * @param effect Effect ID to play at this slot.
     * @return true if successful, false otherwise.
     */
    virtual bool setSequence(uint8_t slot, HapticEffectId effect)
    {
        (void)slot;
        (void)effect;
        return false;
    }

    /**
     * @brief Clear the waveform sequence.
     * @return true if successful, false otherwise.
     */
    virtual bool clearSequence()
    {
        return false;
    }

    /**
     * @brief Play the waveform sequence.
     * @return true if successful, false otherwise.
     */
    virtual bool playSequence()
    {
        return false;
    }

    // ==================== Device Information ====================

    /**
     * @brief Get the chip name.
     * @return Null-terminated string with chip name.
     */
    virtual const char *getChipName() const = 0;

    /**
     * @brief Get the chip ID.
     * @return Chip identifier value.
     */
    virtual uint8_t getChipID() const = 0;

    /**
     * @brief Get the actuator type.
     * @return Current actuator type (ERM or LRA).
     */
    virtual HapticActuatorType getActuatorType() const = 0;

    /**
     * @brief Set the actuator type.
     * @param type Actuator type to set (ERM or LRA).
     * @return true if successful, false otherwise.
     */
    virtual bool setActuatorType(HapticActuatorType type) = 0;

    /**
     * @brief Get device capabilities.
     * @return HapticCapabilities struct describing supported features.
     */
    virtual HapticCapabilities getCapabilities() const = 0;

    // ==================== Calibration ====================

    /**
     * @brief Perform auto-calibration.
     * @return true if successful, false otherwise.
     */
    virtual bool calibrate()
    {
        return false;
    }

    /**
     * @brief Check if calibration is needed.
     * @return true if calibration is required, false otherwise.
     */
    virtual bool needsCalibration() const
    {
        return false;
    }

    /**
     * @brief Get the measured resonant frequency (F0).
     * @return F0 in Hz, or 0 if not available.
     */
    virtual uint32_t getF0() const
    {
        return 0;
    }

    /**
     * @brief Get the battery voltage.
     * @return Voltage in mV, or 0 if unsupported.
     */
    virtual uint32_t getVbat()
    {
        return 0;
    }

    // ==================== Mode Control ====================

    /**
     * @brief Set the operating mode.
     * @param mode Mode to set.
     * @return true if successful, false otherwise.
     */
    virtual bool setMode(HapticMode mode)
    {
        (void)mode;
        return false;
    }

    /**
     * @brief Get the current operating mode.
     * @return Current HapticMode.
     */
    virtual HapticMode getMode() const
    {
        return HapticMode::INTERNAL_TRIGGER;
    }

    // ==================== RTP (Real-Time Playback) ====================

    /**
     * @brief Set RTP (real-time playback) value.
     *
     * Only meaningful when in REAL_TIME_PLAYBACK mode.
     *
     * @param value RTP drive value (0x00 to 0xFF).
     * @return true if successful, false otherwise.
     */
    virtual bool setRealtimeValue(uint8_t value)
    {
        (void)value;
        return false;
    }

protected:
    /**
     * @brief Constructor for derived classes.
     * @param name Chip name string for getChipName().
     */
    explicit HapticBase(const char* name) : _chipName(name), _isReady(false) {}

    /**
     * @brief Set ready state.
     * @param ready Ready state to set.
     */
    void setReady(bool ready)
    {
        _isReady = ready;
    }

    const char *_chipName;
    bool _isReady;
};


inline void HapticBase::continuousVibration(uint32_t duration_ms, bool blocking)
{
    (void)duration_ms;
    (void)blocking;
}

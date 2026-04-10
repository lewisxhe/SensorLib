/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      HapticDriver_DRV2605.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-04-03
 */
#pragma once

#include "platform/comm/I2CDeviceNoHal.hpp"
#include "HapticBase.hpp"

/**
 * @brief Default I2C slave address for DRV2605/DRV2604 haptic driver.
 */
static constexpr uint8_t DRV2605_SLAVE_ADDRESS = 0x5A;

/**
 * @brief Driver for the TI DRV2605/DRV2604 haptic motor controller.
 *
 * This driver provides a complete interface for controlling the DRV2605/DRV2604
 * haptic motor driver IC via I2C. It inherits from HapticBase to provide a
 * unified interface across different haptic drivers.
 *
 * @par Supported Features:
 * - ROM waveform library playback (117 built-in effects)
 * - Waveform sequencing (8 slots)
 * - Real-Time Playback (RTP) mode for direct amplitude control
 * - Auto-calibration for ERM and LRA actuators
 * - Multiple operating modes (internal trigger, external trigger, audio-to-vibe, etc.)
 * - ERM and LRA actuator support
 *
 * @par Unsupported Features:
 * - Runtime gain adjustment (ROM effect intensity is pre-defined)
 * - Continuous vibration mode (use RTP mode instead)
 * - F0 calibration (use autoCal() instead)
 * - VBAT compensation
 *
 * @par ROM Library Reference:
 * - Library 1: Strong Click (default)
 * - Library 2: Sharp Click
 * - Library 3: Soft Click
 * - Library 4: Medium Click
 * - Library 5: Soft Bump
 * - Library 6: Double Click
 * - Library 7: Triple Click
 * - Library 8: Soft Fuzz
 * - Library 9: Buzz
 * - Library 10: Butterly
 * - Library 11: Crash
 * - Library 12: Tick
 * - Library 13: Pop
 * - Library 14: Generic Click
 * - Library 15: Generic Snap
 * - Library 16: Generic Roll
 * - Library 17: Generic Pitch
 *
 * @note LRA Library (Library 6) is specifically designed for Linear Resonant Actuators.
 *       ERM libraries (1-5) are for Eccentric Rotating Mass motors.
 *
 * @par Usage Example:
 * @code
 * #include <Wire.h>
 * #include "HapticDriver_DRV2605.hpp"
 *
 * HapticDriver_DRV2605 haptic;
 *
 * void setup() {
 *     Serial.begin(115200);
 *     Wire.begin();
 *
 *     if (!haptic.begin(Wire, DRV2605_SLAVE_ADDRESS)) {
 *         Serial.println("Init failed!");
 *         while (1);
 *     }
 *
 *     // Select LRA library for linear resonant actuator
 *     haptic.selectLibrary(6);
 *
 *     // Play a built-in effect
 *     haptic.playEffect(1);  // Strong click
 *
 *     // Or use RTP for direct control
 *     haptic.setMode(HapticMode::REAL_TIME_PLAYBACK);
 *     haptic.setRealtimeValue(0x80);
 * }
 * @endcode
 *
 * @warning You must call begin() successfully before using any other API.
 */
class HapticDriver_DRV2605 : public HapticBase, public I2CDeviceNoHal
{
public:
    /**
     * @brief Construct an uninitialized driver instance.
     */
    HapticDriver_DRV2605();

    /**
     * @brief Destructor. Deinitializes communication backend if created.
     */
    ~HapticDriver_DRV2605() override;

    // ==================== HapticBase Interface - Lifecycle ====================

    /**
     * @brief Deinitialize the driver and release resources.
     */
    void end() override;

    /**
     * @brief Check if the driver is initialized and ready.
     * @return true if ready, false otherwise.
     */
    bool isReady() const override;

    /**
     * @brief Get the chip name.
     * @return Always returns "DRV2605".
     */
    const char *getChipName() const override;

    // ==================== HapticBase Interface - Playback Control ====================

    /**
     * @brief Start haptic playback.
     * @return true if successful, false otherwise.
     */
    bool run() override;

    /**
     * @brief Stop haptic playback immediately.
     * @return true if successful, false otherwise.
     */
    bool stop() override;

    /**
     * @brief Check if haptic is currently playing.
     * @return true if playing, false otherwise.
     */
    bool isPlaying() const override;

    /**
     * @brief Get current playback status.
     * @return HapticStatus::PLAYING if playing, HapticStatus::IDLE otherwise.
     */
    HapticStatus getStatus() const override;

    /**
     * @brief Emergency stop - stops all vibration immediately.
     * @note Calls stop() internally.
     */
    void emergencyStop() override;

    // ==================== HapticBase Interface - Gain Control ====================

    /**
     * @brief Set the gain/amplitude.
     *
     * @note NOT SUPPORTED by DRV2605. This chip uses ROM waveforms with
     *       pre-defined intensities. For amplitude control, use RTP mode
     *       (setRealtimeValue) instead.
     *
     * @param gain Ignored.
     * @return Always returns false.
     */
    bool setGain(uint8_t gain) override;

    /**
     * @brief Get the current gain value.
     *
     * @note NOT SUPPORTED by DRV2605. ROM effect intensities are pre-defined.
     *
     * @return Always returns 0.
     */
    uint8_t getGain() const override;

    // ==================== HapticBase Interface - Effect Playback ====================

    /**
     * @brief Play a haptic effect.
     *
     * Plays a ROM waveform effect with blocking (waits for completion).
     *
     * @param effect Effect ID (1-117 based on selected library).
     * @return true if started successfully, false otherwise.
     */
    bool playEffect(HapticEffectId effect) override;

    /**
     * @brief Play a haptic effect (non-blocking).
     *
     * @note For DRV2605, this behaves the same as playEffect() since
     *       the chip handles playback internally.
     *
     * @param effect Effect ID to play.
     * @return true if started successfully, false otherwise.
     */
    bool playEffectAsync(HapticEffectId effect) override;

    /**
     * @brief Play a continuous vibration for specified duration.
     *
     * @note NOT SUPPORTED by DRV2605. This chip plays pre-defined ROM
     *       effects and cannot produce true continuous vibration.
     *       For amplitude control, use RTP mode instead.
     *
     * @param duration_ms Ignored.
     * @param blocking Ignored.
     */
    void continuousVibration(uint32_t duration_ms, bool blocking = true) override;

    /**
     * @brief Update vibration state (for non-blocking mode).
     *
     * @note NOT SUPPORTED. DRV2605 handles playback internally.
     *       This function does nothing.
     */
    void vibrationUpdate() override;

    // ==================== HapticBase Interface - Waveform Sequence ====================

    /**
     * @brief Set a waveform in the sequence.
     *
     * @param slot Sequence slot index (0 to 7).
     * @param effect Effect ID to play at this slot.
     * @return true if successful, false otherwise.
     */
    bool setSequence(uint8_t slot, HapticEffectId effect) override;

    /**
     * @brief Clear the waveform sequence.
     * @return true if successful, false otherwise.
     */
    bool clearSequence() override;

    /**
     * @brief Play the waveform sequence.
     * @return true if successful, false otherwise.
     */
    bool playSequence() override;

    // ==================== HapticBase Interface - Device Information ====================

    /**
     * @brief Get device capabilities.
     * @return HapticCapabilities struct describing supported features.
     *
     * Supported: RTP, Sequence, AutoCalibration, BreakEffect, Overdrive
     * Unsupported: F0Calibration, VbatCompensation, ContinuousMode
     */
    HapticCapabilities getCapabilities() const override;

    /**
     * @brief Get the chip ID.
     * @return Chip identifier value (shifted from STATUS register).
     */
    uint8_t getChipID() const override;

    /**
     * @brief Get the actuator type.
     * @return HapticActuatorType::ERM or HapticActuatorType::LRA.
     */
    HapticActuatorType getActuatorType() const override;

    /**
     * @brief Set the actuator type.
     *
     * Automatically selects the appropriate ROM library:
     * - ERM: Library 1
     * - LRA: Library 6
     *
     * @param type Actuator type to set (ERM or LRA).
     * @return true if successful, false otherwise.
     */
    bool setActuatorType(HapticActuatorType type) override;

    // ==================== HapticBase Interface - Mode Control ====================

    /**
     * @brief Set the operating mode.
     *
     * @param mode Mode to set:
     *            - INTERNAL_TRIGGER: Waveform playback triggered by GO bit
     *            - EXT_TRIGGER_EDGE: External trigger on edge
     *            - EXT_TRIGGER_LEVEL: External trigger on level
     *            - PWM_ANALOG: PWM or analog input drive
     *            - AUDIO_TO_VIBE: Audio input to vibration
     *            - REAL_TIME_PLAYBACK: Direct RTP amplitude control
     *            - DIAGNOSTICS: Diagnostics mode
     *            - AUTO_CALIBRATE: Auto-calibration mode
     *
     * @return true if successful, false otherwise.
     */
    bool setMode(HapticMode mode) override;

    /**
     * @brief Get the current operating mode.
     * @return Current HapticMode.
     */
    HapticMode getMode() const override;

    // ==================== HapticBase Interface - RTP ====================

    /**
     * @brief Set RTP (real-time playback) value.
     *
     * Provides direct amplitude control in RTP mode.
     *
     * @param value RTP drive value (0x00 to 0xFF).
     * @return true if successful, false otherwise.
     */
    bool setRealtimeValue(uint8_t value) override;

    // ==================== HapticBase Interface - Calibration ====================

    /**
     * @brief Perform auto-calibration.
     *
     * Runs the built-in auto-calibration sequence.
     *
     * @return true if successful, false otherwise.
     */
    bool calibrate() override;

    /**
     * @brief Check if calibration is needed.
     * @return Always returns true for DRV2605.
     */
    bool needsCalibration() const override;

    /**
     * @brief Get the measured resonant frequency (F0).
     *
     * @note For LRA mode, reads the resonance period and calculates F0.
     *       For ERM mode, this returns 0.
     *
     * @return F0 in Hz, or 0 if not available.
     */
    uint32_t getF0() const override;

    /**
     * @brief Get the battery voltage.
     * @return Voltage in mV, or 0 if unsupported.
     */
    uint32_t getVbat() override;

    // ==================== Extended API ====================

    /**
     * @brief Set a waveform entry in the playback sequence table.
     *
     * Playback starts at slot 0 and continues sequentially until a value of 0 is
     * encountered, or slot 7 is reached.
     *
     * @param slot Sequence slot index (0..7).
     * @param w    Waveform index in the device ROM library (0 = end).
     */
    void setWaveform(uint8_t slot, uint8_t w);

    /**
     * @brief Select the ROM waveform library.
     *
     * Different libraries contain different sets of built-in effects.
     *
     * @param lib Library selection:
     *            - 0: Empty
     *            - 1..5: ERM libraries (1=Strong Click, 2=Sharp Click, etc.)
     *            - 6: LRA library (for Linear Resonant Actuators)
     */
    void selectLibrary(uint8_t lib);

    /**
     * @brief Check if playback is done.
     * @return true if GO bit is clear (playback stopped).
     */
    bool isDone() const;

    /**
     * @brief Trigger a single playback cycle.
     *
     * Sets the GO bit to start playback of the waveform sequence.
     * Equivalent to run().
     */
    void trigger();

    /**
     * @brief Get the current RTP value.
     * @return Current RTP register value (0x00 to 0xFF).
     */
    uint8_t getRealtimeValue() const;

    /**
     * @brief Set the rated voltage for auto-calibration.
     *
     * This value should match the rated voltage of your actuator.
     *
     * @param value Rated voltage value (register value).
     */
    void setRatedVoltage(uint8_t value);

    /**
     * @brief Set the overdrive clamp voltage for auto-calibration.
     *
     * This value controls the maximum voltage during auto-calibration.
     *
     * @param value Overdrive clamp voltage value.
     */
    void setOverdriveClamp(uint8_t value);

    /**
     * @brief Perform auto-calibration and return compensation result.
     *
     * Runs the full auto-calibration sequence and returns the
     * compensation value stored in the AUTOCALCOMP register.
     *
     * @return Auto-calibration compensation value (0-255), or -1 on failure.
     */
    int autoCal();

    /**
     * @brief Enable/disable automatic braking.
     *
     * When enabled, the chip automatically applies braking
     * after each waveform effect.
     *
     * @param enable true to enable, false to disable.
     */
    void setAutoBrake(bool enable);

    /**
     * @brief Set the number of overdrive cycles.
     *
     * Controls how long the overdrive period lasts at the
     * beginning of each waveform.
     *
     * @param cycles Number of overdrive cycles (0-255).
     */
    void setOverdriveTime(uint8_t cycles);

    /**
     * @brief Set the brake time offset.
     *
     * Controls the braking period after each waveform.
     *
     * @param time Brake time offset value (0-255).
     */
    void setBrakeTime(uint8_t time);

    /**
     * @brief Enable/disable RTP overdrive.
     *
     * When enabled, RTP mode uses overdrive to reach the
     * target amplitude faster.
     *
     * @param enable true to enable, false to disable.
     */
    void setRtpOverdrive(bool enable);

    /**
     * @brief Enable/disable ERM/LRA mode.
     *
     * @deprecated Use setActuatorType() instead.
     *
     * @param erm true for ERM, false for LRA.
     */
    void setERMLRA(bool erm);

    /**
     * @brief Set the control4 register value.
     *
     * @warning Advanced use only. Incorrect values may cause
     *          unpredictable behavior.
     *
     * @param value Control4 register value.
     */
    void setCtrl4(uint8_t value);

    /**
     * @brief Read the LRA resonance period (for LRA calibration).
     *
     * @return LRA resonance period value (bits 6:0), or 0 if unavailable.
     */
    uint8_t getLRAFrequency() const;

private:
    bool initImpl(uint8_t param) override;

    void afterInitSuccess(uint8_t param) override
    {
        setReady(true);
    }
    uint8_t _currentLibrary;
    uint8_t _waveformSeq[8];
    uint8_t _waveformCount;
    uint8_t _gain;
    bool _isERM;
    bool _isStandby;
};
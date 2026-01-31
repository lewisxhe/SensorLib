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
 * @file      SensorBMA423.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-28
 * @note      BMA423 class based on [BMA456 Sensor API](https://github.com/boschsensortec/BMA456_SensorAPI)
 *            Simplification for Arduino
 */
#pragma once

#include "SensorBMA4XX.hpp"
#include "bosch/bma4xx/bma423.h"

/**
 * @brief Utility class for BMA423-specific conversions
 */
class BMA423Utils
{
public:
    /**
    * @brief Converts threshold values ​​in 5.11g format to milligrams (mg)
    * @note 5.11g format: 11-bit signed fixed-point number, 1 sign bit + 10 decimal places
    * Value range: -1g to +1g (excluding +1g), resolution: ~0.5mg
    * Formula: Physical value (g) = Register value / 2048
    */
    static float threshold5_11ToMg(uint16_t threshold_5_11)
    {
        float g_value = static_cast<float>(threshold_5_11) / 2048.0f;
        return g_value * 1000.0f;  // Convert g to mg
    }

    /**
     * @brief Convert threshold from milli-g to 5.11g format
     * @param threshold_mg Threshold in milli-g (0-1000mg)
     * @return uint16_t Threshold in 5.11g format
     */
    static uint16_t mgToThreshold5_11(float threshold_mg)
    {
        if (threshold_mg < 0 || threshold_mg > 1000.0f) {
            log_e("Threshold out of range (0-1000mg): %.1f mg, will be clamped", threshold_mg);
            threshold_mg = (threshold_mg < 0) ? 0 : 1000.0f;
        }
        float g_value = threshold_mg / 1000.0f;  // Convert mg to g
        return static_cast<uint16_t>(g_value * 2048.0f + 0.5f);
    }

    /**
     * @brief Convert duration from 50Hz samples to milliseconds
     * @param samples Duration in 50Hz samples
     * @return float Duration in milliseconds
     */
    static float samples50HzToMs(uint16_t samples)
    {
        return static_cast<float>(samples) * 20.0f;  // 20ms per sample at 50Hz
    }

    /**
     * @brief Convert duration from milliseconds to 50Hz samples
     * @param ms Duration in milliseconds
     * * @return uint16_t Duration in 50Hz samples
     */
    static uint16_t msToSamples50Hz(float ms)
    {
        return static_cast<uint16_t>(ms / 20.0f + 0.5f);
    }

};

class SensorBMA423 : public SensorBMA4XX
{
    struct MotionCfg {
        uint8_t motion_feature_enabled = 0;        // Any-motion and No-motion feature enable flag
        uint16_t any_motion_threshold = 0x00AA;     // 83mg
        uint16_t any_motion_duration = 0x0005;      // 100ms
        uint16_t no_motion_threshold = 0x00AA;      // 83mg
        uint16_t no_motion_duration = 0x0032;       // 1000ms
    };

    /**
     * @brief  Tap detection type
     */
    enum class BMA423_TapType {
        DOUBLE_TAP = 0,
        SINGLE_TAP,
    };

public:

    /**
     * @brief  Select the platform for the sensor
     * @note   This setting affects tilt detection.
     */
    enum class Platform {
        SMARTPHONE,
        WRISTBAND
    };

    /**
       * @brief  Event callbacks for the sensor
       * @note   This structure holds the callback functions for various sensor events.
       *         Each callback can be set individually and will be called when the
       *         corresponding event occurs.
       */
    struct EventCallbacks {
        std::function<void(TapType)> onTap = nullptr;                     ///< Callback for tap detection events
        std::function<void()> onStepDetected = nullptr;                   ///< Callback for step detection events
        std::function<void(uint32_t)> onStepCount = nullptr;              ///< Callback for step count updates
        std::function<void(ActivityType)> onActivity = nullptr;           ///< Callback for activity recognition events
        std::function<void()> onAnyMotion = nullptr;                      ///< Callback for any-motion detection
        std::function<void()> onNoMotion = nullptr;                       ///< Callback for no-motion detection
        std::function<void()> onTiltDetected = nullptr;                   ///< Callback for tilt detection
        std::function<void(AccelerometerData &)> onDataReady = nullptr;   ///< Callback for new accelerometer data
    };

    /**
     * @brief  Motion axes configuration
     * @note   This structure is used to configure which axes are monitored for
     *         motion and no-motion detection. Each bit enables/disables detection
     *         on a specific axis.
     */
    struct MotionAxesConfig {
        uint8_t x_axis : 1;        ///< Enable motion detection on X-axis (1=enabled, 0=disabled)
        uint8_t y_axis : 1;        ///< Enable motion detection on Y-axis (1=enabled, 0=disabled)
        uint8_t z_axis : 1;        ///< Enable motion detection on Z-axis (1=enabled, 0=disabled)

        /**
         * @brief  Constructor for MotionAxesConfig
         * @param  x_axis    Enable motion detection on X-axis (default: 1)
         * @param  y_axis    Enable motion detection on Y-axis (default: 1)
         * @param  z_axis    Enable motion detection on Z-axis (default: 1)
         */
        MotionAxesConfig(uint8_t x = 1, uint8_t y = 1, uint8_t z = 1)
            : x_axis(x), y_axis(y), z_axis(z) {}
    };


    /**
     * @brief  Constructor for the SensorBMA423 class
     * @note   Initializes the sensor with default settings and sets the remap offset
     *         for BMA423 specific features.
     */
    SensorBMA423() : SensorBMA4XX(), _status(0)
    {
        _remap_reg_offset = BMA423_AXES_REMAP_OFFSET;
        _mon_cfg.any_motion_threshold = 0x00AA;
        _mon_cfg.any_motion_duration = 0x0005;
        _mon_cfg.no_motion_threshold = 0x00AA;
        _mon_cfg.no_motion_duration = 0x0032;
        _tapType = BMA423_TapType::DOUBLE_TAP;
    }

    /**
    * @brief  Destructor for the SensorBMA423 class
    * @note   Cleans up the sensor resources (default implementation).
    */
    ~SensorBMA423() = default;

    /**
    * @brief  Set the callback for tap events
    * @note   The callback will be invoked when a tap (single/double/triple) is detected.
    * @param  cb  Callback function that accepts TapType as parameter
    */
    void setOnTapCallback(std::function<void(TapType)> cb)
    {
        callbacks.onTap = std::move(cb);
    }

    /**
       * @brief  Set the callback for step detection events
       * @note   The callback will be invoked when a step is detected (each step).
       */
    void setOnStepDetectedCallback(std::function<void()> cb)
    {
        callbacks.onStepDetected = std::move(cb);
    }

    /**
    * @brief  Set the callback for step count events
    * @note   The callback will be invoked when step count is updated (provides total step count).
    * @param  cb  Callback function that accepts uint32_t step count as parameter
    */
    void setOnStepCountCallback(std::function<void(uint32_t)> cb)
    {
        callbacks.onStepCount = std::move(cb);
    }

    /**
    * @brief  Set the callback for activity events
    * @note   The callback will be invoked when activity type changes (e.g., walking to running).
    * @param  cb  Callback function that accepts ActivityType as parameter
    */
    void setOnActivityCallback(std::function<void(ActivityType)> cb)
    {
        callbacks.onActivity = std::move(cb);
    }

    /**
    * @brief  Set the any-motion callback for motion events
    * @note   The callback will be invoked when motion state changes.
    * @param  cb  Callback function that accepts void
    */
    void setOnAnyMotionCallback(std::function<void(void)> cb)
    {
        callbacks.onAnyMotion = std::move(cb);
    }

    /**
    * @brief  Set the callback for no-motion events
    * @note   The callback will be invoked when no motion is detected.
    * @param  cb  Callback function that accepts void
    */
    void setOnNoMotionCallback(std::function<void(void)> cb)
    {
        callbacks.onNoMotion = std::move(cb);
    }

    /**
    * @brief  Set the callback for data ready events
    * @note   The callback will be invoked when new accelerometer data is available.
    * @param  cb  Callback function that accepts AccelerometerData reference
    */
    void setOnDataReadyCallback(std::function<void(AccelerometerData)> cb)
    {
        callbacks.onDataReady = std::move(cb);
    }

    /**
     * @brief  Set the callback for tilt detection events
     * @note   The callback will be invoked when a tilt is detected.
     * @param  cb  Callback function that accepts void
     */
    void setOnTiltDetectedCallback(std::function<void(void)> cb)
    {
        callbacks.onTiltDetected = std::move(cb);
    }

    /**
     * @brief  Set all callbacks at once
     * @note   This function copies the entire EventCallbacks structure.
     * @param  cbs  Reference to EventCallbacks structure containing all callbacks
     */
    void setCallbacks(const EventCallbacks &cbs)
    {
        callbacks = cbs;
    }

    /**
     * @brief  Get the current callbacks
     * @return Reference to the EventCallbacks structure
     * @note   Use this to modify callbacks directly or check current settings
     */
    EventCallbacks &getCallbacks()
    {
        return callbacks;
    }

    /**
    * @brief  Get the current callbacks (const version)
    * @return Const reference to the EventCallbacks structure
    */
    const EventCallbacks &getCallbacks() const
    {
        return callbacks;
    }

    /**
    * @brief  Enable or disable a specific feature
    * @note   Enables/disables sensor features using bitmask. For step counter,
    *         also enables the step detector if needed.
    * @warning Ensure that only one of any-motion or no-motion is enabled at a time.
    * @param  feature  Bitmask of features to enable/disable (BMA423_*_EN constants)
    * @param  enable   true to enable, false to disable
    * @return true if successful, false on error
    */
    bool enableFeature(uint16_t feature, bool enable)
    {
        const uint16_t motion_mask = BMA423_ANY_MOTION | BMA423_NO_MOTION;

        if ((feature & (BMA423_ANY_MOTION | BMA423_NO_MOTION)) == (BMA423_ANY_MOTION | BMA423_NO_MOTION)) {
            log_e("Both any-motion and no-motion are specified, only any-motion will be enabled.");
            feature &= (~BMA423_NO_MOTION);
        }

        if (enable && (feature & motion_mask)) {
            uint16_t motion_to_enable = feature & motion_mask;
            if (_mon_cfg.motion_feature_enabled != 0 &&
                    _mon_cfg.motion_feature_enabled != motion_to_enable) {
                if (bma423_feature_enable(_mon_cfg.motion_feature_enabled, 0, dev.get()) != 0) {
                    log_e("Failed to disable motion feature 0x%02X",
                          _mon_cfg.motion_feature_enabled);
                    return false;
                }
                log_d("Disabled previous motion feature 0x%02X",
                      _mon_cfg.motion_feature_enabled);
            }
        }

        if (feature & BMA423_STEP_CNTR) {
            if (bma423_step_detector_enable(enable ? BMA4_ENABLE : BMA4_DISABLE, dev.get()) != 0) {
                log_e("Failed to %s step detector", enable ? "enable" : "disable");
                return false;
            }
        }

        if (bma423_feature_enable(feature, enable, dev.get()) != 0) {
            log_e("Failed to %s feature 0x%04X",
                  enable ? "enable" : "disable", feature);
            return false;
        }

        if (feature & motion_mask) {
            if (enable) {
                _mon_cfg.motion_feature_enabled = feature & motion_mask;
            } else {
                if ((feature & motion_mask) == _mon_cfg.motion_feature_enabled) {
                    _mon_cfg.motion_feature_enabled = 0;
                }
            }
        }

        return true;
    }

    /**
    * @brief  Enable data ready interrupt
    * @note   Configures the interrupt for accelerometer data ready events
    * @param  enable           Enable/disable the data ready feature
    * @param  pin_map          Interrupt pin to use (PIN1 or PIN2)
    * @return true if successful, false on error
    */
    bool enableDataReady(bool enable = true, InterruptPinMap pin_map = InterruptPinMap::PIN1)
    {
        return enableInterrupt(pin_map, BMA4_DATA_RDY_INT, enable);
    }

    /**
    * @brief  Enable or disable step counter
    * @note   Step counter counts total steps since last reset. Step detector
    *         must be enabled separately for step-by-step detection.
    * @param  enable        true to enable, false to disable
    * @param  step_counter_wm Watermark level for step counter interrupt (default: 1)
    *         Setting watermark level 1, the output step resolution is 20 steps.
    *         Eg: 1 means, 1 * 20 = 20. Every 20 steps once output triggers
    * @param  reset_counter true to reset counter to zero
    * @return true if successful, false on error
    */
    bool enableStepCounter(bool enable, uint16_t step_counter_wm = 1, bool reset_counter = false)
    {
        if (enable) {
            if (bma423_step_counter_set_watermark(step_counter_wm, dev.get()) != 0) {
                log_e("Failed to set step counter watermark");
                return false;
            }
        }
        if (reset_counter) {
            if (bma423_reset_step_counter(dev.get()) != 0) {
                log_e("Failed to reset step counter");
                return false;
            }
        }
        return enableFeature(BMA423_STEP_CNTR, enable);
    }

    /**
     * @brief  Get the current step counter watermark
     * @note   This function retrieves the current watermark level for the step counter.
     * @param  &step_counter_wm: Reference to the variable to store the watermark level
     * @retval true if successful, false on error
     */
    bool getStepCounterWatermark(uint16_t &step_counter_wm) const
    {
        return bma423_step_counter_get_watermark(&step_counter_wm, dev.get()) == 0;
    }

    /**
    * @brief  Enable or disable step detector
    * @note   Step detector generates interrupt for each step detected.
    *         Use for real-time step detection rather than total count.
    * @param  enable           true to enable, false to disable
    * @param  interrupt_enable Enable/disable interrupt generation
    * @param  pin_map          Interrupt pin to use (PIN1 or PIN2)
    * @return true if successful, false on error
    */
    bool enableStepDetector(bool enable, bool interrupt_enable = false,
                            InterruptPinMap pin_map = InterruptPinMap::PIN1)
    {
        if (!enableInterrupt(pin_map, BMA423_STEP_CNTR_INT, interrupt_enable)) {
            return false;
        }
        return enableFeature(BMA423_STEP_CNTR, enable);
    }

    /**
    * @brief  Enable or disable tilt detection
    * @note   Enables double tap detection simultaneously.
    * @param  enable           true to enable, false to disable
    * @param  interrupt_enable Enable/disable interrupt generation
    * @param  pin_map          Interrupt pin to use (PIN1 or PIN2)
    * @return true if successful, false on error
    */
    bool enableTiltDetector(bool enable, bool interrupt_enable = false,
                            InterruptPinMap pin_map = InterruptPinMap::PIN1)
    {
        if (!enableInterrupt(pin_map, BMA423_TILT_INT, interrupt_enable)) {
            return false;
        }
        return enableFeature(BMA423_TILT, enable);
    }

    /**
     * @brief  Select the platform for the sensor
     * @note   This function configures the sensor for the specified platform.
     * @warranty  This setting affects tilt detection.
     * @param  platform: The platform to select, allowed values are:
     *         - SMARTPHONE
     *         - WRISTBAND
     * @retval None
     */
    bool selectPlatform(Platform platform)
    {
        return bma423_select_platform(static_cast<uint8_t>(platform), dev.get()) == 0;
    }

    /**
    * @brief  Enable or disable tap detection
    * @note   Enables double tap detection simultaneously.
    * @param  enable           true to enable, false to disable
    * @param  interrupt_enable Enable/disable interrupt generation
    * @param  pin_map          Interrupt pin to use (PIN1 or PIN2)
    * @return true if successful, false on error
    */
    bool enableTapDetector(bool enable, bool interrupt_enable = false,
                           InterruptPinMap pin_map = InterruptPinMap::PIN1)
    {
        if (!enableInterrupt(pin_map, BMA423_WAKEUP_INT, interrupt_enable)) {
            return false;
        }
        return enableFeature(BMA423_WAKEUP, enable);
    }

    /**
     * @brief  Select tap detection tap type
     * @note   This function selects the tap detection tap type.
     * @param  tap: The tap type to select, allowed values are:
     *         - DOUBLE_TAP
     *         - SINGLE_TAP
     * @retval true if successful, false on error
     */
    bool selectTapType(TapType tap)
    {
        switch (tap) {
        case TapType::DOUBLE_TAP:
            _tapType = BMA423_TapType::DOUBLE_TAP;
            break;
        case TapType::SINGLE_TAP:
            _tapType = BMA423_TapType::SINGLE_TAP;
            break;
        default:
            break;
        }
        return bma423_tap_selection(static_cast<uint8_t>(_tapType), dev.get()) == 0;
    }

    /**
    * @brief  Enable or disable activity recognition
    * @note   Activity recognition can detect stationary, walking, running, etc.
    * @param  enable           true to enable, false to disable
    * @param  interrupt_enable Enable/disable interrupt generation
    * @param  pin_map          Interrupt pin to use (PIN1 or PIN2)
    * @return true if successful, false on error
    */
    bool enableActivityRecognition(bool enable, bool interrupt_enable = false,
                                   InterruptPinMap pin_map = InterruptPinMap::PIN1)
    {
        if (!enableInterrupt(pin_map, BMA423_ACTIVITY_INT, interrupt_enable)) {
            return false;
        }
        return enableFeature(BMA423_ACTIVITY, enable);
    }

    /**
     * @brief  Enable or disable any-motion detection
     * @note   Configures motion any-motion detection on selected axes
     * @warning Any-Motion and No-Motion is mutually exclusive
     * @param  &cfg             Configuration for which axes to monitor
     * @param  enable           true to enable, false to disable
     * @param  interrupt_enable Enable/disable interrupt generation
     * @param  pin_map          Interrupt pin to use (PIN1 or PIN2)
     * @return true if successful, false on error
     */
    bool enableAnyMotionDetection(const MotionAxesConfig &cfg, bool enable,
                                  bool interrupt_enable = false,
                                  InterruptPinMap pin_map = InterruptPinMap::PIN1)
    {
        uint16_t axis = 0;

        if (cfg.x_axis == 0 && cfg.y_axis == 0 && cfg.z_axis == 0 && enable) {
            log_e("No motion detection axis enabled");
            return false;
        }

        if (cfg.x_axis) axis |= BMA423_X_AXIS_EN;
        if (cfg.y_axis) axis |= BMA423_Y_AXIS_EN;
        if (cfg.z_axis) axis |= BMA423_Z_AXIS_EN;

        if (bma423_anymotion_enable_axis(axis, dev.get()) != 0) {
            log_e("Failed to configure any-motion axes");
            return false;
        }

        if (!enableInterrupt(pin_map, BMA423_ANY_NO_MOTION_INT, interrupt_enable)) {
            return false;
        }

        if (!configMotion(false, _mon_cfg.any_motion_threshold, _mon_cfg.any_motion_duration)) {
            return false;
        }

        return enableFeature(BMA423_ANY_MOTION, enable);
    }

    /**
     * @brief  Enable or disable no-motion detection
     * @note   Configures motion no-motion detection on selected axes
     * @warning Any-Motion and No-Motion is mutually exclusive
     * @param  &cfg             Configuration for which axes to monitor
     * @param  enable           true to enable, false to disable
     * @param  interrupt_enable Enable/disable interrupt generation
     * @param  pin_map          Interrupt pin to use (PIN1 or PIN2)
     * @return true if successful, false on error
     */
    bool enableNoMotionDetection(const MotionAxesConfig &cfg, bool enable,
                                 bool interrupt_enable = false,
                                 InterruptPinMap pin_map = InterruptPinMap::PIN1)
    {
        uint16_t axis = 0;

        if (cfg.x_axis == 0 && cfg.y_axis == 0 && cfg.z_axis == 0 && enable) {
            log_e("No motion detection axis enabled");
            return false;
        }

        if (cfg.x_axis) axis |= BMA423_X_AXIS_EN;
        if (cfg.y_axis) axis |= BMA423_Y_AXIS_EN;
        if (cfg.z_axis) axis |= BMA423_Z_AXIS_EN;

        if (bma423_anymotion_enable_axis(axis, dev.get()) != 0) {
            log_e("Failed to configure no-motion axes");
            return false;
        }

        if (!enableInterrupt(pin_map, BMA423_ANY_NO_MOTION_INT, interrupt_enable)) {
            return false;
        }

        if (!configMotion(true, _mon_cfg.no_motion_threshold, _mon_cfg.no_motion_duration)) {
            return false;
        }

        return enableFeature(BMA423_NO_MOTION, enable);
    }

    /**
     * @brief Configure no-motion detection using raw register values
     *
     * Sets the threshold and duration for no-motion detection.
     * Threshold: 5.11g format (16-bit)
     * Duration: 50Hz samples (20ms per sample)
     *
    * @param threshold   Slope threshold in 5.11g format (0-2047 = 0-~1g)
    *                    Default: 0x00AA = 170 / 2048 ≈ 0.083g = 83mg
    * @param duration    Number of consecutive 50Hz samples (20ms each) required
    *                    Range: 0-8191 (0-163.82 seconds)
    *                    Default: 0x0005 = 5 samples = 100ms
     * @return This method is compatible with other classes and will always return true.
     */
    bool configNoMotion(uint16_t threshold, uint16_t duration)
    {
        _mon_cfg.no_motion_duration = duration;
        _mon_cfg.no_motion_threshold = threshold;
        return true;
    }

    /**
     * @brief Configure any-motion detection using raw register values
     *
     * Sets the threshold and duration for any-motion detection.
     * Threshold: 5.11g format (16-bit)
     * Duration: 50Hz samples (20ms per sample)
     *
    * @param threshold   Slope threshold in 5.11g format (0-2047 = 0-~1g)
    *                    Default: 0x00AA = 170 / 2048 ≈ 0.083g = 83mg
    * @param duration    Number of consecutive 50Hz samples (20ms each) required
    *                    Range: 0-8191 (0-163.82 seconds)
    *                    Default: 0x0005 = 5 samples = 100ms
     * @return This method is compatible with other classes and will always return true.
     */
    bool configAnyMotion(uint16_t threshold, uint16_t duration)
    {
        _mon_cfg.any_motion_duration = duration;
        _mon_cfg.any_motion_threshold = threshold;
        return true;
    }

    /**
     * @brief Configure motion detection parameters using raw register values
     *
     * This function sets the threshold and duration directly using register values.
     * For physical units, use configMotion() instead.
     *
     * @param no_motion   true for no-motion detection, false for any-motion detection
     * @param threshold   Slope threshold in 5.11g format (0-2047 = 0-~1g)
     *                    Default: 0x00AA = 170 / 2048 ≈ 0.083g = 83mg
     * @param duration    Number of consecutive 50Hz samples (20ms each) required
     *                    Range: 0-8191 (0-163.82 seconds)
     *                    Default: 0x0005 = 5 samples = 100ms
     * @return true if successful, false on error
     */
    bool configMotion(bool no_motion, uint16_t threshold, uint16_t duration)
    {
        struct bma423_anymotion_config any_motion;
        any_motion.threshold = threshold;
        any_motion.duration = duration;
        any_motion.nomotion_sel = no_motion ? 1 : 0;

        if (no_motion) {
            _mon_cfg.no_motion_threshold = threshold;
            _mon_cfg.no_motion_duration = duration;
        } else {
            _mon_cfg.any_motion_threshold = threshold;
            _mon_cfg.any_motion_duration = duration;
        }

        log_d("BMA423 motion config (raw): threshold=0x%04X, duration=%u, type: %s",
              threshold, duration,
              no_motion ? "No-motion" : "Any-motion");

        return bma423_set_any_motion_config(&any_motion, dev.get()) == 0;
    }

    /**
     * @brief Get current no-motion/any-motion detection configuration for BMA423
     * @note Returns parameters in user-friendly units
     *
     * @param[out] threshold   Slope threshold in 5.11g format (0-2047 = 0-~1g)
     *                         Default: 0x00AA = 170 / 2048 ≈ 0.083g = 83mg
     * @param[out] duration    Number of consecutive 50Hz samples (20ms each) required
     *                         Range: 0-8191 (0-163.82 seconds)
     *                         Default: 0x0005 = 5 samples = 100ms
     * @param[out] no_motion true is no-motion,false is any-motion
     * @return true if successful, false on error
     */
    bool getMotionConfig(uint16_t &threshold, uint16_t &duration, bool &no_motion)
    {
        struct bma423_anymotion_config any_motion;
        if (bma423_get_any_motion_config(&any_motion, dev.get()) != 0) {
            log_e("Failed to read BMA423 motion configuration");
            return false;
        }
        threshold = any_motion.threshold;
        duration = any_motion.duration;
        no_motion = any_motion.nomotion_sel;
        return true;
    }


    /**
    * @brief  Get the current step count
    * @note   Reads the accumulated step count from the sensor.
    *         Step counter must be enabled first.
    * @return Current step count, or 0 on error
    */
    uint32_t getStepCount()
    {
        uint32_t step_count = 0;
        if (bma423_step_counter_output(&step_count, dev.get()) != 0) {
            log_e("Failed to read step count");
            return 0;
        }
        return step_count;
    }

    /**
     * @brief  Reset the step counter
     * @note   This function resets the internal step counter to zero.
     * @retval true if successful, false on error
     */
    bool resetStepCounter()
    {
        return bma423_reset_step_counter(dev.get()) == 0;
    }

    /**
    * @brief  Get the current activity type
    * @note   Reads the current activity classification from the sensor.
    *         Activity recognition must be enabled first.
    * @return ActivityType enum value, or UNKNOWN on error
    */
    ActivityType getActivityType()
    {
        uint8_t activity_type = 0;
        if (bma423_activity_output(&activity_type, dev.get()) != 0) {
            log_e("Failed to get activity type");
            return ActivityType::UNKNOWN;
        }
        return static_cast<ActivityType>(activity_type);
    }

    /**
     * @brief  Get which motion detection feature is currently enabled
     * @return 0 = none, BMA423_ANY_MOTION = any-motion, BMA423_NO_MOTION = no-motion
     */
    uint8_t getEnabledMotionFeature() const
    {
        return _mon_cfg.motion_feature_enabled;
    }

    /**
     * @brief  Check if any-motion detection is enabled
     */
    bool isAnyMotionEnabled() const
    {
        return _mon_cfg.motion_feature_enabled == BMA423_ANY_MOTION;
    }

    /**
     * @brief  Check if no-motion detection is enabled
     */
    bool isNoMotionEnabled() const
    {
        return _mon_cfg.motion_feature_enabled == BMA423_NO_MOTION;
    }

    /**
     * @brief  Update sensor state and process interrupts
     * @note   This method should be called periodically (e.g., in main loop)
     *         to check for and process sensor interrupts. It reads the interrupt
     *         status register and invokes the appropriate callbacks.
     * @return void
     */
    void update() override
    {
        if (bma4_read_int_status(&_status, dev.get()) != 0) {
            log_e("Failed to read interrupt status");
            return;
        }

        if (_status == 0) {
            return;
        }

        if ((_status & BMA423_WAKEUP_INT) && callbacks.onTap) {
            callbacks.onTap(_tapType == BMA423_TapType::DOUBLE_TAP ? TapType::DOUBLE_TAP : TapType::SINGLE_TAP);
        }

        if ((_status & BMA423_STEP_CNTR_INT) && callbacks.onStepDetected) {
            callbacks.onStepDetected();
            if (callbacks.onStepCount) {
                callbacks.onStepCount(getStepCount());
            }
        }

        if ((_status & BMA423_ACTIVITY_INT) && callbacks.onActivity) {
            callbacks.onActivity(getActivityType());
        }

        if ((_status & BMA423_ANY_NO_MOTION_INT)) {
            if (_mon_cfg.motion_feature_enabled == BMA423_ANY_MOTION && callbacks.onAnyMotion) {
                callbacks.onAnyMotion();
            } else if (_mon_cfg.motion_feature_enabled == BMA423_NO_MOTION && callbacks.onNoMotion) {
                callbacks.onNoMotion();
            }
        }

        if ((_status & BMA4_ACCEL_DATA_RDY_INT) && callbacks.onDataReady) {
            AccelerometerData data;
            if (readData(data)) {
                callbacks.onDataReady(data);
            }
        }

        if ((_status & BMA423_TILT_INT) && callbacks.onTiltDetected) {
            callbacks.onTiltDetected();
        }

        _status = 0;
    }

private:
    /**
     * @brief  Bosch-specific initialization implementation
     * @note   Initializes the BMA423 sensor and loads its configuration file
     * @return true if initialization successful, false otherwise
     */
    bool boschInitImpl() override
    {
        int8_t rslt = bma423_init(dev.get());
        if ( rslt != 0 ) {
            log_e("bma423_init failed with code %d", rslt);
            return false;
        }

        dev->config_size = BMA423_CONFIG_FILE_SIZE;

        rslt = bma423_write_config_file(dev.get());
        if (rslt != 0) {
            log_e("bma423_write_config_file failed with code %d", rslt);
            return false;
        }
        return true;
    }
protected:
    EventCallbacks callbacks;
    uint16_t _status;
    MotionCfg _mon_cfg;
    BMA423_TapType _tapType;
};

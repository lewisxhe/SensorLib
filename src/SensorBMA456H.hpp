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
 * @file      SensorBMA456H.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-28
 * @note      BMA456H class based on [BMA456 Sensor API](https://github.com/boschsensortec/BMA456_SensorAPI)
 *            Simplification for Arduino
 */
#pragma once

#include "SensorBMA4XX.hpp"
#include "bosch/bma4xx/bma456h.h"

class SensorBMA456H : public SensorBMA4XX
{
public:

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
        std::function<void(void)> onAnyMotion = nullptr;                     ///< Callback for any motion detection
        std::function<void(void)> onNoMotion = nullptr;                      ///< Callback for no-motion detection
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
     * @brief  Constructor for the SensorBMA456H class
     * @note   Initializes the sensor with default settings and sets the remap offset
     *         for BMA456H specific features.
     */
    SensorBMA456H() : SensorBMA4XX(), _status(0)
    {
        _remap_reg_offset = BMA456H_AXES_REMAP_OFFSET;
    }

    /**
    * @brief  Destructor for the SensorBMA456H class
    * @note   Cleans up the sensor resources (default implementation).
    */
    ~SensorBMA456H() = default;

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
    * @param  feature  Bitmask of features to enable/disable (BMA456H_*_EN constants)
    * @param  enable   true to enable, false to disable
    * @return true if successful, false on error
    */
    bool enableFeature(uint16_t feature, bool enable)
    {
        if (feature & BMA456H_STEP_COUNTER_EN) {
            if (bma456h_step_detector_enable(BMA4_ENABLE, dev.get()) != 0) {
                log_e("bma456h_step_detector_enable failed");
                return false;
            }
        }
        if (bma456h_feature_enable(feature, enable, dev.get()) != 0) {
            log_e("bma456h_feature_enable failed");
            return false;
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

        if (bma456h_step_counter_set_watermark(step_counter_wm, dev.get()) != 0) {
            log_e("Failed to set step counter watermark");
            return false;
        }

        if (reset_counter) {
            if (bma456h_reset_step_counter(dev.get()) != 0) {
                log_e("Failed to reset step counter");
                return false;
            }
        }
        return enableFeature(BMA456H_STEP_COUNTER_EN, enable);
    }

    /**
     * @brief  Get the current step counter watermark
     * @note   This function retrieves the current watermark level for the step counter.
     * @param  &step_counter_wm: Reference to the variable to store the watermark level
     * @retval true if successful, false on error
     */
    bool getStepCounterWatermark(uint16_t &step_counter_wm) const
    {
        return bma456h_step_counter_get_watermark(&step_counter_wm, dev.get()) == 0;
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
        if (!enableInterrupt(pin_map, BMA456H_STEP_CNTR_INT, interrupt_enable)) {
            return false;
        }
        return enableFeature(BMA456H_STEP_DETECTOR_EN, enable);
    }

    /**
    * @brief  Enable or disable tap detection
    * @note   Enables single, double, and triple tap detection simultaneously.
    *         Use getTapType() to distinguish between tap types.
    * @param  enable           true to enable, false to disable
    * @param  interrupt_enable Enable/disable interrupt generation
    * @param  pin_map          Interrupt pin to use (PIN1 or PIN2)
    * @return true if successful, false on error
    */
    bool enableTapDetector(bool enable, bool interrupt_enable = false,
                           InterruptPinMap pin_map = InterruptPinMap::PIN1)
    {
        if (!enableInterrupt(pin_map, BMA456H_TAP_OUT_INT, interrupt_enable)) {
            return false;
        }
        return enableFeature(BMA456H_SINGLE_TAP_EN | BMA456H_DOUBLE_TAP_EN | BMA456H_TRIPLE_TAP_EN, enable);
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
        if (!enableInterrupt(pin_map, BMA456H_ACTIVITY_INT, interrupt_enable)) {
            return false;
        }
        return enableFeature(BMA456H_STEP_ACTIVITY_EN, enable);
    }

    /**
     * @brief  Enable or disable any-motion detection
     * @note   Configures motion any-motion detection on selected axes
     * @param  &cfg             Configuration for which axes to monitor
     * @param  enable           true to enable, false to disable
     * @param  interrupt_enable Enable/disable interrupt generation
     * @param  pin_map          Interrupt pin to use (PIN1 or PIN2)
     * @return true if successful, false on error
     */
    bool enableAnyMotionDetection(MotionAxesConfig &cfg, bool enable,
                                  bool interrupt_enable = false,
                                  InterruptPinMap pin_map = InterruptPinMap::PIN1)
    {
        uint16_t interrupt_sources = 0;
        uint16_t feature = 0;
        if (cfg.x_axis) feature |= BMA456H_ANY_MOTION_X_AXIS_EN;
        if (cfg.y_axis) feature |= BMA456H_ANY_MOTION_Y_AXIS_EN;
        if (cfg.z_axis) feature |= BMA456H_ANY_MOTION_Z_AXIS_EN;

        if (cfg.x_axis || cfg.y_axis || cfg.z_axis) {
            interrupt_sources |= BMA456H_ANY_MOT_INT;
        }

        if (!enableInterrupt(pin_map, BMA456H_ANY_MOT_INT, interrupt_enable)) {
            return false;
        }
        return enableFeature(feature, enable);
    }


    /**
     * @brief  Enable or disable no-motion detection
     * @note   Configures motion no-motion detection on selected axes
     * @param  &cfg             Configuration for which axes to monitor
     * @param  enable           true to enable, false to disable
     * @param  interrupt_enable Enable/disable interrupt generation
     * @param  pin_map          Interrupt pin to use (PIN1 or PIN2)
     * @return true if successful, false on error
     */
    bool enableNoMotionDetection(MotionAxesConfig &cfg, bool enable,
                                 bool interrupt_enable = false,
                                 InterruptPinMap pin_map = InterruptPinMap::PIN1)
    {
        uint16_t feature = 0;
        if (cfg.x_axis) feature |= BMA456H_NO_MOTION_X_AXIS_EN;
        if (cfg.y_axis) feature |= BMA456H_NO_MOTION_Y_AXIS_EN;
        if (cfg.z_axis) feature |= BMA456H_NO_MOTION_Z_AXIS_EN;

        if (!enableInterrupt(pin_map, BMA456H_NO_MOT_INT, interrupt_enable)) {
            return false;
        }
        return enableFeature(feature, enable);
    }

    /**
    * @brief  Configure no-motion detection parameters with detailed unit conversion
    * @note   This function sets both the threshold (sensitivity) and duration (time)
    *         for no-motion detection.
    *
    *         = Threshold Units =
    *         The threshold is specified in LSB (Least Significant Bits), where:
    *         1 LSB = 0.48 mG = 0.48 × 0.001 g = 0.00048 g ≈ 0.0047 m/s²
    *
    *         To convert from milli-g to LSB:
    *         LSB = mg_value / 0.48
    *
    *         Example conversions:
    *         - 10 LSB = 10 × 0.48 mG = 4.8 mG = 0.0048 g
    *         - 100 mG = 100 / 0.48 ≈ 208 LSB
    *
    *         = Duration Units =
    *         The duration is specified in LSB, where at 50Hz ODR:
    *         1 LSB = 20 milliseconds
    *
    *         Duration in seconds = duration_LSB × 0.020
    *
    *         Example conversions:
    *         - 4 LSB = 4 × 20ms = 80ms = 0.08s
    *         - 50ms = 50 / 20 = 2.5 LSB (round to nearest integer)
    *
    *         @warning The duration unit (20ms/LSB) is only accurate at 50Hz ODR.
    *                  For other ODR settings, the time per sample changes.
    *                  Time per sample = 1 / ODR
    *                  Example at 100Hz ODR: 1 sample = 10ms, so duration = LSB × 10ms
    *
    * @param  threshold  Motion detection threshold in LSB units (1 LSB = 0.48 mG)
    *                    Recommended range: 10-200 (4.8-96 mG)
    *                    Higher values = less sensitive, lower values = more sensitive
    *
    * @param  duration   Detection duration in LSB units (1 LSB = 20ms at 50Hz ODR)
    *                    Recommended range: 1-20 (20ms to 400ms)
    *                    Longer duration = more stable detection, less false positives
    *
    * @return true if configuration successful, false on error
    */
    bool configNoMotion(uint16_t threshold, uint16_t duration)
    {
        return configMotion(false, threshold, duration);
    }

    /**
    * @brief  Configure any-motion detection parameters with detailed unit conversion
    * @note   This function sets both the threshold (sensitivity) and duration (time)
    *         for any-motion detection.
    *
    *         = Threshold Units =
    *         The threshold is specified in LSB (Least Significant Bits), where:
    *         1 LSB = 0.48 mG = 0.48 × 0.001 g = 0.00048 g ≈ 0.0047 m/s²
    *
    *         To convert from milli-g to LSB:
    *         LSB = mg_value / 0.48
    *
    *         Example conversions:
    *         - 10 LSB = 10 × 0.48 mG = 4.8 mG = 0.0048 g
    *         - 100 mG = 100 / 0.48 ≈ 208 LSB
    *
    *         = Duration Units =
    *         The duration is specified in LSB, where at 50Hz ODR:
    *         1 LSB = 20 milliseconds
    *
    *         Duration in seconds = duration_LSB × 0.020
    *
    *         Example conversions:
    *         - 4 LSB = 4 × 20ms = 80ms = 0.08s
    *         - 50ms = 50 / 20 = 2.5 LSB (round to nearest integer)
    *
    *         @warning The duration unit (20ms/LSB) is only accurate at 50Hz ODR.
    *                  For other ODR settings, the time per sample changes.
    *                  Time per sample = 1 / ODR
    *                  Example at 100Hz ODR: 1 sample = 10ms, so duration = LSB × 10ms
    *
    * @param  threshold  Motion detection threshold in LSB units (1 LSB = 0.48 mG)
    *                    Recommended range: 10-200 (4.8-96 mG)
    *                    Higher values = less sensitive, lower values = more sensitive
    *
    * @param  duration   Detection duration in LSB units (1 LSB = 20ms at 50Hz ODR)
    *                    Recommended range: 1-20 (20ms to 400ms)
    *                    Longer duration = more stable detection, less false positives
    *
    * @return true if configuration successful, false on error
    */
    bool configAnyMotion(uint16_t threshold, uint16_t duration)
    {
        return configMotion(true, threshold, duration);
    }

    /**
     * @brief  Configure any-motion or no-motion detection parameters with detailed unit conversion
     * @note   This function sets both the threshold (sensitivity) and duration (time)
     *         for any-motion/no-motion detection.
     *
     *         = Threshold Units =
     *         The threshold is specified in LSB (Least Significant Bits), where:
     *         1 LSB = 0.48 mG = 0.48 × 0.001 g = 0.00048 g ≈ 0.0047 m/s²
     *
     *         To convert from milli-g to LSB:
     *         LSB = mg_value / 0.48
     *
     *         Example conversions:
     *         - 10 LSB = 10 × 0.48 mG = 4.8 mG = 0.0048 g
     *         - 100 mG = 100 / 0.48 ≈ 208 LSB
     *
     *         = Duration Units =
     *         The duration is specified in LSB, where at 50Hz ODR:
     *         1 LSB = 20 milliseconds
     *
     *         Duration in seconds = duration_LSB × 0.020
     *
     *         Example conversions:
     *         - 4 LSB = 4 × 20ms = 80ms = 0.08s
     *         - 50ms = 50 / 20 = 2.5 LSB (round to nearest integer)
     *
     *         @warning The duration unit (20ms/LSB) is only accurate at 50Hz ODR.
     *                  For other ODR settings, the time per sample changes.
     *                  Time per sample = 1 / ODR
     *                  Example at 100Hz ODR: 1 sample = 10ms, so duration = LSB × 10ms
     *
     * @param  any_motion  true to configure any-motion, false for no-motion
     *
     * @param  threshold  Motion detection threshold in LSB units (1 LSB = 0.48 mG)
     *                    Recommended range: 10-200 (4.8-96 mG)
     *                    Higher values = less sensitive, lower values = more sensitive
     *
     * @param  duration   Detection duration in LSB units (1 LSB = 20ms at 50Hz ODR)
     *                    Recommended range: 1-20 (20ms to 400ms)
     *                    Longer duration = more stable detection, less false positives
     *
     * @return true if configuration successful, false on error
     */
    bool configMotion(bool any_motion, uint16_t threshold, uint16_t duration)
    {
        int8_t rslt = 0;

        if (threshold > 16380) {  // Max threshold based on 0.48mg resolution for 16g range
            log_e("Threshold %u exceeds recommended maximum (16380)", threshold);
        }

        if (duration > 16383) {  // Maximum 14-bit value
            log_e("Duration %u exceeds recommended maximum (16383)", duration);
        }

        struct bma456h_any_no_mot_config any_mot_config = {0, 0, 0, 0};

        if (any_motion) {
            rslt = bma456h_get_any_mot_config(&any_mot_config, dev.get());
            if (rslt != 0) {
                log_e("bma456h_an_get_any_mot_config failed");
                return false;
            }
        } else {
            rslt = bma456h_get_no_mot_config(&any_mot_config, dev.get());
            if (rslt != 0) {
                log_e("bma456h_get_no_mot_config failed");
                return false;
            }
        }

        any_mot_config.threshold = threshold;
        any_mot_config.duration = duration;

        log_d("Configuring motion detection: threshold=%u LSB (%.2f mG), duration=%u LSB (%.2f ms at 50Hz)",
              threshold, threshold * 0.48f,
              duration, duration * 20.0f);

        if (any_motion) {
            if (bma456h_set_any_mot_config(&any_mot_config, dev.get()) != 0) {
                log_e("Failed to configure any-motion detection");
                return false;
            }
        } else {
            if (bma456h_set_no_mot_config(&any_mot_config, dev.get()) != 0) {
                log_e("Failed to configure no-motion detection");
                return false;
            }
        }
        return true;
    }


    /**
    * @brief  Configure any-motion detection using physical units
    * @note   Easier to use than configAnyMotion() as it accepts physical units
    *
    * @param  threshold_mg  Threshold in milli-g (1 mG = 0.001 g)
    *                       Will be converted to LSB: LSB = threshold_mg / 0.48
    * @param  duration_ms   Duration in milliseconds (at 50Hz ODR)
    *                       Will be converted to LSB: LSB = duration_ms / 20
    * @return true if successful, false on error
    */
    bool configAnyMotionInPhysicalUnits(float threshold_mg, float duration_ms)
    {
        uint16_t threshold_lsb = static_cast<uint16_t>(threshold_mg / 0.48f + 0.5f);
        uint16_t duration_lsb = static_cast<uint16_t>(duration_ms / 20.0f + 0.5f);

        log_d("Converting: %.2f mg -> %u LSB, %.2f ms -> %u LSB",
              threshold_mg, threshold_lsb, duration_ms, duration_lsb);

        return configAnyMotion(threshold_lsb, duration_lsb);
    }


    /**
    * @brief  Configure no-motion detection using physical units
    * @note   Easier to use than configNoMotion() as it accepts physical units
    *
    * @param  threshold_mg  Threshold in milli-g (1 mG = 0.001 g)
    *                       Will be converted to LSB: LSB = threshold_mg / 0.48
    * @param  duration_ms   Duration in milliseconds (at 50Hz ODR)
    *                       Will be converted to LSB: LSB = duration_ms / 20
    * @return true if successful, false on error
    */
    bool configNoMotionInPhysicalUnits(float threshold_mg, float duration_ms)
    {
        uint16_t threshold_lsb = static_cast<uint16_t>(threshold_mg / 0.48f + 0.5f);
        uint16_t duration_lsb = static_cast<uint16_t>(duration_ms / 20.0f + 0.5f);

        log_d("Converting: %.2f mg -> %u LSB, %.2f ms -> %u LSB",
              threshold_mg, threshold_lsb, duration_ms, duration_lsb);

        return configNoMotion(threshold_lsb, duration_lsb);
    }

    /**
     * @brief  Get current motion detection configuration
     * @note   Reads back the current threshold and duration settings
     *
     * @param[out] threshold_mg  Returns threshold in milli-g
     * @param[out] duration_ms   Returns duration in milliseconds (at 50Hz ODR)
     * @return true if successful, false on error
     */
    bool getAnyMotionConfig(float &threshold_mg, float &duration_ms)
    {
        struct bma456h_any_no_mot_config any_mot_config;
        if (bma456h_get_any_mot_config(&any_mot_config, dev.get()) != 0) {
            log_e("Failed to read motion configuration");
            return false;
        }

        threshold_mg = any_mot_config.threshold * 0.48f;
        duration_ms = any_mot_config.duration * 20.0f;

        log_d("Current config: threshold=%u LSB (%.2f mg), duration=%u LSB (%.2f ms)",
              any_mot_config.threshold, threshold_mg,
              any_mot_config.duration, duration_ms);

        return true;
    }

    /**
    * @brief  Get current no-motion detection configuration
    * @note   Reads back the current threshold and duration settings
    *
    * @param[out] threshold_mg  Returns threshold in milli-g
    * @param[out] duration_ms   Returns duration in milliseconds (at 50Hz ODR)
    * @return true if successful, false on error
    */
    bool getNoMotionConfig(float &threshold_mg, float &duration_ms)
    {
        struct bma456h_any_no_mot_config no_mot_config;
        if (bma456h_get_no_mot_config(&no_mot_config, dev.get()) != 0) {
            log_e("Failed to read no-motion configuration");
            return false;
        }

        threshold_mg = no_mot_config.threshold * 0.48f;
        duration_ms = no_mot_config.duration * 20.0f;

        log_d("Current no-motion config: threshold=%u LSB (%.2f mg), duration=%u LSB (%.2f ms)",
              no_mot_config.threshold, threshold_mg,
              no_mot_config.duration, duration_ms);

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
        if (bma456h_step_counter_output(&step_count, dev.get()) != 0) {
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
        return bma456h_reset_step_counter(dev.get()) == 0;
    }

    /**
    * @brief  Get the current activity type
    * @note   Reads the current activity classification from the sensor.
    *         Activity recognition must be enabled first.
    * @return ActivityType enum value, or UNKNOWN on error
    */
    ActivityType getActivityType()
    {
        bma456h_out_state  activity_type = {0, 0, 0, 0};
        if (bma456h_output_state(&activity_type, dev.get()) != 0) {
            log_e("bma456h_get_activity_type failed");
            return ActivityType::UNKNOWN;
        }
        return static_cast<ActivityType>(activity_type.activity_type);
    }

    /**
    * @brief  Get the detected tap type
    * @note   Determines what type of tap was detected (single, double, or triple).
    *         Tap detection must be enabled first.
    * @return TapType enum value, or NO_TAP on error or no tap detected
    */
    TapType getTapType()
    {
        bma456h_out_state tap_type = { 0, 0, 0, 0 };
        if (bma456h_output_state(&tap_type, dev.get()) != 0) {
            log_e("bma456h_get_tap_type failed");
            return TapType::NO_TAP;
        }
        if (tap_type.triple_tap) {
            return TapType::TRIPLE_TAP;
        } else if (tap_type.double_tap) {
            return TapType::DOUBLE_TAP;
        } else if (tap_type.single_tap) {
            return TapType::SINGLE_TAP;
        }
        return TapType::NO_TAP;
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
            return;
        }

        if (_status == 0) {
            return;
        }

        if ((_status & BMA456H_TAP_OUT_INT) && callbacks.onTap) {
            callbacks.onTap(getTapType());
        }

        if ((_status & BMA456H_STEP_CNTR_INT) && callbacks.onStepDetected) {
            callbacks.onStepDetected();
            if (callbacks.onStepCount) {
                uint32_t step_count = 0;
                if (bma456h_step_counter_output(&step_count, dev.get()) == 0) {
                    callbacks.onStepCount(static_cast<uint32_t>(step_count));
                }
            }
        }

        if ((_status & BMA456H_ACTIVITY_INT) && callbacks.onActivity) {
            callbacks.onActivity(getActivityType());
        }

        if ((_status & BMA456H_ANY_MOT_INT) && callbacks.onAnyMotion) {
            callbacks.onAnyMotion();
        }

        if ((_status & BMA456H_NO_MOT_INT) && callbacks.onNoMotion) {
            callbacks.onNoMotion();
        }

        if ((_status & BMA4_ACCEL_DATA_RDY_INT) && callbacks.onDataReady) {
            AccelerometerData data;
            if (readData(data)) {
                callbacks.onDataReady(data);
            }
        }

        _status = 0;
    }

private:
    /**
     * @brief  Bosch-specific initialization implementation
     * @note   Initializes the BMA456H sensor and loads its configuration file
     * @return true if initialization successful, false otherwise
     */
    bool boschInitImpl() override
    {
        int8_t rslt = bma456h_init(dev.get());
        if ( rslt != 0 ) {
            log_e("bma456h_init failed with code %d", rslt);
            return false;
        }
        rslt = bma456h_write_config_file(dev.get());
        if (rslt != 0) {
            log_e("bma456h_write_config_file failed with code %d", rslt);
            return false;
        }
        return true;
    }
protected:
    EventCallbacks callbacks;
    uint16_t _status;
};

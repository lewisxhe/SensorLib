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
 * @file      SensorQMI8658.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-15
 *
 */
#pragma once

#include <functional>
#include "../../ImuBase.hpp"
#include "../../../platform/comm/ComplexDeviceWithHal.hpp"

static constexpr uint8_t QMI8658_DEFAULT_I2C_ADDR_LOW  = 0x6B;
static constexpr uint8_t QMI8658_DEFAULT_I2C_ADDR_HIGH = 0x6A;

/**
 * @brief QMI8658 IMU sensor driver class.
 */
class SensorQMI8658 : public ComplexDeviceWithHal, public ImuBase
{
public:

    /**
     * @brief Accelerometer full-scale range enumeration.
     */
    enum class AccelRange : uint8_t {
        FS_2G,   //!< ±2g range
        FS_4G,   //!< ±4g range
        FS_8G,   //!< ±8g range
        FS_16G,  //!< ±16g range
    };

    /**
     * @brief Accelerometer output data rate enumeration.
     */
    enum class AccelODR : uint8_t {
        ODR_7Hz,      //!< 7 Hz
        ODR_14Hz,     //!< 14 Hz
        ODR_28Hz,     //!< 28 Hz
        ODR_56Hz,     //!< 56 Hz
        ODR_112Hz,    //!< 112 Hz
        ODR_224Hz,    //!< 224 Hz
        ODR_448Hz,    //!< 448 Hz
        ODR_896Hz,    //!< 896 Hz
        ODR_1792Hz,   //!< 1792 Hz
        ODR_3584Hz,   //!< 3584 Hz
        ODR_7168Hz,   //!< 7168 Hz
        ODR_LP_3Hz,   //!< Low power 3 Hz
        ODR_LP_11Hz,  //!< Low power 11 Hz
        ODR_LP_21Hz,  //!< Low power 21 Hz
        ODR_LP_128Hz, //!< Low power 128 Hz
    };

    /**
     * @brief Gyroscope full-scale range enumeration.
     */
    enum class GyroRange : uint8_t {
        FS_16DPS,   //!< ±16 degrees per second
        FS_32DPS,   //!< ±32 degrees per second
        FS_64DPS,   //!< ±64 degrees per second
        FS_128DPS,  //!< ±128 degrees per second
        FS_256DPS,  //!< ±256 degrees per second
        FS_512DPS,  //!< ±512 degrees per second
        FS_1024DPS, //!< ±1024 degrees per second
        FS_2048DPS, //!< ±2048 degrees per second
    };

    /**
     * @brief Gyroscope output data rate enumeration.
     */
    enum class GyroODR : uint8_t {
        ODR_7Hz,    //!< 7 Hz
        ODR_14Hz,   //!< 14 Hz
        ODR_28Hz,   //!< 28 Hz
        ODR_56Hz,   //!< 56 Hz
        ODR_112Hz,  //!< 112 Hz
        ODR_224Hz,  //!< 224 Hz
        ODR_448Hz,  //!< 448 Hz
        ODR_896Hz,  //!< 896 Hz
        ODR_1792Hz, //!< 1792 Hz
        ODR_3584Hz, //!< 3584 Hz
        ODR_7168Hz, //!< 7168 Hz
    };

    /**
     * @brief Low-pass filter mode enumeration.
     */
    enum class LpfMode : uint8_t {
        MODE_0, //!< 2.66% of output data rate
        MODE_1, //!< 3.63% of output data rate
        MODE_2, //!< 5.39% of output data rate
        MODE_3, //!< 13.37% of output data rate
        OFF    = 0xFF,       //!< Disable low-pass filter
    };

    /**
     * @brief FIFO mode enumeration.
     */
    enum class FifoMode : uint8_t {
        BYPASS,  //!< Bypass mode (FIFO disabled)
        FIFO,    //!< FIFO mode (stops when full)
        STREAM,  //!< Stream mode (overwrites oldest)
    };

    /**
     * @brief FIFO sample count enumeration.
     */
    enum class FifoSamples : uint8_t {
        SAMPLES_16,  //!< 16 samples
        SAMPLES_32,  //!< 32 samples
        SAMPLES_64,  //!< 64 samples
        SAMPLES_128, //!< 128 samples
    };

    using IntPin = InterruptPinMap;
    using TapEvent = ::TapEvent;

    /**
     * @brief Tap detection axis priority enumeration.
     */
    enum class TapPriority : uint8_t {
        X_GT_Y_GT_Z, //!< X > Y > Z
        X_GT_Z_GT_Y, //!< X > Z > Y
        Y_GT_X_GT_Z, //!< Y > X > Z
        Y_GT_Z_GT_X, //!< Y > Z > X
        Z_GT_X_GT_Y, //!< Z > X > Y
        Z_GT_Y_GT_X, //!< Z > Y > X
    };

    /**
     * @brief Motion detection type enumeration.
     */
    enum class MotionType : uint8_t {
        ANY_MOTION      = 0, //!< Any motion detection
        NO_MOTION       = 1, //!< No motion detection
        SIGNIFICANT     = 2, //!< Significant motion detection
    };

    /**
     * @brief Sensor axis layout enumeration for coordinate transformation.
     * @details Different mounting orientations require different axis mappings.
     * Layout values 0-3: Z-axis pointing up
     * Layout values 4-7: Z-axis pointing down
     * Even layouts (0,2,4,6): X-Y axis not swapped
     * Odd layouts (1,3,5,7): X-Y axis swapped
     * Second group (2,3,6,7): X-axis inverted
     * Third group (1,3,5,7): Y-axis inverted
     */
    enum class Layout : uint8_t {
        LAYOUT_DEFAULT     = 0, //!< Default orientation: X=right, Y=forward, Z=up
        LAYOUT_ROTATE_90   = 1, //!< Rotated 90 degrees: X=forward, Y=left, Z=up
        LAYOUT_ROTATE_180  = 2, //!< Rotated 180 degrees: X=left, Y=backward, Z=up
        LAYOUT_ROTATE_270  = 3, //!< Rotated 270 degrees: X=backward, Y=right, Z=up
        LAYOUT_FLIP_Z      = 4, //!< Flipped: X=right, Y=backward, Z=down
        LAYOUT_FLIP_Z_90   = 5, //!< Flipped + 90: X=backward, Y=left, Z=down
        LAYOUT_FLIP_Z_180  = 6, //!< Flipped + 180: X=left, Y=forward, Z=down
        LAYOUT_FLIP_Z_270  = 7, //!< Flipped + 270: X=forward, Y=right, Z=down
    };

    /**
     * @brief Callback function type for motion events.
     */
    using MotionCallback = std::function<void()>;

    /**
     * @brief Callback function type for tap events.
     */
    using TapCallback = std::function<void(TapEvent event)>;

    /**
     * @brief Callback function type for data ready events.
     */
    using DataReadyCallback = std::function<void()>;

    /**
     * @brief Grouped callbacks for QMI8658 events.
     */
    struct EventCallbacks {
        MotionCallback onAnyMotion = nullptr;
        MotionCallback onNoMotion = nullptr;
        MotionCallback onSignificantMotion = nullptr;
        MotionCallback onWakeOnMotion = nullptr;
        MotionCallback onPedometer = nullptr;
        TapCallback onTap = nullptr;
        DataReadyCallback onAccelDataReady = nullptr;
        DataReadyCallback onGyroDataReady = nullptr;
        DataReadyCallback onDataLocking = nullptr;
    };

    // ==================== Constructor / Destructor ====================

    /**
     * @brief Construct a new SensorQMI8658 object.
     */
    SensorQMI8658();

    /**
     * @brief Destroy the SensorQMI8658 object.
     */
    ~SensorQMI8658() override;

    // ==================== ImuBase Interface Implementation ====================

    /**
     * @brief Reset the IMU sensor to its default state.
     * @return true if reset successful, false otherwise.
     */
    bool reset() override;

    /**
     * @brief Perform self-test on the IMU sensor.
     * @return true if self-test passed, false otherwise.
     */
    bool selfTest() override;

    /**
     * @brief Check if the IMU sensor is responding.
     * @return true if sensor is present, false otherwise.
     */
    bool isPresent() override;

    /**
     * @brief Get the chip ID / WHO_AM_I value.
     * @return Chip ID value.
     */
    uint8_t getChipID() override;

    /**
     * @brief Get the firmware version.
     * @return Firmware version value.
     */
    uint32_t getFirmwareVersion() override;

    /**
     * @brief Set IMU operation mode.
     * @param mode Target operation mode.
     * @return true if mode switch successful, false otherwise.
     */
    bool setOperationMode(OperationMode mode) override;

    /**
     * @brief Get the temperature reading from the sensor.
     * @return Temperature in degrees Celsius, or NaN on error.
     */
    float getTemperature() override;

    /**
     * @brief Get the timestamp from the sensor.
     * @return Timestamp value.
     */
    uint32_t getTimestamp() override;

    /**
     * @brief Check if requested IMU data is available.
     * @param mask Data-ready target mask (ACCEL / GYRO / BOTH).
     * @return true if requested data is ready, false otherwise.
     */
    bool isDataReady(uint8_t mask = static_cast<uint8_t>(ImuBase::DataReadyMask::BOTH)) override;

    /**
     * @brief Read accelerometer data.
     * @param[out] out Reference to store the read data.
     * @return true if read successful, false otherwise.
     */
    bool readAccel(AccelerometerData &out) override;

    /**
     * @brief Enable the accelerometer.
     * @return true if enabled successfully, false otherwise.
     */
    bool enableAccel() override;

    /**
     * @brief Disable the accelerometer.
     * @return true if disabled successfully, false otherwise.
     */
    bool disableAccel() override;

    /**
     * @brief Check if accelerometer is enabled.
     * @return true if enabled, false otherwise.
     */
    bool isAccelEnabled() override;

    /**
     * @brief Set accelerometer full scale range.
     * @param range Full scale range setting.
     * @return true if set successfully, false otherwise.
     */
    bool setAccelFullScaleRange(AccelFullScaleRange range) override;

    /**
     * @brief Set accelerometer output data rate.
     * @param data_rate_hz Output data rate in Hz.
     * @return true if set successfully, false otherwise.
     */
    bool setAccelOutputDataRate(float data_rate_hz) override;

    /**
     * @brief Read raw accelerometer data without scaling.
     * @param[out] x X-axis raw value.
     * @param[out] y Y-axis raw value.
     * @param[out] z Z-axis raw value.
     * @return true if read successful, false otherwise.
     */
    bool readAccelRaw(int16_t &x, int16_t &y, int16_t &z) override;

    /**
     * @brief Read gyroscope data.
     * @param[out] out Reference to store the read data.
     * @return true if read successful, false otherwise.
     */
    bool readGyro(GyroscopeData &out) override;

    /**
     * @brief Enable the gyroscope.
     * @return true if enabled successfully, false otherwise.
     */
    bool enableGyro() override;

    /**
     * @brief Disable the gyroscope.
     * @return true if disabled successfully, false otherwise.
     */
    bool disableGyro() override;

    /**
     * @brief Check if gyroscope is enabled.
     * @return true if enabled, false otherwise.
     */
    bool isGyroEnabled() override;

    /**
     * @brief Set gyroscope full scale range.
     * @param range Full scale range setting.
     * @return true if set successfully, false otherwise.
     */
    bool setGyroFullScaleRange(GyroFullScaleRange range) override;

    /**
     * @brief Set gyroscope output data rate.
     * @param data_rate_hz Output data rate in Hz.
     * @return true if set successfully, false otherwise.
     */
    bool setGyroOutputDataRate(float data_rate_hz) override;

    /**
     * @brief Read raw gyroscope data without scaling.
     * @param[out] x X-axis raw value.
     * @param[out] y Y-axis raw value.
     * @param[out] z Z-axis raw value.
     * @return true if read successful, false otherwise.
     */
    bool readGyroRaw(int16_t &x, int16_t &y, int16_t &z) override;

    /**
     * @brief Enable synchronous sampling mode.
     * @return true if enabled successfully, false otherwise.
     */
    bool enableSyncMode() override;

    /**
     * @brief Disable synchronous sampling mode.
     * @return true if disabled successfully, false otherwise.
     */
    bool disableSyncMode() override;

    /**
     * @brief Configure the FIFO buffer.
     * @param enable true to enable FIFO, false to disable.
     * @param watermark_samples Number of samples for watermark interrupt.
     * @return true if configured successfully, false otherwise.
     */
    bool configureFifo(bool enable, uint8_t watermark_samples = 16) override;

    /**
     * @brief Read data from FIFO buffer.
     * @param[out] accel_data Array to store accelerometer samples.
     * @param accel_count Maximum number of accelerometer samples.
     * @param[out] gyro_data Array to store gyroscope samples.
     * @param gyro_count Maximum number of gyroscope samples.
     * @return Number of samples read, or 0 on error.
     */
    uint16_t readFromFifo(AccelerometerData *accel_data, uint16_t accel_count,
                          GyroscopeData *gyro_data, uint16_t gyro_count) override;

    // ==================== Accelerometer Configuration ====================

    /**
     * @brief Configure the accelerometer with specified parameters.
     * @param range Full-scale range.
     * @param odr Output data rate.
     * @param lpf Low-pass filter mode (default: MODE_0).
     * @note If accelerometer and gyroscope are both enabled (6DOF),
     *       the effective synchronized ODR base is derived from gyroscope natural frequency.
     * @return true if configuration successful, false otherwise.
     */
    bool configAccel(AccelFullScaleRange range, float data_rate_hz, LpfMode lpf = LpfMode::MODE_0);

    // ==================== Gyroscope Configuration ====================

    /**
     * @brief Configure the gyroscope with specified parameters.
     * @param range Full-scale range.
     * @param odr Output data rate.
     * @param lpf Low-pass filter mode (default: MODE_0).
     * @note In 6DOF mode (ACC+GYR enabled), system ODR synchronization
     *       is derived from gyroscope natural frequency.
     * @return true if configuration successful, false otherwise.
     */
    bool configGyro(GyroFullScaleRange range, float data_rate_hz, LpfMode lpf = LpfMode::MODE_0);

    // ==================== Interrupt Configuration ====================

    /**
     * @brief Enable interrupt on specified pin.
     * @param pin Interrupt pin.
     * @return true if successful, false otherwise.
     */
    bool enableInterrupt(IntPin pin);

    /**
     * @brief Disable interrupt on specified pin.
     * @param pin Interrupt pin.
     * @return true if successful, false otherwise.
     */
    bool disableInterrupt(IntPin pin);

    /**
     * @brief Enable data ready interrupt.
     * @param pin Interrupt pin for data ready signal.
     * @note In Non-SyncSample mode, DRDY is only routed to INT2 when CTRL7.bit5=0.
     *       If FIFO mode is enabled (FIFO_MODE != bypass), DRDY is disabled by hardware.
     * @return true if successful, false otherwise.
     */
    bool enableDataReadyInterrupt(IntPin pin = IntPin::PIN2);

    /**
     * @brief Get interrupt status.
     * @return Interrupt status register value.
     */
    uint8_t getInterruptStatus();

    /**
     * @brief Set interrupt pin.
     * @param pin GPIO pin number.
     */
    void setPins(int pin);

    /**
     * @brief Backward-compatible alias of setPins.
     * @param pin GPIO pin number.
     */
    void setIntPin(int pin);

    // ==================== FIFO Operations ====================

    /**
     * @brief Configure FIFO with specified mode.
     * @param mode FIFO mode.
     * @note In Non-SyncSample mode, enabling FIFO (mode != BYPASS) disables DRDY function.
     * @param samples FIFO sample count.
     * @param watermark_samples Watermark threshold.
     * @return true if configuration successful, false otherwise.
     */
    bool configFifo(FifoMode mode, FifoSamples samples = FifoSamples::SAMPLES_16,
                   uint8_t watermark_samples = 16);

    /**
     * @brief Reset the FIFO.
     * @return true if successful, false otherwise.
     */
    bool resetFifo();

    /**
     * @brief Get FIFO status.
     * @return FIFO status register value.
     */
    uint8_t getFifoStatus();

    // ==================== Synchronization ====================

    /**
     * @brief Enable the data locking mechanism.
     * @return true if successful, false otherwise.
     */
    bool enableLockingMechanism();

    /**
     * @brief Disable the data locking mechanism.
     * @return true if successful, false otherwise.
     */
    bool disableLockingMechanism();

    // ==================== Calibration ====================

    /**
     * @brief Perform on-demand calibration.
     * @param[out] gyro_x_gain Optional output for X-axis gyro gain.
     * @param[out] gyro_y_gain Optional output for Y-axis gyro gain.
     * @param[out] gyro_z_gain Optional output for Z-axis gyro gain.
     * @return true if calibration successful, false otherwise.
     */
    bool calibrate(uint16_t *gyro_x_gain = nullptr, uint16_t *gyro_y_gain = nullptr,
                   uint16_t *gyro_z_gain = nullptr);

    /**
     * @brief Write previously saved gyro calibration gains.
     * @param gyro_x_gain X-axis gyro gain.
     * @param gyro_y_gain Y-axis gyro gain.
     * @param gyro_z_gain Z-axis gyro gain.
     * @return true if write successful, false otherwise.
     */
    bool writeCalibration(uint16_t gyro_x_gain, uint16_t gyro_y_gain, uint16_t gyro_z_gain);

    /**
     * @brief Set accelerometer host delta offsets.
     * @param x X-axis offset.
     * @param y Y-axis offset.
     * @param z Z-axis offset.
     */
    void setAccelOffset(int16_t x, int16_t y, int16_t z);

    /**
     * @brief Set gyroscope host delta offsets.
     * @param x X-axis offset.
     * @param y Y-axis offset.
     * @param z Z-axis offset.
     */
    void setGyroOffset(int16_t x, int16_t y, int16_t z);

    // ==================== Self-Test ====================

    /**
     * @brief Perform accelerometer self-test.
     * @return true if self-test passed, false otherwise.
     */
    bool selfTestAccel();

    /**
     * @brief Perform gyroscope self-test.
     * @return true if self-test passed, false otherwise.
     */
    bool selfTestGyro();

    // ==================== Motion Detection ====================

    /**
     * @brief Configure motion detection parameters.
     * @param type Motion detection type.
     * @param threshold_x X-axis threshold in mg.
     * @param threshold_y Y-axis threshold in mg.
     * @param threshold_z Z-axis threshold in mg.
     * @param duration Number of consecutive samples above/below threshold.
     * @return true if configuration successful, false otherwise.
     */
    bool configMotionDetect(MotionType type, float threshold_x, float threshold_y,
                            float threshold_z, uint8_t duration);

    /**
     * @brief Configure motion detection with default parameters.
     * @details Provides sensible defaults for motion detection based on type:
     *          - ANY_MOTION: 100mg threshold, 4 samples duration
     *          - NO_MOTION: 100mg threshold, 10 samples duration
     *          - SIGNIFICANT: 200mg threshold, 8 samples duration
     * @param type Motion detection type.
     * @return true if configuration successful, false otherwise.
     */
    bool configMotionDetectDefault(MotionType type);

    /**
     * @brief Enable motion detection on specified pin.
     * @param pin Interrupt pin for motion events.
     * @return true if enabled successfully, false otherwise.
     */
    bool enableMotionDetect(IntPin pin = IntPin::PIN1);

    /**
     * @brief Disable motion detection.
     * @return true if disabled successfully, false otherwise.
     */
    bool disableMotionDetect();

    /**
     * @brief Configure wake-on-motion (WoM) feature.
     * @param threshold WoM threshold in mg (default: 200mg).
     * @param odr Accelerometer ODR for WoM (default: LP_128Hz).
     * @param pin Interrupt pin.
     * @return true if configuration successful, false otherwise.
     */
    bool configWakeOnMotion(uint8_t threshold = 200, float odr_hz = 128.0f,
                            IntPin pin = IntPin::PIN2);

    /**
     * @brief Configure wake-on-motion (WoM) with full vendor parameters.
     * @param threshold WoM threshold register value.
     * @param odr_hz Accelerometer ODR in Hz.
     * @param pin Interrupt pin routing.
     * @param default_pin_value Initial interrupt pin level (0 or 1).
     * @param blanking_time Interrupt blanking time (accelerometer samples).
     * @param acc_range Accelerometer range used in WoM mode.
     * @return true if configuration successful, false otherwise.
     */
    bool configWakeOnMotionAdvanced(uint8_t threshold, float odr_hz, IntPin pin,
                                    uint8_t default_pin_value, uint8_t blanking_time,
                                    AccelFullScaleRange acc_range);

    // ==================== Legacy Compatibility API ====================
    // Keep old API signatures here only, so they can be removed together later.
    bool configAccel(AccelRange range, AccelODR odr, LpfMode lpf = LpfMode::MODE_0);
    bool configAccel(AccelRange range, float data_rate_hz, LpfMode lpf = LpfMode::MODE_0);
    bool configGyro(GyroRange range, GyroODR odr, LpfMode lpf = LpfMode::MODE_0);
    bool configGyro(GyroRange range, float data_rate_hz, LpfMode lpf = LpfMode::MODE_0);
    bool configWakeOnMotion(uint8_t threshold, AccelODR odr, IntPin pin);
    bool configWakeOnMotion(uint8_t threshold, AccelODR odr, IntPin pin,
                            uint8_t default_pin_value, uint8_t blanking_time,
                            AccelRange acc_range);

    // ==================== Tap Detection ====================

    /**
     * @brief Configure tap detection parameters.
     * @param priority Axis priority for tap detection.
     * @param peak_window Maximum peak duration in samples.
     * @param tap_window Minimum quiet time between taps.
     * @param double_tap_window Maximum time for double tap.
     * @param peak_mag_threshold Peak magnitude threshold in g^2.
     * @param quiet_threshold Undefined motion threshold in g^2.
     * @return true if configuration successful, false otherwise.
     */
    bool configTap(TapPriority priority, uint8_t peak_window, uint16_t tap_window,
                   uint16_t double_tap_window, float peak_mag_threshold, float quiet_threshold);

    /**
     * @brief Configure tap detection with default parameters.
     * @details Provides sensible defaults for tap detection. Uses X>Y>Z axis priority.
     * @param priority Axis priority for tap detection.
     * @return true if configuration successful, false otherwise.
     */
    bool configTapDefault(TapPriority priority = TapPriority::X_GT_Y_GT_Z);

    /**
     * @brief Enable tap detection.
     * @param pin Interrupt pin for tap events.
     * @return true if enabled successfully, false otherwise.
     */
    bool enableTap(IntPin pin = IntPin::PIN1);

    /**
     * @brief Disable tap detection.
     * @return true if disabled successfully, false otherwise.
     */
    bool disableTap();

    /**
     * @brief Get tap status.
     * @return TapEvent indicating the detected tap type.
     */
    TapEvent getTapStatus();

    // ==================== Pedometer ====================

    /**
     * @brief Configure pedometer parameters.
     * @param sample_count Number of samples per calculation window.
     * @param peak_to_peak Peak-to-peak threshold in mg.
     * @param peak_threshold Peak threshold in mg.
     * @param time_up Maximum step duration in samples.
     * @param time_low Minimum step duration in samples.
     * @param entry_count Minimum steps to start counting.
     * @param fix_precision Pedometer precision parameter (recommended 0).
     * @param sig_count Steps per output update.
     * @return true if configuration successful, false otherwise.
     */
    bool configPedometer(uint16_t sample_count, uint16_t peak_to_peak, uint16_t peak_threshold,
                         uint16_t time_up, uint8_t time_low = 20, uint8_t entry_count = 10,
                         uint8_t fix_precision = 0, uint8_t sig_count = 4);

    /**
     * @brief Configure pedometer with default parameters.
     * @details Provides sensible defaults for step counting. Works well for normal walking.
     * @param odr Current accelerometer ODR in Hz (for time calculation).
     * @return true if configuration successful, false otherwise.
     */
    bool configPedometerDefault(float odr = 125.0f);

    /**
     * @brief Enable pedometer.
     * @param pin Interrupt pin for pedometer events.
     * @return true if enabled successfully, false otherwise.
     */
    bool enablePedometer(IntPin pin = IntPin::DISABLE);

    /**
     * @brief Disable pedometer.
     * @return true if disabled successfully, false otherwise.
     */
    bool disablePedometer();

    /**
     * @brief Get pedometer step count.
     * @return Number of steps counted.
     */
    uint32_t getStepCount();

    /**
     * @brief Reset pedometer counter.
     * @return true if reset successful, false otherwise.
     */
    bool resetStepCount();

    // ==================== Callbacks ====================

    /**
     * @brief Set callback for any-motion event.
     * @param callback Callback function.
     */
    void setAnyMotionCallback(MotionCallback callback);

    /**
     * @brief Set callback for no-motion event.
     * @param callback Callback function.
     */
    void setNoMotionCallback(MotionCallback callback);

    /**
     * @brief Set callback for significant motion event.
     * @param callback Callback function.
     */
    void setSignificantMotionCallback(MotionCallback callback);

    /**
     * @brief Set callback for wake-on-motion event.
     * @param callback Callback function.
     */
    void setWakeOnMotionCallback(MotionCallback callback);

    /**
     * @brief Set callback for tap event.
     * @param callback Callback function.
     */
    void setTapCallback(TapCallback callback);

    /**
     * @brief Set callback for pedometer event.
     * @param callback Callback function.
     */
    void setPedometerCallback(MotionCallback callback);

    /**
     * @brief Set callback for accelerometer data ready.
     * @param callback Callback function.
     */
    void setAccelDataReadyCallback(DataReadyCallback callback);

    /**
     * @brief Set callback for gyroscope data ready.
     * @param callback Callback function.
     */
    void setGyroDataReadyCallback(DataReadyCallback callback);

    /**
     * @brief Set callback for data locking.
     * @param callback Callback function.
     */
    void setDataLockingCallback(DataReadyCallback callback);

    /**
     * @brief Set all event callbacks at once.
     */
    void setCallbacks(const EventCallbacks &cbs);

    /**
     * @brief Get current callbacks.
     */
    EventCallbacks &getCallbacks();

    /**
     * @brief Get current callbacks (const).
     */
    const EventCallbacks &getCallbacks() const;

    // ==================== Utility ====================

    /**
     * @brief Update sensor status and process events.
     * @return Combined status flags.
     */
    uint16_t update();

    /**
     * @brief Dump control registers for debugging.
     */
    void dumpRegisters();

    /**
     * @brief Get sensor USID (unique chip ID).
     * @param[out] buffer Buffer to store USID (must be at least 6 bytes).
     * @param length Number of bytes to read.
     */
    void getChipUsid(uint8_t *buffer, uint8_t length);

    /**
     * @brief Get the accelerometer scale factor.
     * @return Scale factor in m/s^2 per LSB.
     */
    float getAccelScale();

    /**
     * @brief Get the gyroscope scale factor.
     * @return Scale factor in rad/s per LSB.
     */
    float getGyroScale();

    // ==================== Axis Conversion ====================

    /**
     * @brief Set the sensor axis layout for coordinate transformation.
     * @param layout The axis layout to apply.
     */
    void setAxisLayout(Layout layout);

    /**
     * @brief Get the current axis layout setting.
     * @return Current axis layout.
     */
    Layout getAxisLayout() const;

    /**
     * @brief Apply axis conversion to raw sensor data.
     * @param[in,out] accel_accel Accel data array [x, y, z].
     * @param[in,out] gyro_data Gyro data array [x, y, z].
     */
    void axisConvert(int16_t accel_data[3], int16_t gyro_data[3]);

    // ==================== Static Calibration ====================

    /**
     * @brief Start static calibration.
     * @details Static calibration calculates accelerometer and gyroscope offsets
     * when the sensor is stationary. Place the sensor in a stable position
     * and call processCalibration() periodically until it returns true.
     * @param enable True to enable static calibration, false to disable.
     */
    void enableStaticCalibration(bool enable);

    /**
     * @brief Check if static calibration is enabled.
     * @return true if enabled, false otherwise.
     */
    bool isStaticCalibrationEnabled() const;

    /**
     * @brief Get the current static calibration status.
     * @return true if calibration is complete, false otherwise.
     */
    bool isStaticCalibrationComplete() const;

    /**
     * @brief Process static calibration with current sensor data.
     * @param accel_data Raw accelerometer data [x, y, z].
     * @param gyro_data Raw gyroscope data [x, y, z].
     * @return true if calibration is complete, false if still collecting data.
     */
    bool processStaticCalibration(int16_t accel_data[3], int16_t gyro_data[3]);

    /**
     * @brief Get static calibration offsets.
     * @param[out] accel_offset Accel offsets [x, y, z] in raw units.
     * @param[out] gyro_offset Gyro offsets [x, y, z] in raw units.
     */
    void getStaticCalibrationOffsets(int16_t accel_offset[3], int16_t gyro_offset[3]);

    /**
     * @brief Reset static calibration.
     */
    void resetStaticCalibration();

    // ==================== Dynamic Gyroscope Calibration ====================

    /**
     * @brief Enable dynamic gyroscope calibration.
     * @details Dynamic calibration continuously updates gyro offsets when
     * the sensor detects static periods. It uses accelerometer data to
     * determine when the device is stationary.
     * @param enable True to enable, false to disable.
     */
    void enableDynamicGyroCalibration(bool enable);

    /**
     * @brief Check if dynamic gyro calibration is enabled.
     * @return true if enabled, false otherwise.
     */
    bool isDynamicGyroCalibrationEnabled() const;

    /**
     * @brief Get dynamic gyroscope calibration offsets.
     * @param[out] gyro_offset Gyro offsets [x, y, z] in raw units.
     */
    void getDynamicGyroCalibrationOffsets(int16_t gyro_offset[3]);

    /**
     * @brief Process dynamic calibration with current sensor data.
     * @param accel_data Raw accelerometer data [x, y, z].
     * @param gyro_data Raw gyroscope data [x, y, z].
     */
    void processDynamicCalibration(int16_t accel_data[3], int16_t gyro_data[3]);

    // ==================== Hardware Self-Test ====================

    /**
     * @brief Perform hardware self-test on both sensors.
     * @param includeAccel True to include accelerometer self-test.
     * @param includeGyro True to include gyroscope self-test.
     * @return true if all tests passed, false otherwise.
     */
    bool hardwareSelfTest(bool includeAccel = true, bool includeGyro = true);

    /**
     * @brief Get hardware self-test results.
     * @param[out] accel_result Accel self-test results [x, y, z] in mg.
     * @param[out] gyro_result Gyro self-test results [x, y, z] in dps.
     */
    void getHardwareSelfTestResults(float accel_result[3], float gyro_result[3]);

protected:
    bool initImpl(uint8_t param) override;
    int writeCommand(uint8_t cmd, uint32_t timeout_ms = 1000);
    uint8_t mgToBytes(float mg);

    uint8_t getAccelRangeRegValue(AccelFullScaleRange range);
    uint8_t getGyroRangeRegValue(GyroFullScaleRange range);
    float getAccelScaleFromRange(AccelFullScaleRange range);
    float getGyroScaleFromRange(GyroFullScaleRange range);
    bool applyMotionConfig();

    float odrToFloat(float odr);
    uint8_t findClosestAccelOdr(float hz);
    uint8_t findClosestGyroOdr(float hz);
    uint8_t findClosestAccelLpOdr(float hz);

    static constexpr uint8_t STATUS_TAP_EVENT          = _BV(1);
    static constexpr uint8_t STATUS_WOM_EVENT          = _BV(2);
    static constexpr uint8_t STATUS_PEDOMETER_EVENT    = _BV(4);
    static constexpr uint8_t STATUS_ANY_MOTION_EVENT   = _BV(5);
    static constexpr uint8_t STATUS_NO_MOTION_EVENT    = _BV(6);
    static constexpr uint8_t STATUS_SIGNIFICANT_MOTION = _BV(7);

    AccelFullScaleRange _accel_range;
    GyroFullScaleRange _gyro_range;
    float _accel_scale;
    float _gyro_scale;
    float _accel_odr;
    float _gyro_odr;

    bool _motion_any_configured;
    bool _motion_no_configured;
    bool _motion_sig_configured;
    uint8_t _motion_any_thr[3];
    uint8_t _motion_no_thr[3];
    uint8_t _motion_any_window;
    uint8_t _motion_no_window;
    uint16_t _motion_sig_wait_window;
    uint16_t _motion_sig_confirm_window;

    bool _accel_enabled;
    bool _gyro_enabled;
    bool _sync_mode;
    bool _fifo_enabled;
    uint8_t _fifo_mode;
    uint8_t _fifo_watermark;

    int _int_pin;
    uint8_t _int_pin_mask;
    bool _int_enabled;
    uint32_t _last_step_count;
    bool _tap_event_active;
    TapEvent _last_tap_event;
    uint32_t _last_tap_event_ms;

    uint32_t _last_timestamp;

    uint32_t _firmware_version;
    uint8_t _usid[6];

    EventCallbacks callbacks;

    uint8_t *_fifo_buffer;
    uint16_t _fifo_buffer_size;

    Layout _axis_layout;

    static constexpr uint16_t MAX_STATIC_CALI_SAMPLES = 100;
    uint16_t _static_cali_sample_count;
    bool _static_cali_enabled;
    bool _static_cali_complete;
    int32_t _accel_cali_sum[3];
    int32_t _gyro_cali_sum[3];
    int16_t _accel_cali_offset[3];
    int16_t _gyro_cali_offset[3];

    static constexpr uint8_t DYN_CAL_BUFFER_SIZE = 10;
    static constexpr uint8_t DYN_CAL_STATIC_COUNT = 12;
    bool _dyn_cal_enabled;
    float _gyro_speed_buffer[DYN_CAL_BUFFER_SIZE];
    float _accel_speed_buffer[DYN_CAL_BUFFER_SIZE];
    int16_t _dyn_gyro_offset[3];
    uint32_t _dyn_static_delay;
    uint8_t _dyn_static_flag;
    uint8_t _dyn_cali_sample_count;
    int32_t _dyn_gyro_sum[3];
    float _gyro_static_deviation;
    float _accel_static_deviation;
    uint8_t _dyn_offset_applied;

    static constexpr float HW_ST_ACCEL_THRESHOLD_MG = 200.0f;
    static constexpr float HW_ST_GYRO_THRESHOLD_DPS = 300.0f;
    float _hw_st_accel_result[3];
    float _hw_st_gyro_result[3];
};

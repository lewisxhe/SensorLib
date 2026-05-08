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
 * @file      HapticDriver_AW86224.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-06
 *
 */

#pragma once

#include "platform/comm/I2CDeviceWithHal.hpp"
#include "HapticBase.hpp"

/**
 * @brief AW86224 I2C slave address
 *
 * Default I2C slave address for AW86224/86225 haptic driver.
 */
static constexpr uint8_t AW8624_SLAVE_ADDRESS = (0x58);

/**
 * @brief AW86224/86225 Haptic Driver Class
 *
 * This class provides a complete interface for controlling the AW86224/86225
 * haptic motor driver. It inherits from HapticBase and I2CDeviceWithHal to
 * provide standardized haptic control and I2C communication.
 *
 * @par Vibration Modes:
 * - STANDBY: Low power standby mode, no vibration
 * - RAM: Single play of RAM waveform
 * - RAM_LOOP: Continuous loop playback of RAM waveform
 * - CONT: Continuous mode using F0 tracking
 * - RTP: Real-time playback mode for custom waveforms
 *
 * @par Calibration:
 * The driver performs F0 (resonant frequency) calibration automatically
 * during initialization. This ensures optimal vibration performance across
 * different motor characteristics.
 *
 * @par Thread Safety:
 * When using non-blocking vibration modes, ensure vibrationUpdate() is called
 * regularly in your main loop to maintain proper timing.
 */
class HapticDriver_AW86224 : public HapticBase, public I2CDeviceWithHal
{
public:

    /**
     * @brief Playback mode enumeration
     *
     * Defines the operating mode of the haptic driver.
     */
    enum class PlayMode : uint8_t {
        STANDBY = 0,  /**< Standby mode, minimal power consumption */
        RAM = 1,      /**< RAM playback mode, single play */
        RAM_LOOP = 2, /**< RAM loop mode, continuous playback */
        CONT = 3,     /**< Continuous mode with F0 tracking */
        RTP = 4       /**< Real-time playback mode */
    };

    /**
     * @brief SRAM size selection
     *
     * Configures the internal SRAM size for waveform storage.
     */
    enum class SramSize : uint8_t {
        SRAM_1K = 0,  /**< 1KB SRAM */
        SRAM_2K = 1,  /**< 2KB SRAM */
        SRAM_3K = 2   /**< 3KB SRAM */
    };

    /**
     * @brief Chip ID enumeration
     *
     * Identifies the connected AWinic haptic chip model.
     */
    enum class ChipID : uint8_t {
        AW86223 = 1,  /**< AW86223 chip */
        AW86224 = 2,  /**< AW86224 chip */
        AW86225 = 3,  /**< AW86225 chip */
        AW86214 = 4   /**< AW86214 chip */
    };

    /**
     * @brief PWM mode selection
     *
     * Defines the PWM sampling rate for waveform playback.
     */
    enum class PwmMode : uint8_t {
        PWM_48K = 0,  /**< 48kHz PWM rate */
        PWM_24K = 1,  /**< 24kHz PWM rate */
        PWM_12K = 2   /**< 12kHz PWM rate (default) */
    };

    //=========================================================================
    // Constructor & Initialization
    //=========================================================================

    /**
     * @brief Default constructor
     *
     * Initializes the driver with default parameters.
     * Call begin() before use.
     */
    HapticDriver_AW86224();

    //=========================================================================
    // HapticBase Interface - Lifecycle
    //=========================================================================

    /**
     * @brief  setPins
     * @note   Sets the reset and interrupt pins for the haptic driver.
     * No support for interrupt pins has been added; they are currently reserved for use as a general interface.
     * @param[in] resetPin: The GPIO pin number for the reset pin.
     * @param[in] intPin: The GPIO pin number for the interrupt pin.
     * @retval None
     */
    void setPins(uint8_t resetPin, uint8_t intPin);

    void end() override;
    bool isReady() const override;
    const char *getChipName() const override;

    //=========================================================================
    // HapticBase Interface - Playback Control
    //=========================================================================

    bool run() override;
    bool stop() override;
    bool isPlaying() const override;
    HapticStatus getStatus() const override;

    //=========================================================================
    // HapticBase Interface - Gain Control
    //=========================================================================

    bool setGain(uint8_t gain) override;
    uint8_t getGain() const override;

    //=========================================================================
    // HapticBase Interface - Effect Playback
    //=========================================================================

    bool playEffect(HapticEffectId effect) override;
    bool playEffectAsync(HapticEffectId effect) override;
    void continuousVibration(uint32_t duration_ms, bool blocking = true) override;
    void vibrationUpdate() override;

    //=========================================================================
    // HapticBase Interface - Waveform Sequence
    //=========================================================================

    bool setSequence(uint8_t slot, HapticEffectId effect) override;
    bool clearSequence() override;
    bool playSequence() override;

    //=========================================================================
    // HapticBase Interface - Device Information
    //=========================================================================

    uint8_t getChipID() const override;
    HapticCapabilities getCapabilities() const override;

    //=========================================================================
    // HapticBase Interface - Calibration
    //=========================================================================

    bool calibrate() override;
    bool needsCalibration() const override;
    uint32_t getF0() const override;
    uint32_t getVbat() override;

    //=========================================================================
    // HapticBase Interface - Mode Control
    //=========================================================================

    bool setMode(HapticMode mode) override;
    HapticMode getMode() const override;
    HapticActuatorType getActuatorType() const override;
    bool setActuatorType(HapticActuatorType type) override;

    //=========================================================================
    // HapticBase Interface - RTP
    //=========================================================================

    bool setRealtimeValue(uint8_t value) override;

    //=========================================================================
    // Core Playback Control
    //=========================================================================

    /**
     * @brief Perform soft reset
     *
     * Resets the chip to default state via software command.
     * This clears any residual configuration from previous operations.
     */
    void softReset();

    //=========================================================================
    // Vibration Effects
    //=========================================================================

    /**
     * @brief Play a short vibration effect
     *
     * Plays a short haptic effect using a RAM waveform with specified loop count.
     * This is a blocking call that waits for the effect to complete.
     *
     * @param[in] index Waveform index (1-4, based on loaded RAM data)
     * @param[in] gain  Vibration intensity (0x00-0xFF, default 0x80)
     * @param[in] loop  Number of repetitions (1-16, default 1)
     *
     * @par Example:
     * @code
     * // Short click
     * haptic.shortVibration(1, 0x80, 1);
     *
     * // Double buzz
     * haptic.shortVibration(2, 0xA0, 2);
     *
     * // Maximum intensity click
     * haptic.shortVibration(1, 0xFF, 1);
     * @endcode
     */
    void shortVibration(uint8_t index, uint8_t gain = 0x80, uint8_t loop = 1);

    /**
     * @brief Play a long continuous vibration effect
     *
     * Plays a continuous vibration effect for the specified duration.
     * Supports both blocking and non-blocking modes.
     *
     * @param[in] index        Waveform index (1-4)
     * @param[in] gain         Vibration intensity (0x00-0xFF)
     * @param[in] duration_ms  Duration in milliseconds
     * @param[in] blocking     true = wait for completion (blocking),
     *                          false = return immediately (non-blocking)
     *
     * @par Blocking Mode Example:
     * @code
     * // Wait 2 seconds for vibration to complete
     * haptic.longVibration(2, 0x80, 2000, true);
     * // Vibration finished, continue with other code
     * @endcode
     *
     * @par Non-Blocking Mode Example:
     * @code
     * // Start vibration, return immediately
     * haptic.longVibration(2, 0x80, 5000, false);
     *
     * // Do other work while vibrating
     * while (haptic.isPlaying()) {
     *     haptic.vibrationUpdate();
     *     // Your code here
     * }
     * @endcode
     *
     * @note In non-blocking mode, call vibrationUpdate() regularly in your loop.
     *       If a new vibration is started while one is running, the previous
     *       vibration will be stopped automatically.
     */
    void longVibration(uint8_t index, uint8_t gain = 0x80, uint32_t duration_ms = 1000, bool blocking = true);

    /**
     * @brief Stop any ongoing vibration
     *
     * Immediately stops vibration and enters standby mode.
     * Works for both blocking and non-blocking modes.
     *
     * @par Example:
     * @code
     * haptic.longVibration(2, 0x80, 10000, false);  // Start long vibration
     * // ...
     * if (emergencyStop) {
     *     haptic.stopVibration();  // Immediate stop
     * }
     * @endcode
     */
    void stopVibration();

    /**
     * @brief Update vibration state (for non-blocking mode)
     *
     * Must be called regularly when using non-blocking vibration modes.
     * This function:
     * - Checks if the vibration duration has elapsed
     * - Auto-restarts playback if it stops prematurely
     * - Stops the vibration when duration is complete
     *
     * @par Usage:
     * @code
     * void loop() {
     *     if (haptic.isPlaying()) {
     *         haptic.vibrationUpdate();
     *     }
     *     // Other loop code...
     * }
     * @endcode
     *
     * @note Call this at least every 10-50ms for smooth vibration.
     */

    //=========================================================================
    // Waveform Configuration
    //=========================================================================

    /**
     * @brief Set waveform sequence
     *
     * Configures which waveform is played for each sequencer slot.
     *
     * @param[in] wav Wave slot (0-7)
     * @param[in] seq Waveform index to play (0-15)
     *
     * @par Example:
     * @code
     * haptic.setWaveSeq(0, 2);  // Slot 0 plays waveform 2
     * haptic.setWaveSeq(1, 0);  // Slot 1 plays waveform 0 (wait)
     * @endcode
     */
    void setWaveSeq(uint8_t wav, uint8_t seq);

    /**
     * @brief Set waveform loop count
     *
     * Configures how many times a waveform repeats before advancing.
     *
     * @param[in] wav Wave slot (0-7)
     * @param[in] loop Loop count (0-15, where 15 = infinite)
     *
     * @par Example:
     * @code
     * haptic.setWaveLoop(0, 15);  // Infinite loop for slot 0
     * haptic.setWaveLoop(1, 0);  // No loop for slot 1
     * @endcode
     */
    void setWaveLoop(uint8_t wav, uint8_t loop);

    /**
     * @brief Set playback mode
     *
     * @param[in] mode Playback mode (see PlayMode enum)
     */
    void setPlayMode(PlayMode mode);

    /**
     * @brief Set LRA frequency trim value
     *
     * Used after F0 calibration to optimize vibration.
     *
     * @param[in] val Trim value from calibration (typically 0x00-0x3F)
     */
    void setTrimLRA(uint8_t val);

    //=========================================================================
    // RTP (Real-Time Playback) Mode
    //=========================================================================

    /**
     * @brief Set RTP waveform data
     *
     * Loads waveform data for RTP mode playback.
     *
     * @param[in] data Pointer to waveform data buffer
     * @param[in] len  Length of data in bytes
     */
    void setRtpData(uint8_t *data, uint32_t len);

    /**
     * @brief Play RTP (Real-Time Playback) waveform
     *
     * Plays a custom waveform in RTP mode.
     *
     * @param[in] data Waveform data buffer
     * @param[in] len  Data length in bytes
     * @param[in] gain Playback gain (0x00-0xFF)
     * @return true if successful
     *
     * @par Example:
     * @code
     * uint8_t sineWave[] = {0x00, 0x40, 0x80, 0xC0, 0xFF, ...};
     * haptic.playRtp(sineWave, sizeof(sineWave), 0x80);
     * delay(1000);
     * haptic.stopVibration();
     * @endcode
     */
    bool playRtp(const uint8_t *data, uint32_t len, uint8_t gain = 0x80);

    //=========================================================================
    // Calibration Functions
    //=========================================================================

    /**
     * @brief Load RAM waveform data
     *
     * Loads pre-compiled waveform data into the haptic chip's RAM.
     *
     * @param[in] data Pointer to RAM data array
     * @param[in] len  Length of RAM data
     * @return true if successful
     */
    bool loadRamData(const uint8_t *data, uint32_t len);

    //=========================================================================
    // Status & Information Getters
    //=========================================================================

    /**
     * @brief Get number of loaded waveforms
     * @return Waveform count
     */
    uint8_t getRamNum() const
    {
        return _ramNum;
    }

    /**
     * @brief Check if RAM is initialized
     * @return true if RAM data is loaded
     */
    bool isRamInit() const
    {
        return _ramInit;
    }

    /**
     * @brief Get current voltage reading
     * @return Voltage in millivolts
     */
    uint32_t getVbatValue() const
    {
        return _vbat;
    }

    /**
     * @brief Get current play mode
     * @return PlayMode enum value
     */
    PlayMode getPlayMode() const
    {
        return _playMode;
    }

    /**
     * @brief Get F0 calibration data
     * @return Calibration value
     */
    int8_t getF0CaliData() const
    {
        return _f0_cali_data;
    }

    /**
     * @brief Get LRA resistance
     * @return LRA value
     */
    uint32_t getLraValue() const
    {
        return _lra;
    }

    //=========================================================================
    // Parameter Setters (for advanced configuration)
    //=========================================================================

    /**
     * @brief Set expected F0 (resonant frequency)
     * @param[in] val F0 in Hz (default: 1700)
     */
    void setF0Pre(uint32_t val)
    {
        _f0_pre = val;
    }
    uint32_t getF0Pre() const
    {
        return _f0_pre;
    }

    /**
     * @brief Set LRA rated voltage
     * @param[in] val Voltage in mV (default: 1000)
     */
    void setLraVrms(uint32_t val)
    {
        _lra_vrms = val;
    }
    uint32_t getLraVrms() const
    {
        return _lra_vrms;
    }

    /**
     * @brief Set continuous mode break time
     * @param[in] val Break time value (0x00-0xFF)
     */
    void setContBrkTime(uint8_t val)
    {
        _cont_brk_time = val;
    }
    uint8_t getContBrkTime() const
    {
        return _cont_brk_time;
    }

    /**
     * @brief Set continuous mode break gain
     * @param[in] val Break gain value (0x00-0xFF)
     */
    void setContBrkGain(uint8_t val)
    {
        _cont_brk_gain = val;
    }
    uint8_t getContBrkGain() const
    {
        return _cont_brk_gain;
    }

    /**
     * @brief Set continuous mode drive level 1
     * @param[in] val Drive level (0x00-0x7F)
     */
    void setContDrv1Lvl(uint8_t val)
    {
        _cont_drv1_lvl = val;
    }
    uint8_t getContDrv1Lvl() const
    {
        return _cont_drv1_lvl;
    }

    /**
     * @brief Set continuous mode drive time 1
     * @param[in] val Drive time value (0x00-0xFF)
     */
    void setContDrv1Time(uint8_t val)
    {
        _cont_drv1_time = val;
    }
    uint8_t getContDrv1Time() const
    {
        return _cont_drv1_time;
    }

    /**
     * @brief Set continuous mode drive time 2
     * @param[in] val Drive time value (0x00-0xFF)
     */
    void setContDrv2Time(uint8_t val)
    {
        _cont_drv2_time = val;
    }
    uint8_t getContDrv2Time() const
    {
        return _cont_drv2_time;
    }

    /**
     * @brief Set continuous mode track margin
     * @param[in] val Margin value (0x00-0xFF)
     */
    void setContTrackMargin(uint8_t val)
    {
        _cont_track_margin = val;
    }
    uint8_t getContTrackMargin() const
    {
        return _cont_track_margin;
    }

    /**
     * @brief Set D2S gain (digital to simulation gain)
     * @param[in] val Gain value (0x00-0x07)
     */
    void setD2sGain(uint8_t val)
    {
        _d2s_gain = val;
    }
    uint8_t getD2sGain() const
    {
        return _d2s_gain;
    }

    /**
     * @brief Set protection time
     * @param[in] val Time value
     */
    void setPwmcfg4Prtime(uint8_t val)
    {
        _pwmcfg4_prtime = val;
    }
    uint8_t getPwmcfg4Prtime() const
    {
        return _pwmcfg4_prtime;
    }

    /**
     * @brief Set protection level
     * @param[in] val Level value
     */
    void setPwmcfg3Prlvl(uint8_t val)
    {
        _pwmcfg3_prlvl = val;
    }
    uint8_t getPwmcfg3Prlvl() const
    {
        return _pwmcfg3_prlvl;
    }

    /**
     * @brief Enable/disable auto break
     * @param[in] val true = enabled, false = disabled
     */
    void setAutoBrkEnabled(bool val)
    {
        _auto_brk_enabled = val;
    }
    bool getAutoBrkEnabled() const
    {
        return _auto_brk_enabled;
    }

    /**
     * @brief Set F0 calibration tolerance percentage
     * @param[in] val Percentage (default: 7)
     */
    void setF0CaliPercent(uint8_t val)
    {
        _f0_cali_percent = val;
    }
    uint8_t getF0CaliPercent() const
    {
        return _f0_cali_percent;
    }

    /**
     * @brief Check if vibration is currently active (alias for isPlaying)
     * @return true if vibration is running, false otherwise
     */
    bool isVibrating() const
    {
        return isPlaying();
    }

private:
    // Internal helper functions
    bool waitRtpGo(uint32_t timeout_ms);

    /**
    * @brief Print register values for debugging
    *
    * Outputs current register contents to Serial.
    * Useful for troubleshooting.
    */
    void printRegs();

    // RAM initialization
    void ramInit(bool enable);
    void playStop();
    void reset();
    bool checkQualify();
    bool offsetCalibration();
    int selectD2sGain(uint8_t reg);
    bool measureF0();
    bool measureVbat();
    void getLraResistance();
    void setSramSize(SramSize size);
    void setFifoAddr();
    void setBaseAddr();
    void setRamAddr();
    bool getBaseAddr(const uint8_t *data, uint32_t len);
    void setRepeatSeq(uint8_t seq);
    void autoBrkConfig(bool enable);
    void setPwm(PwmMode mode);
    void playMode(PlayMode mode);
    void config();
    void protectConfig(uint8_t prtime, uint8_t prlvl);
    void miscParaInit();
    void trigInit();
    void contConfig();
    void interruptSetup();
    void vbatModeConfig(uint8_t flag);
    void calculateCaliData();
    bool parseChipID();
    void hapticInit();
    void f0Calibration();
    bool judgeF0WithinRange();
    uint8_t getGlobalState() const;
    bool waitStandby(uint32_t timeout_ms = 100);
    int getIrqState();
    void irqClear();
    uint8_t rtpGetFifoAfs();
    void setRtpAei(bool enable);
    void cancelVibration();
    void ram_vbat_comp(bool flag);

    // Calculation formulas
    inline uint8_t drv2_lvl_formula(uint32_t f0, uint32_t vrms)
    {
        uint32_t base = (f0 < 1800) ? 1809920 : 1990912;
        return static_cast<uint8_t>((base / 1000 * vrms) / 61000);
    }

    bool initImpl(uint8_t param);

    void afterInitSuccess(uint8_t param) override
    {
        setReady(true);
    }

    inline uint32_t vbat_formula(uint32_t value)
    {
        return (6100 * (value) / 1024);
    }

    inline uint32_t f0_formula(uint32_t value)
    {
        return (384000 * 10 / (value));
    }

    inline uint32_t rl_formula(uint32_t value, uint32_t gain)
    {
        return (((value) * 678 * 100) / (1024 * (gain)));
    }

    inline int os_formula(int value, int d2s_gain)
    {
        return (2440 * (value - 512) / (1024 * (d2s_gain + 1)));
    }

    // Member variables
    int _rstPin;
    int _intPin;
    uint8_t _chipID;
    PlayMode _playMode;
    uint8_t _gain;
    uint32_t _f0;
    uint32_t _vbat;
    uint32_t _lra;
    bool _ramInit;
    uint8_t _cont_drv2_lvl;
    int8_t _f0_cali_data;
    uint32_t _f0_pre;
    uint16_t _ramBaseAddr;
    uint8_t _ramNum;
    uint32_t _lra_vrms;
    uint8_t _cont_brk_time;
    uint8_t _cont_brk_gain;
    uint8_t _cont_drv1_lvl;
    uint8_t _cont_drv1_time;
    uint8_t _cont_drv2_time;
    uint8_t _cont_track_margin;
    uint8_t _d2s_gain;
    uint8_t _pwmcfg4_prtime;
    uint8_t _pwmcfg3_prlvl;
    bool _auto_brk_enabled;
    uint8_t _f0_cali_percent;
    uint32_t _duration;
    uint32_t _vibrationStartTime;
    bool _isCalibrated;
};

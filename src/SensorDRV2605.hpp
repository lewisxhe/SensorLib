/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
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
 * @file      SensorDRV2605.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-03
 * @note      Adapted from https://github.com/adafruit/Adafruit_DRV2605_Library
 */
#pragma once

#include "platform/comm/I2CDeviceNoHal.hpp"
#include "sensor/HapticBase.hpp"

/**
 * @brief Default I2C address for DRV2605/DRV2604 haptic driver.
 */
static const uint8_t DRV2605_SLAVE_ADDRESS = 0x5A;

/**
 * @brief Driver for the TI DRV2605/DRV2604 haptic motor controller.
 *
 * This is a lightweight wrapper around the DRV2605 register interface.
 * It supports:
 * - Selecting the ROM waveform library
 * - Programming waveform sequence slots
 * - Starting/stopping playback
 * - Selecting operating modes (internal trigger, RTP, audio-to-vibe, etc.)
 * - Switching between ERM and LRA actuator type
 *
 * The init routine follows the Adafruit library defaults:
 * - exits standby
 * - clears RTP
 * - sets a default click waveform in slot 0 and terminates in slot 1
 * - enables ERM open-loop
 *
 * @warning You must call begin() successfully before using any other API.
 */
class SensorDRV2605 : public HapticBase, public I2CDeviceNoHal
{
public:
    /**
     * @brief Construct an uninitialized driver instance.
     */
    SensorDRV2605() = default;

    /**
     * @brief Destructor. Deinitializes communication backend if created.
     */
    ~SensorDRV2605() = default;

    /**
     * @brief Set a waveform entry in the playback sequence table.
     *
     * Playback starts at slot 0 and continues sequentially until a value of 0 is
     * encountered, or slot 7 is reached.
     *
     * @param slot Sequence slot index (0..7).
     * @param w    Waveform index in the device ROM library.
     *
     * @note Refer to the DRV2605 datasheet ROM library table for waveform IDs.
     */
    void setWaveform(uint8_t slot, uint8_t w)
    {
        writeReg((uint8_t)(DRV2605_REG_WAVESEQ1 + slot), w);
    }

    /**
     * @brief Select the ROM waveform library.
     *
     * @param lib Library selection:
     *            - 0: Empty
     *            - 1..5: ERM libraries
     *            - 6: LRA library
     *
     * @note See the DRV2605 datasheet for details.
     */
    void selectLibrary(uint8_t lib)
    {
        writeReg(DRV2605_REG_LIBRARY, lib);
    }

    /**
     * @brief Start playback (GO=1).
     */
    bool run() override
    {
        return writeReg(DRV2605_REG_GO, 1) == SENSOR_OK;
    }

    /**
     * @brief Stop playback (GO=0).
     */
    bool stop() override
    {
        return writeReg(DRV2605_REG_GO, (uint8_t)0) == SENSOR_OK;
    }

    /**
     * @brief  Set the haptic mode.
     * @note   This function configures the haptic driver mode.
     * @param  mode: Mode value (MODE_* constants).
     * @retval None
     */
    bool setMode(HapticMode mode) override
    {
        uint8_t value = 0;
        switch (mode) {
        case HapticMode::INTERNAL_TRIGGER:
            value = 0x00;
            break;
        case HapticMode::EXT_TRIGGER_EDGE:
            value = 0x01;
            break;
        case HapticMode::EXT_TRIGGER_LEVEL:
            value = 0x02;
            break;
        case HapticMode::PWM_ANALOG:
            value = 0x03;
            break;
        case HapticMode::AUDIO_TO_VIBE:
            value = 0x04;
            break;
        case HapticMode::REAL_TIME_PLAYBACK:
            value = 0x05;
            break;
        case HapticMode::DIAGNOSTICS:
            value = 0x06;
            break;
        case HapticMode::AUTO_CALIBRATE:
            value = 0x07;
            break;
        }
        return writeReg(DRV2605_REG_MODE, value) == SENSOR_OK;
    }

    /** @brief Get the current operating mode.
     *  @return Current operating mode (HapticMode).
     */
    HapticMode getMode() const override
    {
        int value = readReg(DRV2605_REG_MODE);
        if (value < 0) {
            return HapticMode::INTERNAL_TRIGGER; // default/fallback
        }
        return static_cast<HapticMode>(value & 0x07);
    }

    /**
     * @brief Set the RTP (real-time playback) drive value.
     *
     * @param rtp 8-bit RTP drive value.
     *
     * @note Only meaningful when MODE_REALTIME is selected.
     */
    bool setRealtimeValue(uint8_t rtp) override
    {
        return writeReg(DRV2605_REG_RTPIN, rtp) == SENSOR_OK;
    }

    /**
     * @brief  Set the actuator type.
     * @note   This function configures the haptic actuator type (ERM or LRA).
     * @param  type: Actuator type to set.
     * @retval True if successful, false otherwise.
     */
    bool setActuatorType(HapticActuatorType type) override
    {
        if (type == HapticActuatorType::ERM) {
            // Set ERM library
            writeReg(DRV2605_REG_LIBRARY, 1);
            updateBits(DRV2605_REG_FEEDBACK, 0x80, 0x00);
        } else {
            // Set LRA library
            writeReg(DRV2605_REG_LIBRARY, 6);
            updateBits(DRV2605_REG_FEEDBACK, 0x80, 0x80);
        }
        return true;
    }

    /**
     * @brief Get the current actuator type.
     *  @return Current actuator type (ERM or LRA).
     */
    HapticActuatorType getActuatorType() const override
    {
        int value = readReg(DRV2605_REG_FEEDBACK);
        if (value < 0) {
            return HapticActuatorType::ERM; // default/fallback
        }
        return (value & 0x80) ? HapticActuatorType::LRA : HapticActuatorType::ERM;
    }

    /**
     *  @brief Play a haptic effect.
     *  @param effect Effect ID to play.
     *  @return True if successful, false otherwise.
     */
    bool playEffect(HapticEffectId effect)  override
    {
        // For DRV2605, we can just set the effect ID in the first slot and run.
        setWaveform(0, static_cast<uint8_t>(effect));
        setWaveform(1, 0); // Terminate sequence after one effect
        return run();
    }

    /**
    * @brief Set the haptic mode.
    *
    * @param mode Mode value (MODE_* constants).
    *
    * Common modes:
    * - MODE_INTTRIG: internal trigger, call run() to start
    * - MODE_REALTIME: RTP mode (setRealtimeValue())
    * - MODE_AUDIOVIBE: audio-to-vibe
    * - MODE_AUTOCAL: auto calibration
    */
    void setMode(uint8_t mode) __attribute__((deprecated("use setMode(HapticMode::XXX) instead")))
    {
        writeReg(DRV2605_REG_MODE, mode);
    }

    /**
     * @brief Configure the device for ERM actuator mode.
     *
     * Clears N_ERM_LRA bit in FEEDBACK register.
     */
    void useERM() __attribute__((deprecated("use setActuatorType(HapticActuatorType::ERM) instead")))
    {
        setActuatorType(HapticActuatorType::ERM);
    }

    /**
     * @brief Configure the device for LRA actuator mode.
     *
     * Sets N_ERM_LRA bit in FEEDBACK register.
     */
    void useLRA() __attribute__((deprecated("use setActuatorType(HapticActuatorType::LRA) instead")))
    {
        setActuatorType(HapticActuatorType::LRA);
    }

private:
    /**
     * @brief Internal initialization routine.
     *
     * - Reads STATUS to determine chip ID (top bits)
     * - Verifies supported chip variants
     * - Applies a safe default configuration (similar to Adafruit library)
     *
     * @return true if device is supported and configured, false otherwise.
     */
    bool initImpl(uint8_t param) override
    {
        int chipID = readReg(DRV2605_REG_STATUS);
        if (chipID < 0) {
            return false;
        }

        // Chip ID is encoded in the top bits of STATUS.
        chipID >>= 5;

        switch (chipID) {
        case DRV2604_CHIP_ID:
        case DRV2605_CHIP_ID:
        case DRV2604L_CHIP_ID:
        case DRV2605L_CHIP_ID:
        case DRV2605X_CHIP_ID:
            break;
        default:
            log_e("ChipID:0x%x should be 0x03 or 0x04 or 0x06 or 0x07 or 0x05\n", chipID);
            return false;
        }

        // Exit standby (MODE=0)
        writeReg(DRV2605_REG_MODE, (uint8_t)0x00);

        // Disable RTP by default
        writeReg(DRV2605_REG_RTPIN, (uint8_t)0x00);

        // Default waveform sequence: strong click then end
        writeReg(DRV2605_REG_WAVESEQ1, (uint8_t)1);
        writeReg(DRV2605_REG_WAVESEQ2, (uint8_t)0);

        // No overdrive by default
        writeReg(DRV2605_REG_OVERDRIVE, (uint8_t)0);

        // Default sustain/break/audio settings (matching original/Adafruit defaults)
        writeReg(DRV2605_REG_SUSTAINPOS, (uint8_t)0);
        writeReg(DRV2605_REG_SUSTAINNEG, (uint8_t)0);
        writeReg(DRV2605_REG_BREAK, (uint8_t)0);
        writeReg(DRV2605_REG_AUDIOMAX, (uint8_t)0x64);

        // Default: ERM open loop
        // - clear N_ERM_LRA (ERM selected)
        int value = readReg(DRV2605_REG_FEEDBACK);
        if (value == -1) {
            return false;
        }
        writeReg(DRV2605_REG_FEEDBACK,
                 (uint8_t)(value & 0x7F));

        // - set ERM_OPEN_LOOP (CONTROL3 bit 5)
        int control3 = readReg(DRV2605_REG_CONTROL3);
        if (control3 == -1) {
            return false;
        }
        writeReg(DRV2605_REG_CONTROL3, (uint8_t)(control3 | 0x20));

        return true;
    }

protected:
    // -----------------------
    // Chip IDs (STATUS[7:5])
    // -----------------------

    /// DRV2604 (contains RAM, does not contain licensed ROM library)
    static constexpr uint8_t DRV2604_CHIP_ID = 0x04;
    /// DRV2605 (contains licensed ROM library, does not contain RAM)
    static constexpr uint8_t DRV2605_CHIP_ID = 0x03;
    /// DRV2604L (low-voltage version of DRV2604)
    static constexpr uint8_t DRV2604L_CHIP_ID = 0x06;
    /// DRV2605L (low-voltage version of DRV2605)
    static constexpr uint8_t DRV2605L_CHIP_ID = 0x07;
    /**
     * @brief Possible counterfeit DRV2605 variant reported by community.
     * @note See: https://github.com/lewisxhe/SensorLib/issues/32
     */
    static constexpr uint8_t DRV2605X_CHIP_ID = 0x05;

    // -----------------------
    // Register addresses
    // -----------------------

    static constexpr uint8_t DRV2605_REG_STATUS = 0x00;      //!< Status register
    static constexpr uint8_t DRV2605_REG_MODE = 0x01;        //!< Mode register
    static constexpr uint8_t DRV2605_REG_RTPIN = 0x02;       //!< Real-time playback input register
    static constexpr uint8_t DRV2605_REG_LIBRARY = 0x03;     //!< Waveform library selection register
    static constexpr uint8_t DRV2605_REG_WAVESEQ1 = 0x04;    //!< Waveform sequence register 1
    static constexpr uint8_t DRV2605_REG_WAVESEQ2 = 0x05;    //!< Waveform sequence register 2
    static constexpr uint8_t DRV2605_REG_WAVESEQ3 = 0x06;    //!< Waveform sequence register 3
    static constexpr uint8_t DRV2605_REG_WAVESEQ4 = 0x07;    //!< Waveform sequence register 4
    static constexpr uint8_t DRV2605_REG_WAVESEQ5 = 0x08;    //!< Waveform sequence register 5
    static constexpr uint8_t DRV2605_REG_WAVESEQ6 = 0x09;    //!< Waveform sequence register 6
    static constexpr uint8_t DRV2605_REG_WAVESEQ7 = 0x0A;    //!< Waveform sequence register 7
    static constexpr uint8_t DRV2605_REG_WAVESEQ8 = 0x0B;    //!< Waveform sequence register 8
    static constexpr uint8_t DRV2605_REG_GO = 0x0C;          //!< GO register
    static constexpr uint8_t DRV2605_REG_OVERDRIVE = 0x0D;   //!< Overdrive time offset register
    static constexpr uint8_t DRV2605_REG_SUSTAINPOS = 0x0E;  //!< Sustain time offset (positive)
    static constexpr uint8_t DRV2605_REG_SUSTAINNEG = 0x0F;  //!< Sustain time offset (negative)
    static constexpr uint8_t DRV2605_REG_BREAK = 0x10;       //!< Brake time offset register
    static constexpr uint8_t DRV2605_REG_AUDIOCTRL = 0x11;   //!< Audio-to-vibe control register
    static constexpr uint8_t DRV2605_REG_AUDIOLVL = 0x12;    //!< Audio-to-vibe minimum input level register
    static constexpr uint8_t DRV2605_REG_AUDIOMAX = 0x13;    //!< Audio-to-vibe maximum input level register
    static constexpr uint8_t DRV2605_REG_AUDIOOUTMIN = 0x14; //!< Audio-to-vibe minimum output drive register
    static constexpr uint8_t DRV2605_REG_AUDIOOUTMAX = 0x15; //!< Audio-to-vibe maximum output drive register
    static constexpr uint8_t DRV2605_REG_RATEDV = 0x16;      //!< Rated voltage register
    static constexpr uint8_t DRV2605_REG_CLAMPV = 0x17;      //!< Overdrive clamp voltage register
    static constexpr uint8_t DRV2605_REG_AUTOCALCOMP = 0x18; //!< Auto-calibration compensation result register
    static constexpr uint8_t DRV2605_REG_AUTOCALEMP = 0x19;  //!< Auto-calibration back-EMF result register
    static constexpr uint8_t DRV2605_REG_FEEDBACK = 0x1A;    //!< Feedback control register
    static constexpr uint8_t DRV2605_REG_CONTROL1 = 0x1B;    //!< Control 1 register
    static constexpr uint8_t DRV2605_REG_CONTROL2 = 0x1C;    //!< Control 2 register
    static constexpr uint8_t DRV2605_REG_CONTROL3 = 0x1D;    //!< Control 3 register
    static constexpr uint8_t DRV2605_REG_CONTROL4 = 0x1E;    //!< Control 4 register
    static constexpr uint8_t DRV2605_REG_VBAT = 0x21;        //!< VBAT voltage-monitor register
    static constexpr uint8_t DRV2605_REG_LRARESON = 0x22;    //!< LRA resonance-period register
};

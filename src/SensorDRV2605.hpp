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

#include "SensorPlatform.hpp"

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
class SensorDRV2605
{
public:
    // -----------------------
    // Mode register values
    // -----------------------

    /** @brief Internal trigger mode. */
    static constexpr uint8_t MODE_INTTRIG = 0x00;
    /** @brief External edge trigger mode. */
    static constexpr uint8_t MODE_EXTTRIGEDGE = 0x01;
    /** @brief External level trigger mode. */
    static constexpr uint8_t MODE_EXTTRIGLVL = 0x02;
    /** @brief PWM/Analog input mode. */
    static constexpr uint8_t MODE_PWMANALOG = 0x03;
    /** @brief Audio-to-vibe mode. */
    static constexpr uint8_t MODE_AUDIOVIBE = 0x04;
    /** @brief Real-time playback (RTP) mode. */
    static constexpr uint8_t MODE_REALTIME = 0x05;
    /** @brief Diagnostics mode. */
    static constexpr uint8_t MODE_DIAGNOS = 0x06;
    /** @brief Auto-calibration mode. */
    static constexpr uint8_t MODE_AUTOCAL = 0x07;

    /**
     * @brief Construct an uninitialized driver instance.
     */
    SensorDRV2605() : comm(nullptr) {}

    /**
     * @brief Destructor. Deinitializes communication backend if created.
     */
    ~SensorDRV2605()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    /**
     * @brief Initialize the device using Arduino Wire (I2C).
     *
     * @param wire TwoWire instance (e.g. Wire).
     * @param sda  SDA pin (optional, -1 = default).
     * @param scl  SCL pin (optional, -1 = default).
     * @return true if chip ID matches a supported device, false otherwise.
     */
    bool begin(TwoWire &wire, int sda = -1, int scl = -1)
    {
        comm = std::make_unique<SensorCommI2C>(wire, DRV2605_SLAVE_ADDRESS, sda, scl);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialize the device using ESP-IDF legacy I2C API.
     *
     * @param port_num I2C port number.
     * @param sda      SDA pin (optional, -1 = default).
     * @param scl      SCL pin (optional, -1 = default).
     * @return true if chip ID matches a supported device, false otherwise.
     */
    bool begin(i2c_port_t port_num, int sda = -1, int scl = -1)
    {
        comm = std::make_unique<SensorCommI2C>(port_num, DRV2605_SLAVE_ADDRESS, sda, scl);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#else
    /**
     * @brief Initialize the device using ESP-IDF new I2C master bus handle.
     *
     * @param handle I2C master bus handle.
     * @return true if chip ID matches a supported device, false otherwise.
     */
    bool begin(i2c_master_bus_handle_t handle)
    {
        comm = std::make_unique<SensorCommI2C>(handle, DRV2605_SLAVE_ADDRESS);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }
#endif  // USEING_I2C_LEGACY
#endif  // ESP_PLATFORM

    /**
     * @brief Initialize the device using a custom transport callback.
     *
     * @param callback Custom callback used by SensorCommCustom.
     * @return true if chip ID matches a supported device, false otherwise.
     */
    bool begin(SensorCommCustom::CustomCallback callback)
    {
        comm = std::make_unique<SensorCommCustom>(callback, DRV2605_SLAVE_ADDRESS);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl();
    }

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
        comm->writeRegister((uint8_t)(DRV2605_REG_WAVESEQ1 + slot), w);
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
        comm->writeRegister(DRV2605_REG_LIBRARY, lib);
    }

    /**
     * @brief Start playback (GO=1).
     */
    void run()
    {
        comm->writeRegister(DRV2605_REG_GO, 1);
    }

    /**
     * @brief Stop playback (GO=0).
     */
    void stop()
    {
        comm->writeRegister(DRV2605_REG_GO, (uint8_t)0);
    }

    /**
     * @brief Set the device operating mode.
     *
     * @param mode Mode value (MODE_* constants).
     *
     * Common modes:
     * - MODE_INTTRIG: internal trigger, call run() to start
     * - MODE_REALTIME: RTP mode (setRealtimeValue())
     * - MODE_AUDIOVIBE: audio-to-vibe
     * - MODE_AUTOCAL: auto calibration
     */
    void setMode(uint8_t mode)
    {
        comm->writeRegister(DRV2605_REG_MODE, mode);
    }

    /**
     * @brief Set the RTP (real-time playback) drive value.
     *
     * @param rtp 8-bit RTP drive value.
     *
     * @note Only meaningful when MODE_REALTIME is selected.
     */
    void setRealtimeValue(uint8_t rtp)
    {
        comm->writeRegister(DRV2605_REG_RTPIN, rtp);
    }

    /**
     * @brief Configure the device for ERM actuator mode.
     *
     * Clears N_ERM_LRA bit in FEEDBACK register.
     */
    void useERM()
    {
        int value = comm->readRegister(DRV2605_REG_FEEDBACK);
        if (value == -1) {
            return;
        }
        comm->writeRegister(DRV2605_REG_FEEDBACK,
                            (uint8_t)(value & 0x7F));
    }

    /**
     * @brief Configure the device for LRA actuator mode.
     *
     * Sets N_ERM_LRA bit in FEEDBACK register.
     */
    void useLRA()
    {
        int value = comm->readRegister(DRV2605_REG_FEEDBACK);
        if (value == -1) {
            return;
        }
        comm->writeRegister(DRV2605_REG_FEEDBACK,
                            (uint8_t)(value | 0x80));
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
    bool initImpl()
    {
        int chipID = comm->readRegister(DRV2605_REG_STATUS);
        if (chipID < 0) {
            return false;
        }

        // Chip ID is encoded in the top bits of STATUS.
        chipID >>= 5;

        if (chipID != DRV2604_CHIP_ID &&
                chipID != DRV2605_CHIP_ID &&
                chipID != DRV2604L_CHIP_ID &&
                chipID != DRV2605L_CHIP_ID &&
                chipID != DRV2605X_CHIP_ID) {
            log_e("ChipID:0x%x should be 0x03 or 0x04 or 0x06 or 0x07 or 0x05\n", chipID);
            return false;
        }

        // Exit standby (MODE=0)
        comm->writeRegister(DRV2605_REG_MODE, (uint8_t)0x00);

        // Disable RTP by default
        comm->writeRegister(DRV2605_REG_RTPIN, (uint8_t)0x00);

        // Default waveform sequence: strong click then end
        comm->writeRegister(DRV2605_REG_WAVESEQ1, (uint8_t)1);
        comm->writeRegister(DRV2605_REG_WAVESEQ2, (uint8_t)0);

        // No overdrive by default
        comm->writeRegister(DRV2605_REG_OVERDRIVE, (uint8_t)0);

        // Default sustain/break/audio settings (matching original/Adafruit defaults)
        comm->writeRegister(DRV2605_REG_SUSTAINPOS, (uint8_t)0);
        comm->writeRegister(DRV2605_REG_SUSTAINNEG, (uint8_t)0);
        comm->writeRegister(DRV2605_REG_BREAK, (uint8_t)0);
        comm->writeRegister(DRV2605_REG_AUDIOMAX, (uint8_t)0x64);

        // Default: ERM open loop
        // - clear N_ERM_LRA (ERM selected)
        int value = comm->readRegister(DRV2605_REG_FEEDBACK);
        if (value == -1) {
            return false;
        }
        comm->writeRegister(DRV2605_REG_FEEDBACK,
                            (uint8_t)(value & 0x7F));

        // - set ERM_OPEN_LOOP (CONTROL3 bit 5)
        int control3 = comm->readRegister(DRV2605_REG_CONTROL3);
        if (control3 == -1) {
            return false;
        }
        comm->writeRegister(DRV2605_REG_CONTROL3,
                            (uint8_t)(control3 | 0x20));

        return true;
    }

protected:
    /// Communication backend (I2C or custom callback).
    std::unique_ptr<SensorCommBase> comm;

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

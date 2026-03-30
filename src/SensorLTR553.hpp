/**
 *
 * @license MIT License
 *
 * Copyright (c) 2023 lewis he
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
 * @file      SensorLTR553ALS.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-09
 *
 */
#pragma once

#include "platform/comm/I2CDeviceNoHal.hpp"

// Unique I2C device address
static constexpr uint8_t LTR553_SLAVE_ADDRESS = 0x23;

/**
 * @brief Driver for the LTR-553ALS-01 ambient light + proximity sensor (I2C).
 *
 * This class provides a lightweight register-level driver for:
 * - ALS (Ambient Light Sensing) configuration and reading
 * - PS (Proximity Sensing) configuration and reading
 * - Threshold window configuration
 * - Interrupt configuration (polarity + source selection)
 *
 * The register map and bit fields follow the LTR-553ALS-01 datasheet.
 *
 * @warning You must call begin() successfully before calling any other methods.
 */
class SensorLTR553 : public I2CDeviceNoHal
{
public:
    /**
     * @brief Interrupt pin active level.
     */
    enum IrqLevel {
        ALS_IRQ_ACTIVE_LOW,  //!< INT pin is active-low (default)
        ALS_IRQ_ACTIVE_HIGH  //!< INT pin is active-high
    };

    /**
     * @brief Interrupt trigger source.
     *
     * This selects which measurement(s) can assert the interrupt pin.
     */
    enum IrqMode {
        ALS_IRQ_DISABLE = 0,   //!< No measurement can trigger interrupt (default)
        ALS_IRQ_ONLY_PS = 1,   //!< Only PS measurement can trigger interrupt
        ALS_IRQ_ONLY_ALS = 2,  //!< Only ALS measurement can trigger interrupt
        ALS_IRQ_BOTH = 3,      //!< Both ALS and PS measurement can trigger interrupt
    };

    /**
     * @brief ALS gain selection.
     *
     * Note: Actual lux range depends on gain and integration time.
     */
    enum LightSensorGain {
        ALS_GAIN_1X  = 0x00,  //!< 1 lux to 64k lux (default)
        ALS_GAIN_2X  = 0x01,  //!< 0.5 lux to 32k lux
        ALS_GAIN_4X  = 0x02,  //!< 0.25 lux to 16k lux
        ALS_GAIN_8X  = 0x03,  //!< 0.125 lux to 8k lux
        ALS_GAIN_48X = 0x06,  //!< 0.02 lux to 1.3k lux
        ALS_GAIN_96X = 0x07,  //!< 0.01 lux to 600 lux
    };

    /**
     * @brief PS LED pulse modulation frequency (REG_PS_LED bits 7:5).
     *
     * Datasheet: 000=30kHz ... 111=100kHz.
     */
    enum PsLedPeriod {
        PS_LED_PLUSE_30KHZ,   //!< 30kHz
        PS_LED_PLUSE_40KHZ,   //!< 40kHz
        PS_LED_PLUSE_50KHZ,   //!< 50kHz
        PS_LED_PLUSE_60KHZ,   //!< 60kHz (default)
        PS_LED_PLUSE_70KHZ,   //!< 70kHz
        PS_LED_PLUSE_80KHZ,   //!< 80kHz
        PS_LED_PLUSE_90KHZ,   //!< 90kHz
        PS_LED_PLUSE_100KHZ,  //!< 100kHz
    };

    /**
     * @brief PS LED duty cycle (REG_PS_LED bits 4:3).
     *
     * Datasheet: 00=25%, 01=50%, 10=75%, 11=100%.
     */
    enum PsLedDuty {
        PS_LED_DUTY_25,   //!< 25%
        PS_LED_DUTY_50,   //!< 50%
        PS_LED_DUTY_75,   //!< 75%
        PS_LED_DUTY_100,  //!< 100% (default)
    };

    /**
     * @brief PS LED peak current selection (REG_PS_LED bits 2:0).
     *
     * @note The enum values are kept consistent with the original library design.
     *       Please refer to the datasheet current encoding table if you need strict mapping.
     */
    enum PsLedCurrent {
        PS_LED_CUR_5MA,    //!< 5mA
        PS_LED_CUR_10MA,   //!< 10mA
        PS_LED_CUR_20MA,   //!< 20mA
        PS_LED_CUR_50MA,   //!< 50mA
        PS_LED_CUR_100MA,  //!< 100mA
    };

    /**
     * @brief PS measurement rate (REG_PS_MEAS_RATE field).
     *
     * @note PS_MEAS_RATE_10MS uses value 8, matching the original implementation.
     */
    enum PsRate {
        PS_MEAS_RATE_50MS,
        PS_MEAS_RATE_70MS,
        PS_MEAS_RATE_100MS,
        PS_MEAS_RATE_200MS,
        PS_MEAS_RATE_500MS,
        PS_MEAS_RATE_1000MS,
        PS_MEAS_RATE_2000MS,
        PS_MEAS_RATE_10MS = 8,
    };

    /**
     * @brief ALS integration time (REG_ALS_MEAS_RATE bits 5:3).
     *
     * Datasheet mapping (bits 5:3):
     * - 000: 100ms (default)
     * - 001: 50ms
     * - 010: 200ms
     * - 011: 400ms
     * - 100: 150ms
     * - 101: 250ms
     * - 110: 300ms
     * - 111: 350ms
     */
    enum IntegrationTime {
        ALS_INTEGRATION_TIME_100MS,
        ALS_INTEGRATION_TIME_50MS,
        ALS_INTEGRATION_TIME_200MS,
        ALS_INTEGRATION_TIME_400MS,
        ALS_INTEGRATION_TIME_150MS,
        ALS_INTEGRATION_TIME_250MS,
        ALS_INTEGRATION_TIME_300MS,
        ALS_INTEGRATION_TIME_350MS,
    };

    /**
     * @brief ALS measurement repeat rate (REG_ALS_MEAS_RATE bits 2:0).
     *
     * Datasheet mapping (bits 2:0):
     * - 000: 50ms
     * - 001: 100ms
     * - 010: 200ms
     * - 011: 500ms (default)
     * - 100: 1000ms
     * - 110: 2000ms
     *
     * @note Some values may be reserved per datasheet.
     */
    enum MeasurementRate {
        ALS_MEASUREMENT_TIME_50MS,
        ALS_MEASUREMENT_TIME_100MS,
        ALS_MEASUREMENT_TIME_200MS,
        ALS_MEASUREMENT_TIME_500MS,
        ALS_MEASUREMENT_TIME_1000MS,
        ALS_MEASUREMENT_TIME_2000MS,
    };

    /**
     * @brief Construct an uninitialized driver instance.
     */
    SensorLTR553() = default;

    /**
     * @brief Destructor. Deinitializes the communication backend if created.
     */
    ~SensorLTR553() = default;

    /**
     * @brief Set interrupt pin polarity.
     *
     * @param level Desired active level.
     */
    void setIRQLevel(IrqLevel  level)
    {
        if (level) {
            setRegBit(REG_INTERRUPT, 2);
        } else {
            clrRegBit(REG_INTERRUPT, 2);
        }
    }

    /**
     * @brief Enable interrupts and select which measurement sources can trigger them.
     *
     * @param mode Interrupt source mode.
     */
    void enableIRQ(IrqMode mode)
    {
        // RESERVED:7:3
        // Interrupt Polarity : 2
        // Interrupt Mode : 1:0
        updateBits(REG_INTERRUPT, 0x03, mode);
    }

    /**
     * @brief Disable all interrupt sources (mode bits = 0).
     */
    void disableIRQ()
    {
        updateBits(REG_INTERRUPT, 0x03, 0x00);
    }

    // -----------------------
    // ALS (Ambient Light)
    // -----------------------

    /**
     * @brief Check PS data ready flag.
     *
     * @return true if PS data is ready (REG_ALS_PS_STATUS bit0), false otherwise.
     *
     * @note Function name follows original implementation.
     */
    bool psAvailable()
    {
        return getRegBit(REG_ALS_PS_STATUS, 0);
    }

    /**
     * @brief Set ALS threshold window.
     *
     * If ALS measurement is outside [low, high], interrupt may be asserted depending on IRQ settings.
     *
     * @param low  Low threshold.
     * @param high High threshold.
     */
    void setLightSensorThreshold(uint16_t low, uint16_t high)
    {
        uint8_t buffer[4] = {
            lowByte(high), highByte(high),
            lowByte(low),  highByte(low)
        };
        writeRegBuff(REG_ALS_THRES_UP_0, buffer, 4);
    }

    /**
     * @brief Configure ALS interrupt persistence count.
     *
     * Datasheet (REG_INTERRUPT_PERSIST 0x9E):
     * - ALS Persist bits [3:0]
     *
     * @param count Number of consecutive ALS out-of-range events before asserting interrupt.
     *
     * @warning Passing 0 will underflow due to (count - 1). Keep as-is to match original behavior.
     */
    void setLightSensorPersists(uint8_t count)
    {
        updateBits(REG_INTERRUPT_PERSIST, 0x0F, count - 1);
    }

    /**
     * @brief Configure ALS integration time and measurement repeat rate.
     *
     * Datasheet (REG_ALS_MEAS_RATE 0x85):
     * - ALS integration time bits [5:3]
     * - ALS measurement repeat rate bits [2:0]
     *
     * @param alsIntegrationTime ALS integration time.
     * @param alsMeasurementRate ALS measurement repeat rate.
     */
    void setLightSensorRate(IntegrationTime alsIntegrationTime, MeasurementRate alsMeasurementRate)
    {
        uint8_t value = (uint8_t)(((alsIntegrationTime & 0x07) << 3) | (alsMeasurementRate & 0x07));
        writeReg(REG_ALS_MEAS_RATE, value);
    }

    /**
     * @brief Enable ALS measurement.
     */
    void enableLightSensor()
    {
        setRegBit(REG_ALS_CONTR, 0);
    }

    /**
     * @brief Disable ALS measurement.
     */
    void disableLightSensor()
    {
        clrRegBit(REG_ALS_CONTR, 0);
    }

    /**
     * @brief Set ALS gain.
     *
     * @param gain Gain selection.
     */
    void setLightSensorGain(LightSensorGain gain)
    {
        updateBits(REG_ALS_CONTR, 0x1C, gain << 2);
    }

    /**
     * @brief Read ALS raw data from channel 0 or 1.
     *
     * @param ch Channel index (0 or 1). Any non-1 value will be treated as channel 0.
     * @return Raw 16-bit value on success, -1 on invalid data or I2C error.
     */
    int getLightSensor(uint8_t ch)
    {
        uint8_t buffer[2] = {0};

        // Check ALS Data is Valid (datasheet bit definition should be checked if needed)
        if (getRegBit(REG_ALS_PS_STATUS, 7) != false) {
            return -1;
        }

        int val = readRegBuff(ch == 1 ? REG_ALS_DATA_CH1_0 : REG_ALS_DATA_CH0_0, buffer, 2);
        if (val == -1) {
            return -1;
        }

        return (int)buffer[0] | ((int)buffer[1] << 8);
    }

    // -----------------------
    // PS (Proximity)
    // -----------------------

    /**
     * @brief Configure PS interrupt persistence count.
     *
     * Datasheet (REG_INTERRUPT_PERSIST 0x9E):
     * - PS Persist bits [7:4]
     *
     * @param count Number of consecutive PS out-of-range events before asserting interrupt.
     *              count=0 maps to register value 0 (every PS out of range).
     */
    void setProximityPersists(uint8_t count)
    {
        uint8_t val = (count == 0) ? 0 : (count - 1);
        if (val > 0x0F) {
            val = 0x0F;
        }
        updateBits(REG_INTERRUPT_PERSIST, 0xF0, val << 4);
    }

    /**
     * @brief Set PS threshold window.
     *
     * @param low  Low threshold (12-bit effective).
     * @param high High threshold (12-bit effective).
     */
    void setProximityThreshold(uint16_t low, uint16_t high)
    {
        writeReg(REG_PS_THRES_UP_0, lowByte(high));
        writeReg(REG_PS_THRES_UP_1, lowByte(high >> 8) & 0x0F);
        writeReg(REG_PS_THRES_LOW_0, lowByte(low));
        writeReg(REG_PS_THRES_LOW_1, lowByte(low >> 8) & 0x0F);
    }

    /**
     * @brief Set PS measurement rate.
     *
     * @param rate Proximity measurement rate selection.
     */
    void setProximityRate(PsRate rate)
    {
        updateBits(REG_PS_MEAS_RATE, 0x0F, rate & 0x0F);
    }

    /**
     * @brief Enable proximity measurement.
     */
    void enableProximity()
    {
        updateBits(REG_PS_CONTR, 0x03, 0x03);
    }

    /**
     * @brief Disable proximity measurement.
     */
    void disableProximity()
    {
        updateBits(REG_PS_CONTR, 0x03, 0x00);
    }

    /**
     * @brief Enable PS indicator.
     */
    void enablePsIndicator()
    {
        setRegBit(REG_PS_CONTR, 5);
    }

    /**
     * @brief Disable PS indicator.
     */
    void disablePsIndicator()
    {
        clrRegBit(REG_PS_CONTR, 5);
    }

    /**
     * @brief Read proximity raw data.
     *
     * Proximity data is 11-bit:
     * - Low 8 bits: REG_PS_DATA_0
     * - High 3 bits: REG_PS_DATA_1 bits [2:0]
     *
     * @param saturated Optional pointer; will be set true if saturation flag is asserted (bit7).
     * @return Proximity value on success, -1 on I2C error.
     */
    int getProximity(bool *saturated = NULL )
    {
        uint8_t buffer[2] = {0};
        int val = readRegBuff(REG_PS_DATA_0, buffer, 2);
        if (val == -1) {
            return -1;
        }
        if (saturated) {
            *saturated = (buffer[1] & 0x80) == 0x80;
        }
        return (int)buffer[0] | ((int)(buffer[1] & 0x07) << 8);
    }

    // -----------------------
    // PS LED configuration
    // -----------------------

    /**
     * @brief Set PS LED pulse modulation frequency (REG_PS_LED bits 7:5).
     *
     * @param period LED pulse modulation frequency selection.
     */
    void setPsLedPulsePeriod(PsLedPeriod period)
    {
        updateBits(REG_PS_LED, 0xE0, static_cast<uint8_t>(period) << 5);
    }

    /**
     * @brief Set PS LED duty cycle (REG_PS_LED bits 4:3).
     *
     * @param duty LED duty cycle selection.
     */
    void setPsLedDutyCycle(PsLedDuty duty)
    {
        updateBits(REG_PS_LED, 0x18, static_cast<uint8_t>(duty) << 3);
    }

    /**
     * @brief Set PS LED peak current (REG_PS_LED bits 2:0).
     *
     * @param cur LED peak current selection.
     */
    void setPsLedCurrent(PsLedCurrent cur)
    {
        updateBits(REG_PS_LED, 0x07, static_cast<uint8_t>(cur));
    }

    /**
     * @brief Set number of PS LED pulses (REG_PS_N_PULSES bits 3:0).
     *
     * @param pulesNum Number of pulses (0..15).
     */
    void setPsLedPulses(uint8_t pulesNum)
    {
        updateBits(REG_PS_N_PULSES, 0x0F, pulesNum & 0x0F);
    }

    // -----------------------
    // Identification / reset
    // -----------------------

    /**
     * @brief Get Part ID (REG_PART_ID high nibble).
     *
     * @return Part ID on success, -1 on error.
     */
    int getPartID()
    {
        int val = readReg(REG_PART_ID);
        if (val == -1) {
            return -1;
        }
        return (val >> 4) & 0x0F;
    }

    /**
     * @brief Get Revision ID (REG_PART_ID low nibble).
     *
     * @return Revision ID on success, -1 on error.
     */
    int getRevisionID()
    {
        int val = readReg(REG_PART_ID);
        if (val == -1) {
            return -1;
        }
        return (val) & 0x0F;
    }

    /**
     * @brief Read Manufacturer ID (REG_MANUFAC_ID).
     *
     * Expected value is LTR553_DEFAULT_MAN_ID.
     *
     * @return Manufacturer ID (or -1 on I2C error depending on backend).
     */
    int getManufacturerID()
    {
        // Manufacturer ID (0x05H)
        return readReg(REG_MANUFAC_ID);
    }

    /**
     * @brief Software reset (REG_ALS_CONTR bit1).
     */
    void reset()
    {
        setRegBit(REG_ALS_CONTR, 1);
    }

private:
    /**
     * @brief Internal initialization.
     *
     * - Configure I2C params
     * - Reset device
     * - Verify manufacturer ID
     */
    bool initImpl(uint8_t param)
    {
        setAck(false);
        reset();
        return getManufacturerID() == LTR553_DEFAULT_MAN_ID;
    }

protected:
    // Register map
    static constexpr uint8_t REG_ALS_CONTR = 0x80;
    static constexpr uint8_t REG_PS_CONTR = 0x81;
    static constexpr uint8_t REG_PS_LED = 0x82;
    static constexpr uint8_t REG_PS_N_PULSES = 0x83;
    static constexpr uint8_t REG_PS_MEAS_RATE = 0x84;
    static constexpr uint8_t REG_ALS_MEAS_RATE = 0x85;
    static constexpr uint8_t REG_PART_ID = 0x86;
    static constexpr uint8_t REG_MANUFAC_ID = 0x87;
    static constexpr uint8_t REG_ALS_DATA_CH1_0 = 0x88;
    static constexpr uint8_t REG_ALS_DATA_CH1_1 = 0x89;
    static constexpr uint8_t REG_ALS_DATA_CH0_0 = 0x8A;
    static constexpr uint8_t REG_ALS_DATA_CH0_1 = 0x8B;
    static constexpr uint8_t REG_ALS_PS_STATUS = 0x8C;
    static constexpr uint8_t REG_PS_DATA_0 = 0x8D;
    static constexpr uint8_t REG_PS_DATA_1 = 0x8E;
    static constexpr uint8_t REG_INTERRUPT = 0x8F;
    static constexpr uint8_t REG_PS_THRES_UP_0 = 0x90;
    static constexpr uint8_t REG_PS_THRES_UP_1 = 0x91;
    static constexpr uint8_t REG_PS_THRES_LOW_0 = 0x92;
    static constexpr uint8_t REG_PS_THRES_LOW_1 = 0x93;
    static constexpr uint8_t REG_PS_OFFSET_1 = 0x94;
    static constexpr uint8_t REG_PS_OFFSET_0 = 0x95;
    static constexpr uint8_t REG_ALS_THRES_UP_0 = 0x97;
    static constexpr uint8_t REG_ALS_THRES_UP_1 = 0x98;
    static constexpr uint8_t REG_ALS_THRES_LOW_0 = 0x99;
    static constexpr uint8_t REG_ALS_THRES_LOW_1 = 0x9A;
    static constexpr uint8_t REG_INTERRUPT_PERSIST = 0x9E;

    /// Default manufacturer ID.
    static constexpr uint8_t LTR553_DEFAULT_MAN_ID = 0x05;
};

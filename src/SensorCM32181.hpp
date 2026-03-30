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
 * @file      SensorCM32181.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-14
 *
 */
#pragma once

#include "platform/comm/I2CDeviceNoHal.hpp"

/**
 * @brief I2C addresses for CM32181.
 *
 * The address is determined by the ADDR pin configuration.
 */
/**
 * @brief Primary I2C address (0x10).
 */
static constexpr uint8_t CM32181_ADDR_PRIMARY = 0x10;

/**
 * @brief Secondary I2C address (0x48), determined by ADDR pin.
 */
static constexpr uint8_t CM32181_ADDR_SECONDARY = 0x48;

/**
 * @brief Backward-compatible default address definition (0x10).
 */
static constexpr uint8_t CM32181_SLAVE_ADDRESS = 0x10;

/**
 * @brief High Sensitivity I2C Ambient Light Sensor (CM32181) driver.
 *
 * This driver provides:
 * - ALS configuration (sensitivity / integration time)
 * - Power on/off
 * - Interrupt threshold window configuration and interrupt enable/disable
 * - IRQ status read (also clears IRQ per datasheet behavior)
 * - Raw ALS read and approximate lux conversion
 *
 * @note Lux conversion uses a fixed calibration factor, because the datasheet/manual
 *       does not provide a readable calibration register.
 *
 * @warning Always call begin() and ensure it returns true before using other APIs.
 */
class SensorCM32181 : public I2CDeviceNoHal
{
public:
    /**
     * @brief ALS sensitivity selection (REG_ALS_CONF bits 12:11).
     *
     * Datasheet mapping:
     * - 00: x1
     * - 01: x2
     * - 10: x1/8
     * - 11: x1/4
     */
    enum Sampling : uint8_t {
        SAMPLING_X1 = 0,    //!< ALS Sensitivity x1
        SAMPLING_X2 = 1,    //!< ALS Sensitivity x2
        SAMPLING_X1_8 = 2,  //!< ALS Sensitivity x(1/8)
        SAMPLING_X1_4 = 3,  //!< ALS Sensitivity x(1/4)
    };

    /**
     * @brief ALS integration time (REG_ALS_CONF bits 9:6).
     *
     * Datasheet mapping (bits 9:6):
     * - 1100: 25ms  (0x0C)
     * - 1000: 50ms  (0x08)
     * - 0000: 100ms (0x00)
     * - 0001: 200ms (0x01)
     * - 0010: 400ms (0x02)
     * - 0011: 800ms (0x03)
     */
    enum IntegrationTime : uint8_t {
        INTEGRATION_TIME_25MS  = 0x0C, //!< 25ms
        INTEGRATION_TIME_50MS  = 0x08, //!< 25ms
        INTEGRATION_TIME_100MS = 0x00, //!< 100ms
        INTEGRATION_TIME_200MS = 0x01, //!< 200ms
        INTEGRATION_TIME_400MS = 0x02, //!< 400ms
        INTEGRATION_TIME_800MS = 0x03, //!< 800ms
    };

    /**
     * @brief Power saving mode (REG_ALS_PSM bits 2:1).
     *
     * Datasheet mapping:
     * - 00: Mode1
     * - 01: Mode2
     * - 10: Mode3
     * - 11: Mode4
     */
    enum PowerSaveMode : uint8_t {
        PowerSaveMode1 = 0,
        PowerSaveMode2 = 1,
        PowerSaveMode3 = 2,
        PowerSaveMode4 = 3
    };

    /**
     * @brief Interrupt event type reported by getIrqStatus().
     */
    enum InterruptEvent : uint8_t {
        ALS_EVENT_LOW_TRIGGER = 0, //!< Low threshold window interrupt
        ALS_EVENT_HIGH_TRIGGER,    //!< High threshold window interrupt
        ALS_EVENT_NULL,            //!< No interrupt event
    };

    /**
     * @brief Construct a CM32181 driver instance.
     */
    SensorCM32181() = default;

    /**
     * @brief Destructor. Deinitializes communication backend if created.
     */
    ~SensorCM32181() = default;

    /**
     * @brief Configure ALS sensitivity and integration time.
     *
     * This updates REG_ALS_CONF bits:
     * - bits 12:11: sensitivity
     * - bits 9:6: integration time
     *
     * @param sampling ALS sensitivity.
     * @param int_time Integration time.
     */
    void setSampling(Sampling sampling = SAMPLING_X1,
                     IntegrationTime int_time = INTEGRATION_TIME_200MS)
    {
        uint16_t data = 0;
        readRegBuff(REG_ALS_CONF, (uint8_t *)&data, 2);

        // Sensitivity bits [12:11]
        data &= (uint16_t)~ALS_CONF_SENS_MASK;
        data |= (uint16_t)(sampling & 0x03u) << ALS_CONF_SENS_SHIFT;

        // Integration time bits [9:6]
        data &= (uint16_t)~ALS_CONF_IT_MASK;
        data |= (uint16_t)(int_time & 0x0Fu) << ALS_CONF_IT_SHIFT;

        // FIX: Write back configuration (original code mistakenly read instead of write)
        writeRegBuff(REG_ALS_CONF, (uint8_t *)&data, 2);
    }

    /**
     * @brief Set ALS interrupt thresholds.
     *
     * @param low_threshold  Low threshold (raw 16-bit).
     * @param high_threshold High threshold (raw 16-bit).
     */
    void setIntThreshold(uint16_t low_threshold, uint16_t high_threshold)
    {
        uint8_t buffer[2] = {0};

        // High threshold window (REG_ALS_THDH)
        buffer[1] = highByte(high_threshold);
        buffer[0] = lowByte(high_threshold);
        writeRegBuff(REG_ALS_THDH, buffer, 2);

        // Low threshold window (REG_ALS_THDL)
        buffer[1] = highByte(low_threshold);
        buffer[0] = lowByte(low_threshold);
        writeRegBuff(REG_ALS_THDL, buffer, 2);
    }

    /**
     * @brief Configure power saving mode.
     *
     * REG_ALS_PSM (0x03):
     * - bits 2:1: mode (Mode1..Mode4)
     * - bit 0: enable power saving
     *
     * @param mode   Power saving mode.
     * @param enable true to enable power saving, false to disable.
     */
    void powerSave(PowerSaveMode mode, bool enable)
    {
        uint16_t data = 0;
        readRegBuff(REG_ALS_PSM, (uint8_t *)&data, 2);

        // Clear mode bits [2:1] and enable bit [0]
        data &= (uint16_t)~ALS_PSM_MODE_MASK;
        data &= (uint16_t)~ALS_PSM_EN_BIT;

        // Set mode
        data |= (uint16_t)(mode & 0x03u) << ALS_PSM_MODE_SHIFT;

        // Set enable
        if (enable) {
            data |= ALS_PSM_EN_BIT;
        }

        writeRegBuff(REG_ALS_PSM, (uint8_t *)&data, 2);
    }

    /**
     * @brief Read IRQ status (and clear interrupt by reading status register).
     *
     * @return Interrupt event type.
     */
    InterruptEvent getIrqStatus()
    {
        uint16_t data = 0;
        readRegBuff(REG_ALS_STATUS, (uint8_t *)&data, 2);

        // Keep original mapping:
        // bit15 => low threshold trigger
        // bit14 => high threshold trigger
        if (bitRead(data, 15)) {
            return ALS_EVENT_LOW_TRIGGER;
        }
        if (bitRead(data, 14)) {
            return ALS_EVENT_HIGH_TRIGGER;
        }
        return ALS_EVENT_NULL;
    }

    /**
     * @brief Enable interrupt function (REG_ALS_CONF bit1).
     */
    void enableINT()
    {
        uint16_t data = 0;
        readRegBuff(REG_ALS_CONF, (uint8_t *)&data, 2);
        bitWrite(data, 1, 1);
        writeRegBuff(REG_ALS_CONF, (uint8_t *)&data, 2);
    }

    /**
     * @brief Disable interrupt function (REG_ALS_CONF bit1).
     */
    void disableINT()
    {
        uint16_t data = 0;
        readRegBuff(REG_ALS_CONF, (uint8_t *)&data, 2);
        bitWrite(data, 1, 0);
        writeRegBuff(REG_ALS_CONF, (uint8_t *)&data, 2);
    }

    /**
     * @brief Power on sensor (REG_ALS_CONF bit0 = 0).
     */
    void powerOn()
    {
        uint16_t data = 0;
        readRegBuff(REG_ALS_CONF, (uint8_t *)&data, 2);
        bitClear(data, 0);
        writeRegBuff(REG_ALS_CONF, (uint8_t *)&data, 2);
    }

    /**
     * @brief Shutdown sensor (REG_ALS_CONF bit0 = 1).
     */
    void powerDown()
    {
        uint16_t data = 0;
        readRegBuff(REG_ALS_CONF, (uint8_t *)&data, 2);
        bitSet(data, 0);
        writeRegBuff(REG_ALS_CONF, (uint8_t *)&data, 2);
    }

    /**
     * @brief Read raw ALS data (16-bit).
     *
     * @return Raw data.
     */
    uint16_t getRaw()
    {
        uint8_t buffer[2] = {0};
        readRegBuff(REG_ALS_DATA, buffer, 2);
        return (uint16_t)buffer[0] | (uint16_t)(buffer[1] << 8);
    }

    /**
     * @brief Convert raw ALS to lux (approximate).
     *
     * @return Lux value.
     */
    float getLux()
    {
        return getRaw() * calibration_factor;
    }

    /**
     * @brief Read chip ID.
     *
     * @return Chip ID (8-bit).
     */
    int getChipID()
    {
        uint8_t buffer[2] = {0};
        readRegBuff(REG_ID, buffer, 2);
        return lowByte(buffer[0]);
    }

private:
    // ---- Register addresses (datasheet command codes) ----
    static constexpr uint8_t REG_ALS_CONF   = 0x00;
    static constexpr uint8_t REG_ALS_THDH   = 0x01;
    static constexpr uint8_t REG_ALS_THDL   = 0x02;
    static constexpr uint8_t REG_ALS_PSM    = 0x03;
    static constexpr uint8_t REG_ALS_DATA   = 0x04;
    static constexpr uint8_t REG_ALS_STATUS = 0x06;
    static constexpr uint8_t REG_ID         = 0x07;

    // ---- Chip ID ----
    static constexpr uint8_t CM32181_CHIP_ID = 0x81;

    // ---- Bit field masks/shifts (from datasheet screenshot) ----
    // REG_ALS_CONF (0x00): bits 12:11 sensitivity, bits 9:6 integration time
    static constexpr uint16_t ALS_CONF_SENS_SHIFT = 11;
    static constexpr uint16_t ALS_CONF_SENS_MASK  = (uint16_t)(0x03u << ALS_CONF_SENS_SHIFT);

    static constexpr uint16_t ALS_CONF_IT_SHIFT   = 6;
    static constexpr uint16_t ALS_CONF_IT_MASK    = (uint16_t)(0x0Fu << ALS_CONF_IT_SHIFT);

    // REG_ALS_PSM (0x03): bits 2:1 mode, bit0 enable
    static constexpr uint16_t ALS_PSM_EN_BIT      = (uint16_t)(1u << 0);
    static constexpr uint16_t ALS_PSM_MODE_SHIFT  = 1;
    static constexpr uint16_t ALS_PSM_MODE_MASK   = (uint16_t)(0x03u << ALS_PSM_MODE_SHIFT);

    /**
     * @brief Internal initialization: configure I2C params and verify chip ID.
     */
    bool initImpl(uint8_t param) override
    {
        setAck(false);

        int chipID = getChipID();
        log_i("chipID:%d\n", chipID);

        if (chipID < 0) {
            return false;
        }
        if (chipID != CM32181_CHIP_ID) {
            return false;
        }
        return true;
    }

protected:
    /**
     * @brief Fixed raw-to-lux conversion factor (datasheet/manual derived).
     */
    const float calibration_factor = 0.286f;
};

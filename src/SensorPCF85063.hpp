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
 * @file      SensorPCF85063.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-07
 *
 */
#pragma once

#include "platform/comm/I2CDeviceNoHal.hpp"
#include "SensorRTC.h"

/**
 * @class PCF85063 I2C addresses.
 * @brief Unique address, cannot be changed by hardware configuration.
 */
const uint8_t PCF85063_SLAVE_ADDRESS = 0x51;

/**
 * @class SensorPCF85063
 * @brief Driver for the PCF85063 real-time clock (RTC) over I2C.
 */
class SensorPCF85063 : public SensorRTC, public I2CDeviceNoHal
{
public:
    using SensorRTC::setDateTime;
    using SensorRTC::getDateTime;

    /**
     * @brief Clock output frequency selection.
     *
     * Values map to the PCF85063 clock output control bits (see datasheet).
     */
    enum ClockHz {
        CLK_32768HZ = 0,  //!< 32.768 kHz output
        CLK_16384HZ,      //!< 16.384 kHz output
        CLK_8192HZ,       //!< 8.192 kHz output
        CLK_4096HZ,       //!< 4.096 kHz output
        CLK_2048HZ,       //!< 2.048 kHz output
        CLK_1024HZ,       //!< 1.024 kHz output
        CLK_1HZ,          //!< 1 Hz output
        CLK_LOW,          //!< Lowest/low-power output option (chip-specific)
    };

    /**
     * @brief Construct a SensorPCF85063 instance.
     *
     * The instance is not usable until begin() succeeds.
     */
    SensorPCF85063() = default;

    /**
     * @brief Destructor. Deinitializes the communication backend if created.
     */
    ~SensorPCF85063() = default;

#if defined(ARDUINO)
    /**
     * @brief Initialize device using Arduino Wire (I2C).
     *
     * @param wire TwoWire instance (e.g. Wire).
     * @param sda  SDA pin; pass -1 to keep default.
     * @param scl  SCL pin; pass -1 to keep default.
     * @return true if the device is found and initialization succeeds, false otherwise.
     */
    bool begin(TwoWire &wire, int sda = -1, int scl = -1)
    {
        return I2CDeviceNoHal::begin(wire, PCF85063_SLAVE_ADDRESS, sda, scl);
    }
#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialize device using ESP-IDF legacy I2C driver.
     *
     * @param port_num I2C port number.
     * @param sda      SDA pin; pass -1 to keep default.
     * @param scl      SCL pin; pass -1 to keep default.
     * @return true if the device is found and initialization succeeds, false otherwise.
     */
    bool begin(i2c_port_t port_num, int sda = -1, int scl = -1)
    {
        return I2CDeviceNoHal::begin(port_num, PCF85063_SLAVE_ADDRESS, sda, scl);
    }
#else
    /**
     * @brief Initialize device using ESP-IDF new I2C master bus handle.
     *
     * @param handle I2C master bus handle.
     * @return true if the device is found and initialization succeeds, false otherwise.
     */
    bool begin(i2c_master_bus_handle_t handle)
    {
        return I2CDeviceNoHal::begin(handle, PCF85063_SLAVE_ADDRESS);
    }
#endif  // USEING_I2C_LEGACY
#endif  // ESP_PLATFORM

    /**
     * @brief Initialize device using a custom transport callback.
     *
     * @param callback User-provided callback used by SensorCommCustom.
     * @return true if the device is found and initialization succeeds, false otherwise.
     */
    bool begin(SensorCommCustom::CustomCallback callback)
    {
        return I2CDeviceNoHal::begin(callback, PCF85063_SLAVE_ADDRESS);
    }

    /**
     * @brief Set the RTC date and time.
     *
     * This method converts @ref RTC_DateTime into the PCF85063 register format (BCD)
     * and writes to the seconds register onward.
     *
     * @param datetime Date-time object to set.
     */
    void setDateTime(RTC_DateTime datetime)
    {
        uint8_t buffer[7];
        buffer[0] = DEC2BCD(datetime.getSecond()) & 0x7F;  // seconds (mask OS flag)
        buffer[1] = DEC2BCD(datetime.getMinute());         // minutes
        buffer[2] = DEC2BCD(datetime.getHour());           // hours
        buffer[3] = DEC2BCD(datetime.getDay());            // day of month
        buffer[4] = getDayOfWeek(datetime.getDay(), datetime.getMonth(), datetime.getYear());  // weekday
        buffer[5] = DEC2BCD(datetime.getMonth());          // month
        buffer[6] = DEC2BCD(datetime.getYear() % 100);     // year (00-99)

        writeRegBuff(PCF85063_SEC_REG, buffer, 7);
    }

    /**
     * @brief Read the RTC date and time.
     *
     * Reads 7 bytes starting from the seconds register and converts BCD values to
     * a @ref RTC_DateTime instance.
     *
     * @return Current date-time from the chip.
     */
    RTC_DateTime getDateTime()
    {
        // Note: variable kept for compatibility with existing style (not used)
        RTC_DateTime datetime;

        uint8_t buffer[7];
        uint8_t hour = 0;

        readRegBuff(PCF85063_SEC_REG, buffer, 7);

        uint8_t second = BCD2DEC(buffer[0] & 0x7F);
        uint8_t minute = BCD2DEC(buffer[1] & 0x7F);

        if (is24Hour) {
            hour = BCD2DEC(buffer[2] & 0x3F);  // 24-hour mode
        } else {
            // datetime.AMPM = (buffer[2] & 0x20) == 0x20 ? 'A' : 'P';
            hour = BCD2DEC(buffer[2] & 0x1F);  // 12-hour mode (AM/PM bit not exposed here)
        }

        uint8_t day   = BCD2DEC(buffer[3] & 0x3F);
        uint8_t week  = BCD2DEC(buffer[4] & 0x07);
        uint8_t month = BCD2DEC(buffer[5] & 0x1F);
        uint16_t year = BCD2DEC(buffer[6]) + 2000;

        return RTC_DateTime(year, month, day, hour, minute, second, week);
    }

    /**
     * @brief Check whether the clock integrity is guaranteed.
     *
     * Typically this is determined by the "OS" (oscillator stop) flag in the seconds register.
     * If the oscillator has stopped, the time registers may not be reliable.
     *
     * @return true if the oscillator has not stopped (clock is reliable), false otherwise.
     */
    bool isClockIntegrityGuaranteed()
    {
        return getRegBit(PCF85063_SEC_REG, 7) == 0;
    }

    /*
     * 24H/12H mode APIs were intentionally kept disabled in original code.
     * If needed, enable and document them similarly.
     */

    /**
     * @brief Stop the RTC oscillator (timekeeping stops).
     *
     * Sets the STOP bit in CTRL1.
     */
    void stop()
    {
        setRegBit(PCF85063_CTRL1_REG, 5);
    }

    /**
     * @brief Start the RTC oscillator (timekeeping runs).
     *
     * Clears the STOP bit in CTRL1.
     */
    void start()
    {
        clrRegBit(PCF85063_CTRL1_REG, 5);
    }

    /**
     * @brief Check whether the RTC oscillator is currently running.
     *
     * @return true if running, false if stopped.
     */
    bool isRunning()
    {
        return !getRegBit(PCF85063_CTRL1_REG, 5);
    }

    /**
     * @brief Enable alarm interrupt/event generation.
     */
    void enableAlarm()
    {
        setRegBit(PCF85063_CTRL2_REG, 7);
    }

    /**
     * @brief Disable alarm interrupt/event generation.
     */
    void disableAlarm()
    {
        clrRegBit(PCF85063_CTRL2_REG, 7);
    }

    /**
     * @brief Clear/reset the alarm flag.
     */
    void resetAlarm()
    {
        clrRegBit(PCF85063_CTRL2_REG, 6);
    }

    /**
     * @brief Check whether the alarm flag is active.
     *
     * @return true if alarm flag is set, false otherwise.
     */
    bool isAlarmActive()
    {
        return getRegBit(PCF85063_CTRL2_REG, 6);
    }

    /**
     * @brief Read current alarm configuration from the device.
     *
     * @return Alarm configuration as @ref RTC_Alarm.
     *
     * @note The chip supports enabling/disabling each alarm field via MSB bits.
     *       The conversion logic here follows original implementation.
     */
    RTC_Alarm getAlarm()
    {
        uint8_t buffer[5];
        readRegBuff(PCF85063_ALRM_MIN_REG, buffer, 5);
        buffer[0] = BCD2DEC(buffer[0] & 0x80);  // second
        buffer[1] = BCD2DEC(buffer[1] & 0x40);  // minute
        buffer[2] = BCD2DEC(buffer[2] & 0x40);  // hour
        buffer[3] = BCD2DEC(buffer[3] & 0x08);  // day
        buffer[4] = BCD2DEC(buffer[4] & 0x08);  // weekday

        return RTC_Alarm(buffer[2], buffer[1], buffer[0], buffer[3], buffer[4]);
    }

    /**
     * @brief Set alarm using @ref RTC_Alarm object.
     *
     * @param alarm Alarm configuration object.
     */
    void setAlarm(RTC_Alarm alarm)
    {
        setAlarm(alarm.getHour(), alarm.getMinute(), alarm.getSecond(),
                 alarm.getDay(), alarm.getWeek());
    }

    /**
     * @brief Set the alarm fields (hour/minute/second/day/weekday).
     *
     * Any field can be disabled by passing @c PCF85063_NO_ALARM.
     * Values out of range will be clamped where appropriate.
     *
     * @param hour   Hour [0..23] (24H mode) or chip-specific in 12H mode; or PCF85063_NO_ALARM.
     * @param minute Minute [0..59]; or PCF85063_NO_ALARM.
     * @param second Second [0..59]; or PCF85063_NO_ALARM.
     * @param day    Day of month [1..daysInMonth]; or PCF85063_NO_ALARM.
     * @param week   Weekday [0..6]; or PCF85063_NO_ALARM.
     */
    void setAlarm(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day, uint8_t week)
    {
        uint8_t buffer[5] = {0};

        RTC_DateTime datetime = getDateTime();
        uint8_t daysInMonth = getDaysInMonth(datetime.getMonth(), datetime.getYear());

        // Seconds
        if (second != PCF85063_NO_ALARM) {
            if (second > 59) {
                second = 59;
            }
            buffer[0] = DEC2BCD(second);
            buffer[0] &= ~PCF85063_ALARM_ENABLE;
        } else {
            buffer[0] = PCF85063_ALARM_ENABLE;
        }

        // Minutes
        if (minute != PCF85063_NO_ALARM) {
            if (minute > 59) {
                minute = 59;
            }
            buffer[1] = DEC2BCD(minute);
            buffer[1] &= ~PCF85063_ALARM_ENABLE;
        } else {
            buffer[1] = PCF85063_ALARM_ENABLE;
        }

        // Hours
        if (hour != PCF85063_NO_ALARM) {
            if (is24Hour) {
                if (hour > 23) {
                    hour = 23;
                }
                buffer[2] = DEC2BCD(hour);
                buffer[2] &= ~PCF85063_ALARM_ENABLE;
            }
        } else {
            buffer[2] = PCF85063_ALARM_ENABLE;
        }

        // Day of month
        if (day != PCF85063_NO_ALARM) {
            buffer[3] = DEC2BCD(((day) < (1) ? (1) : ((day) > (daysInMonth) ? (daysInMonth) : (day))));
            buffer[3] &= ~PCF85063_ALARM_ENABLE;
        } else {
            buffer[3] = PCF85063_ALARM_ENABLE;
        }

        // Weekday
        if (week != PCF85063_NO_ALARM) {
            if (week > 6) {
                week = 6;
            }
            buffer[4] = DEC2BCD(week);
            buffer[4] &= ~PCF85063_ALARM_ENABLE;
        } else {
            buffer[4] = PCF85063_ALARM_ENABLE;
        }

        // Write alarm registers
        writeRegBuff(PCF85063_ALRM_SEC_REG, buffer, 4);
    }

    /**
     * @brief Convenience: enable only hour alarm.
     *
     * @param hour Hour [0..23].
     */
    void setAlarmByHours(uint8_t hour)
    {
        setAlarm(hour,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM);
    }

    /**
     * @brief Convenience: enable only second alarm.
     *
     * @param second Second [0..59].
     */
    void setAlarmBySecond(uint8_t second)
    {
        setAlarm(PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 second,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM);
    }

    /**
     * @brief Convenience: enable only minute alarm.
     *
     * @param minute Minute [0..59].
     */
    void setAlarmByMinutes(uint8_t minute)
    {
        setAlarm(PCF85063_NO_ALARM,
                 minute,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM);
    }

    /**
     * @brief Convenience: enable only day-of-month alarm.
     *
     * @param day Day of month [1..31] (will be clamped to valid range for current month).
     */
    void setAlarmByDays(uint8_t day)
    {
        setAlarm(PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 day,
                 PCF85063_NO_ALARM);
    }

    /**
     * @brief Convenience: enable only weekday alarm.
     *
     * @param week Weekday [0..6].
     */
    void setAlarmByWeekDay(uint8_t week)
    {
        setAlarm(PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 PCF85063_NO_ALARM,
                 week);
    }

    /**
     * @brief Configure the clock output frequency.
     *
     * @param hz Output frequency selection.
     */
    void setClockOutput(ClockHz hz)
    {
        int val = readReg(PCF85063_CTRL2_REG);
        if (val == -1) return;

        val &= 0xF8;  // clear frequency bits
        val |= hz;    // set new frequency
        writeReg(PCF85063_CTRL2_REG, val);
    }

    /**
     * @brief Get RTC chip name.
     *
     * @return Constant string "PCF85063".
     */
    const char *getChipName()
    {
        return "PCF85063";
    }

private:
    /**
     * @brief Internal initialization routine.
     *
     * - Checks device presence via a RAM register
     * - Verifies that RAM register bit 7 is writable to distinguish PCF85063 vs others
     * - Forces 24-hour mode (if needed)
     * - Starts the oscillator
     *
     * @return true if initialization succeeds and RTC is running, false otherwise.
     */
    bool initImpl(uint8_t param) override
    {
        // Check device is online
        int val = readReg(PCF85063_RAM_REG);
        if (val < 0) {
            log_e("Device is offline!");
            return false;
        }

        // Backup original RAM register value
        uint8_t tmp = readReg(PCF85063_RAM_REG);

        bool rlst = false;

        // Determine whether this is PCF85063 by testing RAM register bit writability:
        // If bit 7 can be set/cleared, it is likely PCF85063.
        writeReg(PCF85063_RAM_REG, val | _BV(7));
        val = readReg(PCF85063_RAM_REG);
        if (val & 0x80) {
            writeReg(PCF85063_RAM_REG, val & ~_BV(7));
            val = readReg(PCF85063_RAM_REG);
            if ((val & 0x80) == 0) {
                rlst = true;
            }
        }

        if (!rlst) {
            log_e("Failed to write to RAM memory register. Maybe this chip is pcf8563.");
            return false;
        }

        // Restore RAM register
        writeReg(PCF85063_RAM_REG, tmp);

        // Default to 24-hour mode
        is24Hour = !getRegBit(PCF85063_CTRL1_REG, 1);
        if (!is24Hour) {
            // Force 24H Mode
            clrRegBit(PCF85063_CTRL1_REG, 1);
            is24Hour = true;
        }

        // Turn on RTC
        start();

        return isRunning();
    }

protected:
    /**
     * @brief Hour mode flag (true = 24-hour mode).
     *
     * This is detected (and forced to 24H) during initialization.
     */
    bool is24Hour;

    // Register addresses
    static constexpr uint8_t PCF85063_CTRL1_REG = 0x00;
    static constexpr uint8_t PCF85063_CTRL2_REG = 0x01;
    static constexpr uint8_t PCF85063_OFFSET_REG = 0x02;
    static constexpr uint8_t PCF85063_RAM_REG = 0x03;
    static constexpr uint8_t PCF85063_SEC_REG = 0x04;
    static constexpr uint8_t PCF85063_MIN_REG = 0x05;
    static constexpr uint8_t PCF85063_HR_REG = 0x06;
    static constexpr uint8_t PCF85063_DAY_REG = 0x07;
    static constexpr uint8_t PCF85063_WEEKDAY_REG = 0x08;
    static constexpr uint8_t PCF85063_MONTH_REG = 0x09;
    static constexpr uint8_t PCF85063_YEAR_REG = 0x0A;
    static constexpr uint8_t PCF85063_ALRM_SEC_REG = 0x0B;
    static constexpr uint8_t PCF85063_ALRM_MIN_REG = 0x0C;
    static constexpr uint8_t PCF8563_ALRM_HR_REG = 0x0D;
    static constexpr uint8_t PCF85063_ALRM_DAY_REG = 0x0E;
    static constexpr uint8_t PCF85063_ALRM_WEEK_REG = 0x0F;
    static constexpr uint8_t PCF85063_TIMER_VAL_REG = 0x10;
    static constexpr uint8_t PCF85063_TIMER_MD_REG = 0x11;

    // Mask values
    static constexpr uint8_t PCF85063_CTRL1_TEST_EN_MASK = (1 << 7u);
    static constexpr uint8_t PCF85063_CTRL1_CLOCK_EN_MASK = (1 << 5u);
    static constexpr uint8_t PCF85063_CTRL1_SOFTRST_EN_MASK = (1 << 4u);
    static constexpr uint8_t PCF85063_CTRL1_CIE_EN_MASK = (1 << 2u);
    static constexpr uint8_t PCF85063_CTRL1_HOUR_FORMAT_12H_MASK = (1 << 1u);

    // Other constants
    static constexpr uint8_t PCF85063_NO_ALARM = 0xFF;
    static constexpr uint8_t PCF85063_ALARM_ENABLE = 0x80;
};

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
 * @file      SensorPCF8563.hpp
 * @brief     PCF8563 RTC driver wrapper for SensorLib.
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-12-09
 *
 */
#pragma once

#include "platform/comm/I2CDeviceNoHal.hpp"
#include "SensorRTC.h"

/**
 * @class PCF8563 I2C addresses.
 * @brief Unique address, cannot be changed by hardware configuration.
 */
static const uint8_t PCF8563_SLAVE_ADDRESS = 0x51;

/**
 * @class SensorPCF8563
 * @brief Driver for the PCF8563 real-time clock (RTC) over I2C.
 */
class SensorPCF8563 : public SensorRTC, public I2CDeviceNoHal
{
public:
    using SensorRTC::setDateTime;
    using SensorRTC::getDateTime;
    using I2CDeviceNoHal::begin;

    /**
     * @brief Clock output frequencies for the CLKOUT pin.
     */
    enum ClockHz {
        CLK_32768HZ, /**< 32.768 kHz output */
        CLK_1024HZ,  /**< 1.024 kHz output */
        CLK_32HZ,    /**< 32 Hz output */
        CLK_1HZ,     /**< 1 Hz output */
        CLK_DISABLE, /**< Disable CLKOUT */
    };

    /**
     * @brief Construct a PCF8563 driver instance.
     */
    SensorPCF8563() = default;

    /**
     * @brief Destructor.
     *
     * Deinitializes the underlying communication bus if it has been created.
     */
    ~SensorPCF8563() = default;

#if defined(ARDUINO)
    /**
     * @brief Initialize the device using Arduino TwoWire.
     *
     * @param wire TwoWire instance.
     * @param sda  SDA pin (optional, -1 keeps default).
     * @param scl  SCL pin (optional, -1 keeps default).
     * @return true if the device responds and passes basic sanity checks.
     */
    bool begin(TwoWire &wire, int sda = -1, int scl = -1)
    {
        return I2CDeviceNoHal::begin(wire, PCF8563_SLAVE_ADDRESS, sda, scl);
    }
#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialize the device using ESP-IDF legacy I2C driver.
     *
     * @param port_num I2C port number.
     * @param sda      SDA pin (optional, -1 keeps default).
     * @param scl      SCL pin (optional, -1 keeps default).
     * @return true if initialization succeeds.
     */
    bool begin(i2c_port_t port_num, int sda = -1, int scl = -1)
    {
        return I2CDeviceNoHal::begin(port_num, PCF8563_SLAVE_ADDRESS, sda, scl);
    }
#else
    /**
     * @brief Initialize the device using ESP-IDF I2C master bus handle.
     *
     * @param handle I2C master bus handle.
     * @return true if initialization succeeds.
     */
    bool begin(i2c_master_bus_handle_t handle)
    {
        return I2CDeviceNoHal::begin(handle, PCF8563_SLAVE_ADDRESS);
    }
#endif  // USEING_I2C_LEGACY
#endif  // ESP_PLATFORM

    /**
     * @brief Initialize the device using a user-provided transport callback.
     *
     * @param callback Custom communication callback.
     * @return true if initialization succeeds.
     */
    bool begin(SensorCommCustom::CustomCallback callback)
    {
        return I2CDeviceNoHal::begin(callback, PCF8563_SLAVE_ADDRESS);
    }

    /**
     * @brief Set the RTC date/time.
     *
     * Converts the provided datetime to BCD and writes it to the device.
     *
     * @param datetime Date/time to write.
     */
    void setDateTime(RTC_DateTime datetime)
    {
        uint8_t buffer[7];
        buffer[0] = DEC2BCD(datetime.getSecond()) & 0x7F;
        buffer[1] = DEC2BCD(datetime.getMinute());
        buffer[2] = DEC2BCD(datetime.getHour());
        buffer[3] = DEC2BCD(datetime.getDay());
        buffer[4] = getDayOfWeek(datetime.getDay(), datetime.getMonth(), datetime.getYear());
        buffer[5] = DEC2BCD(datetime.getMonth());
        buffer[6] = DEC2BCD(datetime.getYear() % 100);

        // Set century bit according to year.
        if ((2000 % datetime.getYear()) == 2000) {
            buffer[5] &= 0x7F;
        } else {
            buffer[5] |= 0x80;
        }
        writeRegBuff(SEC_REG, buffer, 7);
    }

    /**
     * @brief Get the current RTC date/time.
     *
     * Reads the registers and converts the BCD values into a RTC_DateTime.
     *
     * @return Current date/time.
     */
    RTC_DateTime getDateTime()
    {
        uint8_t buffer[7];
        readRegBuff(SEC_REG, buffer, 7);
        uint8_t second = BCD2DEC(buffer[0] & 0x7F);
        uint8_t minute = BCD2DEC(buffer[1] & 0x7F);
        uint8_t hour   = BCD2DEC(buffer[2] & 0x3F);
        uint8_t day    = BCD2DEC(buffer[3] & 0x3F);
        uint8_t week   = BCD2DEC(buffer[4] & 0x07);
        uint8_t month  = BCD2DEC(buffer[5] & 0x1F);
        uint16_t year  = BCD2DEC(buffer[6]);

        // Century: 0 = 1900, 1 = 2000.
        year += (buffer[5] & CENTURY_MASK) ? 1900 : 2000;
        return RTC_DateTime(year, month, day, hour, minute, second, week);
    }

    /**
     * @brief Check whether the clock integrity is guaranteed.
     *
     * The PCF8563 sets the VL (Voltage Low) flag when the supply voltage has
     * dropped below the minimum for reliable timekeeping.
     *
     * @return true if the VL flag is clear; false otherwise.
     */
    bool isClockIntegrityGuaranteed()
    {
        return getRegBit(SEC_REG, 7) == 0;
    }

    /**
     * @brief Read the currently configured alarm settings.
     *
     * @note This returns the currently stored alarm registers; it does not imply
     *       that the alarm is enabled.
     *
     * @return Alarm configuration.
     */
    RTC_Alarm getAlarm()
    {
        uint8_t buffer[4];
        readRegBuff(ALRM_MIN_REG, buffer, 4);
        buffer[0] = BCD2DEC(buffer[0] & 0x80); // minute
        buffer[1] = BCD2DEC(buffer[1] & 0x40); // hour
        buffer[2] = BCD2DEC(buffer[2] & 0x40); // day
        buffer[3] = BCD2DEC(buffer[3] & 0x08); // weekday
        // RTC_Alarm(uint8_t hour, uint8_t minute, uint8_t second, uint8_t day, uint8_t week)
        return RTC_Alarm(buffer[1], buffer[0], 0, buffer[2], buffer[3]);
    }

    /**
     * @brief Enable the alarm interrupt.
     */
    void enableAlarm()
    {
        setRegBit(STAT2_REG, 1);
    }

    /**
     * @brief Disable the alarm interrupt.
     */
    void disableAlarm()
    {
        clrRegBit(STAT2_REG, 1);
    }

    /**
     * @brief Clear the alarm flag.
     */
    void resetAlarm()
    {
        clrRegBit(STAT2_REG, 3);
    }

    /**
     * @brief Check whether the alarm flag is active.
     * @return true if active.
     */
    bool isAlarmActive()
    {
        return getRegBit(STAT2_REG, 3);
    }

    /**
     * @brief Set the alarm configuration.
     * @param alarm Alarm configuration object.
     */
    void setAlarm(RTC_Alarm alarm)
    {
        setAlarm(alarm.getHour(), alarm.getMinute(), alarm.getDay(), alarm.getWeek());
    }

    /**
     * @brief Set the alarm configuration.
     *
     * Any field can be disabled by passing NO_ALARM.
     *
     * @param hour   Hour value [0..23] or NO_ALARM.
     * @param minute Minute value [0..59] or NO_ALARM.
     * @param day    Day-of-month [1..31] or NO_ALARM (clamped to the current month).
     * @param week   Weekday [0..6] or NO_ALARM.
     */
    void setAlarm(uint8_t hour, uint8_t minute, uint8_t day, uint8_t week)
    {
        uint8_t buffer[4] = {0};

        RTC_DateTime datetime = getDateTime();

        uint8_t daysInMonth = getDaysInMonth(datetime.getMonth(), datetime.getYear());

        if (minute != NO_ALARM) {
            if (minute > 59) {
                minute = 59;
            }
            buffer[0] = DEC2BCD(minute);
            buffer[0] &= ~ALARM_ENABLE;
        } else {
            buffer[0] = ALARM_ENABLE;
        }

        if (hour != NO_ALARM) {
            if (hour > 23) {
                hour = 23;
            }
            buffer[1] = DEC2BCD(hour);
            buffer[1] &= ~ALARM_ENABLE;
        } else {
            buffer[1] = ALARM_ENABLE;
        }
        if (day != NO_ALARM) {
            buffer[2] = DEC2BCD(((day) < (1) ? (1) : ((day) > (daysInMonth) ? (daysInMonth) : (day))));
            buffer[2] &= ~ALARM_ENABLE;
        } else {
            buffer[2] = ALARM_ENABLE;
        }
        if (week != NO_ALARM) {
            if (week > 6) {
                week = 6;
            }
            buffer[3] = DEC2BCD(week);
            buffer[3] &= ~ALARM_ENABLE;
        } else {
            buffer[3] = ALARM_ENABLE;
        }
        writeRegBuff(ALRM_MIN_REG, buffer, 4);
    }

    /**
     * @brief Convenience API to set an alarm that matches only minutes.
     * @param minute Minute value [0..59].
     */
    void setAlarmByMinutes(uint8_t minute)
    {
        setAlarm(NO_ALARM, minute, NO_ALARM, NO_ALARM);
    }

    /**
     * @brief Convenience API to set an alarm that matches only day-of-month.
     * @param day Day-of-month value.
     */
    void setAlarmByDays(uint8_t day)
    {
        setAlarm(NO_ALARM, NO_ALARM, day, NO_ALARM);
    }

    /**
     * @brief Convenience API to set an alarm that matches only hours.
     * @param hour Hour value.
     */
    void setAlarmByHours(uint8_t hour)
    {
        setAlarm(hour, NO_ALARM, NO_ALARM, NO_ALARM);
    }

    /**
     * @brief Convenience API to set an alarm that matches only weekday.
     * @param week Weekday [0..6].
     */
    void setAlarmByWeekDay(uint8_t week)
    {
        setAlarm(NO_ALARM, NO_ALARM, NO_ALARM, week);
    }

    /**
     * @brief Check whether the countdown timer is enabled.
     *
     * This checks both the timer interrupt enable (TIE) and timer enable (TE)
     * bits.
     *
     * @return true if enabled.
     */
    bool isCountdownTimerEnable()
    {
        uint8_t buffer[2];
        buffer[0] = readReg(STAT2_REG);
        buffer[1] = readReg(TIMER1_REG);
        if (buffer[0] & TIMER_TIE) {
            return buffer[1] & TIMER_TE ? true : false;
        }
        return false;
    }

    /**
     * @brief Check whether the countdown timer flag is active.
     * @return true if active.
     */
    bool isCountdownTimerActive()
    {
        return getRegBit(STAT2_REG, 2);
    }

    /**
     * @brief Enable the countdown timer interrupt.
     */
    void enableCountdownTimer()
    {
        setRegBit(STAT2_REG, 0);
    }

    /**
     * @brief Disable the countdown timer interrupt.
     */
    void disableCountdownTimer()
    {
        clrRegBit(STAT2_REG, 0);
    }

    /**
     * @brief Configure countdown timer.
     *
     * @param val  Countdown value.
     * @param freq Timer frequency selection (see PCF8563Constants).
     */
    void setCountdownTimer(uint8_t val, uint8_t freq)
    {
        uint8_t buffer[3];
        buffer[1] = readReg(TIMER1_REG);
        buffer[1] |= (freq & TIMER_TD10);
        buffer[2] = val;
        writeReg(TIMER1_REG, buffer[1]);
        writeReg(TIMER2_REG, buffer[2]);
    }

    /**
     * @brief Clear and disable the countdown timer.
     */
    void clearCountdownTimer()
    {
        uint8_t val;
        val = readReg(STAT2_REG);
        val &= ~(TIMER_TF | TIMER_TIE);
        val |= ALARM_AF;
        writeReg(STAT2_REG, val);
        writeReg(TIMER1_REG, (uint8_t)0x00);
    }

    /**
     * @brief Configure the clock output (CLKOUT) pin.
     * @param freq Output frequency selection.
     */
    void setClockOutput(ClockHz freq)
    {
        if (freq == CLK_DISABLE) {
            clrRegBit(SQW_REG, 7);
        } else {
            writeReg(SQW_REG, freq | CLK_ENABLE);
        }
    }

    /**
     * @brief Get the chip name string.
     * @return Constant chip name string.
     */
    const char *getChipName()
    {
        return "PCF8563";
    }

private:
    /**
     * @brief Common initialization routine shared by all begin() overloads.
     *
     * Performs a basic probe and sanity-check of the seconds register.
     *
     * @return true if the device responds and the register contents are sane.
     */
    bool initImpl(uint8_t param) override
    {
        // Check device is online.
        int ret = readReg(SEC_REG);
        if (ret < 0) {
            return false;
        }
        if (BCD2DEC(ret & 0x7F) > 59) {
            return false;
        }
        return true;
    }

protected:
    // Register addresses
    static constexpr uint8_t STAT1_REG = 0x00;
    static constexpr uint8_t STAT2_REG = 0x01;
    static constexpr uint8_t SEC_REG = 0x02;
    static constexpr uint8_t MIN_REG = 0x03;
    static constexpr uint8_t HR_REG = 0x04;
    static constexpr uint8_t DAY_REG = 0x05;
    static constexpr uint8_t WEEKDAY_REG = 0x06;
    static constexpr uint8_t MONTH_REG = 0x07;
    static constexpr uint8_t YEAR_REG = 0x08;
    static constexpr uint8_t ALRM_MIN_REG = 0x09;
    static constexpr uint8_t SQW_REG = 0x0D;
    static constexpr uint8_t TIMER1_REG = 0x0E;
    static constexpr uint8_t TIMER2_REG = 0x0F;

    // Mask values
    static constexpr uint8_t VOL_LOW_MASK = 0x80;
    static constexpr uint8_t MINUTES_MASK = 0x7F;
    static constexpr uint8_t HOUR_MASK = 0x3F;
    static constexpr uint8_t WEEKDAY_MASK = 0x07;
    static constexpr uint8_t CENTURY_MASK = 0x80;
    static constexpr uint8_t DAY_MASK = 0x3F;
    static constexpr uint8_t MONTH_MASK = 0x1F;
    static constexpr uint8_t TIMER_CTL_MASK = 0x03;

    // Alarm and Timer flags
    static constexpr uint8_t ALARM_AF = 0x08;
    static constexpr uint8_t TIMER_TF = 0x04;
    static constexpr uint8_t ALARM_AIE = 0x02;
    static constexpr uint8_t TIMER_TIE = 0x01;
    static constexpr uint8_t TIMER_TE = 0x80;
    static constexpr uint8_t TIMER_TD10 = 0x03;

    // Other constants
    static constexpr uint8_t NO_ALARM = 0xFF;
    static constexpr uint8_t ALARM_ENABLE = 0x80;
    static constexpr uint8_t CLK_ENABLE = 0x80;
};

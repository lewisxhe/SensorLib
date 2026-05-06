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
 * @file      PmicGpioBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 * @brief PMIC GPIO capability interface (single GPIO pin abstraction).
 */
#pragma once
#include <stdint.h>

class PmicGpioBase
{
public:
    /**
     * @brief Virtual destructor
     */
    virtual ~PmicGpioBase() = default;

    /**
     * @brief GPIO pin direction
     */
    enum class Direction : uint8_t {
        Input = 0,   ///< Configure pin as input (read external signals)
        Output = 1,  ///< Configure pin as output (drive external circuits)
    };

    /**
     * @brief GPIO drive type
     */
    enum class DriveType : uint8_t {
        PushPull = 0,  ///< Push-pull output: actively drives high or low
        OpenDrain = 1, ///< Open-drain output: only pulls low, high-impedance for high
        Keep = 0xFF,   ///< Keep current drive type (do not change)
    };

    /**
     * @brief Output level control
     *
     * Some PMICs support Hi-Z/Low/High; others only Low/Hi-Z for open-drain.
     */
    enum class Level : uint8_t {
        Low = 0,   ///< Logic low (active drive low or open-drain pulled low)
        HiZ = 1,   ///< High-impedance (input mode or open-drain not pulling low)
        High = 2,  ///< Logic high (active drive high)
    };

    /**
     * @brief Internal pull resistor configuration
     */
    enum class Pull : uint8_t {
        None = 0,  ///< No pull resistor
        Down = 1,  ///< Pull-down enabled
        Up = 2,    ///< Pull-up enabled
        Keep = 0xFF, ///< Keep current pull setting (do not change)
    };

    /**
     * @brief GPIO operation status code.
     */
    enum class Status : uint8_t {
        Ok = 0,
        InvalidPin = 1,
        Failed = 2,
    };

    static constexpr bool isOk(Status s)
    {
        return s == Status::Ok;
    }

    /**
     * @brief Configure a pin with a single high-level call.
     * @param pin GPIO pin number
     * @param dir Direction
     * @param drive Drive type. Use DriveType::Keep to skip drive configuration.
     */
    Status pinMode(uint8_t pin, Direction dir, DriveType drive = DriveType::Keep,
                   Pull pull = Pull::Keep, uint8_t func = 0xFF)
    {
        if (drive != DriveType::Keep) {
            Status st = setDrive(pin, drive);
            if (!isOk(st) && getDrive(pin) != drive) return st;
        }

        if (pull != Pull::Keep) {
            Status st = setPull(pin, pull);
            if (!isOk(st) && getPull(pin) != pull) return st;
        }

        return Status::Ok;
    }

    /**
     * @brief Get number of GPIO pins supported by this PMIC
     * @return Number of GPIO pins
     */
    virtual uint8_t getPinCount() const = 0;

    /**
     * @brief Set GPIO pin direction
     * @param pin GPIO pin number (if multiple pins are supported by the PMIC)
     * @param dir Direction to set (Input or Output)
     * @return Operation status
     *
     * @note Must be called before reading or writing the pin
     * @see getDirection()
     */
    virtual Status setDirection(uint8_t pin, Direction dir) = 0;

    /**
     * @brief Get current GPIO pin direction
     * @param pin GPIO pin number
     * @return Current direction setting
     *
     * @note On I2C error, returns Direction::Input as a safe default.
     *       Callers cannot distinguish this from a genuine Input setting.
     * @see setDirection()
     */
    virtual Direction getDirection(uint8_t pin) = 0;

    /**
     * @brief Set GPIO drive type
     * @param pin GPIO pin number
     * @param drive Drive type (PushPull or OpenDrain)
     * @return Operation status
     *
     * @note Some PMICs have fixed drive type (e.g. AXP192 is always open-drain,
     *       AXP202 is always push-pull). In such cases, unsupported drive types
     *       return Status::Failed.
     * @see getDrive()
     */
    virtual Status setDrive(uint8_t pin, DriveType drive) = 0;

    /**
     * @brief Get current GPIO drive type
     * @param pin GPIO pin number
     * @return Current drive type setting
     *
     * @see setDrive()
     */
    virtual DriveType getDrive(uint8_t pin) = 0;

    /**
     * @brief Read input level
     * @param pin GPIO pin number
     * @param high Reference to store the read level (true = high, false = low)
     * @return Operation status
     *
     * @note Pin must be configured as Input direction first
     */
    virtual Status read(uint8_t pin, bool &high) = 0;

    /**
     * @brief Set output level
     * @param pin GPIO pin number
     * @param level Level to set (HiZ, Low, or High)
     * @return Operation status
     *
     * @note Only meaningful if pin is configured as Output direction
     * @see setDirection()
     */
    virtual Status write(uint8_t pin, Level level) = 0;

    /**
     * @brief Set GPIO pin function mode (chip-specific multiplexing)
     * @param pin GPIO pin number
     * @param func Function mode value (chip-specific, see chip header for valid values)
     * @return Operation status
     *
     * @note Default implementation returns Status::Failed
     *       (no function select support).
     *       Chip-specific subclasses define valid function values.
     *       For AXP192 GPIO0-2: 0=NMOS open-drain, 1=input, 2=LDO/PWM, 4=ADC, 5=output low, 7=floating
     *       For AXP202 GPIO0-2: 0=output low, 1=output high, 2=input, 3=LDO, 4=ADC, 7=floating
     */
    virtual Status setFunction(uint8_t pin, uint8_t func)
    {
        (void)pin;
        (void)func;
        return Status::Failed;
    }

    /**
     * @brief Get current GPIO pin function mode
     * @param pin GPIO pin number
     * @return Function mode value, or 0xFF if not supported
     */
    virtual uint8_t getFunction(uint8_t pin)
    {
        (void)pin;
        return 0xFF;
    }

    /**
     * @brief Set internal pull resistor
     * @param pin GPIO pin number
     * @param pull Pull configuration (None, Down, or Up)
     * @return Operation status
     *
     * @note Default implementation returns Status::Failed
     *       (no pull resistor support).
     *       Currently only AXP192 supports pull-down on GPIO0-2.
     */
    virtual Status setPull(uint8_t pin, Pull pull)
    {
        (void)pin;
        (void)pull;
        return Status::Failed;
    }

    /**
     * @brief Get current internal pull resistor setting
     * @param pin GPIO pin number
     * @return Current pull configuration
     */
    virtual Pull getPull(uint8_t pin)
    {
        (void)pin;
        return Pull::None;
    }
};

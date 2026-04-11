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
    enum class Drive : uint8_t {
        Floating = 0,   ///< Push-pull output: actively drives high or low
        OpenDrain = 1,  ///< Open-drain output: only pulls low, high-impedance for high
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
     * @brief Set GPIO pin direction
     * @param pin GPIO pin number (if multiple pins are supported by the PMIC)
     * @param dir Direction to set (Input or Output)
     * @return true on success, false on failure
     *
     * @note Must be called before reading or writing the pin
     * @see getDirection()
     */
    virtual bool setDirection(uint8_t pin, Direction dir) = 0;

    /**
     * @brief Get current GPIO pin direction
     * @param pin GPIO pin number
     * @return Current direction setting
     *
     * @see setDirection()
     */
    virtual Direction getDirection(uint8_t pin) = 0;

    /**
     * @brief Set GPIO drive type
     * @param pin GPIO pin number
     * @param drive Drive type (Floating/Push-Pull or Open-Drain)
     * @return true on success, false on failure
     *
     * @note Open-drain is useful for I2C-like open-collector signaling
     * @see getDrive()
     */
    virtual bool setDrive(uint8_t pin, Drive drive) = 0;

    /**
     * @brief Get current GPIO drive type
     * @param pin GPIO pin number
     * @return Current drive type setting
     *
     * @see setDrive()
     */
    virtual Drive getDrive(uint8_t pin) = 0;

    /**
     * @brief Read input level
     * @param pin GPIO pin number
     * @param high Reference to store the read level (true = high, false = low)
     * @return true on success, false on failure
     *
     * @note Pin must be configured as Input direction first
     */
    virtual bool read(uint8_t pin, bool &high) = 0;

    /**
     * @brief Set output level
     * @param pin GPIO pin number
     * @param level Level to set (HiZ, Low, or High)
     * @return true on success, false on failure
     *
     * @note Only meaningful if pin is configured as Output direction
     * @see setDirection()
     */
    virtual bool write(uint8_t pin, Level level) = 0;
};

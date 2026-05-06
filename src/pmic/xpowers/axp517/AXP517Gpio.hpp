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
 * @file      AXP517Gpio.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */

#pragma once

#include "../../PmicGpioBase.hpp"
#include "AXP517Core.hpp"

/**
 * @brief AXP517 GPIO module (REG 0x11).
 *
 * From manual:
 *  - bit7: OD function configure: 0 floating, 1 open drain output
 *  - bit6: OD output configure: 0 hiz, 1 low
 *  - bit5: reserved (RO)
 *  - bit4: GPIO function: 0 input, 1 output
 *  - bit3:2: GPIO output source select:
 *        0: by reg11H[1:0]
 *        1: PD_IRQ
 *  - bit1: GPIO input status (RO): 0 low, 1 high
 *  - bit1:0: GPIO output configure:
 *        00: hiz
 *        01: low
 *        10: high
 *        11: reserved
 *
 * Notes:
 *  - AXP517 has only 1 GPIO pin. Pin parameter must be 0.
 *  - When output source is PD_IRQ, manual output config bits are ignored by HW.
 *  - When OD mode enabled (bit7=1), OD output uses bit6 (hiz/low). "high" is not driven.
 */
class AXP517Gpio : public PmicGpioBase
{
public:

    enum class OutputSource : uint8_t {
        ByReg11 = 0,  ///< by reg11[1:0]
        PdIrq   = 1,  ///< PD_IRQ
    };

    explicit AXP517Gpio(AXP517Core &core);

    ~AXP517Gpio() = default;

    /**
     * @brief Get number of GPIO pins (1)
     * @return 1
     */
    uint8_t getPinCount() const override;

    /**
     * @brief Set GPIO pin direction
     * @param pin GPIO pin number (must be 0)
     * @param dir Direction (Input or Output)
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed on I2C/register error
     */
    Status setDirection(uint8_t pin, Direction dir) override;

    /**
     * @brief Get GPIO pin direction
     * @param pin GPIO pin number (must be 0)
     * @return Current direction setting, Direction::Input on error
     */
    Direction getDirection(uint8_t pin) override;

    /**
     * @brief Set GPIO pin drive type
     * @param pin GPIO pin number (must be 0)
     * @param drive Drive type (PushPull or OpenDrain)
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed on I2C/register error
     */
    Status setDrive(uint8_t pin, DriveType drive) override;

    /**
     * @brief Get GPIO pin drive type
     * @param pin GPIO pin number (must be 0)
     * @return Current drive type setting, DriveType::PushPull on error
     */
    DriveType getDrive(uint8_t pin) override;

    /**
     * @brief Read input level
     * @param pin GPIO pin number (must be 0)
     * @param high Reference to store the read level (true = high, false = low)
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed on I2C/register error
     *
     * @note Pin must be configured as Input direction first
     */
    Status read(uint8_t pin, bool &high) override;

    /**
     * @brief Set output level
     * @param pin GPIO pin number (must be 0)
     * @param level Level to set (HiZ, Low, or High)
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed if output source is PD_IRQ, direction not output,
     *         or on I2C/register error
     *
     * @note Only meaningful if pin is configured as Output direction
     * @note When output source is PD_IRQ, manual write is ignored by hardware
     * @see setDirection()
     */
    Status write(uint8_t pin, Level level) override;

    /**
     * @brief Select GPIO output source
     * @param pin GPIO pin number (must be 0)
     * @param src Output source:
     *            OutputSource::ByReg11 — output controlled by write()
     *            OutputSource::PdIrq    — output follows PD_IRQ hardware signal
     * @return true on success, false if pin != 0 or I2C error
     *
     * @note When source is PdIrq, manual write() is ignored by hardware.
     * @see getOutputSource()
     */
    bool setOutputSource(uint8_t pin, OutputSource src);

    /**
     * @brief Get current GPIO output source
     * @param pin GPIO pin number (must be 0)
     * @return Current OutputSource value (OutputSource::ByReg11 on error)
     *
     * @see setOutputSource()
     */
    OutputSource getOutputSource(uint8_t pin);

    /**
     * @brief Read raw REG11 for debug.
     */
    int readRaw();

private:
    AXP517Core &_core;
};

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
 * @file      AXP202Gpio.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP202 GPIO Interface
 *
 * Controls up to four GPIO pins (GPIO0-GPIO3) on the AXP202 PMIC.
 * GPIO0 can additionally be configured as an LDO output (LDOio)
 * with 1800-3300mV output in 100mV steps via register 0x91.
 * Each GPIO supports direction (input/output), drive strength,
 * digital read, and digital write operations.
 *
 * @section GPIO Pin Mapping
 * - GPIO0: Also functions as LDOio voltage output (reg 0x91)
 * - GPIO1: General purpose I/O
 * - GPIO2: General purpose I/O
 * - GPIO3: NMOS open-drain or input only (REG 95H)
 *
 * @note LDOio voltage control is handled through AXP202Channel, not this class.
 */
#pragma once
#include "../../PmicGpioBase.hpp"
#include "AXP202Core.hpp"

/**
 * @brief AXP202 GPIO pin control
 *
 * Implements the PmicGpioBase interface to manage GPIO direction,
 * drive current, and digital level for the AXP202's four GPIO pins.
 *
 * @section FunctionValues GPIO0-2 Function Select Values (REG 90H/92H/93H [2:0])
 * - 0x00: Output low
 * - 0x01: Output high (3.3V)
 * - 0x02: Input
 * - 0x03: Low-noise LDO
 * - 0x04: ADC input
 * - 0x06/0x07: Floating (high-impedance)
 *
 * @section GPIO3Func GPIO3 (REG 95H)
 * - bit2: 0=NMOS open-drain, 1=input
 * - bit1: output: 0=low NMOS on, 1=floating NMOS off
 * - bit0: input status (R): 0=high, 1=low (active-low!)
 *
 * @note GPIO3 does not support function multiplexing or push-pull output.
 *       Level::High on GPIO3 returns Status::Failed
 *       (not supported by hardware).
 */
class AXP202Gpio : public PmicGpioBase
{
public:
    /**
     * @brief Construct AXP202 GPIO interface
     * @param core Reference to AXP202 core communication
     */
    explicit AXP202Gpio(AXP202Core &core);

    ~AXP202Gpio() = default;

    /**
     * @brief Get number of GPIO pins (4: GPIO0-GPIO3)
     * @return 4
     */
    uint8_t getPinCount() const override;

    /**
     * @brief Set GPIO pin direction
     * @param pin Pin number (0-3)
     * @param dir Direction (Input or Output)
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed on I2C/register error
     */
    Status setDirection(uint8_t pin, Direction dir) override;

    /**
     * @brief Get GPIO pin direction
     * @param pin Pin number (0-3)
     * @return Current Direction enum value
     */
    Direction getDirection(uint8_t pin) override;

    /**
     * @brief Set GPIO pin drive type
     * @param pin Pin number (0-3)
     * @param drive Drive type
     * @return Status::Failed for unsupported drive setting,
     *         Status::InvalidPin for invalid pin
     */
    Status setDrive(uint8_t pin, DriveType drive) override;

    /**
     * @brief Get GPIO pin drive type
     * @param pin Pin number (0-3)
     * @return PushPull for GPIO0-2, OpenDrain for GPIO3
     */
    DriveType getDrive(uint8_t pin) override;

    /**
     * @brief Read digital level from a GPIO pin
     * @param pin Pin number (0-3)
     * @param high Reference set to true if pin is high, false if low
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed on I2C/register error
     *
     * @note GPIO3 input is active-low per AXP202 datasheet (REG 95H bit0: 0=high, 1=low).
     *       This is handled transparently — high=true means pin is electrically high.
     */
    Status read(uint8_t pin, bool &high) override;

    /**
     * @brief Write digital level to a GPIO pin
     * @param pin Pin number (0-3)
     * @param level Level to set (Low, High, or HiZ)
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed on I2C/register error or unsupported level
     *
     * @note GPIO3 does not support Level::High (hardware limitation).
     *       GPIO3 supports Level::Low (NMOS on) and Level::HiZ (NMOS off/floating).
     */
    Status write(uint8_t pin, Level level) override;

    /**
     * @brief Set GPIO pin function mode
     * @param pin Pin number (0-2 only; GPIO3 has no function multiplexing)
     * @param func Function mode value (see @ref FunctionValues for valid values)
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed if pin=3 or on I2C/register error
     */
    Status setFunction(uint8_t pin, uint8_t func) override;

    /**
     * @brief Get current GPIO pin function mode
     * @param pin Pin number (0-3)
     * @return Function mode value, or 0xFF on error or for GPIO3
     */
    uint8_t getFunction(uint8_t pin) override;

private:
    /**
     * @brief Get the control register address for a given GPIO pin
     * @param pin Pin number (0-3)
     * @return Register address (0x90 for GPIO0, 0x92 for GPIO1, etc.)
     */
    uint8_t getCtrlReg(uint8_t pin) const;

    AXP202Core &_core;
};

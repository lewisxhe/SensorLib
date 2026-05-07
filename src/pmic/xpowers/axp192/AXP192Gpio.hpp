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
 * @file      AXP192Gpio.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP192 GPIO Interface
 *
 * Controls the AXP192's general-purpose I/O pins (GPIO0-GPIO5). Supports
 * direction (input/output), drive strength, signal level read/write, and
 * internal pull-down configuration. GPIO0 can alternatively function as
 * a programmable LDO output (LDOio) for external peripheral power.
 *
 * @note When configured as LDOio, voltage is controlled via the
 *       AXP192Channel class rather than this GPIO interface.
 *
 * @see AXP192Channel.hpp
 * @see AXP192Regs.hpp (gpio struct)
 */
#pragma once
#include "../../PmicGpioBase.hpp"
#include "AXP192Core.hpp"

/**
 * @brief AXP192 GPIO Interface
 *
 * Provides pin-level control for AXP192 GPIO0 through GPIO5, including
 * direction, drive strength, digital read/write, function multiplexing,
 * and pull-down settings.
 *
 * @section FunctionValues GPIO0-2 Function Select Values (REG 90H/92H/93H [2:0])
 * - 0x00: NMOS open-drain output
 * - 0x01: Input
 * - 0x02: Low-noise LDO (GPIO0) / PWM1 output (GPIO1) / PWM2 output (GPIO2)
 * - 0x04: ADC input
 * - 0x05: Output low
 * - 0x06/0x07: Floating (high-impedance)
 *
 * @section GPIO34Func GPIO3-4 Function Select (REG 95H)
 * - bit7=1 enables GPIO mode
 * - GPIO3 [1:0]: 00=ext charge, 01=NMOS out, 10=input, 11=ADC
 * - GPIO4 [3:2]: 00=ext charge, 01=NMOS out, 10=input, 11=undefined
 *
 * @section GPIO5Func GPIO5 / N_RSTO (REG 9EH)
 * - bit7: 0=N_RSTO/LDO1 monitor, 1=GPIO5 mode
 * - bit6: 0=NMOS open-drain output, 1=input
 */
class AXP192Gpio : public PmicGpioBase
{
public:
    /**
     * @brief Construct GPIO interface
     * @param core Reference to AXP192 core communication object
     */
    explicit AXP192Gpio(AXP192Core &core);
    ~AXP192Gpio() = default;

    /**
     * @brief Get number of GPIO pins (6: GPIO0-GPIO5)
     * @return 6
     */
    uint8_t getPinCount() const override;

    /**
     * @brief Set GPIO pin direction
     * @param pin GPIO pin number (0-5)
     * @param dir Direction (Input or Output)
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed on I2C/register error
     */
    Status setDirection(uint8_t pin, Direction dir) override;

    /**
     * @brief Get GPIO pin direction
     * @param pin GPIO pin number (0-5)
     * @return Current direction setting
     */
    Direction getDirection(uint8_t pin) override;

    /**
     * @brief Set GPIO pin drive type
     * @param pin GPIO pin number (0-5)
     * @param drive Drive type
     * @return Status::Failed — AXP192 GPIO drive type is implicit in
     *         function select and cannot be independently configured
     */
    Status setDrive(uint8_t pin, DriveType drive) override;

    /**
     * @brief Get GPIO pin drive type
     * @param pin GPIO pin number (0-5)
     * @return DriveType::OpenDrain — AXP192 GPIOs are NMOS open-drain when in GPIO output mode
     */
    DriveType getDrive(uint8_t pin) override;

    /**
     * @brief Read digital level of a GPIO pin
     * @param pin GPIO pin number (0-5)
     * @param[out] high true if pin is high, false if low
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed on I2C/register error
     */
    Status read(uint8_t pin, bool &high) override;

    /**
     * @brief Write digital level to a GPIO pin
     * @param pin GPIO pin number (0-5)
     * @param level Level to set (High or Low)
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed on I2C/register error
     */
    Status write(uint8_t pin, Level level) override;

    /**
     * @brief Set GPIO pin function mode
     * @param pin GPIO pin number (0-5)
     * @param func Function mode value (see @ref FunctionValues for valid values)
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed on I2C/register error or unsupported function
     */
    Status setFunction(uint8_t pin, uint8_t func) override;

    /**
     * @brief Get current GPIO pin function mode
     * @param pin GPIO pin number (0-5)
     * @return Function mode value, or 0xFF on error
     */
    uint8_t getFunction(uint8_t pin) override;

    /**
     * @brief Set internal pull-down resistor
     * @param pin GPIO pin number (0-2, only GPIO0-2 support pull-down)
     * @param pull Pull::Down to enable, Pull::None to disable
     * @return Status::Ok on success, Status::InvalidPin for invalid pin,
     *         Status::Failed if pull unsupported or on I2C/register error
     */
    Status setPull(uint8_t pin, Pull pull) override;

    /**
     * @brief Get current pull-down setting
     * @param pin GPIO pin number (0-2)
     * @return Current pull configuration
     */
    Pull getPull(uint8_t pin) override;

    /**
     * @brief Set PWM output frequency for GPIO1 (PWM1) or GPIO2 (PWM2)
     * @param pin GPIO pin number (1 for PWM1, 2 for PWM2)
     * @param x Frequency divisor X (0-255)
     * @return true on success, false if pin is not 1 or 2, or on I2C error
     *
     * @note PWM frequency = 2.25MHz / (X + 1) / Y1
     *       Default X=0 → X+1=1
     * @see setPwmDutyCycle()
     */
    bool setPwmFrequency(uint8_t pin, uint8_t x);

    /**
     * @brief Set PWM duty cycle for GPIO1 (PWM1) or GPIO2 (PWM2)
     * @param pin GPIO pin number (1 for PWM1, 2 for PWM2)
     * @param y1 Duty cycle period Y1 (1-255, must be > y2)
     * @param y2 Duty cycle high-time Y2 (0-255, must be <= y1)
     * @return true on success, false if pin is not 1 or 2, y2 > y1, or on I2C error
     *
     * @note Duty cycle = Y2 / Y1
     *       REG 99H/9CH [7:0] = Y1
     *       REG 9AH/9DH [7:6] = Y2 (only bits [7:6] are used in REG 9AH/9DH!)
     *
     *       Actually per datasheet: Y2 is in REG 9AH[7:6] (2 bits, not 8 bits).
     *       So Y2 range is 0-3 when using REG 9AH/9DH directly.
     *       For fine-grained duty cycle, use the full register layout.
     *
     *       Default: Y1=0x16(22), Y2=0x0B(11) → 50% duty
     * @see setPwmFrequency()
     */
    bool setPwmDutyCycle(uint8_t pin, uint8_t y1, uint8_t y2);

    /**
     * @brief Configure PWM output parameters in one call
     * @param pin GPIO pin number (1 for PWM1, 2 for PWM2)
     * @param x Frequency divisor X (0-255)
     * @param y1 Duty cycle period Y1 (1-255)
     * @param y2 Duty cycle high-time Y2 (0-255, must be <= y1)
     * @return true on success
     *
     * @note Convenience wrapper. Also sets pin function to PWM mode (0x02).
     *       Frequency = 2.25MHz / (X+1) / Y1
     *       Duty cycle = Y2 / Y1
     *
     *       Examples:
     *         setPwm(pin, 0, 100, 50) → 22.5kHz, 50%
     *         setPwm(pin, 0, 100, 10) → 22.5kHz, 10%
     *         setPwm(pin, 4, 100, 50) → 4.5kHz, 50%
     */
    bool setPwm(uint8_t pin, uint8_t x, uint8_t y1, uint8_t y2);

private:
    AXP192Core &_core;
};

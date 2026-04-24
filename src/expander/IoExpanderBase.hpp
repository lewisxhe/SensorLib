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
 * @file      IoExpanderBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-03
 */
#pragma once

#include "SensorPlatform.hpp"

/**
 * @class IoExpanderBase
 * @brief Abstract base class for GPIO expander devices.
 *
 * This class provides a common interface for controlling GPIO expanders
 * with any number of pins. It manages pin mode caching, output state caching,
 * and delegates hardware‑specific operations to pure virtual functions
 * implemented by derived classes.
 */
class IoExpanderBase
{
public:
    /**
     * @brief Construct a new IoExpanderBase object.
     *
     * Allocates internal arrays for pin modes and output states.
     * If allocation fails, _pinCount is set to 0 and subsequent operations
     * will be safely rejected.
     *
     * @param pinCount Number of GPIO pins provided by the expander.
     */
    IoExpanderBase(uint8_t pinCount) :  _pinCount(pinCount)
    {
        _pinModes = std::unique_ptr<uint8_t[]>(new (std::nothrow) uint8_t[pinCount]());
        _outputStates = std::unique_ptr<bool[]>(new (std::nothrow) bool[pinCount]());
        if (!_pinModes || !_outputStates) {
            _pinCount = 0;
        }
    }

    /**
     * @brief Virtual destructor.
     *
     * Ensures proper cleanup of derived classes. The std::unique_ptr members
     * automatically release their memory.
     */
    virtual ~IoExpanderBase() = default;

    /**
    * @brief Deinitialize the expander.
    *
    * Can be overridden by derived classes to perform chip‑specific cleanup
    * (e.g., power‑down, reset). The base implementation does nothing.
    */
    virtual void deinit()
    {
    }

    /**
     * @brief Configure multiple pins as input or output simultaneously.
     *
     * This function sets the direction of all pins specified in the mask to the
     * given mode. It validates the arguments and updates the internal pin mode
     * cache before delegating the hardware configuration to the pure virtual
     * function configPinsImpl(), which must be implemented by derived classes.
     *
     * @param pinMask Bitmask where a 1 indicates the pin should be configured.
     *                Bit 0 corresponds to pin 0, bit 15 to pin 15 (if applicable).
     *                Only the lowest _pinCount bits are considered.
     * @param mode    Desired mode: INPUT or OUTPUT (as defined in the Arduino framework).
     *
     * @note This function logs errors and returns early if:
     *       - The expander is not initialized (comm is null).
     *       - The internal pin mode cache is uninitialized.
     *       - An invalid mode is provided.
     */
    void configPins(uint16_t pinMask, uint8_t mode)
    {
        if (_pinModes == nullptr) {
            log_e("Pin modes array not initialized");
            return;
        }
        if (mode != INPUT && mode != OUTPUT) {
            log_e("Invalid pin mode");
            return;
        }
        configPinsImpl(pinMask, mode);
    }


    /**
    * @brief Set the direction of a GPIO pin.
    *
    * The mode is cached in _pinModes and the hardware is updated via
    * the pure virtual function pinModeImpl().
    *
    * @param pin  Pin number (0 .. pinCount()-1).
    * @param mode Pin mode: INPUT or OUTPUT (as defined in Arduino framework).
    */
    void pinMode(uint8_t pin, uint8_t mode)
    {
        if (pin >= _pinCount) {
            log_e("Invalid pin number");
            return;
        }
        if (_pinModes == nullptr) {
            log_e("Pin modes array not initialized");
            return;
        }
        if (mode != INPUT && mode != OUTPUT) {
            log_e("Invalid pin mode");
            return;
        }

        _pinModes[pin] = mode;
        pinModeImpl(pin, mode);
    }

    /**
    * @brief Write a digital value to an output pin.
    *
    * The value is cached in _outputStates and the hardware is updated via
    * digitalWriteImpl().
    *
    * @param pin   Pin number (0 .. pinCount()-1). Must be configured as OUTPUT.
    * @param value true for HIGH, false for LOW.
    */
    void digitalWrite(uint8_t pin, bool value)
    {
        if (pin >= _pinCount) {
            log_e("Invalid pin number");
            return;
        }
        if (_pinModes == nullptr) {
            log_e("Pin modes array not initialized");
            return;
        }
        if (_outputStates == nullptr) {
            log_e("Output states array not initialized");
            return;
        }
        if (_pinModes[pin] != OUTPUT) {
            log_e("Pin %d is not configured as OUTPUT", pin);
            return;
        }

        _outputStates[pin] = value;
        digitalWriteImpl(pin, value);
    }

    /**
    * @brief Read the digital value from an input pin.
    *
    * Reads the current state from hardware via digitalReadImpl().
    *
    * @param pin Pin number (0 .. pinCount()-1). Must be configured as INPUT.
    * @return true  Pin is HIGH.
    * @return false Pin is LOW (or error, logged).
    */
    bool digitalRead(uint8_t pin)
    {
        if (pin >= _pinCount) {
            log_e("Invalid pin number");
            return false;
        }
        if (_pinModes == nullptr) {
            log_e("Pin modes array not initialized");
            return false;
        }
        if (_pinModes[pin] != INPUT) {
            log_e("Pin %d is not configured as INPUT", pin);
            return false;
        }

        return digitalReadImpl(pin);
    }

    /**
    * @brief Toggle the state of an output pin.
    *
    * Inverts the current cached output state and writes it to the pin.
    *
    * @param pin Pin number (0 .. pinCount()-1). Must be configured as OUTPUT.
    */
    void digitalToggle(uint8_t pin)
    {
        if (!_pinModes || !_outputStates) {
            log_e("Expander not properly initialized");
            return;
        }
        if (_pinModes[pin] != OUTPUT) {
            log_e("Pin %d is not configured as OUTPUT", pin);
            return;
        }
        int state = 1 - _outputStates[pin];
        digitalWrite(pin, state);
    }

    /**
    * @brief Write to multiple pins simultaneously using a mask.
    *
    * The default implementation writes each selected pin individually.
    * Derived classes may override this with a more efficient register‑based
    * write when the hardware supports it.
    *
    * @param mask   Bitmask where a 1 indicates the pin should be updated.
    * @param values Bitmask of the new values for the selected pins.
    */
    virtual void digitalWritePort(uint16_t mask, uint16_t values)
    {
        for (uint8_t i = 0; i < _pinCount; i++) {
            if (mask & (1UL << i)) {
                digitalWrite(i, (values >> i) & 1);
            }
        }
    }

    /**
    * @brief Read the values of all input pins.
    *
    * The default implementation reads each input pin individually.
    * Derived classes may override this to perform a faster register read.
    *
    * @return uint16_t Bitmask where bit i is 1 if input pin i is HIGH.
    */
    virtual uint16_t digitalReadPort()
    {
        uint16_t result = 0;
        if (!_pinModes || !_outputStates) {
            log_e("Expander not properly initialized");
            return 0;
        }
        for (uint8_t i = 0; i < _pinCount; i++) {
            if (_pinModes[i] == INPUT) {
                result |= (digitalRead(i) ? (1UL << i) : 0);
            }
        }
        return result;
    }

    /**
    * @brief Get the total number of pins provided by this expander.
    *
    * @return uint8_t Pin count.
    */
    uint8_t pinCount() const
    {
        return _pinCount;
    }


    /**
     * @brief Convert a pin number to its corresponding bitmask.
     *
     * This utility function returns a 16-bit mask with a single bit set at the
     * position corresponding to the given pin number. The mask can be used with
     * batch functions such as digitalWritePort() or configPins().
     *
     * @param pin Pin number (0 to pinCount()-1).
     * @return uint16_t Bitmask with bit (pin) set to 1. Returns 0 and logs an error
     *                  if the pin number is out of range.
     */
    uint16_t pinToMask(uint8_t pin) const
    {
        if (pin >= _pinCount) {
            log_e("Invalid pin number");
            return 0;
        }
        return (1UL << pin);
    }

protected:
    /**
      * @brief Chip‑specific initialization.
      *
      * Called by all begin() overloads after the communication object is set up.
      * Derived classes should perform any necessary hardware configuration
      * (e.g., reset, verify device ID, set default registers).
      *
      * @param param Opaque parameter passed from begin() (typically I2C address).
      * @return true  Initialization successful.
      * @return false Initialization failed.
      */
    virtual bool initImpl(uint8_t param) = 0;

    /**
     * @brief Hardware‑specific implementation of batch pin mode configuration.
     *
     * @param pinMask Bitmask of pins to configure.
     * @param mode    Desired mode (INPUT or OUTPUT).
     */
    virtual void configPinsImpl(uint16_t pinMask, uint8_t mode) = 0;

    /**
     * @brief Hardware‑specific implementation of pinMode().
     *
     * Called by pinMode() after validating arguments and updating the cache.
     *
     * @param pin  Pin number.
     * @param mode Desired mode (INPUT or OUTPUT).
     */
    virtual void pinModeImpl(uint8_t pin, uint8_t mode) = 0;

    /**
     * @brief Hardware‑specific implementation of digitalWrite().
     *
     * Called by digitalWrite() after validating arguments and updating the cache.
     *
     * @param pin   Pin number.
     * @param value true for HIGH, false for LOW.
     */
    virtual void digitalWriteImpl(uint8_t pin, bool value) = 0;

    /**
     * @brief Hardware‑specific implementation of digitalRead().
     *
     * Called by digitalRead() after validating arguments.
     *
     * @param pin Pin number.
     * @return true  Pin is HIGH.
     * @return false Pin is LOW.
     */
    virtual bool digitalReadImpl(uint8_t pin) = 0;

    uint8_t _pinCount;                      ///< Number of GPIO pins
    std::unique_ptr<uint8_t[]> _pinModes;   ///< Cached pin modes (INPUT/OUTPUT)
    std::unique_ptr<bool[]> _outputStates;  ///< Cached output states (true = HIGH)
};

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
 * @file      IoExpanderPCA9570.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-03
 */

#pragma once

#include "platform/comm/I2CDeviceNoHal.hpp"
#include "IoExpanderBase.hpp"

/** @brief Default I2C slave address for PCA9570 (hardwired, cannot be changed). */
static constexpr uint8_t  PCA9570_SLAVE_ADDRESS = (0x24);

/**
 * @class IoExpanderPCA9570
 * @brief Concrete driver for the PCA9570 4‑bit output‑only I2C GPIO expander.
 *
 * The PCA9570 provides 4 output‑only pins. All pins are configured as outputs
 * by default; pinMode() will accept only OUTPUT. Reading a pin returns the
 * last written value (output register).
 */
class IoExpanderPCA9570 : public IoExpanderBase, public I2CDeviceNoHal
{
public:

    enum Port {
        PORT_ALL = 0x0F
    };
    
    /**
     * @brief Construct a new IoExpanderPCA9570 object.
     *
     * Initializes the base class with the fixed pin count of 4.
     */
    IoExpanderPCA9570() : IoExpanderBase(PCA9570_PINS_COUNT) {}

    /**
     * @brief Destroy the IoExpanderPCA9570 object.
     */
    ~IoExpanderPCA9570() = default;

    /**
    * @brief Write to multiple pins simultaneously using a mask.
    *
    * Reads the current output register, modifies only the bits specified by
    * the mask, and writes the result back. The internal output state cache
    * is also updated.
    *
    * @param mask   Bitmask where a 1 indicates the pin should be updated.
    * @param values Bitmask of the new values for the selected pins.
    */
    void digitalWritePort(uint16_t mask, uint16_t values) override
    {
        // Read current output state
        uint8_t current;
        if (readBuff(&current, 1) < 0) {
            log_e("Failed to read current output state");
            return;
        }

        // Update only the bits specified by mask (mask is 16-bit, but only low 4 bits used)
        uint8_t newVal = current;
        for (uint8_t i = 0; i < PCA9570_PINS_COUNT; i++) {
            if (mask & (1UL << i)) {
                if ((values >> i) & 1) {
                    newVal |= (1 << i);
                } else {
                    newVal &= ~(1 << i);
                }
            }
        }

        // Write back the new value
        if (writeRegBuff(newVal, nullptr, 0) < 0) {
            log_e("Failed to write output state");
            return;
        }

        // Update cache
        for (uint8_t i = 0; i < PCA9570_PINS_COUNT; i++) {
            if (mask & (1UL << i)) {
                _outputStates[i] = (values >> i) & 1;
            }
        }
    }

    /**
     * @brief Read the current output register value.
     *
     * @return uint16_t Low 4 bits contain the output state of pins 0‑3.
     */
    uint16_t digitalReadPort() override
    {
        uint8_t state;
        if (readBuff(&state, 1) < 0) {
            log_e("Failed to read output state");
            return 0;
        }
        return state & 0x0F;  // Only low 4 bits are valid
    }

protected:
    /**
     * @brief Set the direction of a single pin.
     *
     * The PCA9570 only supports output mode; therefore, any request for INPUT mode
     * is rejected with an error. No hardware configuration is performed because
     * all pins are fixed as outputs.
     *
     * @param pin  Pin number (0‑3). (Not used, as all pins behave identically.)
     * @param mode Desired mode. Only OUTPUT is accepted.
     *
     */
    void pinModeImpl(uint8_t pin, uint8_t mode) override
    {
        if (mode != OUTPUT) {
            log_e("Invalid pin mode, only OUTPUT is supported");
            return;
        }
        // No hardware configuration needed; all pins are fixed as outputs.
    }

    /**
    * @brief Hardware‑specific implementation for configuring multiple pins simultaneously.
    *
    * The PCA9570 has fixed output‑only pins; therefore, this function does not
    * perform any hardware configuration. It validates that the requested mode is
    * OUTPUT (as INPUT is not supported) and updates the internal pin mode cache
    * accordingly.
    *
    * @param pinMask Bitmask of pins to configure (ignored, as all pins behave identically).
    * @param mode    Desired mode. Only OUTPUT is accepted; INPUT will log an error.
    *
    * @note This function assumes that comm and _pinModes are valid (checked in the
    *       base class configPins() before calling here).
    */
    void configPinsImpl(uint16_t pinMask, uint8_t mode) override
    {
        // PCA9570 only supports output mode; ignore any input request
        if (mode != OUTPUT) {
            log_e("PCA9570 only supports OUTPUT mode");
            return;
        }
        // No hardware configuration needed; all pins are fixed as outputs.
        // Update the internal cache for all pins specified in the mask.
        for (uint8_t i = 0; i < PCA9570_PINS_COUNT; i++) {
            if (pinMask & (1UL << i)) {
                _pinModes[i] = OUTPUT;
            }
        }
    }

    /**
     * @brief Write a digital value to an output pin.
     *
     * Reads the current output register, updates the specified bit, and writes
     * the new value back.
     *
     * @param pin   Pin number (0‑3).
     * @param value true for HIGH, false for LOW.
     */
    void digitalWriteImpl(uint8_t pin, bool value) override
    {
        uint8_t current = digitalReadPort();  // Read current output state
        if (value) {
            current |= (1 << pin);
        } else {
            current &= ~(1 << pin);
        }
        writeRegBuff(current, nullptr, 0);  // Write back to output register
    }

    /**
     * @brief Read the current output value of a pin.
     *
     * Since PCA9570 has no input capability, this returns the last written value
     * (output register state).
     *
     * @param pin Pin number (0‑3).
     * @return true  Pin is currently set HIGH.
     * @return false Pin is currently set LOW.
     */
    bool digitalReadImpl(uint8_t pin) override
    {
        uint8_t value = digitalReadPort();
        return (value >> pin) & 1;
    }

    /**
     * @brief Perform chip‑specific initialization.
     *
     * Reads the device ID registers to verify communication and logs the
     * manufacturer ID, part ID, and revision.
     *
     * @param param  No use
     * @return true  Initialization successful (device ID read succeeded).
     * @return false Initialization failed.
     */
    bool initImpl(uint8_t param) override
    {
        uint8_t buffer[3] = {};
        if (readRegBuff(PCA9570_DEVICE_ID_READ, buffer, 3) < 0 ) {
            log_e("Failed to read device ID");
            return false;
        }
        uint16_t manufacturerID, partID;
        uint8_t revision;
        manufacturerID = (buffer[0] << 4) | ((buffer[1] >> 4) & 0x0F);  // 12‑bit manufacturer ID
        partID         = ((buffer[1] & 0x0F) << 5) | ((buffer[2] >> 3) & 0x1F); // 9‑bit part ID
        revision       = buffer[2] & 0x07;                               // 3‑bit revision
        log_d("ManufacturerID:0x%03X, PartID:0x%03X, Revision:0x%02X",
              manufacturerID, partID, revision);
        return true;
    }

private:
    // Special addresses and commands (refer to PCA9570 datasheet)
    static constexpr uint8_t PCA9570_GENERAL_CALL_ADDR = 0x00;   ///< General call address (software reset)
    static constexpr uint8_t PCA9570_DEVICE_ID_WRITE   = 0xF8;   ///< Device ID write address
    static constexpr uint8_t PCA9570_DEVICE_ID_READ    = 0xF9;   ///< Device ID read address
    static constexpr uint8_t PCA9570_SOFT_RESET_CMD    = 0x06;   ///< Software reset command (see datasheet §7.2.1)
    static constexpr uint8_t PCA9570_PINS_COUNT        = 4;      ///< Number of GPIO pins on the chip
};
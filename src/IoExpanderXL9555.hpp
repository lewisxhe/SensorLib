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
 * @file      IoExpanderXL9555.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-03
 */
#pragma once

#include "platform/comm/I2CDeviceNoHal.hpp"
#include "IoExpanderBase.hpp"

/**
 * @defgroup XL9555_Constants XL9555 I2C Address Constants
 * @{
 */

/** @brief Unknown / auto‑discovery address marker. */
static constexpr uint8_t  XL9555_UNKNOWN_ADDRESS = (0xFF);

/** @brief I2C slave address when ADDR pins are 000. */
static constexpr uint8_t  XL9555_SLAVE_ADDRESS0 = (0x20);

/** @brief I2C slave address when ADDR pins are 001. */
static constexpr uint8_t  XL9555_SLAVE_ADDRESS1 = (0x21);

/** @brief I2C slave address when ADDR pins are 010. */
static constexpr uint8_t  XL9555_SLAVE_ADDRESS2 = (0x22);

/** @brief I2C slave address when ADDR pins are 011. */
static constexpr uint8_t  XL9555_SLAVE_ADDRESS3 = (0x23);

/** @brief I2C slave address when ADDR pins are 100. */
static constexpr uint8_t  XL9555_SLAVE_ADDRESS4 = (0x24);

/** @brief I2C slave address when ADDR pins are 101. */
static constexpr uint8_t  XL9555_SLAVE_ADDRESS5 = (0x25);

/** @brief I2C slave address when ADDR pins are 110. */
static constexpr uint8_t  XL9555_SLAVE_ADDRESS6 = (0x26);

/** @brief I2C slave address when ADDR pins are 111. */
static constexpr uint8_t  XL9555_SLAVE_ADDRESS7 = (0x27);


/** @} */ // end of XL9555_Constants

#ifdef __GNUC__
/** @brief Attribute to mark a declaration as deprecated with a custom message. */
#define DEPRECATED_ATTR __attribute__((deprecated("use XL9555_UNKNOWN_ADDRESS instead")))
#else
#define DEPRECATED_ATTR
#endif

/**
 * @brief Deprecated spelling of XL9555_UNKNOWN_ADDRESS.
 *
 * This constant is kept for backward compatibility. New code should use
 * XL9555_UNKNOWN_ADDRESS.
 */
static const int XL9555_UNKOWN_ADDRESS DEPRECATED_ATTR = XL9555_UNKNOWN_ADDRESS;



/**
 * @class IoExpanderXL9555
 * @brief Concrete driver for the XL9555 16‑bit I2C GPIO expander.
 *
 * The XL9555 provides 16 bidirectional I/O pins, organized as two 8‑bit ports.
 * This class implements the hardware‑specific operations required by the
 * IoExpanderBase interface using I2C communication.
 */
class IoExpanderXL9555 : public IoExpanderBase, public I2CDeviceNoHal
{
public:
    enum  Port {
        PORT0 = 0x00FF,
        PORT1 = 0xFF00,
        PORT_ALL = 0xFFFF
    };

    enum IoExpanderGPIO {
        IO0  __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO1  __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO2  __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO3  __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO4  __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO5  __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO6  __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO7  __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO8  __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO9  __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO10 __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO11 __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO12 __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO13 __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO14 __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
        IO15 __attribute__((deprecated("Use integer pin numbers (0-15) directly instead."))) ,
    };

    /**
     * @brief Construct a new IoExpanderXL9555 object.
     *
     * Initializes the base class with the fixed pin count of 16.
     */
    IoExpanderXL9555() : IoExpanderBase(XL9555_PINS_COUNT) {}

    /**
     * @brief Destroy the IoExpanderXL9555 object.
     */
    ~IoExpanderXL9555() = default;

    /**
    * @brief Write to multiple pins simultaneously using a mask.
    *
    * This override writes whole output registers when the mask covers
    * an entire port, and updates the internal output state cache.
    *
    * @param mask   Bitmask where a 1 indicates the pin should be updated.
    * @param values Bitmask of the new values for the selected pins.
    */
    void digitalWritePort(uint16_t mask, uint16_t values) override
    {
        uint8_t lowByte = values & 0xFF;
        uint8_t highByte = (values >> 8) & 0xFF;
        if (!_outputStates) {
            log_e("Output states array not initialized");
            return;
        }
        if (mask & 0x00FF) {
            writeReg(XL9555_CTRL_OUTP0, lowByte);
        }
        if (mask & 0xFF00) {
            writeReg(XL9555_CTRL_OUTP1, highByte);
        }
        for (uint8_t i = 0; i < XL9555_PINS_COUNT; i++) {
            if (mask & (1UL << i)) {
                _outputStates[i] = (values >> i) & 1;
            }
        }
    }

    /**
     * @brief Read the values of all 16 input pins as a single 16-bit word.
     *
     * Reads both input registers (INP0 and INP1) and combines them into a 16-bit
     * value where the lower 8 bits correspond to Port 0 (pins 0‑7) and the upper
     * 8 bits correspond to Port 1 (pins 8‑15).
     *
     * @return uint16_t Bitmask of all input pin states. Bit 0 is pin 0, bit 15 is pin 15.
     *                  Returns 0 and logs an error if the expander is not initialized.
     */
    uint16_t digitalReadPort() override
    {
        uint16_t result = 0;
        uint8_t lowByte = readReg(XL9555_CTRL_INP0);
        uint8_t highByte = readReg(XL9555_CTRL_INP1);
        result = (highByte << 8) | lowByte;
        return result;
    }

protected:
    /**
     * @brief Set the direction of a single pin.
     *
     * This function configures the corresponding bit in the configuration
     * register (CFG0 for pins 0‑7, CFG1 for pins 8‑15). A 0 in the
     * configuration register means output, a 1 means input.
     *
     * @param pin  Pin number (0‑15).
     * @param mode Desired mode: OUTPUT (0) or INPUT (1).
     */
    void pinModeImpl(uint8_t pin, uint8_t mode) override
    {
        if (pin >= XL9555_PINS_COUNT) {
            log_e("Invalid pin number, pin range is 0-%d", XL9555_PINS_COUNT - 1);
            return;
        }
        uint8_t reg = (pin < 8) ? XL9555_CTRL_CFG0 : XL9555_CTRL_CFG1;
        uint8_t bit = pin % 8;
        if (mode == OUTPUT) {
            clrRegBit(reg, bit);
        } else {
            setRegBit(reg, bit);
        }
    }


    /**
     * @brief Hardware‑specific implementation for configuring multiple pins simultaneously.
     *
     * This function modifies the configuration registers (CFG0/CFG1) of the XL9555.
     * It reads the current register values, updates only the bits specified in
     * pinMask, and writes the new values back. This ensures that the direction of
     * pins not listed in the mask remains unchanged.
     *
     * After updating the hardware, the internal pin mode cache (_pinModes) is also
     * updated for consistency, so that subsequent individual pin operations see the
     * correct cached state.
     *
     * @param pinMask Bitmask of pins to configure. Bits 0‑7 correspond to Port 0
     *                (pins 0‑7), bits 8‑15 correspond to Port 1 (pins 8‑15).
     * @param mode    Desired mode: INPUT or OUTPUT.
     *
     * @note This function assumes that comm and _pinModes are valid (they are
     *       checked in the base class configPins() before calling here).
     */
    void configPinsImpl(uint16_t pinMask, uint8_t mode) override
    {
        uint8_t lowMask  = pinMask & 0xFF;          // Bits for Port 0 (pins 0‑7)
        uint8_t highMask = (pinMask >> 8) & 0xFF;   // Bits for Port 1 (pins 8‑15)

        // --- Configure Port 0 (CFG0) ---
        if (lowMask) {
            // Read current configuration
            uint8_t cur = readReg(XL9555_CTRL_CFG0);
            // Modify only the bits specified in lowMask
            if (mode == INPUT) {
                cur |= lowMask;          // Set bits to 1 for INPUT
            } else {
                cur &= ~lowMask;         // Clear bits to 0 for OUTPUT
            }
            // Write back updated value
            writeReg(XL9555_CTRL_CFG0, cur);

            // Update the internal pin mode cache for Port 0 pins
            for (uint8_t i = 0; i < 8; i++) {
                if (lowMask & (1 << i)) {
                    _pinModes[i] = mode;
                }
            }
        }

        // --- Configure Port 1 (CFG1) ---
        if (highMask) {
            uint8_t cur = readReg(XL9555_CTRL_CFG1);
            if (mode == INPUT) {
                cur |= highMask;
            } else {
                cur &= ~highMask;
            }
            writeReg(XL9555_CTRL_CFG1, cur);

            // Update the internal pin mode cache for Port 1 pins
            for (uint8_t i = 0; i < 8; i++) {
                if (highMask & (1 << i)) {
                    _pinModes[i + 8] = mode;
                }
            }
        }
    }

    /**
     * @brief Write a digital value to an output pin.
     *
     * Updates the output register (OUTP0 for pins 0‑7, OUTP1 for pins 8‑15)
     * accordingly.
     *
     * @param pin   Pin number (0‑15).
     * @param value true for HIGH, false for LOW.
     */
    void digitalWriteImpl(uint8_t pin, bool value) override
    {
        if (pin >= XL9555_PINS_COUNT) {
            log_e("Invalid pin number, pin range is 0-%d", XL9555_PINS_COUNT - 1);
            return;
        }
        uint8_t reg = (pin < 8) ? XL9555_CTRL_OUTP0 : XL9555_CTRL_OUTP1;
        uint8_t bit = pin % 8;
        if (value) {
            setRegBit(reg, bit);
        } else {
            clrRegBit(reg, bit);
        }
    }

    /**
     * @brief Read the digital value from an input pin.
     *
     * Reads the input register (INP0 for pins 0‑7, INP1 for pins 8‑15).
     *
     * @param pin Pin number (0‑15).
     * @return true  Pin is HIGH.
     * @return false Pin is LOW.
     */
    bool digitalReadImpl(uint8_t pin) override
    {
        if (pin >= XL9555_PINS_COUNT) {
            log_e("Invalid pin number, pin range is 0-%d", XL9555_PINS_COUNT - 1);
            return false;
        }
        if (!ensureValid()) {
            return false;
        }
        uint8_t reg = (pin < 8) ? XL9555_CTRL_INP0 : XL9555_CTRL_INP1;
        uint8_t bit = pin % 8;
        uint8_t value = readReg(reg);
        return (value >> bit) & 1;
    }

    /**
     * @brief Perform chip‑specific initialization.
     *
     * If the provided address is XL9555_UNKNOWN_ADDRESS, this function
     * attempts to auto‑discover the device by scanning the standard XL9555
     * I2C addresses (0x20‑0x27). Otherwise, it verifies communication by
     * reading the input port 0 register.
     *
     * @param param I2C device address, or XL9555_UNKNOWN_ADDRESS for auto‑discovery.
     * @return true  Initialization successful.
     * @return false Initialization failed (device not found or communication error).
     */
    bool initImpl(uint8_t param) override
    {
        if (param == XL9555_UNKNOWN_ADDRESS) {
            log_d("Try to automatically discover the device");
            for (uint8_t address = XL9555_SLAVE_ADDRESS0; address <= XL9555_SLAVE_ADDRESS7; ++address) {
                setAddress(address);
                log_d("Try to use 0x%02x address.", address);
                if (readReg(XL9555_CTRL_INP0) != -1) {
                    log_d("Found the xl9555 chip address is 0x%X", address);
                    return true;
                }
            }
            log_e("No found xl9555 chip ...");
            return false;
        }
        if (readReg(XL9555_CTRL_INP0) < 0 ) {
            return false;
        }
        return true;
    }

private:
    // Register addresses (as per XL9555 datasheet)
    static constexpr uint8_t XL9555_CTRL_INP0   = (0x00);  ///< Input Port 0  /R
    static constexpr uint8_t XL9555_CTRL_INP1   = (0x01);  ///< Input Port 1  /R
    static constexpr uint8_t XL9555_CTRL_OUTP0  = (0x02);  ///< Output Port 0 /RW
    static constexpr uint8_t XL9555_CTRL_OUTP1  = (0x03);  ///< Output Port 1 /RW
    static constexpr uint8_t XL9555_CTRL_PIP0   = (0x04);  ///< Polarity Inversion Port 0 /RW
    static constexpr uint8_t XL9555_CTRL_PIP1   = (0x05);  ///< Polarity Inversion Port 1 /RW
    static constexpr uint8_t XL9555_CTRL_CFG0   = (0x06);  ///< Configuration Port 0 /RW
    static constexpr uint8_t XL9555_CTRL_CFG1   = (0x07);  ///< Configuration Port 1 /RW
    static constexpr uint8_t XL9555_PINS_COUNT  = 16;      ///< Number of GPIO pins on the chip
};

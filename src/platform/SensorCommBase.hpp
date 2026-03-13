/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
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
 * @file      SensorCommBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 * @note
 *      This header provides utility functions and base classes for sensor
 *      communication and hardware abstraction.
 *
 *      It includes:
 *        - A fallback std::make_unique for C++11.
 *        - A compile-time array size helper.
 *        - Base parameter classes for I2C and SPI settings.
 *        - Abstract interface for sensor communication (SensorCommBase).
 *        - Hardware abstraction layers (SensorHalCustom and SensorHal).
 *
 */
#pragma once
#include "SensorLib.h"
#include <memory>
#include "SensorErrorMixin.hpp"

//--------------------------------------------------------------------------
// C++11 compatibility: provide std::make_unique if not yet in the standard.
//--------------------------------------------------------------------------
#if __cplusplus == 201103L
namespace std
{
/**
 * @brief  Creates a unique_ptr for a given type, forwarding constructor arguments.
 * @tparam T        Type of the object to create.
 * @tparam Args     Types of the constructor arguments.
 * @param  args     Arguments to forward to T's constructor.
 * @return std::unique_ptr<T> owning the newly created object.
 * @note   This is a C++11 fallback; from C++14 onward std::make_unique is part of the standard.
 */
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}
#endif

//--------------------------------------------------------------------------
// Compile‑time array size utility
//--------------------------------------------------------------------------
/**
 * @brief  Returns the number of elements in a C‑style array at compile time.
 * @tparam T    Element type.
 * @tparam N    Array size (deduced).
 * @param  arr  Reference to the array.
 * @return constexpr size_t  Number of elements (N).
 */
template <typename T, size_t N>
constexpr size_t arraySize(const T (&arr)[N])
{
    return N;
}

//============================================================================
// Communication parameter base classes
//============================================================================

/**
 * @brief  Abstract base for all communication parameter objects.
 *         Used to pass configuration (I2C, SPI, etc.) to a SensorCommBase instance.
 */
class CommParamsBase
{
public:
    virtual ~CommParamsBase() = default;
};

//--------------------------------------------------------------------------
// I2C specific parameters
//--------------------------------------------------------------------------
/**
 * @brief  I2C communication parameters.
 *         Encapsulates a type (which setting to change) and a value.
 */
class I2CParam : public CommParamsBase
{
public:
    /**
     * @brief  Identifiers for the various I2C settings that can be changed.
     */
    enum I2CParamType {
        I2C_SET_ADDR,            //!< Set device address
        I2C_SET_FLAG,            //!< Set a generic flag (interpretation depends on driver)
        I2C_SET_CLOCK,           //!< Set I2C clock frequency
        I2C_SET_WRITE_DELAY_US,  //!< Delay after write operations (microseconds)
        I2C_SET_READ_DELAY_US,   //!< Delay after read operations (microseconds)
    };

    /**
     * @brief  Construct an I2C parameter object.
     * @param type   Which parameter to change (see I2CParamType).
     * @param params The value associated with the parameter.
     */
    I2CParam(I2CParamType type, uint32_t params) : type(type), params(params) {}

    /**
     * @brief  Retrieve the stored parameter value.
     * @return uint32_t  The parameter value.
     */
    uint32_t getParams() const
    {
        return params;
    }

    /**
     * @brief  Retrieve the parameter type.
     * @return uint8_t  The I2CParamType (cast to uint8_t).
     */
    uint8_t getType() const
    {
        return type;
    }

protected:
    enum I2CParamType type;   //!< Which I2C parameter this object represents.
    uint32_t params;          //!< The value of the parameter.
};

//--------------------------------------------------------------------------
// SPI settings emulation for non‑Arduino environments
//--------------------------------------------------------------------------
#if !defined(ARDUINO)

// Standard SPI mode constants (mirror Arduino's definitions)
#define SPI_MODE0 0   //!< CPOL = 0, CPHA = 0
#define SPI_MODE1 1   //!< CPOL = 0, CPHA = 1
#define SPI_MODE2 2   //!< CPOL = 1, CPHA = 0
#define SPI_MODE3 3   //!< CPOL = 1, CPHA = 1

#define SPI_LSB 0     //!< Least significant bit first
#define SPI_MSB 1     //!< Most significant bit first

/**
 * @brief  Emulation of Arduino's SPISettings class.
 *         Bundles clock speed, bit order, and SPI mode.
 */
class SPISettings
{
public:
    SPISettings() : clock(1000000), bitOrder(SPI_MSB), dataMode(SPI_MODE0) {}
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) :
        clock(clock), bitOrder(bitOrder), dataMode(dataMode) {}

    uint32_t clock;    //!< SPI clock frequency in Hz.
    uint8_t  bitOrder; //!< SPI_LSB or SPI_MSB.
    uint8_t  dataMode; //!< SPI_MODE0 .. SPI_MODE3.
};
#endif

//--------------------------------------------------------------------------
// SPI specific parameters
//--------------------------------------------------------------------------
/**
 * @brief  SPI communication parameters.
 *         Wraps an SPISettings object.
 */
class SPIParam : public CommParamsBase
{
private:
    SPISettings setting;   //!< The SPI settings (clock, bit order, mode).

public:
    /**
     * @brief  Construct an SPI parameter object from an SPISettings.
     * @param setting  The SPI configuration to use.
     */
    SPIParam(SPISettings setting) : setting(setting) {}

    /**
     * @brief  Retrieve the stored SPI settings.
     * @return SPISettings  The current SPI configuration.
     */
    SPISettings getSetting() const
    {
        return setting;
    }
};

//============================================================================
// Abstract sensor communication interface
//============================================================================
// Note: SensorErrorMixin is assumed to be defined elsewhere and provides
//       error handling / status reporting.

/**
 * @brief  Abstract base class for low‑level sensor communication.
 *         Defines the interface for reading/writing registers, buffers,
 *         and performing combined transactions.
 */
class SensorCommBase : public SensorErrorMixin
{
public:
    //----------------------------------------------------------------------
    // Initialisation / deinitialisation
    //----------------------------------------------------------------------
    /**
     * @brief  Initialise the communication interface (open device, etc.).
     * @return true if successful, false otherwise.
     */
    virtual bool init() = 0;

    /**
     * @brief  Deinitialise the communication interface (close device, etc.).
     */
    virtual void deinit() = 0;

    //----------------------------------------------------------------------
    // Read operations
    //----------------------------------------------------------------------
    /**
     * @brief  Read a single byte from a register.
     * @param reg  Register address.
     * @return int  The byte value on success, or a negative error code.
     */
    virtual int readRegister(const uint8_t reg)
    {
        int err = 0;
        uint8_t value = 0x00;
        err = readRegister(reg, &value, 1);
        if (err < 0) {
            return err; // Propagate read error
        }
        return value;
    }

    /**
     * @brief  Read multiple bytes from a register into a buffer.
     * @param reg  Register address.
     * @param buf  Destination buffer.
     * @param len  Number of bytes to read.
     * @return int  Number of bytes read on success, negative error code otherwise.
     */
    virtual int readRegister(const uint8_t reg, uint8_t *buf, size_t len) = 0;

    /**
     * @brief  Read bytes directly from the device (no register address sent).
     * @param buf  Destination buffer.
     * @param len  Number of bytes to read.
     * @return int  Number of bytes read on success, negative error code otherwise.
     */
    virtual int readBuffer(uint8_t *buf, size_t len) = 0;

    //----------------------------------------------------------------------
    // Write operations
    //----------------------------------------------------------------------
    /**
     * @brief  Write a single byte to a register.
     * @param reg  Register address.
     * @param val  Value to write.
     * @return int  0 on success, negative error code otherwise.
     */
    virtual int writeRegister(const uint8_t reg, uint8_t val)
    {
        return writeRegister(reg, &val, 1);
    }

    /**
     * @brief  Write multiple bytes to a register.
     * @param reg  Register address.
     * @param buf  Source buffer containing data to write.
     * @param len  Number of bytes to write.
     * @return int  Number of bytes written on success, negative error code otherwise.
     */
    virtual int writeRegister(const uint8_t reg, uint8_t *buf, size_t len) = 0;

    /**
     * @brief  Modify a register using a NOR and OR mask.
     *         Typically implemented as: new_val = (old_val & norVal) | orVal.
     * @param reg     Register address.
     * @param norVal  Bits to clear (NOR mask).
     * @param orVal   Bits to set (OR mask).
     * @return int    0 on success, negative error code otherwise.
     */
    virtual int writeRegister(const uint8_t reg, uint8_t norVal, uint8_t orVal)
    {
        int val = readRegister(reg);
        if (val < 0) {
            return val; // Propagate read error
        }
        val &= norVal;
        val |= orVal;
        return writeRegister(reg, reinterpret_cast<uint8_t *>(&val), 1);
    }

    /**
     * @brief  Write a buffer directly to the device (no register address).
     * @param buffer  Source buffer.
     * @param len     Number of bytes to write.
     * @return int    Number of bytes written on success, negative error code otherwise.
     */
    virtual int writeBuffer(uint8_t *buffer, size_t len) = 0;

    //----------------------------------------------------------------------
    // Combined transactions
    //----------------------------------------------------------------------
    /**
     * @brief  Perform a write followed by a read (e.g., for register‑based sensors).
     * @param write_buffer  Data to write (usually register address + optional command).
     * @param write_len     Length of write buffer.
     * @param read_buffer   Buffer to store read data.
     * @param read_len      Number of bytes to read.
     * @return int          Number of bytes read on success, negative error code otherwise.
     */
    virtual int writeThenRead(const uint8_t *write_buffer, size_t write_len,
                              uint8_t *read_buffer, size_t read_len) = 0;

    //----------------------------------------------------------------------
    // Bit‑level register manipulation
    //----------------------------------------------------------------------
    /**
     * @brief  Set a specific bit in a register.
     * @param reg  Register address.
     * @param bit  Bit index (0‑based, LSB).
     * @return true if the operation succeeded, false otherwise.
     */
    virtual bool setRegisterBit(const uint8_t reg, uint8_t bit)
    {
        int value = readRegister(reg);
        if (value < 0)
            return false;
        value |= (1 << bit);
        return writeRegister(reg, reinterpret_cast<uint8_t *>(&value), 1) == 0;
    }

    /**
     * @brief  Clear a specific bit in a register.
     * @param reg  Register address.
     * @param bit  Bit index (0‑based, LSB).
     * @return true if the operation succeeded, false otherwise.
     */
    virtual bool clrRegisterBit(const uint8_t reg, uint8_t bit)
    {
        int value = readRegister(reg);
        if (value < 0)
            return false;
        value &= ~(1 << bit);
        return writeRegister(reg, reinterpret_cast<uint8_t *>(&value), 1) == 0;
    }

    /**
     * @brief  Read the state of a specific bit in a register.
     * @param reg  Register address.
     * @param bit  Bit index (0‑based, LSB).
     * @return true if the bit is set, false if cleared or on error.
     */
    virtual bool getRegisterBit(const uint8_t reg, uint8_t bit)
    {
        int value = readRegister(reg);
        if (value < 0)
            return false;
        return (value & (1 << bit)) != 0;
    }

    /**
     * @brief  Update specific bits in a register.
     * @param reg  Register address.
     * @param mask Bitmask to select bits to update.
     * @param value_shifted New values for the selected bits (must be shifted into position).
     * @return int  0 on success, negative error code otherwise.
     */
    virtual int updateBits(uint8_t reg, uint8_t mask, uint8_t value_shifted)
    {
        int cur = readRegister(reg);
        if (cur < 0) return cur; // Propagate read error
        uint8_t newv = (uint8_t(cur) & ~mask) | (value_shifted & mask);
        return writeRegister(reg, newv);
    }

    //----------------------------------------------------------------------
    // Parameter configuration
    //----------------------------------------------------------------------
    /**
     * @brief  Apply communication parameters (e.g., I2C address, SPI settings).
     * @param params  A CommParamsBase derived object (I2CParam or SPIParam).
     */
    virtual void setParams(const CommParamsBase &params) = 0;

    //----------------------------------------------------------------------
    // Destructor
    //----------------------------------------------------------------------
    virtual ~SensorCommBase() = default;
};

//============================================================================
// Hardware abstraction layers
//============================================================================

/**
 * @brief  Customisation point for low‑level GPIO callbacks.
 *         Allows injection of user‑defined functions for pin manipulation.
 */
class SensorHalCustom
{
public:
    // Type aliases for callback signatures
    using CustomWrite = void(*)(uint8_t pin, uint8_t level);   //!< Set pin level (0/1)
    using CustomRead  = uint8_t(*)(uint8_t pin);               //!< Read pin level
    using CustomMode  = void(*)(uint8_t pin, uint8_t mode);    //!< Set pin mode (INPUT/OUTPUT, etc.)

    SensorHalCustom() : writeCallback(nullptr), readCallback(nullptr), modeCallback(nullptr) {}

    /**
     * @brief  Set a custom digitalWrite implementation.
     * @param callback  Function pointer to be used as digitalWrite.
     */
    void setCustomWrite(CustomWrite callback)
    {
        writeCallback = callback;
    }

    /**
     * @brief  Set a custom digitalRead implementation.
     * @param callback  Function pointer to be used as digitalRead.
     */
    void setCustomRead(CustomRead callback)
    {
        readCallback = callback;
    }

    /**
     * @brief  Set a custom pinMode implementation.
     * @param callback  Function pointer to be used as pinMode.
     */
    void setCustomMode(CustomMode callback)
    {
        modeCallback = callback;
    }

protected:
    CustomWrite writeCallback;  //!< Custom digitalWrite, nullptr if not set.
    CustomRead  readCallback;   //!< Custom digitalRead,  nullptr if not set.
    CustomMode  modeCallback;   //!< Custom pinMode,      nullptr if not set.
};

/**
 * @brief  Abstract hardware abstraction layer.
 *         Provides the basic I/O and timing functions required by sensor drivers.
 */
class SensorHal : public SensorHalCustom
{
public:
    /**
     * @brief  Set the mode of a digital pin.
     * @param pin   Pin number.
     * @param mode  Mode (e.g., INPUT, OUTPUT, INPUT_PULLUP).
     */
    virtual void pinMode(uint8_t pin, uint8_t mode) = 0;

    /**
     * @brief  Write a digital value to a pin.
     * @param pin    Pin number.
     * @param level  Logic level (0 = LOW, 1 = HIGH).
     */
    virtual void digitalWrite(uint8_t pin, uint8_t level) = 0;

    /**
     * @brief  Read the digital value from a pin.
     * @param pin  Pin number.
     * @return uint8_t  The read level (0 = LOW, 1 = HIGH).
     */
    virtual uint8_t digitalRead(uint8_t pin) = 0;

    /**
     * @brief  Get the current system time in milliseconds.
     * @return uint32_t  Milliseconds since some epoch (e.g., boot).
     */
    virtual uint32_t millis() = 0;

    /**
     * @brief  Blocking delay for a given number of milliseconds.
     * @param ms  Delay duration in milliseconds.
     */
    virtual void delay(uint32_t ms) = 0;

    /**
     * @brief  Blocking delay for a given number of microseconds.
     * @param us  Delay duration in microseconds.
     */
    virtual void delayMicroseconds(uint32_t us) = 0;
};

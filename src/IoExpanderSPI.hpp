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
 * @file      IoExpanderSPI.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-03
 */
#pragma once

#include "IoExpanderBase.hpp"

/**
 * @class IoExpanderSPI
 * @brief Software SPI master class for driving GPIO expanders via bit-banging.
 *
 * This class implements a software SPI master using the pin control functions
 * provided by any IoExpanderBase-derived class. It allows expanders without
 * dedicated hardware SPI to be controlled via general-purpose I/O pins.
 */
class IoExpanderSPI
{
public:
    /**
     * @brief Construct a new IoExpanderSPI object.
     *
     * Initializes all pin numbers to -1 (unused) and bit order to 0 (MSB first).
     */
    IoExpanderSPI(): _mosi(-1),  _sck(-1), _miso(-1), _order(0), _cs(-1) {}

    /**
     * @brief Destroy the IoExpanderSPI object.
     */
    ~IoExpanderSPI() = default;

    /**
     * @brief Initialize the software SPI interface.
     *
     * Configures the specified pins for SPI operation using the provided
     * expander instance. MOSI and SCK are mandatory and set as outputs;
     * CS is optional (if not -1) and set as output high; MISO is optional
     * and set as input if provided.
     *
     * @param base Reference to the IoExpanderBase instance used for pin control.
     * @param mosi MOSI pin number (must be specified).
     * @param miso MISO pin number (set to -1 if not used).
     * @param sck  SCK pin number (must be specified).
     * @param cs   Chip select pin number (set to -1 if not used).
     * @return true  Initialization successful.
     * @return false Initialization failed (e.g., missing MOSI or SCK).
     */
    bool beginSPI(IoExpanderBase &base, int mosi, int miso, int sck, int cs)
    {
        _base = &base;
        _mosi = mosi;
        _miso = miso;
        _sck = sck;
        _cs = cs;

        if (_mosi == -1 || sck == -1) {
            log_e("MOSI and SCK pins must be specified");
            return false;
        }

        _base->pinMode(_mosi, OUTPUT);
        _base->pinMode(_sck, OUTPUT);

        if (_cs != -1) {
            _base->pinMode(_cs, OUTPUT);
            _base->digitalWrite(_cs, HIGH);
        }

        if (_miso != -1) {
            _base->pinMode(_miso, INPUT);
        }
        _base->digitalWrite(_mosi, HIGH);
        _base->digitalWrite(_sck, HIGH);
        return true;
    }

    /**
     * @brief Set the bit order for SPI transfers.
     *
     * @param order 0 for MSB first, 1 for LSB first. Only the least significant
     *              bit is used; any other bits are ignored.
     */
    void setBitOrder(uint8_t order)
    {
        _order = order & 1;
    }

    /**
     * @brief Transfer 8 bits over SPI.
     *
     * @param val 8-bit data to send.
     * @return uint8_t 8-bit data received.
     */
    uint8_t transfer8(uint8_t val)
    {
        return transferDataBits(val, 8);
    }

    /**
     * @brief Transfer 9 bits over SPI.
     *
     * Useful for devices requiring 9-bit transfers (e.g., certain sensors).
     *
     * @param val 9-bit data to send (only lower 9 bits are used).
     * @return uint16_t 9-bit data received (only lower 9 bits are valid).
     */
    uint16_t transfer9(uint16_t val)
    {
        return transferDataBits(val, 9);
    }

    /**
     * @brief Transfer an arbitrary number of bits over SPI.
     *
     * Performs a full-duplex software SPI transfer of the specified number of bits.
     * The data is shifted out MSB-first or LSB-first according to the current bit order.
     *
     * @param val  Data to send (only the lower 'bits' bits are used).
     * @param bits Number of bits to transfer (1-32).
     * @return uint32_t Data received (only the lower 'bits' bits are valid).
     */
    uint32_t transferDataBits(uint32_t val, uint32_t bits)
    {
        if (!_base) {
            log_e("SPI not initialized (call beginSPI first)");
            return 0;
        }
        uint32_t out = 0;
        if (_cs != -1) _base->digitalWrite(_cs, LOW);
        for (uint32_t i = 0; i < bits; i++) {
            _base->digitalWrite(_sck, LOW);
            out <<= 1;
            if (_miso != -1 && _base->digitalRead(_miso)) out |= 0x01;
            bool bitVal;
            if (_order == 0) // MSB first
                bitVal = (val >> (bits - 1 - i)) & 1;
            else             // LSB first
                bitVal = (val >> i) & 1;
            _base->digitalWrite(_mosi, bitVal ? HIGH : LOW);
            _base->digitalWrite(_sck, HIGH);
        }
        if (_cs != -1) _base->digitalWrite(_cs, HIGH);
        return out;
    }

private:
    int _mosi;          ///< MOSI pin number
    int _sck;           ///< SCK pin number
    int _miso;          ///< MISO pin number (-1 if unused)
    int _order;         ///< Bit order: 0 = MSB first, 1 = LSB first
    int _cs;            ///< Chip select pin number (-1 if unused)
    IoExpanderBase *_base;  ///< Pointer to the expander used for pin I/O
};
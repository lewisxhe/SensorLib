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
 * @file      SensorLibUtils.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-12
 */
#pragma once

#include <cstdint>
#include <type_traits>

namespace SensorLib {

//============================================================================
// Bit manipulation functions
//============================================================================

inline constexpr uint32_t makeBit(uint8_t b) { return 1UL << static_cast<uint32_t>(b); }

template<typename T>
inline typename std::enable_if<sizeof(T) <= 2, uint8_t>::type
getLowByte(T w) { return static_cast<uint8_t>(w & 0xFF); }

template<typename T>
inline typename std::enable_if<sizeof(T) == 4, uint8_t>::type
getLowByte(T w) { return static_cast<uint8_t>(w & 0xFF); }

template<typename T>
inline typename std::enable_if<sizeof(T) <= 2, uint8_t>::type
getHighByte(T w) { return static_cast<uint8_t>(w >> 8); }

template<typename T>
inline typename std::enable_if<sizeof(T) == 4, uint8_t>::type
getHighByte(T w) { return static_cast<uint8_t>((w >> 8) & 0xFF); }

template<typename T>
inline typename std::enable_if<std::is_integral<T>::value, bool>::type
getBit(T value, uint8_t bit) { return ((value >> bit) & 0x01) != 0; }

template<typename T>
inline typename std::enable_if<std::is_integral<T>::value>::type
setBit(T& value, uint8_t b) { value |= static_cast<T>(makeBit(b)); }

template<typename T>
inline typename std::enable_if<std::is_integral<T>::value>::type
clearBit(T& value, uint8_t b) { value &= static_cast<T>(~makeBit(b)); }

template<typename T>
inline typename std::enable_if<std::is_integral<T>::value>::type
toggleBit(T& value, uint8_t b) { value ^= static_cast<T>(makeBit(b)); }

template<typename T>
inline typename std::enable_if<std::is_integral<T>::value>::type
writeBit(T& value, uint8_t b, bool bitvalue)
{
    if (bitvalue) setBit(value, b);
    else clearBit(value, b);
}

template<typename T>
inline typename std::enable_if<std::is_integral<T>::value, bool>::type
checkBit(T value, uint8_t b) { return (value & makeBit(b)) == makeBit(b); }

}
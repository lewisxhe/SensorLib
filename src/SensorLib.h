/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
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
 * @file      SensorLib.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-05
 */

#pragma once

#if defined(ARDUINO)
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#else
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#endif

#include "SensorLib_Version.h"

#if defined(INCLUDE_DEVICES_PINS)
#include "DevicesPins.h"
#endif

#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
#define SPIClass SPIClassRP2040
#endif

// ═══════════════════════════════════════════════════════════════════════════
//  Core utilities in namespace sensorlib
// ═══════════════════════════════════════════════════════════════════════════

namespace sensorlib {

// ── Bit shift helpers (constexpr, type-safe) ────────────────────────────────

constexpr uint32_t _bv(uint8_t b) { return static_cast<uint32_t>(1) << b; }

// ── Byte extraction ─────────────────────────────────────────────────────────

constexpr uint8_t _lowByte(uint16_t w)  { return static_cast<uint8_t>(w & 0xFF); }
constexpr uint8_t _highByte(uint16_t w) { return static_cast<uint8_t>(w >> 8);   }

// ── Bit manipulation (template, type-safe) ──────────────────────────────────

template<typename T>
constexpr T _bitRead(T value, uint8_t bit)
{
    return (value >> bit) & static_cast<T>(1);
}

template<typename T>
inline void _bitSet(T &value, uint8_t bit)
{
    value |= (static_cast<T>(1) << bit);
}

template<typename T>
inline void _bitClear(T &value, uint8_t bit)
{
    value &= ~(static_cast<T>(1) << bit);
}

template<typename T>
inline void _bitToggle(T &value, uint8_t bit)
{
    value ^= (static_cast<T>(1) << bit);
}

template<typename T>
inline void _bitWrite(T &value, uint8_t bit, bool bitvalue)
{
    if (bitvalue) _bitSet(value, bit);
    else          _bitClear(value, bit);
}

template<typename T>
constexpr bool _isBitSet(T value, uint8_t bit)
{
    return (value & (static_cast<T>(1) << bit)) == (static_cast<T>(1) << bit);
}

} // namespace sensorlib

#if !defined(ARDUINO)  && defined(ESP_PLATFORM)

#ifndef INPUT
#define INPUT                 (0x0)
#endif

#ifndef OUTPUT
#define OUTPUT                (0x1)
#endif

#ifndef RISING
#define RISING                (0x01)
#endif

#ifndef FALLING
#define FALLING               (0x02)
#endif

#ifndef LOW
#define LOW                   (0)
#endif

#ifndef HIGH
#define HIGH                  (1)
#endif

#endif

#include "platform/SensorLibLog.hpp"

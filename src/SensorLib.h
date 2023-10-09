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

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#ifdef ARDUINO_ARCH_MBED
// Not supported at the moment
#error The Arduino RP2040 MBED board package is not supported when PIO is used. Use the community package by Earle Philhower.
#endif

#if defined(ARDUINO_ARCH_RP2040)
#define PLATFORM_SPI_TYPE               SPIClassRP2040
#define PLATFORM_WIRE_TYPE              TwoWire
#define SPI_DATA_ORDER  SPI_MSB_FIRST
#define DEFAULT_SDA     (0xFF)
#define DEFAULT_SCL     (0xFF)
#define DEFAULT_SPISETTING  SPISettings()
#elif defined(NRF52840_XXAA) || defined(NRF52832_XXAA)
#define SPI_DATA_ORDER  MSBFIRST
#define DEFAULT_SDA     (0xFF)
#define DEFAULT_SCL     (0xFF)
#define DEFAULT_SPISETTING  SPISettings()
#else
#define PLATFORM_SPI_TYPE               SPIClass
#define PLATFORM_WIRE_TYPE              TwoWire
#define SPI_DATA_ORDER  SPI_MSBFIRST
#define DEFAULT_SDA     (SDA)
#define DEFAULT_SCL     (SCL)
#define DEFAULT_SPISETTING  SPISettings(__freq, __dataOrder, __dataMode);
#endif

enum SensorLibInterface {
    SENSORLIB_SPI_INTERFACE = 1,
    SENSORLIB_I2C_INTERFACE
};

typedef struct __SensorLibPins {
    int irq;
    int rst;
    union   __ {
        struct  {
            int sda;
            int scl;
            int addr;
            PLATFORM_WIRE_TYPE *wire;
        } i2c_dev;
        struct  {
            int cs;
            int miso;
            int mosi;
            int sck;
            PLATFORM_SPI_TYPE *spi;
        } spi_dev;
    } u  ;
    SensorLibInterface intf;
} SensorLibConfigure;




#define SENSOR_PIN_NONE     (-1)
#define DEV_WIRE_NONE       (0)
#define DEV_WIRE_ERR        (-1)
#define DEV_WIRE_TIMEOUT    (-2)




#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

#ifdef _BV
#undef _BV
#endif
#define _BV(b)                          (1UL << (uint32_t)(b))

// #define LOG_PORT Serial
#ifdef LOG_PORT
#define LOG(fmt, ...) LOG_PORT.printf("[%s] " fmt "\n", __func__, ##__VA_ARGS__)
#define LOG_BIN(x)    LOG_PORT.println(x,BIN);
#else
#define LOG(fmt, ...)
#define LOG_BIN(x)
#endif

#ifndef lowByte
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#endif

#ifndef highByte
#define highByte(w) ((uint8_t) ((w) >> 8))
#endif

#ifndef bitRead
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#endif

#ifndef bitSet
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#endif

#ifndef bitClear
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#endif

#ifndef bitToggle
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#endif

#ifndef bitWrite
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))
#endif

#define SENSORLIB_ATTR_NOT_IMPLEMENTED    __attribute__((error("Not implemented")))


#ifndef ESP32
#ifndef log_e
#define log_e(...)          Serial.printf(__VA_ARGS__)
#endif
#ifndef log_i
#define log_i(...)          Serial.printf(__VA_ARGS__)
#endif
#ifndef log_d
#define log_d(...)          Serial.printf(__VA_ARGS__)
#endif
#endif

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






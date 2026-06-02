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
* @file      SensorLibLog.hpp
* @author    Lewis He (lewishe@outlook.com)
* @date      2026-05-30
* @brief     Platform-isolated logging and debug-dump macros for SensorLib.
*/

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <ctype.h>

#if !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)
#define PLATFORM_HAS_PRINTF
#endif

// ─── Arduino (non-MBED, non-Zephyr) ────────────────────────────────────────
#if !defined(ARDUINO_ARCH_ESP32) && defined(LOG_PORT) && defined(ARDUINO) \
    && !defined(ARDUINO_ARCH_MBED) && !defined(ARDUINO_ARCH_ZEPHYR)

#define SENSORLIB_LOG_FILE_LINE  __FILE__, __LINE__

#define SENSORLIB_LOG_E(fmt, ...)  LOG_PORT.printf("[E][%s:%d] " fmt "\n", SENSORLIB_LOG_FILE_LINE, ##__VA_ARGS__)
#define SENSORLIB_LOG_I(fmt, ...)  LOG_PORT.printf("[I][%s:%d] " fmt "\n", SENSORLIB_LOG_FILE_LINE, ##__VA_ARGS__)
#define SENSORLIB_LOG_D(fmt, ...)  LOG_PORT.printf("[D][%s:%d] " fmt "\n", SENSORLIB_LOG_FILE_LINE, ##__VA_ARGS__)
#define SENSORLIB_LOG_W(fmt, ...)  LOG_PORT.printf("[W][%s:%d] " fmt "\n", SENSORLIB_LOG_FILE_LINE, ##__VA_ARGS__)

static inline void _dump_buffer(const uint8_t *buf, size_t size)
{
    const size_t bytesPerLine = 16;
    for (size_t i = 0; i < size; i += bytesPerLine) {
        LOG_PORT.printf("%08x  ", (unsigned int)i);
        for (size_t j = 0; j < bytesPerLine; ++j) {
            if (i + j < size)
                LOG_PORT.printf("%02x ", buf[i + j]);
            else
                LOG_PORT.printf("   ");
            if (j == 7)
                LOG_PORT.printf(" ");
        }
        LOG_PORT.printf(" ");
        for (size_t j = 0; j < bytesPerLine; ++j) {
            if (i + j < size)
                LOG_PORT.printf("%c", isprint(buf[i + j]) ? (char)buf[i + j] : '.');
            else
                LOG_PORT.printf(" ");
        }
        LOG_PORT.printf("\n");
    }
}

#define SENSORLIB_DUMP_BUFFER(buf, size)  _dump_buffer((const uint8_t *)(buf), (size_t)(size))

// ─── Mbed / Zephyr ─────────────────────────────────────────────────────────
#elif defined(ARDUINO_ARCH_MBED) || defined(ARDUINO_ARCH_ZEPHYR)

#define SENSORLIB_LOG_FILE_LINE  __FILE__, __LINE__

#define SENSORLIB_LOG_E(fmt, ...)  printf("[E][%s:%d] " fmt "\n", SENSORLIB_LOG_FILE_LINE, ##__VA_ARGS__)
#define SENSORLIB_LOG_I(fmt, ...)  printf("[I][%s:%d] " fmt "\n", SENSORLIB_LOG_FILE_LINE, ##__VA_ARGS__)
#define SENSORLIB_LOG_D(fmt, ...)  printf("[D][%s:%d] " fmt "\n", SENSORLIB_LOG_FILE_LINE, ##__VA_ARGS__)
#define SENSORLIB_LOG_W(fmt, ...)  printf("[W][%s:%d] " fmt "\n", SENSORLIB_LOG_FILE_LINE, ##__VA_ARGS__)

static inline void _dump_buffer(const uint8_t *buf, size_t size)
{
    const size_t bytesPerLine = 16;
    for (size_t i = 0; i < size; i += bytesPerLine) {
        printf("%08x  ", (unsigned int)i);
        for (size_t j = 0; j < bytesPerLine; ++j) {
            if (i + j < size)
                printf("%02x ", buf[i + j]);
            else
                printf("   ");
            if (j == 7)
                printf(" ");
        }
        printf(" ");
        for (size_t j = 0; j < bytesPerLine; ++j) {
            if (i + j < size)
                printf("%c", isprint(buf[i + j]) ? (char)buf[i + j] : '.');
            else
                printf(" ");
        }
        printf("\n");
    }
}

#define SENSORLIB_DUMP_BUFFER(buf, size)  _dump_buffer((const uint8_t *)(buf), (size_t)(size))



#elif defined(ESP_PLATFORM) && defined(ARDUINO)

#define SENSORLIB_LOG_E(format, ...)  log_e(format, ##__VA_ARGS__)
#define SENSORLIB_LOG_W(format, ...)  log_w(format, ##__VA_ARGS__)
#define SENSORLIB_LOG_I(format, ...)  log_i(format, ##__VA_ARGS__)
#define SENSORLIB_LOG_D(format, ...)  log_d(format, ##__VA_ARGS__)
#define SENSORLIB_LOG_V(format, ...)  log_v(format, ##__VA_ARGS__)


static inline void _dump_buffer(const uint8_t *buf, size_t size)
{
    const size_t bytesPerLine = 16;
    for (size_t i = 0; i < size; i += bytesPerLine) {
        char hex[3 * 16 + 2 + 1] = {0};   // "xx "×16 + " " + NUL
        char ascii[16 + 1] = {0};
        size_t pos_h = 0, pos_a = 0;
        for (size_t j = 0; j < bytesPerLine; ++j) {
            if (i + j < size) {
                pos_h += snprintf(hex + pos_h, sizeof(hex) - pos_h, "%02x ", buf[i + j]);
                ascii[pos_a++] = isprint(buf[i + j]) ? (char)buf[i + j] : '.';
            } else {
                pos_h += snprintf(hex + pos_h, sizeof(hex) - pos_h, "   ");
                ascii[pos_a++] = ' ';
            }
            if (j == 7) {
                hex[pos_h++] = ' ';
                hex[pos_h] = '\0';
            }
        }
        ascii[pos_a] = '\0';
        (void)ascii;
        log_d("%08x  %s %s", (unsigned int)i, hex, ascii);
    }
}

#define SENSORLIB_DUMP_BUFFER(buf, size)  _dump_buffer((const uint8_t *)(buf), (size_t)(size))

// ─── ESP-IDF (native, not Arduino) ─────────────────────────────────────────
#elif defined(ESP_PLATFORM) && !defined(ARDUINO)

#include "esp_log.h"

#define SENSORLIB_LOG_TAG  "SensorLib"

#if defined(__cplusplus) && (__cplusplus > 201703L)
#define SENSORLIB_LOG_E(format, ...)  ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   SENSORLIB_LOG_TAG, format __VA_OPT__(,) __VA_ARGS__)
#define SENSORLIB_LOG_W(format, ...)  ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    SENSORLIB_LOG_TAG, format __VA_OPT__(,) __VA_ARGS__)
#define SENSORLIB_LOG_I(format, ...)  ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    SENSORLIB_LOG_TAG, format __VA_OPT__(,) __VA_ARGS__)
#define SENSORLIB_LOG_D(format, ...)  ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   SENSORLIB_LOG_TAG, format __VA_OPT__(,) __VA_ARGS__)
#define SENSORLIB_LOG_V(format, ...)  ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, SENSORLIB_LOG_TAG, format __VA_OPT__(,) __VA_ARGS__)
#else
#define SENSORLIB_LOG_E(format, ...)  ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR,   SENSORLIB_LOG_TAG, format, ##__VA_ARGS__)
#define SENSORLIB_LOG_W(format, ...)  ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN,    SENSORLIB_LOG_TAG, format, ##__VA_ARGS__)
#define SENSORLIB_LOG_I(format, ...)  ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO,    SENSORLIB_LOG_TAG, format, ##__VA_ARGS__)
#define SENSORLIB_LOG_D(format, ...)  ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG,   SENSORLIB_LOG_TAG, format, ##__VA_ARGS__)
#define SENSORLIB_LOG_V(format, ...)  ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, SENSORLIB_LOG_TAG, format, ##__VA_ARGS__)
#endif

static inline void _dump_buffer(const uint8_t *buf, size_t size)
{
    const size_t bytesPerLine = 16;
    for (size_t i = 0; i < size; i += bytesPerLine) {
        char hex[3 * 16 + 2 + 1] = {0};   // "xx "×16 + " " + NUL
        char ascii[16 + 1] = {0};
        size_t pos_h = 0, pos_a = 0;
        for (size_t j = 0; j < bytesPerLine; ++j) {
            if (i + j < size) {
                pos_h += snprintf(hex + pos_h, sizeof(hex) - pos_h, "%02x ", buf[i + j]);
                ascii[pos_a++] = isprint(buf[i + j]) ? (char)buf[i + j] : '.';
            } else {
                pos_h += snprintf(hex + pos_h, sizeof(hex) - pos_h, "   ");
                ascii[pos_a++] = ' ';
            }
            if (j == 7) {
                hex[pos_h++] = ' ';
                hex[pos_h] = '\0';
            }
        }
        ascii[pos_a] = '\0';
        (void)ascii;
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, SENSORLIB_LOG_TAG,
                            "%08x  %s %s", (unsigned int)i, hex, ascii);
    }
}

#define SENSORLIB_DUMP_BUFFER(buf, size)  _dump_buffer((const uint8_t *)(buf), (size_t)(size))

// ─── Fallback (no-op) ──────────────────────────────────────────────────────
#else

#define SENSORLIB_LOG_E(...)
#define SENSORLIB_LOG_I(...)
#define SENSORLIB_LOG_D(...)
#define SENSORLIB_LOG_W(...)

#define SENSORLIB_DUMP_BUFFER(buf, size)

#endif

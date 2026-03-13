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
 * @file      SensorErr.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-08
 *
 */
#pragma once

#include <stdint.h>

// 0 = OK, negative = error
typedef enum SENSOR_ERROR_CODE : int16_t {
    // OK
    SENSOR_OK = 0,

    // ===== GENERIC / STATE (-100..-199) =====
    SENSOR_ERR = -100,
    SENSOR_ERR_NOT_INITIALIZED = -101,
    SENSOR_ERR_ALREADY_INITIALIZED = -102,
    SENSOR_ERR_INVALID_ARG = -103,
    SENSOR_ERR_OUT_OF_RANGE = -104,
    SENSOR_ERR_NOT_SUPPORTED = -105,
    SENSOR_ERR_BUSY = -106,
    SENSOR_ERR_TIMEOUT = -107,
    SENSOR_ERR_INTERNAL = -108,
    SENSOR_ERR_UNKNOWN = -109,

    // ===== COMMUNICATION (-200..-299) =====
    SENSOR_ERR_COMM = -200,
    SENSOR_ERR_COMM_INIT = -201,
    SENSOR_ERR_COMM_TIMEOUT = -202,
    SENSOR_ERR_COMM_NACK = -203,
    SENSOR_ERR_COMM_ARBITRATION = -204,
    SENSOR_ERR_COMM_BUS = -205,
    SENSOR_ERR_COMM_CRC = -206,
    SENSOR_ERR_COMM_FRAME = -207,
    SENSOR_ERR_COMM_UNAVAILABLE = -208,
    SENSOR_ERR_COMM_DATA_TOO_LONG = -209,

    // ===== DEVICE PROBE / IDENTITY (-300..-399) =====
    SENSOR_ERR_DEVICE_NOT_FOUND = -300,
    SENSOR_ERR_DEVICE_ID_MISMATCH = -301,
    SENSOR_ERR_DEVICE_RESET_FAIL = -302,
    SENSOR_ERR_DEVICE_CONFIG_FAIL = -303,
    SENSOR_ERR_DEVICE_SELF_TEST_FAIL = -304,

    // ===== REGISTER ACCESS (-400..-499) =====
    SENSOR_ERR_REG_READ = -400,
    SENSOR_ERR_REG_WRITE = -401,
    SENSOR_ERR_REG_VERIFY = -402,

    // ===== DATA PATH (-500..-599) =====
    SENSOR_ERR_NO_DATA = -500,
    SENSOR_ERR_DATA_INVALID = -501,
    SENSOR_ERR_DATA_OVERFLOW = -502,
    SENSOR_ERR_DATA_UNDERFLOW = -503,
    SENSOR_ERR_FIFO = -504,
    SENSOR_ERR_CALIBRATION_REQUIRED = -505,
    SENSOR_ERR_CALIBRATION_FAIL = -506,

    // ===== BOSCH SMART SENSOR / FIRMWARE (-600..-699) =====
    SENSOR_ERR_FW_MISSING = -600,
    SENSOR_ERR_FW_INVALID = -601,
    SENSOR_ERR_FW_UPLOAD_FAIL = -602,
    SENSOR_ERR_FW_BOOT_FAIL = -603,
    SENSOR_ERR_FW_VERSION_UNSUPPORTED = -604,
    SENSOR_ERR_FEATURE_NOT_FOUND = -605,
    SENSOR_ERR_FEATURE_ENABLE_FAIL = -606,
    SENSOR_ERR_META_EVENT = -607,

    // ===== TOUCH (-700..-799) =====
    SENSOR_ERR_TOUCH_NO_POINT = -700,
    SENSOR_ERR_TOUCH_COORD_INVALID = -701,
    SENSOR_ERR_TOUCH_RESOLUTION_NOT_SET = -702,
    SENSOR_ERR_TOUCH_CHIPID_MISMATCH = -703,
    SENSOR_ERR_TOUCH_MODE_INVALID = -704,
    SENSOR_ERR_TOUCH_INT_CONFIG = -705,

    // ===== IO EXPANDER / GPIO (-800..-899) =====
    SENSOR_ERR_GPIO_INVALID_PIN = -800,
    SENSOR_ERR_GPIO_INVALID_MODE = -801,
    SENSOR_ERR_GPIO_WRITE_FAIL = -802,
    SENSOR_ERR_GPIO_READ_FAIL = -803,

    // ===== MEMORY / RESOURCES (-900..-999) =====
    SENSOR_ERR_NO_MEMORY = -900,
    SENSOR_ERR_BUFFER_TOO_SMALL = -901,
    SENSOR_ERR_RESOURCE_EXHAUSTED = -902
} SENSOR_ERROR_CODE;

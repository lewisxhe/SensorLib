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
 * @file      SensorErrUtils.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-08
 *
 * SensorErrUtils.hpp - Utility functions for SENSOR_ERROR_CODE handling.
 *
 * Provides:
 * - isOk(), isErr() for success/failure checks
 * - Category checks (isCommErr(), isDeviceErr(), etc.) for telemetry/log routing
 * - toString() to convert code to ALL-CAPS name (for logs/UI)
 * - hint() for optional short human-readable hints (NOT all caps; for docs/serial output)
 */
#pragma once

#include "SensorErr.hpp"

namespace SensorErrUtils
{
    // 0 = OK, negative = error
    static inline bool isOk(SENSOR_ERROR_CODE code)
    {
        return code == SENSOR_OK;
    }

    static inline bool isErr(SENSOR_ERROR_CODE code)
    {
        return code < 0;
    }

    // Category checks (ranges)
    static inline bool isGenericErr(SENSOR_ERROR_CODE code)
    {
        return code <= -100 && code > -200;
    }
    static inline bool isCommErr(SENSOR_ERROR_CODE code)
    {
        return code <= -200 && code > -300;
    }
    static inline bool isDeviceErr(SENSOR_ERROR_CODE code)
    {
        return code <= -300 && code > -400;
    }
    static inline bool isRegErr(SENSOR_ERROR_CODE code)
    {
        return code <= -400 && code > -500;
    }
    static inline bool isDataErr(SENSOR_ERROR_CODE code)
    {
        return code <= -500 && code > -600;
    }
    static inline bool isFwErr(SENSOR_ERROR_CODE code)
    {
        return code <= -600 && code > -700;
    }
    static inline bool isTouchErr(SENSOR_ERROR_CODE code)
    {
        return code <= -700 && code > -800;
    }
    static inline bool isGpioErr(SENSOR_ERROR_CODE code)
    {
        return code <= -800 && code > -900;
    }
    static inline bool isResourceErr(SENSOR_ERROR_CODE code)
    {
        return code <= -900 && code > -1000;
    }

    // Convert to ALL-CAPS token for logging/telemetry/UI
    static inline const char *toString(SENSOR_ERROR_CODE code)
    {
        switch (code) {
        case SENSOR_OK: return "SENSOR_OK";

        case SENSOR_ERR: return "SENSOR_ERR";
        case SENSOR_ERR_NOT_INITIALIZED: return "SENSOR_ERR_NOT_INITIALIZED";
        case SENSOR_ERR_ALREADY_INITIALIZED: return "SENSOR_ERR_ALREADY_INITIALIZED";
        case SENSOR_ERR_INVALID_ARG: return "SENSOR_ERR_INVALID_ARG";
        case SENSOR_ERR_OUT_OF_RANGE: return "SENSOR_ERR_OUT_OF_RANGE";
        case SENSOR_ERR_NOT_SUPPORTED: return "SENSOR_ERR_NOT_SUPPORTED";
        case SENSOR_ERR_BUSY: return "SENSOR_ERR_BUSY";
        case SENSOR_ERR_TIMEOUT: return "SENSOR_ERR_TIMEOUT";
        case SENSOR_ERR_INTERNAL: return "SENSOR_ERR_INTERNAL";
        case SENSOR_ERR_UNKNOWN: return "SENSOR_ERR_UNKNOWN";

        case SENSOR_ERR_COMM: return "SENSOR_ERR_COMM";
        case SENSOR_ERR_COMM_INIT: return "SENSOR_ERR_COMM_INIT";
        case SENSOR_ERR_COMM_TIMEOUT: return "SENSOR_ERR_COMM_TIMEOUT";
        case SENSOR_ERR_COMM_NACK: return "SENSOR_ERR_COMM_NACK";
        case SENSOR_ERR_COMM_ARBITRATION: return "SENSOR_ERR_COMM_ARBITRATION";
        case SENSOR_ERR_COMM_BUS: return "SENSOR_ERR_COMM_BUS";
        case SENSOR_ERR_COMM_CRC: return "SENSOR_ERR_COMM_CRC";
        case SENSOR_ERR_COMM_FRAME: return "SENSOR_ERR_COMM_FRAME";
        case SENSOR_ERR_COMM_UNAVAILABLE: return "SENSOR_ERR_COMM_UNAVAILABLE";

        case SENSOR_ERR_DEVICE_NOT_FOUND: return "SENSOR_ERR_DEVICE_NOT_FOUND";
        case SENSOR_ERR_DEVICE_ID_MISMATCH: return "SENSOR_ERR_DEVICE_ID_MISMATCH";
        case SENSOR_ERR_DEVICE_RESET_FAIL: return "SENSOR_ERR_DEVICE_RESET_FAIL";
        case SENSOR_ERR_DEVICE_CONFIG_FAIL: return "SENSOR_ERR_DEVICE_CONFIG_FAIL";
        case SENSOR_ERR_DEVICE_SELF_TEST_FAIL: return "SENSOR_ERR_DEVICE_SELF_TEST_FAIL";

        case SENSOR_ERR_REG_READ: return "SENSOR_ERR_REG_READ";
        case SENSOR_ERR_REG_WRITE: return "SENSOR_ERR_REG_WRITE";
        case SENSOR_ERR_REG_VERIFY: return "SENSOR_ERR_REG_VERIFY";

        case SENSOR_ERR_NO_DATA: return "SENSOR_ERR_NO_DATA";
        case SENSOR_ERR_DATA_INVALID: return "SENSOR_ERR_DATA_INVALID";
        case SENSOR_ERR_DATA_OVERFLOW: return "SENSOR_ERR_DATA_OVERFLOW";
        case SENSOR_ERR_DATA_UNDERFLOW: return "SENSOR_ERR_DATA_UNDERFLOW";
        case SENSOR_ERR_FIFO: return "SENSOR_ERR_FIFO";
        case SENSOR_ERR_CALIBRATION_REQUIRED: return "SENSOR_ERR_CALIBRATION_REQUIRED";
        case SENSOR_ERR_CALIBRATION_FAIL: return "SENSOR_ERR_CALIBRATION_FAIL";

        case SENSOR_ERR_FW_MISSING: return "SENSOR_ERR_FW_MISSING";
        case SENSOR_ERR_FW_INVALID: return "SENSOR_ERR_FW_INVALID";
        case SENSOR_ERR_FW_UPLOAD_FAIL: return "SENSOR_ERR_FW_UPLOAD_FAIL";
        case SENSOR_ERR_FW_BOOT_FAIL: return "SENSOR_ERR_FW_BOOT_FAIL";
        case SENSOR_ERR_FW_VERSION_UNSUPPORTED: return "SENSOR_ERR_FW_VERSION_UNSUPPORTED";
        case SENSOR_ERR_FEATURE_NOT_FOUND: return "SENSOR_ERR_FEATURE_NOT_FOUND";
        case SENSOR_ERR_FEATURE_ENABLE_FAIL: return "SENSOR_ERR_FEATURE_ENABLE_FAIL";
        case SENSOR_ERR_META_EVENT: return "SENSOR_ERR_META_EVENT";

        case SENSOR_ERR_TOUCH_NO_POINT: return "SENSOR_ERR_TOUCH_NO_POINT";
        case SENSOR_ERR_TOUCH_COORD_INVALID: return "SENSOR_ERR_TOUCH_COORD_INVALID";
        case SENSOR_ERR_TOUCH_RESOLUTION_NOT_SET: return "SENSOR_ERR_TOUCH_RESOLUTION_NOT_SET";
        case SENSOR_ERR_TOUCH_CHIPID_MISMATCH: return "SENSOR_ERR_TOUCH_CHIPID_MISMATCH";
        case SENSOR_ERR_TOUCH_MODE_INVALID: return "SENSOR_ERR_TOUCH_MODE_INVALID";
        case SENSOR_ERR_TOUCH_INT_CONFIG: return "SENSOR_ERR_TOUCH_INT_CONFIG";

        case SENSOR_ERR_GPIO_INVALID_PIN: return "SENSOR_ERR_GPIO_INVALID_PIN";
        case SENSOR_ERR_GPIO_INVALID_MODE: return "SENSOR_ERR_GPIO_INVALID_MODE";
        case SENSOR_ERR_GPIO_WRITE_FAIL: return "SENSOR_ERR_GPIO_WRITE_FAIL";
        case SENSOR_ERR_GPIO_READ_FAIL: return "SENSOR_ERR_GPIO_READ_FAIL";

        case SENSOR_ERR_NO_MEMORY: return "SENSOR_ERR_NO_MEMORY";
        case SENSOR_ERR_BUFFER_TOO_SMALL: return "SENSOR_ERR_BUFFER_TOO_SMALL";
        case SENSOR_ERR_RESOURCE_EXHAUSTED: return "SENSOR_ERR_RESOURCE_EXHAUSTED";

        default: return "SENSOR_ERR_UNKNOWN";
        }
    }

    // Optional: map error code to a stable numeric category (for compact logs)
    static inline int16_t category(SENSOR_ERROR_CODE code)
    {
        if (!isErr(code)) return 0;
        // e.g. -203 -> 200, -701 -> 700 (absolute hundred group)
        int16_t v = (code < 0) ? (int16_t)(-code) : (int16_t)code;
        return (int16_t)((v / 100) * 100);
    }

    // Compatibility helper: turn a bool result + default error into code
    static inline SENSOR_ERROR_CODE fromBool(bool ok, SENSOR_ERROR_CODE errIfFalse = SENSOR_ERR)
    {
        return ok ? SENSOR_OK : errIfFalse;
    }
} // namespace SensorErrUtils

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
 * @file      SensorBhy2Define.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-30
 *
 */
#pragma once

#include "../SensorLib.h"

#define BHI260AP_SLAVE_ADDRESS_L          0x28
#define BHI260AP_SLAVE_ADDRESS_H          0x29

#define BHY2_RLST_CHECK(ret, str, val) \
    do                                 \
    {                                  \
        if (ret)                       \
        {                              \
            log_e(str);                \
            return val;                 \
        }                               \
    } while (0)

typedef void (*BhyEventCb)(uint8_t event, uint8_t sensor_id, uint8_t data);
typedef void (*BhyParseDataCallback)(uint8_t sensor_id, uint8_t *data, uint32_t size, uint64_t *timestamp);
typedef void (*BhyDebugMessageCallback)(const char * message);

typedef struct ParseCallBackList {
    static uint8_t current_id;
    uint8_t id;
    BhyParseDataCallback cb;
    uint32_t data_length;
    uint8_t *data;
    ParseCallBackList() : id(current_id++), cb(NULL),  data_length(0), data(NULL)
    {
    }
} ParseCallBackList_t;

enum BhySensorEvent {
    BHY2_EVENT_FLUSH_COMPLETE           = 1,
    BHY2_EVENT_SAMPLE_RATE_CHANGED,
    BHY2_EVENT_POWER_MODE_CHANGED,
    BHY2_EVENT_ALGORITHM_EVENTS         = 5,
    BHY2_EVENT_SENSOR_STATUS,
    BHY2_EVENT_BSX_DO_STEPS_MAIN,
    BHY2_EVENT_BSX_DO_STEPS_CALIB,
    BHY2_EVENT_BSX_GET_OUTPUT_SIGNAL,
    BHY2_EVENT_SENSOR_ERROR             = 11,
    BHY2_EVENT_FIFO_OVERFLOW,
    BHY2_EVENT_DYNAMIC_RANGE_CHANGED,
    BHY2_EVENT_FIFO_WATERMARK,
    BHY2_EVENT_INITIALIZED              = 16,
    BHY2_EVENT_TRANSFER_CAUSE,
    BHY2_EVENT_SENSOR_FRAMEWORK,
    BHY2_EVENT_RESET,
};






















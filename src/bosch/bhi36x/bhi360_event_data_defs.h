/**
* Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bhi360_event_data_defs.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef __BHI360_EVENT_DATA_DEFS_H__
#define __BHI360_EVENT_DATA_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdio.h>
#include <stdint.h>

#ifndef BHI360_PACKED
#define BHI360_PACKED  __attribute__ ((__packed__))
#endif

struct bhi360_event_data_xyz {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct bhi360_event_data_quaternion {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;
    uint16_t accuracy;
};

struct bhi360_event_data_orientation {
    int16_t heading;
    int16_t pitch;
    int16_t roll;
};

/*! Sensor Structure for Head Orientation Quaternion */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;
} BHI360_PACKED bhi360_event_data_head_orientation_quat;

/*! Sensor Structure for Head Orientation Euler */
typedef struct {
    int16_t heading;
    int16_t pitch;
    int16_t roll;
} BHI360_PACKED bhi360_event_data_head_orientation_eul;

/*!
 * Multi Tap Setting.
 */
typedef enum {
    BHI360_NO_TAP,
    BHI360_SINGLE_TAP,
    BHI360_DOUBLE_TAP,
    BHI360_DOUBLE_SINGLE_TAP,
    BHI360_TRIPLE_TAP,
    BHI360_TRIPLE_SINGLE_TAP,
    BHI360_TRIPLE_DOUBLE_TAP,
    BHI360_TRIPLE_DOUBLE_SINGLE_TAP
} bhi360_event_data_multi_tap;

/*!
 * Multi Tap Output.
 */
static const char *const bhi360_event_data_multi_tap_string_out[] = {
    "NO_TAP",
    "SINGLE_TAP",
    "DOUBLE_TAP",
    "DOUBLE_SINGLE_TAP",
    "TRIPLE_TAP",
    "TRIPLE_SINGLE_TAP",
    "TRIPLE_DOUBLE_TAP",
    "TRIPLE_DOUBLE_SINGLE_TAP"
};

enum bhi360_event_data_wrist_gesture_activity {
    BHI360_NO_GESTURE,
    BHI360_WRIST_SHAKE_JIGGLE = 0x03,
    BHI360_FLICK_IN,
    BHI360_FLICK_OUT
};

typedef struct bhi360_event_data_wrist_gesture_detect {
    enum bhi360_event_data_wrist_gesture_activity wrist_gesture;
} __attribute__ ((packed)) bhi360_event_data_wrist_gesture_detect_t;

static const char *const bhi360_event_data_wrist_gesture_detect_output[] = {
    "NO_GESTURE",
    "WRIST_SHAKE_JIGGLE",
    "FLICK_IN",
    "FLICK_OUT"
};

typedef struct {
    uint16_t iaq;
    uint16_t siaq;
    uint16_t voc;
    uint32_t co2;
    uint8_t iaq_accuracy;
    int16_t comp_temperature;
    uint16_t comp_humidity;
    uint32_t raw_gas;
} BHI360_PACKED bhi360_event_data_iaq_output_t;

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_EVENT_DATA_DEFS_H__ */

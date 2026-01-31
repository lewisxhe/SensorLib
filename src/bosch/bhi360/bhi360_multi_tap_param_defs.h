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
* @file       bhi360_multi_tap_param_defs.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef _BHI360_MULTI_TAP_PARAM_DEFS_H_
#define _BHI360_MULTI_TAP_PARAM_DEFS_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <string.h>

#include "bhi360_defs.h"
#include "bhi360_event_data.h"

/*! Sensor ID for Multi-Tap */
#define BHI360_SENSOR_ID_MULTI_TAP                                   UINT8_C(153)

/*! Multi-Tap Parameter Page Base Address*/
#define BHI360_MULTI_TAP_PARAM_PAGE_BASE                             UINT16_C(0xD00)

/*! Multi-Tap Configuration pages */
#define BHI360_MULTI_TAP_PARAM_ENABLE_PARAM_ID                       UINT8_C(0x01)
#define BHI360_MULTI_TAP_PARAM_DETECTOR_CONFIG_PARAM_ID              UINT8_C(0x02)

#define BHI360_MULTI_TAP_PARAM_ENABLE_LENGTH                         UINT8_C(4)
#define BHI360_MULTI_TAP_PARAM_DETECTOR_CONFIG_LENGTH                UINT8_C(6)

#define BHI360_MULTI_TAP_PARAM_AXIS_SEL_X                            UINT8_C(0)
#define BHI360_MULTI_TAP_PARAM_AXIS_SEL_Y                            UINT8_C(1)
#define BHI360_MULTI_TAP_PARAM_AXIS_SEL_Z                            UINT8_C(2)

#define BHI360_MULTI_TAP_PARAM_WAIT_TIMEOUT_DISABLE                  UINT8_C(0)
#define BHI360_MULTI_TAP_PARAM_WAIT_TIMEOUT_ENABLE                   UINT8_C(1)

#define BHI360_MULTI_TAP_PARAM_FILTER_MODE_SENSITIVE                 UINT8_C(0)
#define BHI360_MULTI_TAP_PARAM_FILTER_MODE_NORMAL                    UINT8_C(1)
#define BHI360_MULTI_TAP_PARAM_FILTER_MODE_ROBUST                    UINT8_C(2)

#define BHI360_MULTI_TAP_PARAM_SINGLE_TAP_AXIS_SEL_MASK              0x03
#define BHI360_MULTI_TAP_PARAM_SINGLE_TAP_WAIT_TIMEOUT_MASK          0x04
#define BHI360_MULTI_TAP_PARAM_SINGLE_TAP_MAX_PEAKS_FOR_TAP_MASK     0x38
#define BHI360_MULTI_TAP_PARAM_SINGLE_TAP_FILTER_MODE_MASK           0xC0

#define BHI360_MULTI_TAP_PARAM_DOUBLE_TAP_TAP_PEAK_DUR_MASK          0x03FF
#define BHI360_MULTI_TAP_PARAM_DOUBLE_TAP_MAX_GES_DUR_MASK           0xFC00

#define BHI360_MULTI_TAP_PARAM_TRIPLE_TAP_MAX_DUR_BW_PEAKS_MASK      0x0F
#define BHI360_MULTI_TAP_PARAM_TRIPLE_TAP_TAP_SHOCK_SETL_DUR_MASK    0xF0
#define BHI360_MULTI_TAP_PARAM_TRIPLE_TAP_MIN_QT_DUR_BW_PEAKS_MASK   0x0F
#define BHI360_MULTI_TAP_PARAM_TRIPLE_TAP_QT_TM_AFTER_GESTURE_MASK   0xF0

#define BHI360_MULTI_TAP_PARAM_SINGLE_TAP_AXIS_SEL_SHIFT             UINT8_C(0)
#define BHI360_MULTI_TAP_PARAM_SINGLE_TAP_WAIT_TIMEOUT_SHIFT         UINT8_C(2)
#define BHI360_MULTI_TAP_PARAM_SINGLE_TAP_MAX_PEAKS_FOR_TAP_SHIFT    UINT8_C(3)
#define BHI360_MULTI_TAP_PARAM_SINGLE_TAP_FILTER_MODE_SHIFT          UINT8_C(6)

#define BHI360_MULTI_TAP_PARAM_DOUBLE_TAP_TAP_PEAK_DUR_SHIFT         UINT8_C(0)
#define BHI360_MULTI_TAP_PARAM_DOUBLE_TAP_MAX_GES_DUR_SHIFT          UINT8_C(10)

#define BHI360_MULTI_TAP_PARAM_TRIPLE_TAP_MAX_DUR_BW_PEAKS_SHIFT     UINT8_C(0)
#define BHI360_MULTI_TAP_PARAM_TRIPLE_TAP_TAP_SHOCK_SETL_DUR_SHIFT   UINT8_C(4)
#define BHI360_MULTI_TAP_PARAM_TRIPLE_TAP_MIN_QT_DUR_BW_PEAKS_SHIFT  UINT8_C(0)
#define BHI360_MULTI_TAP_PARAM_TRIPLE_TAP_QT_TM_AFTER_GESTURE_SHIFT  UINT8_C(4)

/*!
 * Single Tap Configuration.
 */
typedef union
{
    uint16_t as_uint16;
    struct
    {
        uint16_t axis_sel                : 2; /* Tap axis selection */
        uint16_t wait_for_timeout        : 1; /* Wait for gesture confirmation */
        uint16_t max_peaks_for_tap       : 3; /* Maximum number of peaks that can occur when a tap is done
                                                           * */
        uint16_t mode                    : 2; /* Filter configuration for various detection mode: SENSITIVE, NORMAL and
                                               * ROBUST */
        uint16_t reserved8               : 8;
    } as_s;
} BHI360_PACKED bhi360_multi_tap_param_singletap_detector_settings;

/*!
 * Double Tap Configuration.
 */
typedef union
{
    uint16_t as_uint16;
    struct
    {
        uint16_t tap_peak_thres          : 10; /* Minimum threshold for peak detection */
        uint16_t max_gesture_dur         : 6; /* Maximum time duration from first tap within which matching
                                               * tap/s should happen to be detected as double/triple tap */
    } as_s;
} BHI360_PACKED bhi360_multi_tap_param_doubletap_detector_settings;

/*!
 * Triple Tap Configuration.
 */
typedef union
{
    uint16_t as_uint16;
    struct
    {
        uint16_t max_dur_between_peaks       : 4; /* Maximum time duration within which matching peaks of tap
                                                   * should occur */
        uint16_t tap_shock_settling_dur      : 4; /* Maximum duration to wait for tap shock settling */
        uint16_t min_quite_dur_between_taps  : 4; /* Minimum quite time between detection of consecutive taps
                                                   * of double/triple taps*/
        uint16_t quite_time_after_gesture    : 4; /* Minimum quite time between detection of 2 consecutive
                                                   * selected gesture */
    } as_s;
} BHI360_PACKED bhi360_multi_tap_param_tripletap_detector_settings;

/*!
 * Multi Tap Configuration.
 */
typedef struct
{
    bhi360_multi_tap_param_singletap_detector_settings stap_setting;
    bhi360_multi_tap_param_doubletap_detector_settings dtap_setting;
    bhi360_multi_tap_param_tripletap_detector_settings ttap_setting;
} BHI360_PACKED bhi360_multi_tap_param_detector;

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHI360_MULTI_TAP_PARAM_DEFS_H_ */

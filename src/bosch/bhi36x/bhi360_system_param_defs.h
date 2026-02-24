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
* @file       bhi360_system_param_defs.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef __BHI360_SYSTEM_PARAM_DEFS_H__
#define __BHI360_SYSTEM_PARAM_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include "bhi360_defs.h"

#define BHI360_PHYS_SENSOR_ID_ACCELEROMETER                       UINT8_C(1)

/*! System parameters */
#define  BHI360_SYSTEM_PARAM_META_EVENT_CONTROL_NON_WAKE_UP_FIFO  UINT16_C(0x101)
#define  BHI360_SYSTEM_PARAM_META_EVENT_CONTROL_WAKE_UP_FIFO      UINT16_C(0x102)
#define  BHI360_SYSTEM_PARAM_FIFO_CONTROL                         UINT16_C(0x103)
#define  BHI360_SYSTEM_PARAM_FIRM_VERSION                         UINT16_C(0x104)
#define  BHI360_SYSTEM_PARAM_TIMESTAMPS                           UINT16_C(0x105)
#define  BHI360_SYSTEM_PARAM_FRAME_WORK_STATUS                    UINT16_C(0x106)
#define  BHI360_SYSTEM_PARAM_VIR_SENSOR_PRESENT                   UINT16_C(0x11F)
#define  BHI360_SYSTEM_PARAM_PHY_SENSOR_PRESENT                   UINT16_C(0x120)

#define  BHI360_SYSTEM_PARAM_PHY_SENSOR_INFO_BASE                 UINT16_C(0x120)

struct bhi360_system_param_phys_sensor_info
{
    uint8_t sensor_type;
    uint8_t driver_id;
    uint8_t driver_version;
    uint8_t power_current;
    union bhi360_u16_conv curr_range;
    uint8_t flags;
    uint8_t slave_address;
    uint8_t gpio_assignment;
    union bhi360_float_conv curr_rate;
    uint8_t num_axis;
    uint8_t orientation_matrix[5];
    uint8_t reserved;
};

struct bhi360_system_param_orient_matrix
{
    int8_t c[9];
};

typedef union bhi360_system_param_meta_event_states
{
    uint8_t as_uint8;
    struct
    {
        uint8_t meta_event1_int_enable_state        : 1;
        uint8_t meta_event1_enable_state            : 1;
        uint8_t meta_event2_int_enable_state        : 1;
        uint8_t meta_event2_enable_state            : 1;
        uint8_t meta_event3_int_enable_state        : 1;
        uint8_t meta_event3_enable_state            : 1;
        uint8_t meta_event4_int_enable_state        : 1;
        uint8_t meta_event4_enable_state            : 1;

    } as_s;
} __attribute__ ((packed)) bhi360_system_param_meta_event_states_t;

/*!
 * Multi BHY meta events control.
 */
typedef struct bhi360_system_param_multi_meta_event_ctrl
{
    bhi360_system_param_meta_event_states_t group[8];
} __attribute__ ((packed)) bhi360_system_param_multi_meta_event_ctrl_t;

struct bhi360_system_param_fifo_control
{
    uint32_t wakeup_fifo_watermark;
    uint32_t wakeup_fifo_size;
    uint32_t non_wakeup_fifo_watermark;
    uint32_t non_wakeup_fifo_size;
};

struct bhi360_system_param_firmware_version
{
    uint16_t custom_ver_num;
    uint64_t em_hash;
    uint64_t bst_hash;
    uint64_t user_hash;
};

struct bhi360_system_param_timestamp
{
    uint64_t host_int_ts;
    uint64_t cur_ts;
    uint64_t event_ts;
};

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_SYSTEM_PARAM_DEFS_H__ */

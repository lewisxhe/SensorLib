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
* @file       bhi360_head_orientation_param_defs.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef _BHI360_HEAD_ORIENTATION_PARAM_DEFS_H_
#define _BHI360_HEAD_ORIENTATION_PARAM_DEFS_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stdlib.h>
#include <stdlib.h>

#include "bhi360_defs.h"

/*! Virtual Sensor Macros */
#define BHI360_SENSOR_ID_HEAD_ORI_MIS_ALG                             UINT8_C(120)                     /*Head Orientation
                                                                                                     * Misalignment*/
#define BHI360_SENSOR_ID_IMU_HEAD_ORI_Q                               UINT8_C(121)                     /*IMU Head
                                                                                                     * Orientation
                                                                                                     * Quaternion*/
#define BHI360_SENSOR_ID_NDOF_HEAD_ORI_Q                              UINT8_C(122)                     /*NDOF Head
                                                                                                     * Orientation
                                                                                                     * Quaternion*/
#define BHI360_SENSOR_ID_IMU_HEAD_ORI_E                               UINT8_C(123)                     /*IMU Head
                                                                                                     * Orientation
                                                                                                     * Euler*/
#define BHI360_SENSOR_ID_NDOF_HEAD_ORI_E                              UINT8_C(124)                     /*NDOF Head
                                                                                                     * Orientation
                                                                                                     * Euler*/

#define BHI360_HEAD_ORIENTATION_PARAM_PAGE_BASE                       UINT16_C(0xC00)

#define BHI360_HEAD_ORIENTATION_PARAM_HMC_TRIGGER_CALIB_ID            UINT8_C(1)
#define BHI360_HEAD_ORIENTATION_PARAM_HMC_TRIGGER_CALIB_LENGTH        UINT8_C(4)
#define BHI360_HEAD_ORIENTATION_PARAM_HMC_TRIGGER_CALIB_SET           UINT8_C(1)

#define BHI360_HEAD_ORIENTATION_PARAM_HMC_CONFIG_ID                   UINT8_C(2)
#define BHI360_HEAD_ORIENTATION_PARAM_HMC_CONFIG_LENGTH               UINT8_C(8)

#define BHI360_HEAD_ORIENTATION_PARAM_HMC_SET_DEF_CONFIG_ID           UINT8_C(3)
#define BHI360_HEAD_ORIENTATION_PARAM_HMC_SET_DEF_CONFIG_LENGTH       UINT8_C(4)
#define BHI360_HEAD_ORIENTATION_PARAM_HMC_SET_DEF_CONFIG_SET          UINT8_C(1)

#define BHI360_HEAD_ORIENTATION_PARAM_HMC_VERSION_ID                  UINT8_C(4)
#define BHI360_HEAD_ORIENTATION_PARAM_HMC_VERSION_LENGTH              UINT8_C(4)

#define BHI360_HEAD_ORIENTATION_PARAM_HMC_QUAT_CALIB_CORR_ID          UINT8_C(5)
#define BHI360_HEAD_ORIENTATION_PARAM_HMC_QUAT_CALIB_CORR_WLENGTH     UINT8_C(16)
#define BHI360_HEAD_ORIENTATION_PARAM_HMC_QUAT_CALIB_CORR_RLENGTH     UINT8_C(20)

#define BHI360_HEAD_ORIENTATION_PARAM_HMC_PARAM_SET_MODE_ID           UINT8_C(6)
#define BHI360_HEAD_ORIENTATION_PARAM_HMC_PARAM_SET_MODE_LENGTH       UINT8_C(13)

#define BHI360_HEAD_ORIENTATION_PARAM_QUAT_INITIAL_HEAD_CORR_ID       UINT8_C(10)
#define BHI360_HEAD_ORIENTATION_PARAM_QUAT_INITIAL_HEAD_CORR_LENGTH   UINT8_C(4)
#define BHI360_HEAD_ORIENTATION_PARAM_QUAT_INITIAL_HEAD_CORR_DISABLE  UINT8_C(0)
#define BHI360_HEAD_ORIENTATION_PARAM_QUAT_INITIAL_HEAD_CORR_ENABLE   UINT8_C(1)

#define BHI360_HEAD_ORIENTATION_PARAM_VERSION_ID                      UINT8_C(11)
#define BHI360_HEAD_ORIENTATION_PARAM_VERSION_LENGTH                  UINT8_C(4)

#define BHI360_HEAD_ORIENTATION_PARAM_EUL_INITIAL_HEAD_CORR_ID        UINT8_C(12)
#define BHI360_HEAD_ORIENTATION_PARAM_EUL_INITIAL_HEAD_CORR_LENGTH    UINT8_C(4)
#define BHI360_HEAD_ORIENTATION_PARAM_EUL_INITIAL_HEAD_CORR_DISABLE   UINT8_C(0)
#define BHI360_HEAD_ORIENTATION_PARAM_EUL_INITIAL_HEAD_CORR_ENABLE    UINT8_C(1)

/*! Structure for Head Misalignment Configuration */
typedef struct
{
    uint8_t still_phase_max_dur;
    uint8_t still_phase_min_dur;
    uint8_t still_phase_max_samples;
    int32_t acc_diff_threshold;
    uint8_t reserved;
} BHI360_PACKED bhi360_head_orientation_param_misalignment_config;

/*! Structure for Head Orientation /Head Misalignment version */
typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t reserved;
} BHI360_PACKED bhi360_head_orientation_param_ver;

/*! Structure for Head Misalignment Quaternion Correction */
typedef struct
{
    union bhi360_float_conv quaternion_x;
    union bhi360_float_conv quaternion_y;
    union bhi360_float_conv quaternion_z;
    union bhi360_float_conv quaternion_w;
    union bhi360_float_conv accuracy;
} BHI360_PACKED bhi360_head_orientation_param_misalignment_quat_corr;

/*! Structure for Head Misalignment Mode and Vector X value */
typedef struct
{
    uint8_t mode;
    union bhi360_float_conv vector_x_0;
    union bhi360_float_conv vector_x_1;
    union bhi360_float_conv vector_x_2;
    uint8_t reserved[3];
} BHI360_PACKED bhi360_head_misalignment_mode_vector_x;

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHI360_HEAD_ORIENTATION_PARAM_DEFS_H_ */

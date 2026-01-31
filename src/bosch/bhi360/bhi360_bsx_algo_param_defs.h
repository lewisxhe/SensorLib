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
* @file       bhi360_bsx_algo_param_defs.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef __BHI360_BSX_ALGO_PARAM_DEFS_H__
#define __BHI360_BSX_ALGO_PARAM_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhi360_defs.h"

#define BHI360_BSX_CALIBRATE_STATE_BASE     UINT16_C(0x200)

#define BHI360_BSX_SIC_MATRIX               UINT16_C(0x27D)
#define BHI360_BSX_VERSION                  UINT16_C(0x27E)

#define BHI360_BSX_STATE_STRUCT_LEN         UINT8_C(68)
#define BHI360_BSX_STATE_BLOCK_LEN          UINT8_C(64)
#define BHI360_BSX_STATE_TRANSFER_COMPLETE  UINT8_C(0x80)
#define BHI360_BSX_STATE_MAX_BLOCKS         UINT8_C(20)

typedef struct
{
    uint8_t block_info;
    uint8_t block_len;
    uint16_t struct_len;
    uint8_t state_data[BHI360_BSX_STATE_BLOCK_LEN];
} BHI360_PACKED bhi360_bsx_algo_param_state_exg;

struct BHI360_PACKED bhi360_bsx_algo_param_version
{
    uint8_t major_version;
    uint8_t minor_version;
    uint8_t major_bug_fix_version;
    uint8_t minor_bug_fix_version;
};

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_BSX_ALGO_PARAM_DEFS_H__ */

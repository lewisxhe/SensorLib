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
* @file       bhi360_bsec_param_defs.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef __BHI360_BSEC_PARAM_DEFS_H__
#define __BHI360_BSEC_PARAM_DEFS_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhi360_defs.h"

/*! BSEC Parameter Page Base Address*/
#define BHI360_BSEC_PARAM_PAGE_BASE               UINT16_C(0x800)

#define BHI360_BSEC_PARAM_IAQ_PARAM_STATE_ID      UINT16_C(0x00)
#define BHI360_BSEC_PARAM_IAQ_TEMP_OFFSET_ID      UINT16_C(0x05)
#define BHI360_BSEC_PARAM_IAQ_SAMPLE_RATE_ID      UINT16_C(0x06)

/*! BSEC Parameter Length */
#define BHI360_BSEC_PARAM_IAQ_PARAM_STATE_LENGTH  UINT8_C(164)
#define BHI360_BSEC_PARAM_IAQ_TEMP_OFFSET_LENGTH  UINT8_C(4)
#define BHI360_BSEC_PARAM_IAQ_SAMPLE_RATE_LENGTH  UINT8_C(4)

#define BHI360_BSEC_PARAM_BSEC_ALGO_STATE_LENGTH  UINT8_C(163)

/*! BSEC algo state structure */
typedef struct
{
    uint8_t algo_state[BHI360_BSEC_PARAM_BSEC_ALGO_STATE_LENGTH];
    uint8_t padding;
} BHI360_PACKED bhi360_bsec_param_algo_state;

/*! BSEC Sample rate index */
typedef enum
{
    BSEC_PARAM_SAMPLE_RATE_CONT = 0U,
    BSEC_PARAM_SAMPLE_RATE_LP   = 1U,
    BSEC_PARAM_SAMPLE_RATE_ULP  = 2U
} bhi360_bsec_param_sample_rate_index;

/*! BSEC Sample rate structure */
typedef struct
{
    bhi360_bsec_param_sample_rate_index sample_rate_index;
    uint8_t padding[3];
} BHI360_PACKED bhi360_bsec_param_sample_rate;

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_BSEC_PARAM_DEFS_H__ */

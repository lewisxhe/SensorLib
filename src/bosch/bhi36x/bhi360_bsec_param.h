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
* @file       bhi360_bsec_param.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef __BHI360_BSEC_PARAM_H__
#define __BHI360_BSEC_PARAM_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhi360_bsec_param_defs.h"
#include "../bhi260x/bhy2.h"

/**
 * Sets the BSEC algorithm state.
 *
 * @param[in] algo_state : Reference to BSEC algorithm state.
 * @param[in] dev        : Device instance.
 * @return API error codes.
 */
int8_t bhi360_bsec_param_set_algo_state(const bhi360_bsec_param_algo_state *algo_state, struct bhy2_dev *dev);

/**
 * Retrieves the BSEC algorithm state from the sensor payload.
 *
 * @param[out] algo_state : Reference to BSEC algorithm state.
 * @param[in]  dev        : Device instance.
 * @return API error codes.
 */
int8_t bhi360_bsec_param_get_algo_state(bhi360_bsec_param_algo_state *algo_state, struct bhy2_dev *dev);

/**
 * Sets the BSEC temperature offset.
 *
 * @param[in] offset : Reference to BSEC temperature offset.
 * @param[in] dev    : Device instance.
 * @return API error codes.
 */
int8_t bhi360_bsec_param_set_temp_offset(const union bhi360_float_conv *offset, struct bhy2_dev *dev);

/**
 * Retrieves the BSEC temperature offset.
 *
 * @param[out] offset : Reference to BSEC temperature offset.
 * @param[in]  dev    : Device instance.
 * @return API error codes.
 */
int8_t bhi360_bsec_param_get_temp_offset(union bhi360_float_conv *offset, struct bhy2_dev *dev);

/**
 * Sets the BSEC sample rate.
 *
 * @param[in] sample_rate : Reference to BSEC sample rate.
 * @param[in] dev         : Device instance.
 * @return API error codes.
 */
int8_t bhi360_bsec_param_set_sample_rate(const bhi360_bsec_param_sample_rate *sample_rate, struct bhy2_dev *dev);

/**
 * Retrieves the BSEC sample rate.
 *
 * @param[in] sample_rate : Reference to BSEC sample rate.
 * @param[in] dev         : Device instance.
 * @return API error codes.
 */
int8_t bhi360_bsec_param_get_sample_rate(bhi360_bsec_param_sample_rate *sample_rate, struct bhy2_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_BSEC_PARAM_H__ */

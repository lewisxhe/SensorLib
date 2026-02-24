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
* @file       bhi360_bsx_algo_param.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef __BHI360_BSX_ALGO_PARAM_H__
#define __BHI360_BSX_ALGO_PARAM_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhi360_bsx_algo_param_defs.h"
#include "../bhi260x/bhy2.h"

/**
 * @brief Function to get the calibration profile of BSX fusion for physical sensors
 * @param[in] param_id      : Parameter ID for the physical sensors
 * @param[out] bsx_state    : Reference to the data buffer to store the calibration profile
 * @param[in] state_len     : Length of the data buffer
 * @param[out] actual_len   : Actual length of the calibration profile
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_bsx_algo_param_get_bsx_states(uint16_t param_id,
                                            bhi360_bsx_algo_param_state_exg *bsx_state,
                                            uint16_t state_len,
                                            uint32_t *actual_len,
                                            struct bhy2_dev *dev);

/**
 * @brief Function to set the calibration profile of physical sensors to the BSX fusion
 * @param[in] param_id      : Parameter ID for the physical sensors
 * @param[in] bsx_state    : Reference to the data buffer storing the calibration profile
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_bsx_algo_param_set_bsx_states(uint16_t param_id,
                                            const bhi360_bsx_algo_param_state_exg *bsx_state,
                                            struct bhy2_dev *dev);

/**
 * @brief Function to get the BSX version
 * @param[out] bsx_ver      : Reference to the data buffer to store the BSX version
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_bsx_algo_param_get_bsx_version(struct bhi360_bsx_algo_param_version *bsx_ver, struct bhy2_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_BSX_ALGO_PARAM_H__ */

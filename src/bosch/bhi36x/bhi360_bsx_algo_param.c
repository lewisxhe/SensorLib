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
* @file       bhi360_bsx_algo_param.c
* @date       2025-03-28
* @version    v2.2.0
*
*/

#include "bhi360_bsx_algo_param.h"
#include "bhi360_hif.h"
#include "../bhi260x/bhy2_hif.h"

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
                                            struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    bhi360_bsx_algo_param_state_exg bsx_state_exg;

    if ((dev == NULL) || (bsx_state == NULL) || (actual_len == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        uint8_t complete_flag = 0;
        uint16_t tmp_state_len = 0;
        uint32_t read_len;
        uint16_t state_index = 0;

        while (complete_flag == 0)
        {
            rslt = bhy2_hif_get_parameter(param_id,
                                            (uint8_t *)&bsx_state_exg,
                                            BHI360_BSX_STATE_STRUCT_LEN,
                                            &read_len,
                                            &dev->hif);
            if (rslt != BHY2_OK)
            {
                break;
            }

            if (read_len > BHI360_BSX_STATE_STRUCT_LEN)
            {
                rslt = BHY2_E_INVALID_PARAM - 50;
                break;
            }

            complete_flag = bsx_state_exg.block_info & BHI360_BSX_STATE_TRANSFER_COMPLETE;
            tmp_state_len = bsx_state_exg.struct_len;
            if (tmp_state_len == 0)
            {
                rslt = BHY2_E_INVALID_PARAM - 70;
                break;
            }

            *actual_len = tmp_state_len; /* Report actual length, in case of invalid buffer */

            if (state_len < tmp_state_len)
            {
                rslt = BHI360_E_BUFFER;
                break;
            }

            bsx_state[state_index] = bsx_state_exg;
            state_index++;
        }
    }

    return rslt;
}

/**
 * @brief Function to set the calibration profile of physical sensors to the BSX fusion
 * @param[in] param_id      : Parameter ID for the physical sensors
 * @param[in] bsx_state    : Reference to the data buffer storing the calibration profile
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_bsx_algo_param_set_bsx_states(uint16_t param_id,
                                            const bhi360_bsx_algo_param_state_exg *bsx_state,
                                            struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t complete_flag = 0;
    uint16_t state_index = 0;
    bhi360_bsx_algo_param_state_exg bsx_state_exg;

    if ((dev == NULL) || (bsx_state == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {

        while (complete_flag == 0)
        {
            bsx_state_exg = bsx_state[state_index];
            state_index++;

            if (bsx_state_exg.struct_len == 0)
            {
                rslt = BHY2_E_INVALID_PARAM - 70;
                break;
            }

            complete_flag = bsx_state_exg.block_info & BHI360_BSX_STATE_TRANSFER_COMPLETE;

            rslt = bhy2_hif_set_parameter(param_id, (uint8_t *)&bsx_state_exg, sizeof(bsx_state_exg), &dev->hif);
            if (rslt != BHY2_OK)
            {
                break;
            }

            if ((bsx_state_exg.block_len == 0) || (complete_flag != 0))
            {
                break;
            }
        }
    }

    return rslt;
}

/**
 * @brief Function to get the BSX version
 * @param[out] bsx_ver  : Reference to the data buffer to store the BSX version
 * @param[in]  dev      : Device reference
 * @return API error codes
 */
int8_t bhi360_bsx_algo_param_get_bsx_version(struct bhi360_bsx_algo_param_version *bsx_ver, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[4] = { 0 };

    if ((dev == NULL) || (bsx_ver == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(BHI360_BSX_VERSION, buffer, 4, &bytes_read, &dev->hif);
        if (rslt != BHY2_OK)
        {
            return rslt;
        }
        else
        {
            bsx_ver->major_version = buffer[0]; /* BSX major version */
            bsx_ver->minor_version = buffer[1]; /* BSX minor version */
            bsx_ver->major_bug_fix_version = buffer[2]; /* BSX major bug fix version */
            bsx_ver->minor_bug_fix_version = buffer[3]; /* BSX minor bug fix version */
        }
    }

    return rslt;
}

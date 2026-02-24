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
* @file       bhi360_activity_param.c
* @date       2025-03-28
* @version    v2.2.0
*
*/

#include "bhi360_activity_param.h"
// #include "bhi360.h"

/**
 * Sets the hearble activity configuration.
 *
 * This function is used to set the hearable activity configuration in the BHY device.
 *
 * @param[in] hearable_conf : Reference to hearable activity configuration.
 * @param[in] dev           : Pointer to the BHY device structure.
 * @return API error codes.
 */
int8_t bhi360_activity_param_set_hearable_config(const bhi360_activity_param_hearable *hearable_conf,
                                                 struct bhy2_dev *dev)
{
    int8_t rslt;
    uint16_t param_id = BHI360_ACTIVITY_PARAM_PAGE_BASE | BHI360_ACTIVITY_PARAM_HEARABLE_ID;

    if ((hearable_conf == NULL) || (dev == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_set_parameter(param_id, (const uint8_t*)hearable_conf, BHI360_ACTIVITY_PARAM_LENGTH, dev);
    }

    return rslt;
}

/**
 * Retrieves the hearable activity configuration from the sensor payload.
 *
 * @param[out] hearable_conf : Reference to hearable activity configuration.
 * @param[in] dev            : Pointer to the BHY device structure.
 * @return API error codes.
 */
int8_t bhi360_activity_param_get_hearable_config(bhi360_activity_param_hearable *hearable_conf, struct bhy2_dev *dev)
{
    int8_t rslt;
    uint32_t act_len;
    uint16_t param_id = BHI360_ACTIVITY_PARAM_PAGE_BASE | BHI360_ACTIVITY_PARAM_HEARABLE_ID;
    uint8_t buffer[BHI360_ACTIVITY_PARAM_LENGTH] = { 0U };

    if ((hearable_conf == NULL) || (dev == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_get_parameter(param_id, (uint8_t*)buffer, BHI360_ACTIVITY_PARAM_LENGTH, &act_len, dev);
        if (rslt == BHY2_OK)
        {
            hearable_conf->seg_size = BHY2_LE2U16(&buffer[0]);
            hearable_conf->post_process_en = BHY2_LE2U16(&buffer[2]);
            hearable_conf->min_gdi_thre = BHY2_LE2U16(&buffer[4]);
            hearable_conf->max_gdi_thre = BHY2_LE2U16(&buffer[6]);
            hearable_conf->out_buff_size = BHY2_LE2U16(&buffer[8]);
            hearable_conf->min_seg_moder_conf = BHY2_LE2U16(&buffer[10]);
        }
    }

    return rslt;
}

/**
 * Sets the wearble activity configuration.
 *
 * This function is used to set the wearable activity configuration in the BHY device.
 *
 * @param[in] wearable_conf : Reference to wearable activity configuration.
 * @param[in] dev           : Pointer to the BHY device structure.
 * @return API error codes.
 */
int8_t bhi360_activity_param_set_wearable_config(const bhi360_activity_param_wearable *wearable_conf,
                                                 struct bhy2_dev *dev)
{
    int8_t rslt;
    uint16_t param_id = BHI360_ACTIVITY_PARAM_PAGE_BASE | BHI360_ACTIVITY_PARAM_WEARABLE_ID;

    if ((wearable_conf == NULL) || (dev == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_set_parameter(param_id, (const uint8_t*)wearable_conf, BHI360_ACTIVITY_PARAM_LENGTH, dev);
    }

    return rslt;
}

/**
 * Retrieves the wearable activity configuration from the sensor payload.
 *
 * @param[out] wearable_conf : Reference to wearable activity configuration.
 * @param[in] dev            : Pointer to the BHY device structure.
 * @return API error codes.
 */
int8_t bhi360_activity_param_get_wearable_config(bhi360_activity_param_wearable *wearable_conf, struct bhy2_dev *dev)
{
    int8_t rslt;
    uint32_t act_len;
    uint16_t param_id = BHI360_ACTIVITY_PARAM_PAGE_BASE | BHI360_ACTIVITY_PARAM_WEARABLE_ID;
    uint8_t buffer[BHI360_ACTIVITY_PARAM_LENGTH] = { 0U };

    if ((wearable_conf == NULL) || (dev == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_get_parameter(param_id, (uint8_t*)buffer, BHI360_ACTIVITY_PARAM_LENGTH, &act_len, dev);
        if (rslt == BHY2_OK)
        {
            wearable_conf->post_process_en = BHY2_LE2U16(&buffer[0]);
            wearable_conf->min_gdi_thre = BHY2_LE2U16(&buffer[2]);
            wearable_conf->max_gdi_thre = BHY2_LE2U16(&buffer[4]);
            wearable_conf->out_buff_size = BHY2_LE2U16(&buffer[6]);
            wearable_conf->min_seg_moder_conf = BHY2_LE2U16(&buffer[8]);
        }
    }

    return rslt;
}

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
* @file       bhi360_virtual_sensor_info_param.c
* @date       2025-03-28
* @version    v2.2.0
*
*/

#include "bhi360_virtual_sensor_info_param.h"
// #include "bhi360_hif.h"
#include "../bhi260x/bhy2_hif.h"

/**
 * @brief Function get information of a virtual sensor
 * @param[in] sensor_id : Sensor ID of the virtual sensor
 * @param[out] info     : Reference to the data buffer to store the sensor info
 * @param[in] dev       : BHY device instance
 * @return API error codes
 */

int8_t bhi360_virtual_sensor_info_param_get_info(uint8_t sensor_id,
                                                 struct bhi360_virtual_sensor_info_param_info *info,
                                                 struct bhy2_dev *dev)
{
    int8_t rslt;
    uint32_t length;
    uint8_t bytes[BHI360_PARAM_NUM_BYTES];

    if (sensor_id > BHI360_SPECIAL_SENSOR_ID_OFFSET)
    {
        rslt = BHY2_E_INVALID_PARAM;
    }
    else if ((info == NULL) || (dev == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter((uint16_t)(BHI360_PARAM_VIRTUAL_SENSOR_INFO + sensor_id),
                                        bytes,
                                        BHI360_PARAM_NUM_BYTES,
                                        &length,
                                        &dev->hif);
        if (rslt == BHY2_OK)
        {
            if (length != BHI360_PARAM_NUM_BYTES)
            {
                rslt = BHY2_E_INVALID_PARAM;
            }
            else
            {
                info->sensor_type = bytes[0];
                info->driver_id = bytes[1];
                info->driver_version = bytes[2];
                info->power = bytes[3];
                info->max_range.bytes[0] = bytes[4];
                info->max_range.bytes[1] = bytes[5];
                info->resolution.bytes[0] = bytes[6];
                info->resolution.bytes[1] = bytes[7];
                info->max_rate.bytes[0] = bytes[8];
                info->max_rate.bytes[1] = bytes[9];
                info->max_rate.bytes[2] = bytes[10];
                info->max_rate.bytes[3] = bytes[11];
                info->fifo_reserved.bytes[0] = bytes[12];
                info->fifo_reserved.bytes[1] = bytes[13];
                info->fifo_reserved.bytes[2] = bytes[14];
                info->fifo_reserved.bytes[3] = bytes[15];
                info->fifo_max.bytes[0] = bytes[16];
                info->fifo_max.bytes[1] = bytes[17];
                info->fifo_max.bytes[2] = bytes[18];
                info->fifo_max.bytes[3] = bytes[19];
                info->event_size = bytes[20];
                info->min_rate.bytes[0] = bytes[21];
                info->min_rate.bytes[1] = bytes[22];
                info->min_rate.bytes[2] = bytes[23];
                info->min_rate.bytes[3] = bytes[24];
                info->max_range.u16_val = BHY2_LE2U16(info->max_range.bytes);
                info->resolution.u16_val = BHY2_LE2U16(info->resolution.bytes);
                info->max_rate.u32_val = BHY2_LE2U32(info->max_rate.bytes);
                info->fifo_reserved.u32_val = BHY2_LE2U32(info->fifo_reserved.bytes);
                info->fifo_max.u32_val = BHY2_LE2U32(info->fifo_max.bytes);
                info->min_rate.u32_val = BHY2_LE2U32(info->min_rate.bytes);
            }
        }
    }

    return rslt;
}

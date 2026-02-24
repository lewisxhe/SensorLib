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
* @file       bhi360_virtual_sensor_conf_param.c
* @date       2025-03-28
* @version    v2.2.0
*
*/

#include "bhi360_virtual_sensor_conf_param.h"
// #include "bhi360_hif.h"
#include "../bhi260x/bhy2_hif.h"
#include "../bhi260x/bhy2.h"
/**
 * @brief Function to set the sample rate and latency of the virtual sensor
 * @param[in] sensor_id     : Sensor ID of the virtual sensor
 * @param[in] sample_rate   : Sample rate in Hz
 * @param[in] latency       : Report latency in milliseconds
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_virtual_sensor_conf_param_set_cfg(uint8_t sensor_id,
                                                const struct bhi360_virtual_sensor_conf_param_conf *sensor_conf,
                                                struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if ((dev != NULL) && (sensor_conf != NULL))
    {
        rslt = bhy2_hif_exec_sensor_conf_cmd(sensor_id, sensor_conf->sample_rate, sensor_conf->latency, &dev->hif);
    }
    else
    {
        rslt = BHY2_E_NULL_PTR;
    }

    return rslt;
}

/**
 * @brief Function to get the virtual sensor configuration
 * @param[in] sensor_id         : Sensor ID of the virtual sensor
 * @param[out] virt_sensor_conf : Reference to the data buffer to store the virtual sensor configuration
 * @param[in] dev               : Device reference
 * @return API error codes
 */
int8_t bhi360_virtual_sensor_conf_param_get_cfg(uint8_t sensor_id,
                                                struct bhi360_virtual_sensor_conf_param_conf *virt_sensor_conf,
                                                struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    union bhi360_virtual_sensor_conf_param_float_u32 sample_rate;
    uint8_t buffer[12] = { 0 }; /* Size of parameter to get sensor config */

    if ((dev == NULL) || (virt_sensor_conf == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter((uint16_t)(BHI360_VIRTUAL_SENSOR_CONF_BASE + sensor_id),
                                        buffer,
                                        12,
                                        &bytes_read,
                                        &dev->hif);
        sample_rate.reg = BHY2_LE2U32(buffer);
        virt_sensor_conf->sample_rate = sample_rate.as_float;
        virt_sensor_conf->latency = BHY2_LE2U32(&buffer[4]);
        virt_sensor_conf->reserved = BHY2_LE2U16(&buffer[8]);
        virt_sensor_conf->range = BHY2_LE2U16(&buffer[10]);
    }

    return rslt;
}

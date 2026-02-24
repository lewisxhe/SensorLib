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
* @file       bhi360_system_param.c
* @date       2025-03-28
* @version    v2.2.0
*
*/

#include "bhi360_system_param.h"
// #include "bhi360_hif.h"
#include "../bhi260x/bhy2_hif.h"
#include <stdio.h>

/*!
 * @brief Set Meta Event Control for FIFO (wake-up and non-wake-up)
 *
 * @param[in] param          : Parameter to set.
 * @param[in] info           : Buffer containing new parameter value.
 * @param[in] dev            : Device reference.
 *
 * @return BHY API error codes.
 */
int8_t bhi360_system_param_set_meta_event_control(uint16_t param,
                                                  const bhi360_system_param_multi_meta_event_ctrl_t *info,
                                                  struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint8_t buffer[8] = { 0 };

    if (dev == NULL || info == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        for (uint8_t idx = 0; idx < 8; idx++)
        {
            buffer[idx] = info->group[idx].as_uint8;
        }

        rslt = bhy2_hif_set_parameter(param, buffer, sizeof(buffer), &dev->hif);

        if (rslt != BHY2_OK)
        {
            return rslt;
        }
    }

    return rslt;
}

/*!
 * @brief Get Meta Event Control for FIFOs (wake-up and non-wake-up)
 *
 * @param[in] param          : Parameter to set.
 * @param[out] info          : Buffer containing new parameter value.
 * @param[in] dev            : Device reference.
 *
 * @return BHY API error codes.
 */
int8_t bhi360_system_param_get_meta_event_control(uint16_t param,
                                                  bhi360_system_param_multi_meta_event_ctrl_t *info,
                                                  struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[8] = { 0 };

    if (dev == NULL || info == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(param, buffer, sizeof(buffer), &bytes_read, &dev->hif);

        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        for (uint8_t idx = 0; idx < 8; idx++)
        {
            info->group[idx].as_s.meta_event4_enable_state = ((buffer[idx] & 0x80) >> 7);
            info->group[idx].as_s.meta_event4_int_enable_state = ((buffer[idx] & 0x40) >> 6);

            info->group[idx].as_s.meta_event3_enable_state = ((buffer[idx] & 0x20) >> 5);
            info->group[idx].as_s.meta_event3_int_enable_state = ((buffer[idx] & 0x10) >> 4);

            info->group[idx].as_s.meta_event2_enable_state = ((buffer[idx] & 0x08) >> 3);
            info->group[idx].as_s.meta_event2_int_enable_state = ((buffer[idx] & 0x04) >> 2);

            info->group[idx].as_s.meta_event1_enable_state = ((buffer[idx] & 0x02) >> 1);
            info->group[idx].as_s.meta_event1_int_enable_state = buffer[idx] & 0x01;
        }
    }

    return rslt;
}

/*!
 * @brief Function set the Wakeup FIFO control register
 * @param[in] fifo_ctrl     : Fifo control
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_set_wakeup_fifo_control(const struct bhi360_system_param_fifo_control *fifo_ctrl,
                                                   struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[20] = { 0 };
    uint32_t read_wakeup_fifo_watermark = 0;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        /* Setting watermark value for Wake-up FIFO*/
        rslt =
            bhy2_hif_get_parameter(BHI360_SYSTEM_PARAM_FIFO_CONTROL, buffer, sizeof(buffer), &bytes_read, &dev->hif);
        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        buffer[0] = (uint8_t)((fifo_ctrl->wakeup_fifo_watermark & 0xFF));
        buffer[1] = (uint8_t)(((fifo_ctrl->wakeup_fifo_watermark >> 8) & 0xFF));
        buffer[2] = (uint8_t)(((fifo_ctrl->wakeup_fifo_watermark >> 16) & 0xFF));
        buffer[3] = (uint8_t)(((fifo_ctrl->wakeup_fifo_watermark >> 24) & 0xFF));

        rslt = bhy2_hif_set_parameter(BHI360_SYSTEM_PARAM_FIFO_CONTROL, buffer, sizeof(buffer), &dev->hif);

        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        rslt =
            bhy2_hif_get_parameter(BHI360_SYSTEM_PARAM_FIFO_CONTROL, buffer, sizeof(buffer), &bytes_read, &dev->hif);

        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        read_wakeup_fifo_watermark = BHY2_LE2U32(buffer);

        if (read_wakeup_fifo_watermark != fifo_ctrl->wakeup_fifo_watermark)
        {
            rslt = BHY2_E_PARAM_NOT_SET;

            return rslt;
        }
    }

    return rslt;
}

/*!
 * @brief Function set the non-wakeup FIFO control register
 * @param[in] fifo_ctrl     : FIFO control
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_set_nonwakeup_fifo_control(const struct bhi360_system_param_fifo_control *fifo_ctrl,
                                                      struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[20] = { 0 };
    uint32_t read_nonwakeup_fifo_watermark = 0;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt =
            bhy2_hif_get_parameter(BHI360_SYSTEM_PARAM_FIFO_CONTROL, buffer, sizeof(buffer), &bytes_read, &dev->hif);
        if (rslt < 0)
        {
            return rslt;
        }

        buffer[8] = (uint8_t)((fifo_ctrl->non_wakeup_fifo_watermark & 0xFF));
        buffer[9] = (uint8_t)(((fifo_ctrl->non_wakeup_fifo_watermark >> 8) & 0xFF));
        buffer[10] = (uint8_t)(((fifo_ctrl->non_wakeup_fifo_watermark >> 16) & 0xFF));
        buffer[11] = (uint8_t)(((fifo_ctrl->non_wakeup_fifo_watermark >> 24) & 0xFF));

        rslt = bhy2_hif_set_parameter(BHI360_SYSTEM_PARAM_FIFO_CONTROL, buffer, sizeof(buffer), &dev->hif);

        if (rslt < 0)
        {
            return rslt;
        }

        rslt =
            bhy2_hif_get_parameter(BHI360_SYSTEM_PARAM_FIFO_CONTROL, buffer, sizeof(buffer), &bytes_read, &dev->hif);

        if (rslt < 0)
        {
            return rslt;
        }

        read_nonwakeup_fifo_watermark = BHY2_LE2U32(&buffer[8]);

        if (read_nonwakeup_fifo_watermark != fifo_ctrl->non_wakeup_fifo_watermark)
        {
            rslt = BHY2_E_PARAM_NOT_SET;
        }
    }

    return rslt;
}

/*!
 * @brief Function get the FIFO control register
 * @param[out] fifo_ctrl    : FIFO control
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_fifo_control(struct bhi360_system_param_fifo_control *fifo_ctrl, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[20] = { 0 };

    if ((dev == NULL) || (fifo_ctrl == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(BHI360_SYSTEM_PARAM_FIFO_CONTROL, buffer, 20, &bytes_read, &dev->hif);

        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        fifo_ctrl->wakeup_fifo_watermark = BHY2_LE2U32(buffer);
        fifo_ctrl->wakeup_fifo_size = BHY2_LE2U32(&buffer[4]);
        fifo_ctrl->non_wakeup_fifo_watermark = BHY2_LE2U32(&buffer[8]);
        fifo_ctrl->non_wakeup_fifo_size = BHY2_LE2U32(&buffer[12]);
    }

    return rslt;
}

/*!
 * @brief Function get the firmware version
 * @param[out] fw_ver       : Reference to the data buffer to store firmware version
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_firmware_version(struct bhi360_system_param_firmware_version *fw_ver,
                                                struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[20] = { 0 };

    if ((dev == NULL) || (fw_ver == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(BHI360_SYSTEM_PARAM_FIRM_VERSION, buffer, 20, &bytes_read, &dev->hif);

        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        fw_ver->custom_ver_num = BHY2_LE2U16(buffer);
        fw_ver->em_hash = BHY2_LE2U48(&buffer[2]);
        fw_ver->bst_hash = BHY2_LE2U48(&buffer[8]);
        fw_ver->user_hash = BHY2_LE2U48(&buffer[14]);
    }

    return rslt;
}

/*!
 * @brief Function get the timestamp
 * @param[out] ts           : Reference to the data buffer to store timestamp
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_timestamps(struct bhi360_system_param_timestamp *ts, struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;
    uint8_t buffer[16] = { 0 };

    if ((dev == NULL) || (ts == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(BHI360_SYSTEM_PARAM_TIMESTAMPS, buffer, 16, &bytes_read, &dev->hif);

        if (rslt != BHY2_OK)
        {
            return rslt;
        }

        ts->host_int_ts = BHY2_LE2U40(buffer); /* Host interrupt timestamp */
        ts->cur_ts = BHY2_LE2U40(&buffer[5]); /* Current timestamp */
        ts->event_ts = BHY2_LE2U40(&buffer[10]); /* Timestamp event */
    }

    return rslt;
}

/*!
 * @brief Function get the virtual sensor present
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_virtual_sensor_present(struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t bytes_read = 0;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(BHI360_SYSTEM_PARAM_VIR_SENSOR_PRESENT,
                                        dev->present_buff,
                                        32,
                                        &bytes_read,
                                        &dev->hif);
    }

    return rslt;
}

/*!
 * @brief Function get the physical sensor present
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_physical_sensor_present(struct bhy2_dev *dev)
{
    uint32_t bytes_read = 0;

    int8_t rslt = BHY2_OK;

    if (dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter(BHI360_SYSTEM_PARAM_PHY_SENSOR_PRESENT,
                                        dev->phy_present_buff,
                                        8,
                                        &bytes_read,
                                        &dev->hif);
    }

    return rslt;
}

/**
 * @brief Function to get the physical sensor information of a virtual sensor
 * @param[in] sensor_id : Sensor ID of the physical sensor
 * @param[out] info     : Reference to the data buffer to store the physical sensor info
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_physical_sensor_info(uint8_t sensor_id,
                                                    struct bhi360_system_param_phys_sensor_info *info,
                                                    struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;
    uint32_t length;
    uint8_t bytes[20];

    if (info == NULL || dev == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        rslt = bhy2_hif_get_parameter((uint16_t)(BHI360_SYSTEM_PARAM_PHY_SENSOR_INFO_BASE + sensor_id),
                                        bytes,
                                        20,
                                        &length,
                                        &dev->hif);
        if (rslt == BHY2_OK)
        {
            if (length != 20)
            {
                rslt = BHY2_E_INVALID_PARAM;
            }
            else
            {
                info->sensor_type = bytes[0];
                info->driver_id = bytes[1];
                info->driver_version = bytes[2];
                info->power_current = bytes[3];
                info->curr_range.bytes[0] = bytes[4];
                info->curr_range.bytes[1] = bytes[5];
                info->flags = bytes[6];
                info->slave_address = bytes[7];
                info->gpio_assignment = bytes[8];
                info->curr_rate.bytes[0] = bytes[9];
                info->curr_rate.bytes[1] = bytes[10];
                info->curr_rate.bytes[2] = bytes[11];
                info->curr_rate.bytes[3] = bytes[12];
                info->num_axis = bytes[13];
                info->orientation_matrix[0] = bytes[14];
                info->orientation_matrix[1] = bytes[15];
                info->orientation_matrix[2] = bytes[16];
                info->orientation_matrix[3] = bytes[17];
                info->orientation_matrix[4] = bytes[18];

                info->curr_range.u16_val = BHY2_LE2U16(info->curr_range.bytes);
                info->curr_rate.u32_val = BHY2_LE2U32(info->curr_rate.bytes);
            }
        }
    }

    return rslt;
}

/**
 * @brief Function to set the orientation matrix of physical sensor
 * @param[in] sensor_id : Sensor ID of the virtual sensor
 * @param[out] orient_matrix     : Reference to the data buffer to store the orientation matrix
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_set_physical_sensor_info(uint8_t sensor_id,
                                                    const struct bhi360_system_param_orient_matrix *orient_matrix,
                                                    struct bhy2_dev *dev)
{
    int8_t rslt = BHY2_OK;

    if (dev == NULL || orient_matrix == NULL)
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        uint8_t tmp_matrix_buf[8]; /* Includes 2 reserved bytes */

        memset(tmp_matrix_buf, 0, sizeof(tmp_matrix_buf));

        tmp_matrix_buf[0] = BHY2_BYTE_TO_NIBBLE(&orient_matrix->c[0]);
        tmp_matrix_buf[1] = BHY2_BYTE_TO_NIBBLE(&orient_matrix->c[2]);
        tmp_matrix_buf[2] = BHY2_BYTE_TO_NIBBLE(&orient_matrix->c[4]);
        tmp_matrix_buf[3] = BHY2_BYTE_TO_NIBBLE(&orient_matrix->c[6]);
        tmp_matrix_buf[4] = orient_matrix->c[8] & 0x0F;

        rslt = bhy2_hif_set_parameter((uint16_t)(BHI360_SYSTEM_PARAM_PHY_SENSOR_INFO_BASE + sensor_id),
                                        tmp_matrix_buf,
                                        8,
                                        &dev->hif);
    }

    return rslt;
}

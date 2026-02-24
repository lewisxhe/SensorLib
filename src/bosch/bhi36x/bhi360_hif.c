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
* @file       bhi360_hif.c
* @date       2025-03-28
* @version    v2.2.0
*
*/

#include "bhi360_hif.h"

/*! Mask definitions for SPI read/write address */
#define BHI360_SPI_RD_MASK  UINT8_C(0x80)
#define BHI360_SPI_WR_MASK  UINT8_C(0x7F)

union bhi360_hif_float_u32
{
    bhi360_float as_float;
    uint32_t reg;
};

static void bhi360_hif_exec_cmd_generic_support(const uint16_t *cmd,
                                                uint8_t *cmd_buf,
                                                uint32_t *temp_length,
                                                uint32_t *total_length,
                                                const uint32_t *len,
                                                const uint32_t *pre_len,
                                                const uint32_t *cmd_len)
{
    *total_length = *pre_len + *len;

    if (*cmd_len)
    {
        *temp_length = *pre_len + *cmd_len;
    }
    else
    {
        *temp_length = *total_length;
    }

    /* Align 4 bytes */
    if (*temp_length % 4)
    {

        *temp_length = BHI360_ROUND_WORD_HIGHER(*temp_length);
    }

    cmd_buf[0] = (uint8_t)(*cmd & 0xFF);
    cmd_buf[1] = (uint8_t)((*cmd >> 8) & 0xFF);

    /* Length in word */
    if (*cmd == BHI360_CMD_UPLOAD_TO_PROGRAM_RAM)
    {
        cmd_buf[2] = (*temp_length / 4) & 0xFF;
        cmd_buf[3] = ((*temp_length / 4) >> 8) & 0xFF;
    }
    /* Length in byte */
    else
    {
        cmd_buf[2] = *temp_length & 0xFF;
        cmd_buf[3] = (*temp_length >> 8) & 0xFF;
    }
}

/*
 * If a command need several frames to upload to sensor,
 * this command only be used in the first frame.
 *
 * |------------------------------------------------|---------------|
 * |<--               frame one                  -->|<--frame two-->|
 * |------------------------------------------------|---------------|
 * |                  |<--pre_length-->|<--length-->|               |
 * |------------------|----------------|------------|---------------|
 * | cmd | cmd_length |   pre_payload  |   data 1   |     data 2    |
 * |-----------------------------------|------------|---------------|
 *                                     |          payload           |
 *                                     |------------|---------------|
 *                                     |<--     cmd_length       -->|
 *                                     |----------------------------|
 */
int8_t bhi360_hif_exec_cmd_generic(uint16_t cmd,
                                   const uint8_t *payload,
                                   uint32_t length,
                                   const uint8_t *pre_payload,
                                   uint32_t pre_length,
                                   uint32_t cmd_length,
                                   struct bhy2_hif_dev *hif)
{
    int8_t rslt = BHI360_OK;
    uint32_t remain, trans_len, copy_len, pos, total_len, temp_total_len, loop_remain_len, max_len;
    uint8_t command_buf[BHI360_COMMAND_PACKET_LEN];

    if ((hif != NULL) && !((length != 0) && (payload == NULL)) && !((pre_length != 0) && (pre_payload == NULL)))
    {
        if (hif->read_write_len != 0)
        {
            bhi360_hif_exec_cmd_generic_support(&cmd,
                                                command_buf,
                                                &temp_total_len,
                                                &total_len,
                                                &length,
                                                &pre_length,
                                                &cmd_length);
            pos = BHI360_COMMAND_HEADER_LEN;
            remain = total_len;
            loop_remain_len = remain + pos;
            max_len = BHI360_COMMAND_PACKET_LEN - BHI360_COMMAND_HEADER_LEN;

            if (hif->read_write_len < max_len)
            {
                max_len = hif->read_write_len;
            }

            while ((loop_remain_len > 0) && (rslt == BHI360_OK))
            {
                if ((remain + pos) > max_len)
                {
                    trans_len = max_len;
                    copy_len = max_len - pos;
                }
                else
                {
                    trans_len = remain + pos;
                    copy_len = remain;

                    /* Align to 4 bytes */
                    if (trans_len % 4)
                    {

                        trans_len = BHI360_ROUND_WORD_HIGHER(trans_len);
                    }

                    if (trans_len > (BHI360_COMMAND_PACKET_LEN - BHI360_COMMAND_HEADER_LEN))
                    {
                        trans_len = BHI360_COMMAND_PACKET_LEN - BHI360_COMMAND_HEADER_LEN;
                    }
                }

                if (copy_len > 0)
                {
                    if (remain >= (length + copy_len))
                    {
                        memcpy(&command_buf[pos], &pre_payload[total_len - remain], copy_len);
                    }
                    else if (remain > length)
                    {
                        memcpy(&command_buf[pos], &pre_payload[total_len - remain], remain - length);
                        memcpy(&command_buf[pos + remain - length], payload, copy_len - (remain - length));
                    }
                    else
                    {
                        memcpy(&command_buf[pos], &payload[length - remain], copy_len);
                    }
                }

                if ((trans_len - (pos + copy_len)) > 0)
                {
                    memset(&command_buf[pos + copy_len], 0, BHI360_COMMAND_PACKET_LEN - (pos + copy_len));
                }

                rslt = bhy2_hif_set_regs(BHI360_REG_CHAN_CMD, command_buf, trans_len, hif);
                if (rslt != BHI360_OK)
                {
                    break;
                }

                pos = 0;
                remain -= copy_len;
                loop_remain_len = remain;
            }
        }
        else
        {
            rslt = BHI360_E_INVALID_PARAM;
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

static int8_t bhi360_hif_get_fifo(uint8_t reg,
                                  uint8_t *fifo,
                                  uint32_t fifo_len,
                                  uint32_t *bytes_read,
                                  uint32_t *bytes_remain,
                                  struct bhy2_hif_dev *hif)
{
    return bhy2_hif_get_fifo(reg, fifo, fifo_len, bytes_read, bytes_remain, hif);
}

int8_t bhi360_hif_exec_cmd(uint16_t cmd, const uint8_t *payload, uint32_t length, struct bhy2_hif_dev *hif)
{
    return bhi360_hif_exec_cmd_generic(cmd, payload, length, NULL, 0, 0, hif);
}

int8_t bhi360_hif_get_nonwakeup_fifo(uint8_t *fifo,
                                     uint32_t fifo_len,
                                     uint32_t *bytes_read,
                                     uint32_t *bytes_remain,
                                     struct bhy2_hif_dev *hif)
{
    return bhi360_hif_get_fifo(BHI360_REG_CHAN_FIFO_NW, fifo, fifo_len, bytes_read, bytes_remain, hif);
}

int8_t bhi360_hif_set_orientation_matrix(uint8_t sensor_id,
                                         struct bhi360_system_param_orient_matrix orient_matrix,
                                         struct bhy2_hif_dev *hif)
{
    int8_t rslt;
    uint8_t tmp_matrix_buf[8]; /* Includes 2 reserved bytes */

    memset(tmp_matrix_buf, 0, sizeof(tmp_matrix_buf));

    tmp_matrix_buf[0] = BHY2_BYTE_TO_NIBBLE(&orient_matrix.c[0]);
    tmp_matrix_buf[1] = BHY2_BYTE_TO_NIBBLE(&orient_matrix.c[2]);
    tmp_matrix_buf[2] = BHY2_BYTE_TO_NIBBLE(&orient_matrix.c[4]);
    tmp_matrix_buf[3] = BHY2_BYTE_TO_NIBBLE(&orient_matrix.c[6]);
    tmp_matrix_buf[4] = orient_matrix.c[8] & 0x0F;

    rslt = bhy2_hif_set_parameter((uint16_t)(BHI360_PARAM_PHYSICAL_SENSOR_BASE + sensor_id), tmp_matrix_buf, 8, hif);

    return rslt;
}

/*lint -e{715} suppressed parameters not referenced info as all the calling functions are parsing the valid values to
 * all parameters*/
int8_t bhi360_hif_set_inject_data_mode(const uint8_t *payload,
                                       uint8_t payload_len,
                                       const uint8_t *work_buf,
                                       uint32_t work_buf_len,
                                       const uint32_t *actual_len,
                                       struct bhy2_hif_dev *hif)
{
    uint8_t tmp_buf;
    int8_t rslt;

    rslt = bhy2_hif_get_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
    if (rslt == BHI360_OK)
    {
        tmp_buf &= (uint8_t)(~(BHI360_HIF_CTRL_ASYNC_STATUS_CHANNEL));
        rslt = bhy2_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
        if (rslt == BHI360_OK)
        {
            /* Set injection mode */
            rslt = bhy2_hif_set_regs(BHI360_REG_CHAN_CMD, payload, payload_len, hif);
            if (rslt != BHI360_OK)
            {
                rslt = BHI360_E_NULL_PTR;
            }
        }
    }

    return rslt;
}

int8_t bhi360_hif_inject_data(const uint8_t *payload, uint32_t payload_len, struct bhy2_hif_dev *hif)
{
    return bhi360_hif_exec_cmd(BHI360_CMD_INJECT_DATA, payload, payload_len, hif);
}

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
                                   struct bhi360_hif_dev *hif)
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

                rslt = bhi360_hif_set_regs(BHI360_REG_CHAN_CMD, command_buf, trans_len, hif);
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

int8_t bhi360_hif_wait_status_ready(struct bhi360_hif_dev *hif)
{
    uint16_t retry;
    uint8_t int_status;
    int8_t rslt;

    /* Wait status ready */
    for (retry = 0; retry < BHI360_QUERY_PARAM_STATUS_READY_MAX_RETRY; ++retry)
    {
        rslt = bhi360_hif_get_interrupt_status(&int_status, hif);
        if (rslt == BHI360_OK)
        {
            if (int_status & BHI360_IST_MASK_STATUS)
            {
                rslt = BHI360_OK;
                break;
            }

            /* 10ms */
            rslt = bhi360_hif_delay_us(10000, hif);
            if (rslt != BHI360_OK)
            {
                return rslt;
            }
        }
        else
        {
            return rslt; /*break; */
        }
    }

    return rslt;
}

static int8_t bhi360_hif_check_boot_status_ram(struct bhi360_hif_dev *hif)
{
    int8_t rslt;
    uint16_t i;
    uint8_t boot_status;

    /* total 5s */
    for (i = 0; i < BHI360_BST_CHECK_RETRY; ++i)
    {
        /* 50ms */
        rslt = bhi360_hif_delay_us(50000, hif);
        if (rslt < 0)
        {
            return rslt;
        }

        rslt = bhi360_hif_get_regs(BHI360_REG_BOOT_STATUS, &boot_status, sizeof(boot_status), hif);
        if (rslt < 0)
        {
            return rslt;
        }

        if ((boot_status & BHI360_BST_HOST_INTERFACE_READY) && (boot_status & BHI360_BST_HOST_FW_VERIFY_DONE) &&
            (!(boot_status & BHI360_BST_HOST_FW_VERIFY_ERROR)))
        {
            break;
        }
    }

    if (i == BHI360_BST_CHECK_RETRY)
    {
        return BHI360_E_TIMEOUT;
    }

    return BHI360_OK;
}

static int8_t bhi360_hif_get_fifo(uint8_t reg,
                                  uint8_t *fifo,
                                  uint32_t fifo_len,
                                  uint32_t *bytes_read,
                                  uint32_t *bytes_remain,
                                  struct bhi360_hif_dev *hif)
{
    int8_t rslt = BHI360_OK;
    uint8_t n_bytes[2];
    uint32_t read_len;
    uint32_t read_write_len;
    uint32_t offset;

    if ((hif != NULL) && (fifo != NULL) && (bytes_read != NULL) && (bytes_remain != NULL))
    {
        read_write_len = hif->read_write_len;

        if (*bytes_remain == 0)
        {
            rslt = bhi360_hif_get_regs(reg, n_bytes, 2, hif); /* Read the the available size */
            *bytes_remain = BHI360_LE2U16(n_bytes);
        }

        if ((*bytes_remain != 0) && (rslt == BHI360_OK))
        {
            if (fifo_len < *bytes_remain)
            {
                *bytes_read = fifo_len;
            }
            else
            {
                *bytes_read = *bytes_remain;
            }

            read_len = *bytes_read;
            offset = 0;
            while (read_len > read_write_len)
            {
                rslt = bhi360_hif_get_regs(reg, &fifo[offset], read_write_len, hif);
                if (rslt != BHI360_OK)
                {
                    break;
                }

                read_len -= read_write_len;
                offset += read_write_len;
            }

            if (read_len != 0)
            {
                rslt = bhi360_hif_get_regs(reg, &fifo[offset], read_len, hif);
            }

            *bytes_remain -= *bytes_read;
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_init(enum bhi360_intf intf,
                       bhi360_read_fptr_t read,
                       bhi360_write_fptr_t write,
                       bhi360_delay_us_fptr_t delay_us,
                       uint32_t read_write_len,
                       void *intf_ptr,
                       struct bhi360_hif_dev *hif)
{
    int8_t rslt = BHI360_OK;

    if ((hif != NULL) && (read != NULL) && (write != NULL) && (delay_us != NULL))
    {
        hif->read = read;
        hif->write = write;
        hif->delay_us = delay_us;
        hif->intf_ptr = intf_ptr;
        hif->intf = intf;
        if (read_write_len % 4)
        {

            hif->read_write_len = BHI360_ROUND_WORD_LOWER(read_write_len);
        }
        else
        {
            hif->read_write_len = read_write_len;
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, struct bhi360_hif_dev *hif)
{
    int8_t rslt = BHI360_OK;

    if ((hif != NULL) && (hif->read != NULL) && (reg_data != NULL))
    {
        if (hif->intf == BHI360_SPI_INTERFACE)
        {
            reg_addr |= BHI360_SPI_RD_MASK;
        }

        hif->intf_rslt = hif->read(reg_addr, reg_data, length, hif->intf_ptr);
        if (hif->intf_rslt != BHI360_INTF_RET_SUCCESS)
        {
            rslt = BHI360_E_IO;
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_set_regs(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, struct bhi360_hif_dev *hif)
{
    int8_t rslt = BHI360_OK;

    if ((hif != NULL) && (hif->write != NULL) && (reg_data != NULL))
    {
        if (hif->intf == BHI360_SPI_INTERFACE)
        {
            reg_addr &= BHI360_SPI_WR_MASK;
        }

        hif->intf_rslt = hif->write(reg_addr, reg_data, length, hif->intf_ptr);
        if (hif->intf_rslt != BHI360_INTF_RET_SUCCESS)
        {
            rslt = BHI360_E_IO;
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_delay_us(uint32_t period_us, const struct bhi360_hif_dev *hif)
{
    int8_t rslt = BHI360_OK;

    if ((hif != NULL) && (hif->delay_us != NULL))
    {
        hif->delay_us(period_us, hif->intf_ptr);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_exec_cmd(uint16_t cmd, const uint8_t *payload, uint32_t length, struct bhi360_hif_dev *hif)
{
    return bhi360_hif_exec_cmd_generic(cmd, payload, length, NULL, 0, 0, hif);
}

int8_t bhi360_hif_get_parameter(uint16_t param,
                                uint8_t *payload,
                                uint32_t payload_len,
                                uint32_t *actual_len,
                                struct bhi360_hif_dev *hif)
{
    uint16_t code = 0;
    uint8_t prev_hif_ctrl, hif_ctrl;
    int8_t rslt = BHI360_OK;

    if ((hif != NULL) && (payload != NULL) && (actual_len != NULL))
    {
        *actual_len = 0;

        rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTERFACE_CTRL, &hif_ctrl, 1, hif);
        if (rslt == BHI360_OK)
        {
            prev_hif_ctrl = hif_ctrl;
            hif_ctrl &= (uint8_t)(~(BHI360_HIF_CTRL_ASYNC_STATUS_CHANNEL)); /* Change the Status FIFO to synchronous mode
                                                                           * */
            if (hif_ctrl != prev_hif_ctrl)
            {
                rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &hif_ctrl, 1, hif);
            }

            if (rslt == BHI360_OK)
            {
                rslt = bhi360_hif_exec_cmd(param | BHI360_PARAM_READ_MASK, NULL, 0, hif);

                if (rslt == BHI360_OK)
                {
                    rslt = bhi360_hif_wait_status_ready(hif);

                    if (rslt == BHI360_OK)
                    {
                        rslt = bhi360_hif_get_status_fifo(&code, payload, payload_len, actual_len, hif);

                        if (rslt == BHI360_OK)
                        {

                            if (hif_ctrl != prev_hif_ctrl)
                            {
                                hif_ctrl = prev_hif_ctrl;
                                rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &hif_ctrl, 1, hif);
                            }

                            if (rslt == BHI360_OK)
                            {
                                if (code != param)
                                {
                                    rslt = BHI360_E_TIMEOUT;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_set_parameter(uint16_t param, const uint8_t *payload, uint32_t length, struct bhi360_hif_dev *hif)
{
    return bhi360_hif_exec_cmd(param, payload, length, hif);
}

int8_t bhi360_hif_get_product_id(uint8_t *product_id, struct bhi360_hif_dev *hif)
{
    return bhi360_hif_get_regs(BHI360_REG_PRODUCT_ID, product_id, 1, hif);
}

int8_t bhi360_hif_get_chip_id(uint8_t *chip_id, struct bhi360_hif_dev *hif)
{
    return bhi360_hif_get_regs(BHI360_REG_CHIP_ID, chip_id, 1, hif);
}

int8_t bhi360_hif_get_rom_version(uint16_t *rom_version, struct bhi360_hif_dev *hif)
{
    uint8_t buffer[2];
    int8_t rslt;

    if (rom_version != NULL)
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_ROM_VERSION_0, buffer, 2, hif);
        *rom_version = BHI360_LE2U16(buffer);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_get_kernel_version(uint16_t *kernel_version, struct bhi360_hif_dev *hif)
{
    uint8_t buffer[2];
    int8_t rslt;

    if (kernel_version != NULL)
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_KERNEL_VERSION_0, buffer, 2, hif);
        *kernel_version = BHI360_LE2U16(buffer);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_get_user_version(uint16_t *user_version, struct bhi360_hif_dev *hif)
{
    uint8_t buffer[2];
    int8_t rslt;

    if (user_version != NULL)
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_USER_VERSION_0, buffer, 2, hif);
        *user_version = BHI360_LE2U16(buffer);
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_get_boot_status(uint8_t *boot_status, struct bhi360_hif_dev *hif)
{
    return bhi360_hif_get_regs(BHI360_REG_BOOT_STATUS, boot_status, 1, hif);
}

int8_t bhi360_hif_get_host_status(uint8_t *host_status, struct bhi360_hif_dev *hif)
{
    return bhi360_hif_get_regs(BHI360_REG_HOST_STATUS, host_status, 1, hif);
}

int8_t bhi360_hif_get_feature_status(uint8_t *feat_status, struct bhi360_hif_dev *hif)
{
    return bhi360_hif_get_regs(BHI360_REG_FEATURE_STATUS, feat_status, 1, hif);
}

int8_t bhi360_hif_get_interrupt_status(uint8_t *int_status, struct bhi360_hif_dev *hif)
{
    return bhi360_hif_get_regs(BHI360_REG_INT_STATUS, int_status, 1, hif);
}

int8_t bhi360_hif_get_fw_error(uint8_t *fw_error, struct bhi360_hif_dev *hif)
{
    return bhi360_hif_get_regs(BHI360_REG_ERROR_VALUE, fw_error, 1, hif);
}

int8_t bhi360_hif_reset(struct bhi360_hif_dev *hif)
{
    uint8_t reset_req = BHI360_REQUEST_RESET;
    uint8_t boot_status = 0;
    int8_t rslt;

    /* Timeout at 150ms (15 * 10000 microseconds) */
    uint16_t count = 15;

    rslt = bhi360_hif_set_regs(BHI360_REG_RESET_REQ, &reset_req, 1, hif);
    if (rslt == BHI360_OK)
    {
        while (count--)
        {
            rslt = bhi360_hif_delay_us(10000, hif);
            if (rslt == BHI360_OK)
            {
                rslt = bhi360_hif_get_regs(BHI360_REG_BOOT_STATUS, &boot_status, 1, hif);
                if (rslt == BHI360_OK)
                {
                    if (boot_status & BHI360_BST_HOST_INTERFACE_READY)
                    {
                        break;
                    }
                }
            }
        }

        if (!(boot_status & BHI360_BST_HOST_INTERFACE_READY))
        {
            rslt = BHI360_E_TIMEOUT;
        }
    }

    return rslt;
}

int8_t bhi360_hif_upload_firmware_to_ram(const uint8_t *firmware, uint32_t length, struct bhi360_hif_dev *hif)
{
    int8_t rslt = BHI360_OK;
    uint16_t magic;

    if ((hif != NULL) && (firmware != NULL))
    {
        magic = BHI360_LE2U16(firmware);
        if (magic != BHI360_FW_MAGIC)
        {
            rslt = BHI360_E_MAGIC;
        }
        else
        {
            rslt = bhi360_hif_exec_cmd(BHI360_CMD_UPLOAD_TO_PROGRAM_RAM, firmware, length, hif);
            if (rslt == BHI360_OK)
            {
                rslt = bhi360_hif_check_boot_status_ram(hif);
            }
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_upload_firmware_to_ram_partly(const uint8_t *firmware,
                                                uint32_t total_size,
                                                uint32_t cur_pos,
                                                uint32_t packet_len,
                                                struct bhi360_hif_dev *hif)
{
    int8_t rslt = BHI360_OK;
    uint16_t magic;

    if ((hif != NULL) && (firmware != NULL))
    {
        if (cur_pos == 0)
        {
            magic = BHI360_LE2U16(firmware);
            if (magic != BHI360_FW_MAGIC)
            {
                rslt = BHI360_E_MAGIC;
            }

            if (rslt == BHI360_OK)
            {
                rslt = bhi360_hif_exec_cmd_generic(BHI360_CMD_UPLOAD_TO_PROGRAM_RAM,
                                                   firmware,
                                                   packet_len,
                                                   NULL,
                                                   0,
                                                   total_size,
                                                   hif);
            }
        }
        else
        {
            rslt = bhi360_hif_set_regs(BHI360_REG_CHAN_CMD, firmware, packet_len, hif);
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_boot_program_ram(struct bhi360_hif_dev *hif)
{
    int8_t rslt;

    rslt = bhi360_hif_exec_cmd(BHI360_CMD_BOOT_PROGRAM_RAM, NULL, 0, hif);
    if (rslt == BHI360_OK)
    {
        rslt = bhi360_hif_check_boot_status_ram(hif);
    }

    return rslt;
}

int8_t bhi360_hif_get_wakeup_fifo(uint8_t *fifo,
                                  uint32_t fifo_len,
                                  uint32_t *bytes_read,
                                  uint32_t *bytes_remain,
                                  struct bhi360_hif_dev *hif)
{
    return bhi360_hif_get_fifo(BHI360_REG_CHAN_FIFO_W, fifo, fifo_len, bytes_read, bytes_remain, hif);
}

int8_t bhi360_hif_get_nonwakeup_fifo(uint8_t *fifo,
                                     uint32_t fifo_len,
                                     uint32_t *bytes_read,
                                     uint32_t *bytes_remain,
                                     struct bhi360_hif_dev *hif)
{
    return bhi360_hif_get_fifo(BHI360_REG_CHAN_FIFO_NW, fifo, fifo_len, bytes_read, bytes_remain, hif);
}

int8_t bhi360_hif_get_status_fifo(uint16_t *status_code,
                                  uint8_t *fifo,
                                  uint32_t fifo_len,
                                  uint32_t *bytes_remain,
                                  struct bhi360_hif_dev *hif)
{
    int8_t rslt;
    uint8_t tmp_fifo[4];

    rslt = bhi360_hif_get_regs(BHI360_REG_CHAN_STATUS, tmp_fifo, 4, hif);
    if (rslt == BHI360_OK)
    {
        *status_code = BHI360_LE2U16(&tmp_fifo[0]);
        *bytes_remain = BHI360_LE2U16(&tmp_fifo[2]);
        if (*bytes_remain != 0)
        {
            if (fifo_len < *bytes_remain)
            {
                rslt = BHI360_E_BUFFER;
            }
            else
            {
                rslt = bhi360_hif_get_regs(BHI360_REG_CHAN_STATUS, fifo, *bytes_remain, hif);
            }
        }
    }

    return rslt;
}

int8_t bhi360_hif_get_status_fifo_async(uint8_t *fifo,
                                        uint32_t fifo_len,
                                        uint32_t *bytes_read,
                                        uint32_t *bytes_remain,
                                        struct bhi360_hif_dev *hif)
{
    return bhi360_hif_get_fifo(BHI360_REG_CHAN_STATUS, fifo, fifo_len, bytes_read, bytes_remain, hif);
}

int8_t bhi360_hif_exec_sensor_conf_cmd(uint8_t sensor_id,
                                       bhi360_float sample_rate,
                                       uint32_t latency,
                                       struct bhi360_hif_dev *hif)
{
    uint8_t tmp_buf[8];
    union bhi360_hif_float_u32 sample_rate_u;

    sample_rate_u.as_float = sample_rate;

    /* Sample rate is 32bits, latency is 24bits */
    tmp_buf[0] = sensor_id;
    tmp_buf[1] = (uint8_t)(sample_rate_u.reg & 0xFF);
    tmp_buf[2] = (uint8_t)((sample_rate_u.reg >> 8) & 0xFF);
    tmp_buf[3] = (uint8_t)((sample_rate_u.reg >> 16) & 0xFF);
    tmp_buf[4] = (uint8_t)((sample_rate_u.reg >> 24) & 0xFF);
    tmp_buf[5] = (uint8_t)(latency & 0xFF);
    tmp_buf[6] = (uint8_t)((latency >> 8) & 0xFF);
    tmp_buf[7] = (uint8_t)((latency >> 16) & 0xFF);

    return bhi360_hif_exec_cmd(BHI360_CMD_CONFIG_SENSOR, tmp_buf, 8, hif);
}

int8_t bhi360_hif_set_fifo_flush(uint8_t sensor_id, struct bhi360_hif_dev *hif)
{
    uint8_t tmp_buf[4] = { sensor_id, 0, 0, 0 };

    return bhi360_hif_exec_cmd(BHI360_CMD_FIFO_FLUSH, tmp_buf, 4, hif);
}

int8_t bhi360_hif_exec_soft_passthrough(const uint8_t *payload,
                                        uint32_t payload_len,
                                        uint8_t *reg_data,
                                        uint32_t length,
                                        uint32_t *actual_len,
                                        struct bhi360_hif_dev *hif)
{
    uint16_t code = 0;
    uint8_t old_status, tmp_buf;
    int8_t rslt;

    /* Enter synchronous mode */
    rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
    if (rslt == BHI360_OK)
    {
        old_status = tmp_buf;
        tmp_buf &= (uint8_t)(~(BHI360_HIF_CTRL_ASYNC_STATUS_CHANNEL));
        rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
        if (rslt == BHI360_OK)
        {
            /* Execute the Software passthrough */
            rslt = bhi360_hif_exec_cmd(BHI360_CMD_SW_PASSTHROUGH, payload, payload_len, hif);
            if (rslt == BHI360_OK)
            {
                rslt = bhi360_hif_wait_status_ready(hif);
                if (rslt == BHI360_OK)
                {
                    /* Read back the response */
                    rslt = bhi360_hif_get_status_fifo(&code, reg_data, length, actual_len, hif);
                    if (rslt == BHI360_OK)
                    {
                        /* Set back to previous mode */
                        tmp_buf = old_status;
                        rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
                        if (rslt == BHI360_OK)
                        {
                            if (code != BHI360_STATUS_SW_PASS_THRU_RES)
                            {
                                rslt = BHI360_E_TIMEOUT;
                            }
                        }
                    }
                }
            }
        }
    }

    return rslt;
}

int8_t bhi360_hif_get_post_mortem(uint16_t *code,
                                  uint32_t *actual_len,
                                  uint8_t *post_mortem,
                                  uint32_t buffer_len,
                                  struct bhi360_hif_dev *hif)
{
    uint8_t old_status, tmp_buf;
    int8_t rslt;

    if (code != NULL)
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
        if (rslt == BHI360_OK)
        {
            old_status = tmp_buf;
            tmp_buf &= (uint8_t)(~(BHI360_HIF_CTRL_ASYNC_STATUS_CHANNEL));
            rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
            if (rslt == BHI360_OK)
            {
                rslt = bhi360_hif_exec_cmd(BHI360_CMD_REQ_POST_MORTEM_DATA, NULL, 0, hif);
                if (rslt == BHI360_OK)
                {
                    rslt = bhi360_hif_wait_status_ready(hif);
                    if (rslt == BHI360_OK)
                    {
                        rslt = bhi360_hif_get_status_fifo(code, post_mortem, buffer_len, actual_len, hif);
                        if (rslt == BHI360_OK)
                        {
                            tmp_buf = old_status;
                            rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
                            if (rslt == BHI360_OK)
                            {
                                if (*code != BHI360_STATUS_CRASH_DUMP)
                                {
                                    rslt = BHI360_E_TIMEOUT;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_do_self_test(uint8_t phys_sensor_id,
                               struct bhi360_self_test_resp *self_test_resp,
                               struct bhi360_hif_dev *hif)
{
    uint16_t code = 0;
    uint8_t old_status, tmp_buf, req[4], ret_data[16];
    uint32_t ret_len = 0;
    int8_t rslt;

    if (self_test_resp != NULL)
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
        if (rslt == BHI360_OK)
        {
            old_status = tmp_buf;
            tmp_buf &= (uint8_t)(~(BHI360_HIF_CTRL_ASYNC_STATUS_CHANNEL));
            rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
            if (rslt == BHI360_OK)
            {
                req[0] = phys_sensor_id;
                req[1] = req[2] = req[3] = 0;
                rslt = bhi360_hif_exec_cmd(BHI360_CMD_REQ_SELF_TEST, req, 4, hif);
                if (rslt == BHI360_OK)
                {
                    /* wait status ready */
                    rslt = bhi360_hif_wait_status_ready(hif);
                    if (rslt == BHI360_OK)
                    {
                        rslt = bhi360_hif_get_status_fifo(&code, ret_data, 16, &ret_len, hif);
                        if (rslt == BHI360_OK)
                        {
                            tmp_buf = old_status;
                            rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
                            if (rslt == BHI360_OK)
                            {
                                if (code != BHI360_STATUS_SELF_TEST_RES)
                                {
                                    rslt = BHI360_E_TIMEOUT;
                                }
                                else
                                {
                                    if (ret_data[0] != phys_sensor_id)
                                    {
                                        rslt = BHI360_E_INVALID_PARAM;
                                    }
                                    else
                                    {
                                        self_test_resp->test_status = ret_data[1];
                                        self_test_resp->x_offset = BHI360_LE2S16(&ret_data[2]);
                                        self_test_resp->y_offset = BHI360_LE2S16(&ret_data[4]);
                                        self_test_resp->z_offset = BHI360_LE2S16(&ret_data[6]);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_do_foc(uint8_t phys_sensor_id, struct bhi360_foc_resp *foc_resp, struct bhi360_hif_dev *hif)
{
    uint16_t code = 0;
    uint8_t old_status, tmp_buf, req[4], ret_data[16];
    uint32_t ret_len = 0;
    int8_t rslt;

    if (foc_resp != NULL)
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
        if (rslt == BHI360_OK)
        {
            old_status = tmp_buf;
            tmp_buf &= (uint8_t)(~(BHI360_HIF_CTRL_ASYNC_STATUS_CHANNEL));
            rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
            if (rslt == BHI360_OK)
            {
                req[0] = phys_sensor_id;
                req[1] = req[2] = req[3] = 0;
                rslt = bhi360_hif_exec_cmd(BHI360_CMD_REQ_FOC, req, 4, hif);
                if (rslt == BHI360_OK)
                {
                    rslt = bhi360_hif_wait_status_ready(hif);
                    if (rslt == BHI360_OK)
                    {
                        rslt = bhi360_hif_get_status_fifo(&code, ret_data, 16, &ret_len, hif);
                        if (rslt == BHI360_OK)
                        {
                            tmp_buf = old_status;
                            rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
                            if (rslt == BHI360_OK)
                            {
                                if (code != BHI360_STATUS_FOC_RES)
                                {
                                    rslt = BHI360_E_TIMEOUT;
                                }
                                else
                                {
                                    if (ret_data[0] != phys_sensor_id)
                                    {
                                        rslt = BHI360_E_INVALID_PARAM;
                                    }
                                    else
                                    {
                                        foc_resp->foc_status = ret_data[1];
                                        foc_resp->x_offset = BHI360_LE2S16(&ret_data[2]);
                                        foc_resp->y_offset = BHI360_LE2S16(&ret_data[4]);
                                        foc_resp->z_offset = BHI360_LE2S16(&ret_data[6]);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_request_hw_timestamp(struct bhi360_hif_dev *hif)
{
    uint8_t time_ev_req = 1;

    return bhi360_hif_set_regs(BHI360_REG_TIME_EV_REQ, &time_ev_req, 1, hif);
}

int8_t bhi360_hif_get_hw_timestamp(uint64_t *ts_ticks, struct bhi360_hif_dev *hif)
{
    int8_t rslt = BHI360_OK;
    uint8_t ts_regs[5];

    if (ts_ticks != NULL)
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTR_TIME_0, ts_regs, 5, hif);
        if (rslt == BHI360_OK)
        {
            *ts_ticks = BHI360_LE2U40(ts_regs);
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_req_and_get_hw_timestamp(uint64_t *ts_ticks, struct bhi360_hif_dev *hif)
{
    int8_t rslt;
    uint8_t retry;
    uint8_t tmp_buf[5];
    uint64_t ts_old;

    if (ts_ticks != NULL)
    {
        rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTR_TIME_0, tmp_buf, 5, hif);
        if (rslt == BHI360_OK)
        {
            ts_old = BHI360_LE2U40(tmp_buf);
            tmp_buf[0] = 1;
            rslt = bhi360_hif_set_regs(BHI360_REG_TIME_EV_REQ, tmp_buf, 1, hif);
            if (rslt == BHI360_OK)
            {
                for (retry = 0; retry < 5; retry++)
                {
                    rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTR_TIME_0, tmp_buf, 5, hif);
                    if (rslt == BHI360_OK)
                    {
                        *ts_ticks = BHI360_LE2U40(tmp_buf);
                        if (*ts_ticks != ts_old)
                        {
                            break;
                        }

                        rslt = bhi360_hif_delay_us(10, hif);
                        if (rslt != BHI360_OK)
                        {
                            break;
                        }
                    }
                }

                if ((retry >= 5) && (*ts_ticks == ts_old))
                {
                    rslt = BHI360_E_TIMEOUT;
                }
            }
        }
    }
    else
    {
        rslt = BHI360_E_NULL_PTR;
    }

    return rslt;
}

int8_t bhi360_hif_set_orientation_matrix(uint8_t sensor_id,
                                         struct bhi360_system_param_orient_matrix orient_matrix,
                                         struct bhi360_hif_dev *hif)
{
    int8_t rslt;
    uint8_t tmp_matrix_buf[8]; /* Includes 2 reserved bytes */

    memset(tmp_matrix_buf, 0, sizeof(tmp_matrix_buf));

    tmp_matrix_buf[0] = BHI360_BYTE_TO_NIBBLE(&orient_matrix.c[0]);
    tmp_matrix_buf[1] = BHI360_BYTE_TO_NIBBLE(&orient_matrix.c[2]);
    tmp_matrix_buf[2] = BHI360_BYTE_TO_NIBBLE(&orient_matrix.c[4]);
    tmp_matrix_buf[3] = BHI360_BYTE_TO_NIBBLE(&orient_matrix.c[6]);
    tmp_matrix_buf[4] = orient_matrix.c[8] & 0x0F;

    rslt = bhi360_hif_set_parameter((uint16_t)(BHI360_PARAM_PHYSICAL_SENSOR_BASE + sensor_id), tmp_matrix_buf, 8, hif);

    return rslt;
}

/*lint -e{715} suppressed parameters not referenced info as all the calling functions are parsing the valid values to
 * all parameters*/
int8_t bhi360_hif_set_inject_data_mode(const uint8_t *payload,
                                       uint8_t payload_len,
                                       const uint8_t *work_buf,
                                       uint32_t work_buf_len,
                                       const uint32_t *actual_len,
                                       struct bhi360_hif_dev *hif)
{
    uint8_t tmp_buf;
    int8_t rslt;

    rslt = bhi360_hif_get_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
    if (rslt == BHI360_OK)
    {
        tmp_buf &= (uint8_t)(~(BHI360_HIF_CTRL_ASYNC_STATUS_CHANNEL));
        rslt = bhi360_hif_set_regs(BHI360_REG_HOST_INTERFACE_CTRL, &tmp_buf, 1, hif);
        if (rslt == BHI360_OK)
        {
            /* Set injection mode */
            rslt = bhi360_hif_set_regs(BHI360_REG_CHAN_CMD, payload, payload_len, hif);
            if (rslt != BHI360_OK)
            {
                rslt = BHI360_E_NULL_PTR;
            }
        }
    }

    return rslt;
}

int8_t bhi360_hif_inject_data(const uint8_t *payload, uint32_t payload_len, struct bhi360_hif_dev *hif)
{
    return bhi360_hif_exec_cmd(BHI360_CMD_INJECT_DATA, payload, payload_len, hif);
}

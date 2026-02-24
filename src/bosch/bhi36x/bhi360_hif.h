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
* @file       bhi360_hif.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef __BHI360_HIF_H__
#define __BHI360_HIF_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhi360_defs.h"
#include "bhi360_system_param_defs.h"
#include "bhi360_param_defs.h"
#include "../bhi260x/bhy2_defs.h"
#include "../bhi260x/bhy2_hif.h"
/**
 * Function to execute a generic command
 * @param[in] cmd        : Command code
 * @param[in] payload    : Reference to the data buffer containing the command's payload
 * @param[in] length     : Length of the data buffer
 * @param[in] pre_payload: Reference to the data buffer containing the command's pre-payload
 * @param[in] pre_length : Length of the pre-payload
 * @param[in] cmd_length : Length of command
 * @param[in] hif        : HIF device reference
 * @return API error codes
 */
int8_t bhi360_hif_exec_cmd_generic(uint16_t cmd,
                                   const uint8_t *payload,
                                   uint32_t length,
                                   const uint8_t *pre_payload,
                                   uint32_t pre_length,
                                   uint32_t cmd_length,
                                   struct bhy2_hif_dev *hif);

/**
 * Function to execute a command
 * @param[in] cmd       : Command code
 * @param[in] payload   : Reference to the data buffer containing the command's payload
 * @param[in] length    : Length of the data buffer
 * @param[in] hif       : HIF device reference
 * @return API error codes
 */
int8_t bhi360_hif_exec_cmd(uint16_t cmd, const uint8_t *payload, uint32_t length, struct bhy2_hif_dev *hif);

/**
 * @brief Function to get data from the Non-wake-up FIFO
 * @param[out] fifo         : Reference to the data buffer to store data from the FIFO
 * @param[in] fifo_len      : Length of the data buffer
 * @param[out] bytes_read   : Number of bytes read into the data buffer
 * @param[out] bytes_remain : Bytes remaining in the sensor FIFO
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhi360_hif_get_nonwakeup_fifo(uint8_t *fifo,
                                     uint32_t fifo_len,
                                     uint32_t *bytes_read,
                                     uint32_t *bytes_remain,
                                     struct bhy2_hif_dev *hif);

                                     /**
 * @brief Function to get the orientation matrix of a virtual sensor
 * @param[in] sensor_id         : Sensor ID of the virtual sensor
 * @param[out] orient_matrix    : Reference to the data buffer to store the orientation matrix
 * @param[in] hif               : HIF device reference
 * @return API error codes
 */
int8_t bhi360_hif_set_orientation_matrix(uint8_t sensor_id,
                                         struct bhi360_system_param_orient_matrix orient_matrix,
                                         struct bhy2_hif_dev *hif);

/**
 * @brief Function to set the data injection mode
 * @param[in] payload       : Reference to the data buffer containing the host injection mode
 * @param[in] payload_len   : Length of the data buffer
 * @param[in/out] work_buf  : Reference to the work buffer
 * @param[in] work_buf_len  : Length of the work buffer
 * @param[out] actual_len   : Expected length of the work buffer
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhi360_hif_set_inject_data_mode(const uint8_t *payload,
                                       uint8_t payload_len,
                                       const uint8_t *work_buf,
                                       uint32_t work_buf_len,
                                       const uint32_t *actual_len,
                                       struct bhy2_hif_dev *hif);

/**
 * @brief Function to inject data
 * @param[in] payload       : Reference to the data buffer
 * @param[in] payload_len   : Length of the data buffer
 * @param[in] hif           : HIF device reference
 * @return API error codes
 */
int8_t bhi360_hif_inject_data(const uint8_t *payload, uint32_t payload_len, struct bhy2_hif_dev *hif);


/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_HIF_H__ */

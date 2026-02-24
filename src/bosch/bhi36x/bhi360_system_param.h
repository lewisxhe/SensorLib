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
* @file       bhi360_system_param.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef __BHI360_SYSTEM_PARAM_H__
#define __BHI360_SYSTEM_PARAM_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stdlib.h>

// #include "bhi360.h"
#include "../bhi260x/bhy2.h"
#include "bhi360_system_param_defs.h"

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
                                                  struct bhy2_dev *dev);

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
                                                  struct bhy2_dev *dev);

/*!
 * @brief Function set the Wakeup FIFO control register
 * @param[in] fifo_ctrl     : Fifo control
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_set_wakeup_fifo_control(const struct bhi360_system_param_fifo_control *fifo_ctrl,
                                                   struct bhy2_dev *dev);

/*!
 * @brief Function set the non-wakeup FIFO control register
 * @param[in] fifo_ctrl     : FIFO control
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_set_nonwakeup_fifo_control(const struct bhi360_system_param_fifo_control *fifo_ctrl,
                                                      struct bhy2_dev *dev);

/*!
 * @brief Function get the FIFO control register
 * @param[out] fifo_ctrl    : FIFO control
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_fifo_control(struct bhi360_system_param_fifo_control *fifo_ctrl, struct bhy2_dev *dev);

/*!
 * @brief Function get the firmware version
 * @param[out] fw_ver       : Reference to the data buffer to store firmware version
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_firmware_version(struct bhi360_system_param_firmware_version *fw_ver,
                                                struct bhy2_dev *dev);

/*!
 * @brief Function get the timestamp
 * @param[out] ts           : Reference to the data buffer to store timestamp
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_timestamps(struct bhi360_system_param_timestamp *ts, struct bhy2_dev *dev);

/*!
 * @brief Function get the virtual sensor present
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_virtual_sensor_present(struct bhy2_dev *dev);

/*!
 * @brief Function get the physical sensor present
 * @param[in] dev           : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_physical_sensor_present(struct bhy2_dev *dev);

/**
 * @brief Function to get the physical sensor information of a virtual sensor
 * @param[in] sensor_id : Sensor ID of the virtual sensor
 * @param[out] info     : Reference to the data buffer to store the physical sensor info
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_get_physical_sensor_info(uint8_t sensor_id,
                                                    struct bhi360_system_param_phys_sensor_info *info,
                                                    struct bhy2_dev *dev);

/**
 * @brief Function to set the orientation matrix of physical sensor
 * @param[in] sensor_id : Sensor ID of the virtual sensor
 * @param[out] orient_matrix     : Reference to the data buffer to store the orientation matrix
 * @param[in] dev       : Device reference
 * @return API error codes
 */
int8_t bhi360_system_param_set_physical_sensor_info(uint8_t sensor_id,
                                                    const struct bhi360_system_param_orient_matrix *orient_matrix,
                                                    struct bhy2_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif

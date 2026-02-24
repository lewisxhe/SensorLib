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
* @file       bhi360_head_orientation_param.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef _BHI360_HEAD_ORIENTATION_PARAM_H_
#define _BHI360_HEAD_ORIENTATION_PARAM_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stdlib.h>
#include <stdlib.h>

// #include "bhi360.h"
#include "../bhi260x/bhy2.h"
#include "bhi360_head_orientation_param_defs.h"

/*!
 * @brief To trigger the Head Misalignment Calibration
 *
 * @param[in] dev  Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_trigger_hmc_calibration(struct bhy2_dev *dev);

/*!
 * @brief To set the Head Misalignment Calibration Configuration
 *
 * @param[in] config  Reference to hold the Head Misalignment Calibration Configuration
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_set_hmc_configuration(
    const bhi360_head_orientation_param_misalignment_config *config,
    struct bhy2_dev *dev);

/*!
 * @brief To get the Head Misalignment Calibration Configuration
 *
 * @param[out] config  Reference to hold the Head Misalignment Calibration Configuration
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_get_hmc_configuration(bhi360_head_orientation_param_misalignment_config *config,
                                                           struct bhy2_dev *dev);

/*!
 * @brief To set Head Misalignment Calibration Configuration to default
 *
 * @param[in] dev  Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_set_default_hmc_cfg(struct bhy2_dev *dev);

/*!
 * @brief To get the Head Misalignment Calibrator Version
 *
 * @param[out] hmc_version  Referenceto hold the Head Misalignment Calibrator Version
 * @param[in] dev           Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_get_hmc_version(bhi360_head_orientation_param_ver *hmc_version,
                                                     struct bhy2_dev *dev);

/*!
 * @brief To set the Head Misalignment Quaternion Calibration Correction Configuration
 *
 * @param[in] config  Reference to hold the Head Misalignment Quaternion Calibration Correction Configuration
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_set_hmc_quat_cal_cor_cfg(
    const bhi360_head_orientation_param_misalignment_quat_corr *config,
    struct bhy2_dev *dev);

/*!
 * @brief To get the Head Misalignment Quaternion Calibration Correction Configuration
 *
 * @param[out] config  Reference to hold the Head Misalignment Quaternion Calibration Correction Configuration
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_get_hmc_quat_cal_cor_cfg(
    bhi360_head_orientation_param_misalignment_quat_corr *config,
    struct bhy2_dev *dev);

/*!
 * @brief To set the Head Misalignment Mode and Vector X value
 *
 * @param[in] config  Reference to hold the Head Misalignment Mode and Vector X value
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_set_hmc_mode_vector_x(const bhi360_head_misalignment_mode_vector_x *config,
                                                           struct bhy2_dev *dev);

/*!
 * @brief To get the Head Misalignment Mode and Vector X value
 *
 * @param[out] config  Reference to hold the Head Misalignment Mode and Vector X value
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_get_hmc_mode_vector_x(bhi360_head_misalignment_mode_vector_x *config,
                                                           struct bhy2_dev *dev);

/*!
 * @brief To set the Head Misalignment Quaternion Initial Head Correction
 *
 * @param[in] config  Reference to hold the Head Misalignment Quaternion Initial Head Correction
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_set_quat_init_head_corr(const uint8_t* config, struct bhy2_dev *dev);

/*!
 * @brief To get the Head Misalignment Quaternion Initial Head Correction
 *
 * @param[out] config  Reference to hold the Head Misalignment Quaternion Initial Head Correction
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_get_quat_init_head_corr(uint8_t *buffer, struct bhy2_dev *dev);

/*!
 * @brief To get the IMU/NDOF Head Orientation Version
 *
 * @param[out] ho_version Reference to hold the IMU/NDOF Head Orientation Version
 * @param[in] dev         Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_get_ho_version(bhi360_head_orientation_param_ver *ho_version,
                                                    struct bhy2_dev *dev);

/*!
 * @brief To set the Head Misalignment Euler Initial Head Correction
 *
 * @param[in] config  Reference to hold the Head Misalignment Euler Initial Head Correction
 * @param[in] dev     Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_set_eul_init_head_corr(const uint8_t *config, struct bhy2_dev *dev);

/*!
 * @brief To get the Head Misalignment Euler Initial Head Correction
 *
 * @param[out] config  Reference  to hold the Head Misalignment Euler Initial Head Correction
 * @param[in] dev      Device instance
 *
 * @return  API error codes
 *
 */
int8_t bhi360_head_orientation_param_get_eul_init_head_corr(uint8_t *config, struct bhy2_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHI360_HEAD_ORIENTATION_PARAM_H_ */

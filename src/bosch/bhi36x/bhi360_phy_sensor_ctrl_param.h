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
* @file       bhi360_phy_sensor_ctrl_param.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef __BHI360_PHY_SENSOR_CTRL_PARAM_H__
#define __BHI360_PHY_SENSOR_CTRL_PARAM_H__

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include "bhi360_phy_sensor_ctrl_param_defs.h"
#include "../bhi260x/bhy2.h"
/**
 * @brief Function to set accelerometer FOC calibration
 * @param[in] calib  : Reference to FOC calibration
 * @param[in] dev    : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_accel_set_foc_calibration(
    const bhi360_phy_sensor_ctrl_param_accel_fast_offset_calib * calib,
    struct bhy2_dev *dev);

/**
 * @brief Function to get accelerometer FOC calibration
 * @param[out] calib  : Reference to FOC calibration
 * @param[in]  dev    : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_accel_get_foc_calibration(
    bhi360_phy_sensor_ctrl_param_accel_fast_offset_calib * calib,
    struct bhy2_dev *dev);

/**
 * @brief Function to set accelerometer power mode
 * @param[in] mode  : Accelerometer power mode
 * @param[in] dev   : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_accel_set_power_mode(uint8_t mode, struct bhy2_dev *dev);

/**
 * @brief Function to get accelerometer mode
 * @param[out] mode  : Reference to accelerometer mode
 * @param[in]  dev   : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_accel_get_power_mode(uint8_t* mode, struct bhy2_dev *dev);

/**
 * @brief Function to set axis remapping for internal imu features
 * @param[in] remap  : Reference to axis remapping
 * @param[in] dev    : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_accel_set_axis_remapping(
    const bhi360_phy_sensor_ctrl_param_accel_axis_remap * remap,
    struct bhy2_dev *dev);

/**
 * @brief Function to get axis remapping for internal imu features
 * @param[in] remap  : Reference to axis remapping
 * @param[in] dev    : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_accel_get_axis_remapping(bhi360_phy_sensor_ctrl_param_accel_axis_remap * remap,
                                                             struct bhy2_dev *dev);

/**
 * @brief Function to trigger a NVM writing for accelerometer
 * @param[in] dev   : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_accel_trigger_nvm_writing(struct bhy2_dev *dev);

/**
 * @brief Function to get NVM writing status for accelerometer
 * @param[out] status  : Reference to NVM writing status
 * @param[in] dev      : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_accel_get_nvm_status(uint8_t* status, struct bhy2_dev *dev);

/**
 * @brief Function to set gyroscope FOC calibration
 * @param[in] calib  : Reference to FOC calibration
 * @param[in] dev    : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_set_foc_calibration(
    const bhi360_phy_sensor_ctrl_param_gyro_fast_offset_calib * calib,
    struct bhy2_dev *dev);

/**
 * @brief Function to get gyroscope FOC calibration
 * @param[out] calib  : Reference to FOC calibration
 * @param[in]  dev    : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_get_foc_calibration(
    bhi360_phy_sensor_ctrl_param_gyro_fast_offset_calib * calib,
    struct bhy2_dev *dev);

/**
 * @brief Function to set gyroscope OIS configuration
 * @param[in] config  : OIS configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_set_ois_config(uint8_t config, struct bhy2_dev *dev);

/**
 * @brief Function to get gyroscope OIS configuration
 * @param[out] config  : Reference to OIS configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_get_ois_config(uint8_t* config, struct bhy2_dev *dev);

/**
 * @brief Function to set gyroscope Fast start up configuration
 * @param[in] config  : Fast start up configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg(uint8_t config, struct bhy2_dev *dev);

/**
 * @brief Function to get gyroscope Fast start up configuration
 * @param[out] config  : Reference to Fast start up configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg(uint8_t* config, struct bhy2_dev *dev);

/**
 * @brief Function to start gyroscope Component ReTrim (CRT)
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_start_comp_retrim(struct bhy2_dev *dev);

/**
 * @brief Function to get gyroscope Component ReTrim (CRT) status
 * @param[out] crt  : Reference to CRT status
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_get_crt_status(bhi360_phy_sensor_ctrl_param_gyro_crt_status* crt,
                                                        struct bhy2_dev *dev);

/**
 * @brief Function to set gyroscope power mode
 * @param[in] mode  : Gyroscope power mode
 * @param[in] dev   : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_set_power_mode(uint8_t mode, struct bhy2_dev *dev);

/**
 * @brief Function to set gyroscope mode
 * @param[out] mode  : Reference to gyroscope mode
 * @param[in]  dev   : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_get_power_mode(uint8_t* mode, struct bhy2_dev *dev);

/**
 * @brief Function to set gyroscope timer auto trim configuration
 * @param[in] config  : Timer auto trim configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg(uint8_t config, struct bhy2_dev *dev);

/**
 * @brief Function to get gyroscope timer auto trim configuration
 * @param[out] config  : Reference to Timer auto trim configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg(uint8_t* config, struct bhy2_dev *dev);

/**
 * @brief Function to trigger a NVM writing for gyroscope
 * @param[in] dev   : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_trigger_nvm_writing(struct bhy2_dev *dev);

/**
 * @brief Function to get NVM writing status for gyroscope
 * @param[out] status  : Reference to NVM writing status
 * @param[in] dev      : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_gyro_get_nvm_status(uint8_t* status, struct bhy2_dev *dev);

/**
 * @brief Function to set magnetometer power mode
 * @param[in] mode  : Magnetometer power mode
 * @param[in] dev   : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_magnet_set_power_mode(uint8_t mode, struct bhy2_dev *dev);

/**
 * @brief Function to get magnetometer mode
 * @param[out] mode  : Reference to magnetometer mode
 * @param[in]  dev   : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_magnet_get_power_mode(uint8_t* mode, struct bhy2_dev *dev);

/**
 * @brief Function to set Wrist Wear Wakeup configuration
 * @param[in] config  : Reference to Wrist Wear Wakeup configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg(
    const bhi360_phy_sensor_ctrl_param_wrist_wear_wakeup* config,
    struct bhy2_dev *dev);

/**
 * @brief Function to get Wrist Wear Wakeup configuration
 * @param[out] config  : Reference to Wrist Wear Wakeup configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg(bhi360_phy_sensor_ctrl_param_wrist_wear_wakeup* config,
                                                              struct bhy2_dev *dev);

/**
 * @brief Function to set Any Motion configuration
 * @param[in] config  : Reference to Any Motion configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_set_any_motion_config(const bhi360_phy_sensor_ctrl_param_any_motion* config,
                                                          struct bhy2_dev *dev);

/**
 * @brief Function to get Any Motion configuration
 * @param[out] config  : Reference to Any Motion configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_get_any_motion_config(bhi360_phy_sensor_ctrl_param_any_motion* config,
                                                          struct bhy2_dev *dev);

/**
 * @brief Function to set No Motion configuration
 * @param[in] config  : Reference to No Motion configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_set_no_motion_config(const bhi360_phy_sensor_ctrl_param_no_motion* config,
                                                         struct bhy2_dev *dev);

/**
 * @brief Function to get No Motion configuration
 * @param[out] config  : Reference to No Motion configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_get_no_motion_config(bhi360_phy_sensor_ctrl_param_no_motion* config,
                                                         struct bhy2_dev *dev);

/**
 * @brief Function to set Wrist Gesture Detector configuration
 * @param[in] config  : Reference to Wrist Gesture Detector configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_set_wrist_gesture_cfg(
    const bhi360_phy_sensor_ctrl_param_wrist_gesture_detector* config,
    struct bhy2_dev *dev);

/**
 * @brief Function to get Wrist Gesture Detector configuration
 * @param[out] config  : Reference to Wrist Gesture Detector configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_get_wrist_gesture_cfg(bhi360_phy_sensor_ctrl_param_wrist_gesture_detector* config,
                                                          struct bhy2_dev *dev);

/**
 * @brief Function to set Barometric Pressure Type 1 configuration
 * @param[in] config  : Reference to Barometric Pressure Type 1 configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_baro_set_press_type_1_cfg(const bhi360_phy_sensor_ctrl_param_baro_type_1* config,
                                                              struct bhy2_dev *dev);

/**
 * @brief Function to get Barometric Pressure Type 1 configuration
 * @param[out] config  : Reference to Barometric Pressure Type 1 configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_baro_get_press_type_1_cfg(bhi360_phy_sensor_ctrl_param_baro_type_1* config,
                                                              struct bhy2_dev *dev);

/**
 * @brief Function to set Barometric Pressure Type 2 configuration
 * @param[in] config  : Reference to Barometric Pressure Type 2 configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_baro_set_press_type_2_cfg(const bhi360_phy_sensor_ctrl_param_baro_type_2* config,
                                                              struct bhy2_dev *dev);

/**
 * @brief Function to get Barometric Pressure Type 2 configuration
 * @param[out] config  : Reference to Barometric Pressure Type 2 configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_baro_get_press_type_2_cfg(bhi360_phy_sensor_ctrl_param_baro_type_2* config,
                                                              struct bhy2_dev *dev);

/**
 * @brief Function to set Step Counter configuration
 * @param[in] config  : Reference to Step Counter configuration
 * @param[in] dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_set_step_counter_config(const bhi360_phy_sensor_ctrl_param_step_counter* config,
                                                            struct bhy2_dev *dev);

/**
 * @brief Function to get Step Counter configuration
 * @param[out] config  : Reference to Step Counter configuration
 * @param[in]  dev     : Device instance
 * @return API error codes
 */
int8_t bhi360_phy_sensor_ctrl_param_get_step_counter_config(bhi360_phy_sensor_ctrl_param_step_counter* config,
                                                            struct bhy2_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* __BHI360_PHY_SENSOR_CTRL_PARAM_H__ */

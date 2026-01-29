/**
* Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
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
* @file       bma456w.h
* @date       2023-07-05
* @version    V2.29.0
*
*/

/**
 * \ingroup bma4xy
 * \defgroup bma456w BMA456W
 * @brief Sensor driver for BMA456W sensor
 */

#ifndef BMA456W_H
#define BMA456W_H

#ifdef __cplusplus
extern "C" {
#endif
#include "bma4.h"

/**\name Chip ID of BMA456W sensor */
#define BMA456W_CHIP_ID                                  UINT8_C(0x16)

/**\ Configuration ID start position of BMA456W sensor */
#define BMA456W_CONFIG_ID_START_ADDR                     UINT8_C(72)

/**\name Sensor feature size */
#define BMA456W_FEATURE_SIZE                             UINT8_C(76)
#define BMA456W_ANY_MOT_LEN                              UINT8_C(4)

/**\name Feature offset address */
#define BMA456W_ANY_MOT_OFFSET                           UINT8_C(0x00)
#define BMA456W_NO_MOT_OFFSET                            UINT8_C(0x04)
#define BMA456W_STEP_CNTR_PARAM_OFFSET                   UINT8_C(0x08)
#define BMA456W_STEP_CNTR_OFFSET                         UINT8_C(0x3A)
#define BMA456W_WRIST_WEAR_WAKEUP_OFFSET                 UINT8_C(0x3C)
#define BMA456W_WRIST_WEAR_WAKEUP_PARAM_OFFSET           UINT8_C(0x3E)
#define BMA456W_CONFIG_ID_OFFSET                         UINT8_C(0x48)
#define BMA456W_AXES_REMAP_OFFSET                        UINT8_C(0x4A)

/**\name Read/Write Lengths */
#define BMA456W_RD_WR_MIN_LEN                            UINT8_C(2)
#define BMA456W_NO_MOT_RD_WR_LEN                         (BMA456W_ANY_MOT_LEN + BMA456W_NO_MOT_OFFSET)

/*! @name Maximum valid read write length is size of config file array */
#define BMA456W_RD_WR_MAX_LEN                            ((uint16_t)sizeof(bma456w_config_file))

/**************************************************************/
/**\name    Step Counter/Detector/Activity */
/**************************************************************/
/**\name Step counter/activity enable macros */
#define BMA456W_STEP_CNTR_EN_MSK                         UINT8_C(0x10)
#define BMA456W_STEP_ACT_EN_MSK                          UINT8_C(0x20)

/**\name Step counter water-mark macros */
#define BMA456W_STEP_CNTR_WM_MSK                         UINT16_C(0x03FF)

/**\name Step counter reset macros */
#define BMA456W_STEP_CNTR_RST_POS                        UINT8_C(2)
#define BMA456W_STEP_CNTR_RST_MSK                        UINT8_C(0x04)

/**\name Step detector enable macros */
#define BMA456W_STEP_DETECTOR_EN_POS                     UINT8_C(3)
#define BMA456W_STEP_DETECTOR_EN_MSK                     UINT8_C(0x08)

/**\name Wrist-wear enable macros */
#define BMA456W_WRIST_WEAR_WAKEUP_EN_MSK                 UINT8_C(0x10)
#define BMA456W_WRIST_WEAR_WAKEUP_EN_POS                 UINT8_C(4)

/**\name Step count output length */
#define BMA456W_STEP_CNTR_DATA_SIZE                      UINT8_C(4)

/**************************************************************/
/**\name    Any/No-Motion */
/**************************************************************/
/**\name Any/No-motion threshold macros */
#define BMA456W_ANY_NO_MOT_THRES_MSK                     UINT16_C(0x07FF)

/**\name Any/No-motion duration macros */
#define BMA456W_ANY_NO_MOT_DUR_MSK                       UINT16_C(0x1FFF)

/**\name Any/No-motion enable macros */
#define BMA456W_ANY_NO_MOT_AXIS_EN_POS                   UINT8_C(0x0D)
#define BMA456W_ANY_NO_MOT_AXIS_EN_MSK                   UINT16_C(0xE000)

/**************************************************************/
/**\name    Wrist wear wakeup */
/**************************************************************/
/**\name Mask definition for minimum angle focus */
#define BMA456W_WRIST_WEAR_WAKEUP_MIN_ANG_FOCUS_MSK      UINT16_C(0xFFFF)

/**\name Mask definition for minimum angle non focus */
#define BMA456W_WRIST_WEAR_WAKEUP_MIN_ANG_NON_FOCUS_MSK  UINT16_C(0xFFFF)

/**\name Mask definition for angle landscape right */
#define BMA456W_WRIST_WEAR_WAKEUP_ANG_LSCAPE_RHT_MSK     UINT8_C(0xFF)

/**\name Mask definition for angle landscape left  */
#define BMA456W_WRIST_WEAR_WAKEUP_ANG_LSCAPE_LEFT_MSK    UINT16_C(0XFF00)

/**\name Position definition for angle landscape left */
#define BMA456W_WRIST_WEAR_WAKEUP_ANG_LSCAPE_LEFT_POS    UINT8_C(8)

/**\name Mask definition for angle portrait down  */
#define BMA456W_WRIST_WEAR_WAKEUP_ANG_PORTRAIT_DOWN_MSK  UINT8_C(0xFF)

/**\name Mask definition for angle portrait up */
#define BMA456W_WRIST_WEAR_WAKEUP_ANG_PORTRAIT_UP_MSK    UINT16_C(0xFF00)

/**\name Position definition for angle portrait up */
#define BMA456W_WRIST_WEAR_WAKEUP_ANG_PORTRAIT_UP_POS    UINT8_C(8)

/**\name Mask definition for minimum duration moved */
#define BMA456W_WRIST_WEAR_WAKEUP_MIN_DUR_MOVED_MSK      UINT8_C(0xFF)

/**\name Mask definition for minimum duration quite */
#define BMA456W_WRIST_WEAR_WAKEUP_MIN_DUR_QUITE_MSK      UINT16_C(0xFF00)

/**\name Position definition for minimum duration quite */
#define BMA456W_WRIST_WEAR_WAKEUP_MIN_DUR_QUITE_POS      UINT8_C(8)

/**************************************************************/
/**\name    User macros */
/**************************************************************/
/**\name Any-motion/No-motion axis enable macros */
#define BMA456W_X_AXIS_EN                                UINT8_C(0x01)
#define BMA456W_Y_AXIS_EN                                UINT8_C(0x02)
#define BMA456W_Z_AXIS_EN                                UINT8_C(0x04)
#define BMA456W_EN_ALL_AXIS                              UINT8_C(0x07)
#define BMA456W_DIS_ALL_AXIS                             UINT8_C(0x00)

/**\name Feature enable macros for the sensor */
#define BMA456W_STEP_CNTR                                UINT8_C(0x01)
#define BMA456W_STEP_ACT                                 UINT8_C(0x02)
#define BMA456W_WRIST_WEAR_WAKEUP                        UINT8_C(0x04)

/**\name Interrupt status macros */
#define BMA456W_STEP_CNTR_INT                            UINT8_C(0x02)
#define BMA456W_ACTIVITY_INT                             UINT8_C(0x04)
#define BMA456W_WRIST_WEAR_WAKEUP_INT                    UINT8_C(0x08)
#define BMA456W_ANY_MOT_INT                              UINT8_C(0x20)
#define BMA456W_NO_MOT_INT                               UINT8_C(0x40)
#define BMA456W_ERROR_INT                                UINT8_C(0x80)

/**\name Activity recognition macros */
#define BMA456W_USER_STATIONARY                          UINT8_C(0x00)
#define BMA456W_USER_WALKING                             UINT8_C(0x01)
#define BMA456W_USER_RUNNING                             UINT8_C(0x02)
#define BMA456W_STATE_INVALID                            UINT8_C(0x03)

/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/

/*!
 * @brief Any/No-motion configuration
 */
struct bma456w_any_no_mot_config
{
    /*! Expressed in 50 Hz samples (20 ms) */
    uint16_t duration;

    /*! Threshold value for Any-motion/No-motion detection in
     * 5.11g format
     */
    uint16_t threshold;

    /*! To enable selected axes */
    uint8_t axes_en;
};

/*!
 * @brief Step counter param settings
 */
struct bma456w_stepcounter_settings
{
    /*! Step Counter param 1 */
    uint16_t param1;

    /*! Step Counter param 2 */
    uint16_t param2;

    /*! Step Counter param 3 */
    uint16_t param3;

    /*! Step Counter param 4 */
    uint16_t param4;

    /*! Step Counter param 5 */
    uint16_t param5;

    /*! Step Counter param 6 */
    uint16_t param6;

    /*! Step Counter param 7 */
    uint16_t param7;

    /*! Step Counter param 8 */
    uint16_t param8;

    /*! Step Counter param 9 */
    uint16_t param9;

    /*! Step Counter param 10 */
    uint16_t param10;

    /*! Step Counter param 11 */
    uint16_t param11;

    /*! Step Counter param 12 */
    uint16_t param12;

    /*! Step Counter param 13 */
    uint16_t param13;

    /*! Step Counter param 14 */
    uint16_t param14;

    /*! Step Counter param 15 */
    uint16_t param15;

    /*! Step Counter param 16 */
    uint16_t param16;

    /*! Step Counter param 17 */
    uint16_t param17;

    /*! Step Counter param 18 */
    uint16_t param18;

    /*! Step Counter param 19 */
    uint16_t param19;

    /*! Step Counter param 20 */
    uint16_t param20;

    /*! Step Counter param 21 */
    uint16_t param21;

    /*! Step Counter param 22 */
    uint16_t param22;

    /*! Step Counter param 23 */
    uint16_t param23;

    /*! Step Counter param 24 */
    uint16_t param24;

    /*! Step Counter param 25 */
    uint16_t param25;
};

/*!
 * @brief Wrist wear wakeup param settings
 */
struct bma456w_wrist_wear_wakeup_params
{
    /*! Cosine of minimum expected attitude change of the
     * device within 1 second time window
     * when moving within focus position. */
    uint16_t min_angle_focus;

    /*!Cosine of minimum expected attitude change of the
     * device within 1 second time window
     * when moving from non-focus to focus position.*/
    uint16_t min_angle_non_focus;

    /*! Sine of the maximum allowed tilt angle in
     * landscape right direction of the device,
     * when it is in focus position */
    uint8_t angle_landscape_right;

    /*! Sine of the maximum allowed tilt angle in
     * landscape left direction of the device,
     * when it is in focus position */
    uint8_t angle_landscape_left;

    /*! Sine of the maximum allowed forward
     * tilt angle in portrait up direction
     * of the device, when it is in focus position */
    uint8_t angle_portrait_up;

    /*! Sine of the maximum allowed backward tilt
     * angle in portrait down direction of
     * the device, when it is in focus position */
    uint8_t angle_portrait_down;

    /*! Minimum duration the arm should
     * be moved while performing gesture.*/
    uint8_t min_dur_moved;

    /*! Minimum duration the arm should be
     * static between two consecutive gestures */
    uint8_t min_dur_quite;
};

/***************************************************************************/

/*!     BMA456W User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bma456w
 * \defgroup bma456wApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bma456wApiInit
 * \page bma456w_api_bma456w_init bma456w_init
 * \code
 * int8_t bma456w_init(struct bma4_dev *dev);
 * \endcode
 * @details This API is the entry point.
 * Call this API before using all other APIs.
 * This API reads the chip-id of the sensor and sets the resolution.
 *
 * @param[in,out] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_init(struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiConfig ConfigFile
 * @brief Write binary configuration in the sensor
 */

/*!
 * \ingroup bma456wApiConfig
 * \page bma456w_api_bma456w_write_config_file bma456w_write_config_file
 * \code
 * int8_t bma456w_write_config_file(struct bma4_dev *dev);
 * \endcode
 * @details This API is used to upload the config file to enable the features of
 * the sensor.
 *
 * @param[in] dev : Structure instance of bma4_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_write_config_file(struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiConfigId ConfigId
 * @brief Get Configuration ID of the sensor
 */

/*!
 * \ingroup bma456wApiConfig
 * \page bma456w_api_bma456w_get_config_id bma456w_get_config_id
 * \code
 * int8_t bma456w_get_config_id(uint16_t *config_id, struct bma4_dev *dev);
 * \endcode
 * @details This API is used to get the configuration id of the sensor.
 *
 * @param[out] config_id : Pointer variable used to store the configuration id.
 * @param[in] dev : Structure instance of bma4_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_get_config_id(uint16_t *config_id, struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiMapInt Map / Unmap Interrupt
 * @brief Map / Unmap user provided interrupt to interrupt pin1 or pin2 of the sensor
 */

/*!
 * \ingroup bma456wApiMapInt
 * \page bma456w_api_bma456w_map_interrupt bma456w_map_interrupt
 * \code
 * int8_t bma456w_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);
 * \endcode
 * @details This API sets/unsets the user provided interrupt to either
 * interrupt pin1 or pin2 in the sensor.
 *
 * @param[in] int_line: Variable to select either interrupt pin1 or pin2.
 *
 *@verbatim
 *  int_line    |   Macros
 *  ------------|-------------------
 *  0x00        |  BMA4_INTR1_MAP
 *  0x01        |  BMA4_INTR2_MAP
 *@endverbatim
 *
 * @param[in] int_map : Variable to specify the interrupts.
 * @param[in] enable : Variable to specify mapping or unmapping of interrupts.
 *
 *@verbatim
 *  enable  |   Macros
 *  --------|-------------------
 *  0x00    |  BMA4_DISABLE
 *  0x01    |  BMA4_ENABLE
 *@endverbatim
 *
 * @param[in] dev : Structure instance of bma4_dev.
 *
 * @note Below macros specify the interrupts.
 *
 * Feature Interrupts
 *  - BMA456W_STEP_CNTR_INT
 *  - BMA456W_ACTIVITY_INT
 *  - BMA456W_WRIST_WEAR_INT
 *  - BMA456W_ANY_MOT_INT
 *  - BMA456W_NO_MOT_INT
 *  - BMA456W_ERROR_INT
 *
 * Hardware Interrupts
 *  - BMA4_FIFO_FULL_INT
 *  - BMA4_FIFO_WM_INT
 *  - BMA4_DATA_RDY_INT
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiIntS Interrupt Status
 * @brief Read interrupt status of the sensor
 */

/*!
 * \ingroup bma456wApiIntS
 * \page bma456w_api_bma456w_read_int_status bma456w_read_int_status
 * \code
 * int8_t bma456w_read_int_status(uint16_t *int_status, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the bma456w interrupt status from the sensor.
 *
 * @param[out] int_status : Variable to store the interrupt status read from
 * the sensor.
 * @param[in] dev : Structure instance of bma4_dev.
 *
 *  @note Below macros are used to check the interrupt status.
 *
 * Feature Interrupts
 *  - BMA456W_STEP_CNTR_INT
 *  - BMA456W_ACTIVITY_INT
 *  - BMA456W_WRIST_WEAR_INT
 *  - BMA456W_ANY_MOT_INT
 *  - BMA456W_NO_MOT_INT
 *  - BMA456W_ERROR_INT
 *
 * Hardware Interrupts
 *  - BMA4_FIFO_FULL_INT
 *  - BMA4_FIFO_WM_INT
 *  - BMA4_DATA_RDY_INT
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_read_int_status(uint16_t *int_status, struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiFeat Sensor Feature
 * @brief Enables / Disables features of the sensor
 */

/*!
 * \ingroup bma456wApiFeat
 * \page bma456w_api_bma456w_feature_enable bma456w_feature_enable
 * \code
 * int8_t bma456w_feature_enable(uint8_t feature, uint8_t enable, struct bma4_dev *dev);
 * \endcode
 * @details This API enables/disables the features of the sensor.
 *
 * @param[in] feature : Variable to specify the features which are to be set in
 * bma456w sensor.
 * @param[in] enable : Variable which specifies whether to enable or disable the
 * features in the bma456w sensor.
 *
 *@verbatim
 *  enable  |   Macros
 *  --------|-------------------
 *  0x00    |  BMA4_DISABLE
 *  0x01    |  BMA4_ENABLE
 *@endverbatim
 *
 * @param[in] dev : Structure instance of bma4_dev.
 *
 * @note User should use the below macros to enable or disable the
 * features of bma456w sensor
 *
 *    - BMA456W_STEP_CNTR
 *    - BMA456W_ACTIVITY
 *    - BMA456W_WAKEUP
 *    - BMA456W_WRIST_WEAR
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_feature_enable(uint8_t feature, uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiRemap Remap Axes
 * @brief Set / Get x, y and z axis re-mapping in the sensor
 */

/*!
 * \ingroup bma456wApiRemap
 * \page bma456w_api_bma456w_set_remap_axes bma456w_set_remap_axes
 * \code
 * int8_t bma456w_set_remap_axes(const struct bma4_remap *remap_data, struct bma4_dev *dev);
 * \endcode
 * @details This API performs x, y and z axis remapping in the sensor.
 *
 * @param[in] remap_axes : Structure instance of bma4_remap
 * @param[in] dev        : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_set_remap_axes(const struct bma4_remap *remap_axes, struct bma4_dev *dev);

/*!
 * \ingroup bma456wApiRemap
 * \page bma456w_api_bma456w_get_remap_axes bma456w_get_remap_axes
 * \code
 * int8_t bma456w_get_remap_axes(struct bma4_remap *remap_data, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the x, y and z axis remap data from the sensor.
 *
 * @param[out] remap_axes : Structure instance of bma4_remap
 * @param[in] dev         : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_get_remap_axes(struct bma4_remap *remap_axes, struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiStepC Step counter
 * @brief Operations of step counter feature of the sensor
 */

/*!
 * \ingroup bma456wApiStepC
 * \page bma456w_api_bma456w_step_counter_set_watermark bma456w_step_counter_set_watermark
 * \code
 * int8_t bma456w_step_counter_set_watermark(uint16_t step_counter_wm, struct bma4_dev *dev);
 * \endcode
 * @details This API sets the watermark level for step counter interrupt in
 * the sensor.
 *
 * @param[in] step_counter_wm : Variable which specifies watermark level
 * count
 * @note Valid values are from 1 to 1023
 * @note Value 0 is used for step detector interrupt
 *
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_step_counter_set_watermark(uint16_t step_counter_wm, struct bma4_dev *dev);

/*!
 * \ingroup bma456wApiStepC
 * \page bma456w_api_bma456w_step_counter_get_watermark bma456w_step_counter_get_watermark
 * \code
 * int8_t bma456w_step_counter_get_watermark(uint16_t *step_counter_wm, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the water mark level set for step counter interrupt
 * in the sensor
 *
 * @param[out] step_counter_wm : Pointer variable which stores the water mark
 * level read from the sensor.
 * @note valid values are from 1 to 1023
 * @note value 0 is used for step detector interrupt
 *
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_step_counter_get_watermark(uint16_t *step_counter_wm, struct bma4_dev *dev);

/*!
 * \ingroup bma456wApiStepC
 * \page bma456w_api_bma456w_reset_step_counter bma456w_reset_step_counter
 * \code
 * int8_t bma456w_reset_step_counter(struct bma4_dev *dev);
 * \endcode
 * @details This API resets the counted steps of step counter.
 *
 * @param[in] dev : structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_reset_step_counter(struct bma4_dev *dev);

/*!
 * \ingroup bma456wApiStepC
 * \page bma456w_api_bma456w_step_counter_output bma456w_step_counter_output
 * \code
 * int8_t bma456w_step_counter_output(uint32_t *step_count, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the number of counted steps of the step counter
 * feature from the sensor.
 *
 * @param[out] step_count : Pointer variable which stores counted steps
 * read from the sensor.
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_step_counter_output(uint32_t *step_count, struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiAct Activity Feature
 * @brief Get output for activity feature of the sensor
 */

/*!
 * \ingroup bma456wApiAct
 * \page bma456w_api_bma456w_activity_output bma456w_activity_output
 * \code
 * int8_t bma456w_activity_output(uint8_t *activity, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the output for activity feature.
 *
 * @param[out] activity : Pointer variable which stores activity output read
 * from the sensor.
 *
 *@verbatim
 *       activity |   State
 *  --------------|------------------------
 *        0x00    | BMA456W_USER_STATIONARY
 *        0x01    | BMA456W_USER_WALKING
 *        0x02    | BMA456W_USER_RUNNING
 *        0x03    | BMA456W_STATE_INVALID
 *@endverbatim
 *
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_activity_output(uint8_t *activity, struct bma4_dev *dev);

/*!
 * \ingroup bma456wApiStepC
 * \page bma456w_api_bma456w_stepcounter_get_parameter bma456w_stepcounter_get_parameter
 * \code
 * int8_t bma456w_stepcounter_get_parameter(struct bma456w_stepcounter_settings *setting, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the parameter1 to parameter7 settings of the step
 * counter feature.
 *
 * @param[out] setting : Pointer to structure variable which stores the
 * parameter1 to parameter7 read from the sensor.
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_stepcounter_get_parameter(struct bma456w_stepcounter_settings *setting, struct bma4_dev *dev);

/*!
 * \ingroup bma456wApiStepC
 * \page bma456w_api_bma456w_stepcounter_set_parameter bma456w_stepcounter_set_parameter
 * \code
 * int8_t bma456w_stepcounter_set_parameter(const struct bma456w_stepcounter_settings *setting, struct bma4_dev *dev);
 * \endcode
 * @details This API sets the parameter1 to parameter7 settings of the step
 * counter feature in the sensor.
 *
 * @param[in] setting : Pointer to structure variable which stores the
 * parameter1 to parameter7 settings read from the sensor.
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_stepcounter_set_parameter(const struct bma456w_stepcounter_settings *setting, struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiStepD Step detector
 * @brief Operations of step detector feature of the sensor
 */

/*!
 * \ingroup bma456wApiStepD
 * \page bma456w_api_bma421_step_detector_enable bma456w_step_detector_enable
 * \code
 * int8_t bma456w_step_detector_enable(uint8_t enable, struct bma4_dev *dev);
 * \endcode
 * @details This API enables or disables the step detector feature in the
 * sensor.
 *
 * @param[in] enable : Variable used to enable or disable step detector
 *
 *@verbatim
 *  enable  |   Macros
 *  --------|-------------------
 *  0x00    |  BMA4_DISABLE
 *  0x01    |  BMA4_ENABLE
 *@endverbatim
 *
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_step_detector_enable(uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiAnyMot Any motion Feature
 * @brief Functions of Any motion feature of the sensor
 */

/*!
 * \ingroup bma456wApiAnyMot
 * \page bma456w_api_bma456w_set_any_motion_config bma456w_set_any_motion_config
 * \code
 * int8_t bma456w_set_any_motion_config(const struct bma456w_anymotion_config *any_motion, struct bma4_dev *dev);
 * \endcode
 * @details This API sets the configuration of any-motion feature in the sensor
 * This API enables/disables the any-motion feature according to the axis set.
 *
 * @param[in] any_mot           : Pointer to structure variable to configure
 *                                any-motion.
 *
 * @verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        Defines the number of
 *                                 |        consecutive data points for
 *                                 |        which the threshold condition
 *         duration                |        must be respected, for interrupt
 *                                 |        assertion. It is expressed in
 *                                 |        50 Hz samples (20 ms).
 *                                 |        Range is 0 to 163sec.
 *                                 |        Default value is 5 = 100ms.
 * --------------------------------|----------------------------------------
 *                                 |        Slope threshold value for
 *                                 |        Any-motion detection
 *         threshold               |        in 5.11g format.
 *                                 |        Range is 0 to 1g.
 *                                 |        Default value is 0xAA = 83mg.
 * --------------------------------|----------------------------------------
 *                                 |        Enables the feature on a per-axis
 *         axis_en                 |        basis.
 * ---------------------------------------------------------------------------
 * @endverbatim
 *
 *@verbatim
 *  Value    |  axis_en
 *  ---------|-------------------------
 *  0x00     |  BMA456W_ANY_NO_MOT_ALL_AXIS_DIS
 *  0x01     |  BMA456W_ANY_NO_MOT_X_AXIS_EN
 *  0x02     |  BMA456W_ANY_NO_MOT_Y_AXIS_EN
 *  0x04     |  BMA456W_ANY_NO_MOT_Z_AXIS_EN
 *  0x07     |  BMA456W_ANY_NO_MOT_ALL_AXIS_EN
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_set_any_mot_config(const struct bma456w_any_no_mot_config *any_mot, struct bma4_dev *dev);

/*!
 * \ingroup bma456wApiAnyMot
 * \page bma456w_api_bma456w_get_any_motion_config bma456w_get_any_motion_config
 * \code
 * int8_t bma456w_get_any_motion_config(struct bma456w_anymotion_config *any_motion, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the configuration of any-motion feature from the
 * sensor.
 *
 * @param[out] any_mot        : Pointer to structure variable to configure
 *                              any-motion.
 *
 * @verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        Defines the number of
 *                                 |        consecutive data points for
 *                                 |        which the threshold condition
 *         duration                |        must be respected, for interrupt
 *                                 |        assertion. It is expressed in
 *                                 |        50 Hz samples (20 ms).
 *                                 |        Range is 0 to 163sec.
 *                                 |        Default value is 5 = 100ms.
 * --------------------------------|----------------------------------------
 *                                 |        Slope threshold value for
 *                                 |        Any-motion detection
 *         threshold               |        in 5.11g format.
 *                                 |        Range is 0 to 1g.
 *                                 |        Default value is 0xAA = 83mg.
 * --------------------------------|-----------------------------------------
 *                                 |        Enables the feature on a per-axis
 *         axis_en                 |        basis.
 * ---------------------------------------------------------------------------
 * @endverbatim
 *
 *@verbatim
 *  Value    |  axis_en
 *  ---------|-------------------------
 *  0x00     |  BMA456W_ANY_NO_MOT_ALL_AXIS_DIS
 *  0x01     |  BMA456W_ANY_NO_MOT_X_AXIS_EN
 *  0x02     |  BMA456W_ANY_NO_MOT_Y_AXIS_EN
 *  0x04     |  BMA456W_ANY_NO_MOT_Z_AXIS_EN
 *  0x07     |  BMA456W_ANY_NO_MOT_ALL_AXIS_EN
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_get_any_mot_config(struct bma456w_any_no_mot_config *any_mot, struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiNomot No-Motion Feature
 * @brief Operations of no-motion feature of the sensor
 */

/*!
 * \ingroup bma456wApiNomot
 * \page bma456w_api_bma456w_set_no_motion_config bma456w_set_no_motion_config
 * \code
 * int8_t bma456w_set_no_motion_config(const struct bma456w_nomotion_config *no_motion, struct bma4_dev *dev);
 * \endcode
 * @details This API sets the configuration of no-motion feature in the sensor
 * This API enables/disables the no-motion feature according to the axis set.
 *
 * @param[in] no_mot                : Pointer to structure variable to configure
 *                                  no-motion.
 *
 * @verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        Defines the number of
 *                                 |        consecutive data points for
 *                                 |        which the threshold condition
 *         duration                |        must be respected, for interrupt
 *                                 |        assertion. It is expressed in
 *                                 |        50 Hz samples (20 ms).
 *                                 |        Range is 0 to 163sec.
 *                                 |        Default value is 5 = 100ms.
 * --------------------------------|----------------------------------------
 *                                 |        Slope threshold value for
 *                                 |        No-motion detection
 *         threshold               |        in 5.11g format.
 *                                 |        Range is 0 to 1g.
 *                                 |        Default value is 0xAA = 83mg.
 * --------------------------------|----------------------------------------
 *                                 |        Enables the feature on a per-axis
 *         axis_en                 |        basis.
 * ---------------------------------------------------------------------------
 * @endverbatim
 *
 *@verbatim
 *  Value    |  axis_en
 *  ---------|-------------------------
 *  0x00     |  BMA456W_ANY_NO_MOT_ALL_AXIS_DIS
 *  0x01     |  BMA456W_ANY_NO_MOT_X_AXIS_EN
 *  0x02     |  BMA456W_ANY_NO_MOT_Y_AXIS_EN
 *  0x04     |  BMA456W_ANY_NO_MOT_Z_AXIS_EN
 *  0x07     |  BMA456W_ANY_NO_MOT_ALL_AXIS_EN
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_set_no_mot_config(const struct bma456w_any_no_mot_config *no_mot, struct bma4_dev *dev);

/*!
 * \ingroup bma456wApiNomot
 * \page bma456w_api_bma456w_get_no_motion_config bma456w_get_no_motion_config
 * \code
 * int8_t bma456w_get_no_motion_config(struct bma456w_nomotion_config *no_motion, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the configuration of no-motion feature from the
 * sensor.
 *
 * @param[out] no_mot        : Pointer to structure variable to configure
 *                              no-motion.
 *
 * @verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        Defines the number of
 *                                 |        consecutive data points for
 *                                 |        which the threshold condition
 *         duration                |        must be respected, for interrupt
 *                                 |        assertion. It is expressed in
 *                                 |        50 Hz samples (20 ms).
 *                                 |        Range is 0 to 163sec.
 *                                 |        Default value is 5 = 100ms.
 * --------------------------------|----------------------------------------
 *                                 |        Slope threshold value for
 *                                 |        No-motion detection
 *         threshold               |        in 5.11g format.
 *                                 |        Range is 0 to 1g.
 *                                 |        Default value is 0xAA = 83mg.
 * --------------------------------|-----------------------------------------
 *                                 |        Enables the feature on a per-axis
 *         axis_en                 |        basis.
 * ---------------------------------------------------------------------------
 * @endverbatim
 *
 *@verbatim
 *  Value    |  axis_en
 *  ---------|-------------------------
 *  0x00     |  BMA456W_ANY_NO_MOT_ALL_AXIS_DIS
 *  0x01     |  BMA456W_ANY_NO_MOT_X_AXIS_EN
 *  0x02     |  BMA456W_ANY_NO_MOT_Y_AXIS_EN
 *  0x04     |  BMA456W_ANY_NO_MOT_Z_AXIS_EN
 *  0x07     |  BMA456W_ANY_NO_MOT_ALL_AXIS_EN
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_get_no_mot_config(struct bma456w_any_no_mot_config *no_mot, struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiVersionConfig Version Config
 * @brief Get version configuration of the sensor
 */

/*!
 * \ingroup bma456wApiVersionConfig
 * \page bma456w_api_bma456w_get_version_config bma456w_get_version_config
 * \code
 * int8_t bma456w_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev);
 * \endcode
 * @details This API is used to get the config file major and minor information.
 *
 * @param[out] config_major    : Pointer to data buffer to store the config major.
 * @param[out] config_minor    : Pointer to data buffer to store the config minor.
 * @param[in, out] dev         : Structure instance of bma4_dev.
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev);

/**
 * \ingroup bma456w
 * \defgroup bma456wApiWristwearwakeup Wrist wear wakeup
 * @brief Wrist wear wakeup configurations of the sensor
 */

/*!
 * \ingroup bma456wApiWristwearwakeup
 * \page bma456w_api_bma456w_set_wrist_wear_wakeup_param_config bma456w_set_wrist_wear_wakeup_param_config
 * \code
 * int8_t bma456w_set_wrist_wear_wakeup_param_config(const struct bma456w_wrist_wear_wakeup_params *setting, struct bma4_dev *dev);
 * \endcode
 * @details This API sets the configuration of wrist wear wakeup feature from the
 * sensor.
 *
 * @param[in] setting        : Pointer to structure variable to configure
 *                              wrist wear wakeup.
 *
 * @verbatim
 * --------------------------------|-----------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        Cosine of minimum expected attitude change of the
 *                                 |        device within 1 second time window
 *      Minimum angle focus        |        when moving within focus position.
 *                                 |        The parameter is scaled by 2048
 *                                 |        i.e. 2048 * cos(angle).
 *                                 |        Range is 1024 to 1774. Default is 1774.
 *                                 |
 * --------------------------------|----------------------------------------
 *                                 |        Cosine of minimum expected attitude change of the
 *                                 |        device within 1 second time window
 *      Minimum angle non focus    |        when moving from non-focus to focus position
 *                                 |        The parameter is scaled by 2048
 *                                 |        i.e. 2048 * cos(angle). Range is 1448 to 1856.
 *                                 |        Default value is 1522.
 * --------------------------------|-----------------------------------------
 *                                 |        Sine of the maximum allowed tilt angle
 *                                 |        in landscape right direction of the device,
 *       Angle landscape right     |        when it is in focus position
 *                                 |        (i.e. user is able to comfortably
 *                                 |        look at the dial of wear device).
 *                                 |        The configuration parameter is scaled
 *                                 |        by 256 i.e. 256 * sin(angle). Range is
 *                                 |        88 to 128. Default value is 128.
 * --------------------------------|-------------------------------------------
 *                                 |        Sine of the maximum allowed tilt angle
 *                                 |        in landscape left direction of the device,
 *        Angle landscape left     |        when it is in focus position
 *                                 |        (i.e. user is able to comfortably look at
 *                                 |        the dial of wear device).
 *                                 |        The configuration parameter is scaled by 256
 *                                 |        i.e. 256 * sin(angle). Range is 88 to 128.
 *                                 |        Default value is 128.
 * --------------------------------|-----------------------------------------
 *                                 |        Sine of the maximum allowed backward tilt
 *                                 |        angle in portrait down direction of the device,
 *        Angle portrait down      |        when it is in focus position (i.e. user is
 *                                 |        able to comfortably look at the dial of wear device).
 *                                 |        The configuration parameter is scaled by 256
 *                                 |        i.e. 256 * sin(angle).
 *                                 |        Range is 0 to179. Default value is 22.
 *                                 |        basis.
 * --------------------------------|-------------------------------------------
 *                                 |        Sine of the maximum allowed forward tilt angle
 *                                 |        in portrait up direction of the device,
 *         Angle portrait up       |        when it is in focus position
 *                                 |        (i.e. user is able to comfortably look at
 *                                 |        the dial of wear device).
 *                                 |        The configuration parameter is scaled by 256 i.e.
 *                                 |        256 * sin(angle).
 *                                 |        Range is 222 to 247. Default value is 241.
 * --------------------------------|-----------------------------------------
 *                                 |        Minimum duration the arm should be moved
 *        Minimum duration moved   |        while performing gesture.
 *                                 |        Range: 1 to 10, resolution = 20 ms
 * --------------------------------|------------------------------------------
 *                                 |        Minimum duration the arm should be
 *        Minimum duration quite   |        static between two consecutive gestures.
 *                                 |        Range: 1 to 10, resolution = 20 ms
 * --------------------------------|-----------------------------------------
 * @endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_set_wrist_wear_wakeup_param_config(const struct bma456w_wrist_wear_wakeup_params *setting,
                                                  struct bma4_dev *dev);

/*!
 * \ingroup bma456wApiWristwearwakeup
 * \page bma456w_api_bma456w_get_wrist_wear_wakeup_param_config bma456w_set_wrist_wear_wakeup_param_config
 * \code
 * int8_t bma456w_get_wrist_wear_wakeup_param_config(struct bma456w_wrist_wear_wakeup_params *setting, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the configuration of wrist wear wakeup feature from the
 * sensor.
 *
 * @param[in] setting        : Pointer to structure variable to store
 *                              wrist wear wakeup.
 *
 * @verbatim
 * --------------------------------|-----------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        Cosine of minimum expected attitude change of the
 *                                 |        device within 1 second time window
 *      Minimum angle focus        |        when moving within focus position.
 *                                 |        The parameter is scaled by 2048
 *                                 |        i.e. 2048 * cos(angle).
 *                                 |        Range is 1024 to 1774. Default is 1774.
 *                                 |
 * --------------------------------|----------------------------------------
 *                                 |        Cosine of minimum expected attitude change of the
 *                                 |        device within 1 second time window
 *      Minimum angle non focus    |        when moving from non-focus to focus position
 *                                 |        The parameter is scaled by 2048
 *                                 |        i.e. 2048 * cos(angle). Range is 1448 to 1856.
 *                                 |        Default value is 1522.
 * --------------------------------|-----------------------------------------
 *                                 |        Sine of the maximum allowed tilt angle
 *                                 |        in landscape right direction of the device,
 *       Angle landscape right     |        when it is in focus position
 *                                 |        (i.e. user is able to comfortably
 *                                 |        look at the dial of wear device).
 *                                 |        The configuration parameter is scaled
 *                                 |        by 256 i.e. 256 * sin(angle). Range is
 *                                 |        88 to 128. Default value is 128.
 * --------------------------------|-------------------------------------------
 *                                 |        Sine of the maximum allowed tilt angle
 *                                 |        in landscape left direction of the device,
 *        Angle landscape left     |        when it is in focus position
 *                                 |        (i.e. user is able to comfortably look at
 *                                 |        the dial of wear device).
 *                                 |        The configuration parameter is scaled by 256
 *                                 |        i.e. 256 * sin(angle). Range is 88 to 128.
 *                                 |        Default value is 128.
 * --------------------------------|-----------------------------------------
 *                                 |        Sine of the maximum allowed backward tilt
 *                                 |        angle in portrait down direction of the device,
 *        Angle portrait down      |        when it is in focus position (i.e. user is
 *                                 |        able to comfortably look at the dial of wear device).
 *                                 |        The configuration parameter is scaled by 256
 *                                 |        i.e. 256 * sin(angle).
 *                                 |        Range is 0 to179. Default value is 22.
 *                                 |        basis.
 * --------------------------------|-------------------------------------------
 *                                 |        Sine of the maximum allowed forward tilt angle
 *                                 |        in portrait up direction of the device,
 *         Angle portrait up       |        when it is in focus position
 *                                 |        (i.e. user is able to comfortably look at
 *                                 |        the dial of wear device).
 *                                 |        The configuration parameter is scaled by 256 i.e.
 *                                 |        256 * sin(angle).
 *                                 |        Range is 222 to 247. Default value is 241.
 * --------------------------------|-----------------------------------------
 *                                 |        Minimum duration the arm should be moved
 *        Minimum duration moved   |        while performing gesture.
 *                                 |        Range: 1 to 10, resolution = 20 ms
 * --------------------------------|------------------------------------------
 *                                 |        Minimum duration the arm should be
 *        Minimum duration quite   |        static between two consecutive gestures.
 *                                 |        Range: 1 to 10, resolution = 20 ms
 * --------------------------------|-----------------------------------------
 * @endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456w_get_wrist_wear_wakeup_param_config(struct bma456w_wrist_wear_wakeup_params *setting,
                                                  struct bma4_dev *dev);

#ifdef __cplusplus
}
#endif /*End of CPP guard */

#endif /*End of header guard macro */

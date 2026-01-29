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
* @file       bma456_tablet.h
* @date       2023-07-05
* @version    V2.29.0
*
*/

/**
 * \ingroup bma4xy
 * \defgroup bma456_tablet BMA456_TABLET
 * @brief Sensor driver for BMA456_TABLET sensor
 */

#ifndef BMA456_TABLET_H
#define BMA456_TABLET_H

#ifdef __cplusplus
extern "C" {
#endif
#include "bma4.h"

/**\name Chip ID */
#define BMA456_TABLET_CHIP_ID                         UINT8_C(0x16)

/**\name Sensor feature size */
#define BMA456_TABLET_FEATURE_SIZE                    UINT8_C(50)

/**\name Feature offset address */
#define BMA456_TABLET_ANY_MOT_OFFSET                  UINT8_C(0x00)
#define BMA456_TABLET_NO_MOT_OFFSET                   UINT8_C(0x04)
#define BMA456_TABLET_LOW_G_OFFSET                    UINT8_C(0x08)
#define BMA456_TABLET_TAP_DETECTOR_OFFSET             UINT8_C(0x0E)
#define BMA456_TABLET_ORIENTATION_OFFSET              UINT8_C(0x28)
#define BMA456_TABLET_CONFIG_ID_OFFSET                UINT8_C(0x2E)
#define BMA456_TABLET_AXES_REMAP_OFFSET               UINT8_C(0x30)

/**\name Read/Write Lengths */
#define BMA456_TABLET_RD_WR_MIN_LEN                   UINT8_C(2)

/*! @name Maximum valid read write length is size of config file array */
#define BMA456_TABLET_RD_WR_MAX_LEN                   ((uint16_t)sizeof(bma456_tablet_config_file))

/**************************************************************/
/**\name    Re-map Axes */
/**************************************************************/
#define BMA456_TABLET_X_AXIS_MASK                     UINT8_C(0x03)
#define BMA456_TABLET_X_AXIS_SIGN_MASK                UINT8_C(0x04)
#define BMA456_TABLET_Y_AXIS_MASK                     UINT8_C(0x18)
#define BMA456_TABLET_Y_AXIS_SIGN_MASK                UINT8_C(0x20)
#define BMA456_TABLET_Z_AXIS_MASK                     UINT8_C(0xC0)
#define BMA456_TABLET_Z_AXIS_SIGN_MASK                UINT8_C(0x01)

/**************************************************************/
/**\name            Any/no Motion                             */
/**************************************************************/
#define BMA456_TABLET_ANY_MOT_LEN                     UINT8_C(4)
#define BMA456_TABLET_NO_MOT_RD_WR_LEN                (BMA456_TABLET_ANY_MOT_LEN + BMA456_TABLET_NO_MOT_OFFSET)

/**\name Any/No motion threshold macros */
#define BMA456_TABLET_ANY_NO_MOT_THRES_MSK            UINT16_C(0x07FF)

/**\name Position and mask of interrupt behavior and slope */
#define BMA456_TABLET_ANY_NO_MOTION_INTR_BHVR_EN_POS  UINT8_C(0x03)
#define BMA456_TABLET_ANY_NO_MOTION_INTR_BHVR_EN_MSK  UINT8_C(0x08)

#define BMA456_TABLET_ANY_MOTION_SLOPE_EN_POS         UINT8_C(0x04)
#define BMA456_TABLET_ANY_MOTION_SLOPE_EN_MSK         UINT8_C(0x10)

/**\name Any/No motion duration macros */
#define BMA456_TABLET_ANY_NO_MOT_DUR_MSK              UINT16_C(0x1FFF)

/**\name Any/No motion enable macros */
#define BMA456_TABLET_ANY_NO_MOT_AXIS_EN_POS          UINT8_C(0x0D)
#define BMA456_TABLET_ANY_NO_MOT_AXIS_EN_MSK          UINT16_C(0xE000)

/**************************************************************/
/**\name         Single/Double/Triple Tap                     */
/**************************************************************/
/**\name Single tap enable macros */
#define BMA456_TABLET_SINGLE_TAP_EN_MSK               UINT8_C(0x01)

/**\name Double tap enable macros */
#define BMA456_TABLET_DOUBLE_TAP_EN_MSK               UINT8_C(0x02)

/**\name Triple tap enable macros */
#define BMA456_TABLET_TRIPLE_TAP_EN_MSK               UINT8_C(0x04)

/**\name Tap averaging enable macros */
#define BMA456_TABLET_TAP_AVG_EN_MSK                  UINT8_C(0x08)

/**\name Tap output macros */
#define BMA456_TABLET_SINGLE_TAP_OUT_MSK              UINT8_C(0x01)

#define BMA456_TABLET_DOUBLE_TAP_OUT_MSK              UINT8_C(0x02)
#define BMA456_TABLET_DOUBLE_TAP_OUT_POS              UINT8_C(0x01)

#define BMA456_TABLET_TRIPLE_TAP_OUT_MSK              UINT8_C(0x04)
#define BMA456_TABLET_TRIPLE_TAP_OUT_POS              UINT8_C(0x02)

/**************************************************************/
/**\name                Low-G                                 */
/**************************************************************/
/**\name Low-G enable macros */
#define BMA456_TABLET_LOW_G_FEAT_EN_OFFSET            UINT8_C(0x03)
#define BMA456_TABLET_LOW_G_EN_POS                    UINT8_C(0x04)
#define BMA456_TABLET_LOW_G_EN_MSK                    UINT8_C(0x10)

/**\name Low-g threshold macro */
#define BMA456_TABLET_LOW_G_THRES_MSK                 UINT16_C(0x7FFF)

/**\name Low-g hysteresis macro */
#define BMA456_TABLET_LOW_G_HYST_MSK                  UINT16_C(0x0FFF)

/**\name Low-g duration macro */
#define BMA456_TABLET_LOW_G_DUR_MSK                   UINT16_C(0x0FFF)

/**************************************************************/
/**\name                Orientation                           */
/**************************************************************/
#define BMA456_TABLET_ORIENTATION_OUT_ADDR            UINT8_C(0x1E)

/**\name Orientation enable macros */
#define BMA456_TABLET_ORIENT_EN_MSK                   UINT8_C(0x01)

/**\name Orientation upside/down detection macros */
#define BMA456_TABLET_ORIENT_UD_POS                   UINT8_C(1)
#define BMA456_TABLET_ORIENT_UD_MSK                   UINT8_C(0x02)

/**\name Orientation mode macros */
#define BMA456_TABLET_ORIENT_MODE_POS                 UINT8_C(2)
#define BMA456_TABLET_ORIENT_MODE_MSK                 UINT8_C(0x0C)

/**\name Orientation blocking macros */
#define BMA456_TABLET_ORIENT_BLOCKING_POS             UINT8_C(4)
#define BMA456_TABLET_ORIENT_BLOCKING_MSK             UINT8_C(0x30)

/**\name Orientation hold time macros */
#define BMA456_TABLET_ORIENT_HOLD_TIME_POS            UINT8_C(6)
#define BMA456_TABLET_ORIENT_HOLD_TIME_MSK            UINT16_C(0x7C0)

/**\name Orientation slope threshold macros */
#define BMA456_TABLET_ORIENT_SLOPE_THRES_MSK          UINT16_C(0x00FF)

/**\name Orientation hysteresis macros */
#define BMA456_TABLET_ORIENT_HYST_POS                 UINT8_C(8)
#define BMA456_TABLET_ORIENT_HYST_MSK                 UINT16_C(0xFF00)

/**\name Orientation theta macros */
#define BMA456_TABLET_ORIENT_THETA_MSK                UINT16_C(0x00FF)

/*  Orientation output macros  */
#define BMA456_TABLET_ORIENT_OUT_POS                  UINT8_C(0)
#define BMA456_TABLET_ORIENT_OUT_MSK                  UINT8_C(0x03)
#define BMA456_TABLET_ORIENT_FACEUP_DOWN_POS          UINT8_C(2)
#define BMA456_TABLET_ORIENT_FACEUP_DOWN_MSK          UINT8_C(0x04)

/**\name Orientation output macros */
/* Bit pos 2 reflects the face-up (0) face-down(1) only if ud_en is enabled */
#define BMA456_TABLET_FACE_UP                         UINT8_C(0x00)
#define BMA456_TABLET_FACE_DOWN                       UINT8_C(0x01)

/* Bit pos 0-1 reflects the have the following value */
#define BMA456_TABLET_PORTRAIT_UP_RIGHT               UINT8_C(0x00)
#define BMA456_TABLET_LANDSCAPE_LEFT                  UINT8_C(0x01)
#define BMA456_TABLET_PORTRAIT_UP_DOWN                UINT8_C(0x02)
#define BMA456_TABLET_LANDSCAPE_RIGHT                 UINT8_C(0x03)

/**************************************************************/
/**\name                 User macros                          */
/**************************************************************/
#define BMA456_TABLET_TAP_FEAT_OUT_ADDR               UINT8_C(0x20)

/**\name Any-motion/No-motion axis enable macros */
#define BMA456_TABLET_X_AXIS_EN                       UINT8_C(0x01)
#define BMA456_TABLET_Y_AXIS_EN                       UINT8_C(0x02)
#define BMA456_TABLET_Z_AXIS_EN                       UINT8_C(0x04)
#define BMA456_TABLET_EN_ALL_AXIS                     UINT8_C(0x07)
#define BMA456_TABLET_DIS_ALL_AXIS                    UINT8_C(0x00)

/**\name Feature enable macros for the sensor */
#define BMA456_TABLET_LOW_G                           UINT8_C(0x01)
#define BMA456_TABLET_ORIENTATION                     UINT8_C(0x02)
#define BMA456_TABLET_SINGLE_TAP                      UINT8_C(0x04)
#define BMA456_TABLET_DOUBLE_TAP                      UINT8_C(0x08)
#define BMA456_TABLET_TRIPLE_TAP                      UINT8_C(0x10)

/**\name Interrupt status macros */
#define BMA456_TABLET_TAP_OUT_INT                     UINT8_C(0x01)
#define BMA456_TABLET_ORIENTATION_INT                 UINT8_C(0x02)
#define BMA456_TABLET_LOW_G_INT                       UINT8_C(0x04)
#define BMA456_TABLET_ANY_MOT_INT                     UINT8_C(0x20)
#define BMA456_TABLET_NO_MOT_INT                      UINT8_C(0x40)
#define BMA456_TABLET_ERROR_INT                       UINT8_C(0x80)

/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/

/*!
 * @brief Any/No motion configuration
 */
struct bma456_tablet_any_no_mot_config
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
 * @brief Tap param settings
 */
struct bma456_tablet_tap_settings
{
    /*! Scaling factor for threshold */
    uint16_t tap_sens_thres;

    /*! Maximum duration after the first tap */
    uint16_t max_gest_dur;

    /*! Settling time for high frequency acceleration signal */
    uint16_t tap_shock_dur;

    /*! Minimum quite time between the two gesture detection */
    uint16_t quite_time_after_gest;

    /*! Wait for the duration set by max_gest_dur after the first tap */
    uint16_t wait_for_timeout;

    /*! Selection of axis from 3D-acceleration signal vector */
    uint16_t axis_sel;
};

/*!
 * @brief Tap output state
 */
struct bma456_tablet_tap_output
{
    /*! Single tap detected */
    uint8_t s_tap;

    /*! Double tap detected */
    uint8_t d_tap;

    /*! Triple tap detected */
    uint8_t t_tap;
};

/*!
 * @brief Low-G configuration
 */
struct bma456_tablet_low_g_config
{
    /*! Threshold value for Low-G feature */
    uint16_t threshold;

    /*! Hysteresis value for Low-G feature */
    uint16_t hysteresis;

    /*! Duration in 50Hz samples(20msec) for
     *  which threshold has to be exceeded
     */
    uint16_t duration;
};

/*!
 * @brief Orientation configuration structure
 */
struct bma456_tablet_orientation_config
{
    /*! Upside/Downside detection */
    uint8_t upside_down;

    /*! Mode
     * Symmetrical (values 0 or 3), High asymmetrical
     * (value 1) or Low asymmetrical (value 2)
     */
    uint8_t mode;

    /*! Blocking mode
     * Sets the blocking mode.
     * If blocking is set, no Orientation interrupt will be triggered.
     * Default value is 3 – the most restrictive blocking mode.
     */
    uint8_t blocking;

    /*! Hold Time
     * The number of samples for which the orientation
     * should be stable in 50Hz. Resolution: 5 = 100msecs
     */
    uint8_t hold_time;

    /*! Slope Threshold
     * The slope threshold for blocking mode.
     * Resolution: 0.4g = 205 lsbs
     */
    uint8_t slope_thres;

    /*! Hysteresis
     * Hysteresis of acceleration for orientation change detection.
     * Resolution: 62.5mg = 32 lsbs
     */
    uint8_t hysteresis;

    /*! Theta
     * Coded value of the threshold angle with horizontal
     * used in Blocking modes;
     * theta = 64 * (tan(angle)^2);
     */
    uint8_t theta;
};

/***************************************************************************/

/*!             BMA456_TABLET User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bma456_tabletApiInit
 * \page bma456_tablet_api_bma456_tablet_init bma456_tablet_init
 * \code
 * int8_t bma456_tablet_init(struct bma4_dev *dev);
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
int8_t bma456_tablet_init(struct bma4_dev *dev);

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApiConfig Config
 * @brief Configuration APIs
 */

/*!
 * \ingroup bma456_tabletApiConfig
 * \page bma456_tablet_api_bma456_tablet_write_config_file bma456_tablet_write_config_file
 * \code
 * int8_t bma456_tablet_write_config_file(struct bma4_dev *dev);
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
int8_t bma456_tablet_write_config_file(struct bma4_dev *dev);

/*!
 * \ingroup bma456_tabletApiConfig
 * \page bma456_tablet_api_bma456_tablet_get_version_config bma456_tablet_get_version_config
 * \code
 *int8_t bma456_tablet_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev);
 * \endcode
 * @details This API is used to get the config file major and minor information.
 *
 * @param[in] dev   : Structure instance of bma4_dev.
 * @param[out] config_major    : Pointer to data buffer to store the config major.
 * @param[out] config_minor    : Pointer to data buffer to store the config minor.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456_tablet_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev);

/*!
 * \ingroup bma456_tabletApiConfig
 * \page bma456_tablet_api_bma456_tablet_get_config_id bma456_tablet_get_config_id
 * \code
 * int8_t bma456_tablet_get_config_id(uint16_t *config_id, struct bma4_dev *dev);
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
int8_t bma456_tablet_get_config_id(uint16_t *config_id, struct bma4_dev *dev);

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApiMapInt Map / Unmap Interrupt
 * @brief Map / Unmap user provided interrupt to interrupt pin1 or pin2 of the sensor
 */

/*!
 * \ingroup bma456_tabletApiMapInt
 * \page bma456_tablet_api_bma456_tablet_map_interrupt bma456_tablet_map_interrupt
 * \code
 * int8_t bma456_tablet_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);
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
 *  - BMA456_TABLET_TAP_OUT_INT
 *  - BMA456_TABLET_ORIENTATION_INT
 *  - BMA456_TABLET_LOW_G_INT
 *  - BMA456_TABLET_ANY_MOT_INT
 *  - BMA456_TABLET_NO_MOT_INT
 *  - BMA456_TABLET_ERROR_INT
 *
 * Hardware Interrupts
 *  - BMA4_FIFO_FULL_INT
 *  - BMA4_FIFO_WM_INT
 *  - BMA4_MAG_DATA_RDY_INT
 *  - BMA4_ACCEL_DATA_RDY_INT
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApiIntS Interrupt Status
 * @brief Read interrupt status of the sensor
 */

/*!
 * \ingroup bma456_tabletApiIntS
 * \page bma456_tablet_api_bma456_tablet_read_int_status bma456_tablet_read_int_status
 * \code
 * int8_t bma456_tablet_read_int_status(uint16_t *int_status, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the bma456 interrupt status from the sensor.
 *
 * @param[out] int_status : Variable to store the interrupt status read from
 * the sensor.
 * @param[in] dev : Structure instance of bma4_dev.
 *
 * @note Below macros are used to check the interrupt status.
 *
 * Feature Interrupts
 *  - BMA456_TABLET_TAP_OUT_INT
 *  - BMA456_TABLET_ORIENTATION_INT
 *  - BMA456_TABLET_LOW_G_INT
 *  - BMA456_TABLET_ANY_MOT_INT
 *  - BMA456_TABLET_NO_MOT_INT
 *  - BMA456_TABLET_ERROR_INT
 *
 * Hardware Interrupts
 *  - BMA4_FIFO_FULL_INT
 *  - BMA4_FIFO_WM_INT
 *  - BMA4_MAG_DATA_RDY_INT
 *  - BMA4_ACCEL_DATA_RDY_INT
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_read_int_status(uint16_t *int_status, struct bma4_dev *dev);

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApiFeat Sensor Feature
 * @brief Enables / Disables features of the sensor
 */

/*!
 * \ingroup bma456_tabletApiFeat
 * \page bma456_tablet_api_bma456_tablet_feature_enable bma456_tablet_feature_enable
 * \code
 * int8_t bma456_tablet_feature_enable(uint8_t feature, uint8_t enable, struct bma4_dev *dev);
 * \endcode
 * @details This API enables/disables the features of the sensor.
 *
 * @param[in] feature : Variable to specify the features which are to be set in
 * bma456 sensor.
 * @param[in] enable : Variable which specifies whether to enable or disable the
 * features in the bma456 sensor.
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
 * features of bma456_tablet sensor
 *
 *   - BMA456_TABLET_SINGLE_TAP
 *   - BMA456_TABLET_DOUBLE_TAP
 *   - BMA456_TABLET_TRIPLE_TAP
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_feature_enable(uint8_t feature, uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApiRemap Remap Axes
 * @brief Set / Get x, y and z axis re-mapping in the sensor
 */

/*!
 * \ingroup bma456_tabletApiRemap
 * \page bma456_tablet_api_bma456_tablet_set_remap_axes bma456_tablet_set_remap_axes
 * \code
 * int8_t bma456_tablet_set_remap_axes(const struct bma4_remap *remap_data, struct bma4_dev *dev);
 * \endcode
 * @details This API performs x, y and z axis remapping in the sensor.
 *
 * @param[in] remap_data : Pointer to store axes remapping data.
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_set_remap_axes(const struct bma4_remap *remap_data, struct bma4_dev *dev);

/*!
 * \ingroup bma456_tabletApiRemap
 * \page bma456_tablet_api_bma456_tablet_get_remap_axes bma456_tablet_get_remap_axes
 * \code
 * int8_t bma456_tablet_get_remap_axes(struct bma4_remap *remap_data, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the x, y and z axis remap data from the sensor.
 *
 * @param[out] remap_data : Pointer to store axis remap data which is read
 * from the bma456 sensor.
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_get_remap_axes(struct bma4_remap *remap_data, struct bma4_dev *dev);

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApiAnyMot Any motion Feature
 * @brief Functions of Any motion feature of the sensor
 */

/*!
 * \ingroup bma456_tabletApiAnyMot
 * \page bma456_tablet_api_bma456_tablet_set_any_mot_config bma456_tablet_set_any_mot_config
 * \code
 * int8_t bma456_tablet_set_any_mot_config(const struct bma456_tablet_any_no_mot_config *any_mot, struct bma4_dev *dev);
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
 *                                 |        Configuration for acceleration
 *          slope                  |        scope computation.
 * ---------------------------------------------------------------------------
 * @endverbatim
 *
 *@verbatim
 *  Value    |  axis_en
 *  ---------|-------------------------
 *  0x00     |  BMA456_TABLET_DIS_ALL_AXIS
 *  0x01     |  BMA456_TABLET_X_AXIS_EN
 *  0x02     |  BMA456_TABLET_Y_AXIS_EN
 *  0x04     |  BMA456_TABLET_Z_AXIS_EN
 *  0x07     |  BMA456_TABLET_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_set_any_mot_config(const struct bma456_tablet_any_no_mot_config *any_mot, struct bma4_dev *dev);

/*!
 * \ingroup bma456_tabletApiAnyMot
 * \page bma456_tablet_api_bma456_tablet_get_any_motion_config bma456_tablet_get_any_motion_config
 * \code
 * int8_t bma456_tablet_get_any_motion_config(struct bma456_tablet_anymotion_config *any_motion, struct bma4_dev *dev);
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
 *                                 |        Configuration for acceleration
 *          slope                  |        scope computation.
 * ---------------------------------------------------------------------------
 * @endverbatim
 *
 *@verbatim
 *  Value    |  axis_en
 *  ---------|-------------------------
 *  0x00     |  BMA456_TABLET_DIS_ALL_AXIS
 *  0x01     |  BMA456_TABLET_X_AXIS_EN
 *  0x02     |  BMA456_TABLET_Y_AXIS_EN
 *  0x04     |  BMA456_TABLET_Z_AXIS_EN
 *  0x07     |  BMA456_TABLET_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_get_any_mot_config(struct bma456_tablet_any_no_mot_config *any_mot, struct bma4_dev *dev);

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApiNomot No-Motion Feature
 * @brief Operations of no-motion feature of the sensor
 */

/*!
 * \ingroup bma456_tabletApiNomot
 * \page bma456_tablet_api_bma456_tablet_set_no_motion_config bma456_tablet_set_no_motion_config
 * \code
 * int8_t bma456_tablet_set_no_motion_config(const struct bma456_tablet_nomotion_config *no_motion, struct bma4_dev *dev);
 * \endcode
 * @details This API sets the configuration of no-motion feature in the sensor
 * This API enables/disables the no-motion feature according to the axis set.
 *
 * @param[in] no_mot                : Pointer to structure variable to configure
 *                                  no-motion.
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
 *  0x00     |  BMA456_TABLET_DIS_ALL_AXIS
 *  0x01     |  BMA456_TABLET_X_AXIS_EN
 *  0x02     |  BMA456_TABLET_Y_AXIS_EN
 *  0x04     |  BMA456_TABLET_Z_AXIS_EN
 *  0x07     |  BMA456_TABLET_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_set_no_mot_config(const struct bma456_tablet_any_no_mot_config *no_mot, struct bma4_dev *dev);

/*!
 * \ingroup bma456_tabletApiNomot
 * \page bma456_tablet_api_bma456_tablet_get_no_motion_config bma456_tablet_get_no_motion_config
 * \code
 * int8_t bma456_tablet_get_no_motion_config(struct bma456_tablet_nomotion_config *no_motion, struct bma4_dev *dev);
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
 *  0x00     |  BMA456_TABLET_DIS_ALL_AXIS
 *  0x01     |  BMA456_TABLET_X_AXIS_EN
 *  0x02     |  BMA456_TABLET_Y_AXIS_EN
 *  0x04     |  BMA456_TABLET_Z_AXIS_EN
 *  0x07     |  BMA456_TABLET_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_get_no_mot_config(struct bma456_tablet_any_no_mot_config *no_mot, struct bma4_dev *dev);

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApOP Tap Output
 * @brief Functions to provide tap feature output status
 */

/*!
 * \ingroup bma456_tabletApOP
 * \page bma456_tablet_api_bma456_tablet_tap_output bma456_tablet_tap_output
 * \code
 * int8_t bma456_tablet_tap_output(struct bma456_tablet_tap_output *out_state, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the output for tap(single, double, triple) feature.
 *
 * @param[out] out_state : Pointer variable which stores feature output values.
 * @param[in] dev        : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_tap_output(struct bma456_tablet_tap_output *out_state, struct bma4_dev *dev);

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApiTap Tap Feature
 * @brief Tap feature operations
 */

/*!
 * \ingroup bma456_tabletApiTap
 * \page bma456_tablet_api_bma456_tablet_tap_get_parameter bma456_tablet_tap_get_parameter
 * \code
 * int8_t bma456_tablet_tap_get_parameter(struct bma456_tablet_tap_settings *setting, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the tap parameter settings.
 *
 * @param[out] setting : Pointer to structure variable which stores the
 * tap parameters read from the sensor
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_tap_get_parameter(struct bma456_tablet_tap_settings *setting, struct bma4_dev *dev);

/*!
 * \ingroup bma456_tabletApiTap
 * \page bma456_tablet_api_bma456_tablet_tap_set_parameter bma456_tablet_tap_set_parameter
 * \code
 * int8_t bma456_tablet_tap_set_parameter(const struct bma456_tablet_tap_settings *setting, struct bma4_dev *dev);
 * \endcode
 * @details This API sets the tap parameter settings in the sensor.
 *
 * @param[in] setting : Pointer to structure variable which stores the
 * tap parameter settings read from the sensor.
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_tablet_tap_set_parameter(const struct bma456_tablet_tap_settings *setting, struct bma4_dev *dev);

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApLowG Low-G Feature
 * @brief Functions of Low-G feature of the sensor
 */

/*!
 * \ingroup bma456_tabletApLowG
 * \page bma456_tablet_api_bma456_tablet_set_low_g_config bma456_tablet_set_low_g_config
 * \code
 * int8_t bma456_tablet_set_low_g_config(const struct bma456_tablet_low_g_config *low_g, struct bma4_dev *dev);
 * \endcode
 * @details  This API sets the configuration of low-g feature of the sensor.
 *
 * @param[in] low_g : Pointer  to structure variable used to
 * store the low-g feature settings to be written to the sensor.
 *
 * Structure members are provided in the table below
 *
 *@verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |       Threshold value for low-g feature.
 *          threshold              |           Range is 0 to 1g.
 *                                 |       Default value is 512 = 0.25g.
 * --------------------------------|----------------------------------------
 *                                 |     Hysteresis value for low_g feature.
 *         hysteresis              |        Range is 0 to 0.5g.
 *                                 |        Default value is 256 = 0.125g.
 * --------------------------------|----------------------------------------
 *                                 | Duration in 50 Hz samples (20 msec) for
 *         duration                |  which the threshold has to be exceeded.
 *                                 |       Range is 0 to 82 sec.
 *                                 |    Default value is 0 = 0 ms.
 * --------------------------------|----------------------------------------
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456_tablet_set_low_g_config(const struct bma456_tablet_low_g_config *low_g, struct bma4_dev *dev);

/*!
 * \ingroup bma456_tabletApLowG
 * \page bma456_tablet_api_bma456_tablet_get_low_g_config bma456_tablet_get_low_g_config
 * \code
 * int8_t bma456_tablet_get_low_g_config(struct bma456_tablet_low_g_config *low_g, struct bma4_dev *dev);
 * \endcode
 * @details  This API gets the configuration of significant motion feature
 *  from the sensor.
 *
 *  @param[out] low_g : Pointer  to structure variable used to
 *  store the low-g feature settings read from the sensor.
 *
 *  Structure members are provided in the table below
 *
 *@verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |       Threshold value for low-g feature.
 *          threshold              |           Range is 0 to 1g.
 *                                 |       Default value is 512 = 0.25g.
 * --------------------------------|----------------------------------------
 *                                 |     Hysteresis value for low_g feature.
 *         hysteresis              |        Range is 0 to 0.5g.
 *                                 |        Default value is 256 = 0.125g.
 * --------------------------------|----------------------------------------
 *                                 | Duration in 50 Hz samples (20 msec) for
 *         duration                |  which the threshold has to be exceeded.
 *                                 |       Range is 0 to 82 sec.
 *                                 |    Default value is 0 = 0 ms.
 * --------------------------------|----------------------------------------
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456_tablet_get_low_g_config(struct bma456_tablet_low_g_config *low_g, struct bma4_dev *dev);

/**
 * \ingroup bma456_tablet
 * \defgroup bma456_tabletApiOrientation Orientation Feature
 * @brief Functions of Orientation feature of the sensor
 */

/*!
 * \ingroup bma456_tabletApiOrientation
 * \page bma456_tablet_api_bma456_tablet_set_orientation_config bma456_tablet_set_orientation_config
 * \code
 * int8_t bma456_tablet_set_orientation_config(struct bma456_tablet_orientation_config *orientation, struct bma4_dev *dev);
 * \endcode
 * @details  This API sets the configuration of orientation feature of the sensor.
 *
 * @param[in] orientation : Pointer  to structure variable used to
 * store the orientation feature settings to be written to sensor.
 *
 * Structure members are provided in the table below
 *
 *@verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *         ud_en                   |        Enables face upside/downside
 *                                 |        detection if set to 1
 * --------------------------------|----------------------------------------
 *                                 |        symmetrical (values 0 or 3),
 *         mode                    |        high asymmetrical (value 1)
 *                                 |        or low asymmetrical (value 2).
 * --------------------------------|----------------------------------------
 *                                 |        Sets the blocking mode.
 *         blocking                |        Default value is 4, the most
 *                                 |        restrictive blocking mode.
 * -------------------------------------------------------------------------
 *                                 |        The number of samples for which
 *        hold_time                |        the orientation should be stable
 *                                 |         in 50Hz. Reolution: 5 = 100msecs
 * -------------------------------------------------------------------------
 *        slope_thres              |        The slope threshold for blocking
 *                                 |        mode. Resolution: 0.4g = 205 lsbs
 * -------------------------------------------------------------------------
 *                                 |        Hysteresis of acceleration for
 *        hysteresis               |        orientation change detection.
 *                                 |        Resolution: 62.5mg = 32 lsbs
 * -------------------------------------------------------------------------
 *                                 |        Coded value of the threshold angle
 *                                 |        with horizontal used in blocking
 *        theta                    |        modes, theta = 64 * (tan(angle)^2).
 *                                 |        Default value is 40 which is
 *                                 |        equivalent to 38 degrees angle.
 * -------------------------------------------------------------------------
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456_tablet_set_orientation_config(const struct bma456_tablet_orientation_config *orientation,
                                            struct bma4_dev *dev);

/*!
 * \ingroup bma456_tabletApiOrientation
 * \page bma456_tablet_api_bma456_tablet_get_orientation_config bma456_tablet_get_orientation_config
 * \code
 * int8_t bma456_tablet_get_orientation_config(struct bma456_tablet_orientation_config *orientation, struct bma4_dev *dev);
 * \endcode
 * @details  This API gets the configuration of orientation feature from the sensor.
 *
 * @param[out] orientation : Pointer  to structure variable used to
 * store the orientation feature settings read from the sensor.
 *
 * Structure members are provided in the table below
 *
 *@verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *         ud_en                   |        Enables face upside/downside
 *                                 |        detection if set to 1
 * --------------------------------|----------------------------------------
 *                                 |        symmetrical (values 0 or 3),
 *         mode                    |        high asymmetrical (value 1)
 *                                 |        or low asymmetrical (value 2).
 * --------------------------------|----------------------------------------
 *                                 |        Sets the blocking mode.
 *         blocking                |        Default value is 4, the most
 *                                 |        restrictive blocking mode.
 * -------------------------------------------------------------------------
 *                                 |        The number of samples for which
 *        hold_time                |        the orientation should be stable
 *                                 |         in 50Hz. Reolution: 5 = 100msecs
 * -------------------------------------------------------------------------
 *        slope_thres              |        The slope threshold for blocking
 *                                 |        mode. Resolution: 0.4g = 205 lsbs
 * -------------------------------------------------------------------------
 *                                 |        Hysteresis of acceleration for
 *        hysteresis               |        orientation change detection.
 *                                 |        Resolution: 62.5mg = 32 lsbs
 * -------------------------------------------------------------------------
 *                                 |        Coded value of the threshold angle
 *                                 |        with horizontal used in blocking
 *        theta                    |        modes, theta = 64 * (tan(angle)^2).
 *                                 |        Default value is 40 which is
 *                                 |        equivalent to 38 degrees angle.
 * -------------------------------------------------------------------------
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456_tablet_get_orientation_config(struct bma456_tablet_orientation_config *orientation, struct bma4_dev *dev);

/*!
 * \ingroup bma422nApiOrient
 * \page bma456_tablet_api_bma456_tablet_orientation_output bma456_tablet_orientation_output
 * \code
 * int8_t bma456_tablet_orientation_output(uint8_t *orientation_output, uint8_t *orientation_faceup_down,
 *                                   struct bma4_dev *dev);
 *
 * \endcode
 * @details This API gets the output of orientation feature from the sensor.
 *
 *  @param orientation_output : pointer used to store the orientation output.
 *
 *@verbatim
 *   orientation_output          |        Values
 *-------------------------------|-----------------------------
 *   Bit pos 0-1 reflects        |   BMA456_TABLET_PORTRAIT_UP_RIGHT
 *   orientation output value    |   BMA456_TABLET_LANDSCAPE_LEFT
 *   only if ud_en is enabled    |   BMA456_TABLET_PORTRAIT_UP_DOWN
 *                               |   BMA456_TABLET_LANDSCAPE_RIGHT
 *-------------------------------|-----------------------------
 *@endverbatim
 *
 * @param orientation_faceup_down : Output value of face down face up orientation
 *
 *@verbatim
 *   orientation_faceup_down     |        Values
 *-------------------------------|-----------------------------
 *   Bit pos 2 reflects          |   BMA456_TABLET_FACE_UP
 *   face-up (0) or face-down(1) |   BMA456_TABLET_FACE_DOWN
 *   only if ud_en is enabled    |
 *-------------------------------|-----------------------------
 *@endverbatim
 *
 *  @param dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456_tablet_orientation_output(uint8_t *orientation_output,
                                        uint8_t *orientation_faceup_down,
                                        struct bma4_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* End of header guard macro */

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
* @file       bma456mm.h
* @date       2023-07-05
* @version    V2.29.0
*
*/

/**
 * \ingroup bma4xy
 * \defgroup bma456mm BMA456MM
 * @brief Sensor driver for BMA456MM sensor
 */

#ifndef BMA456MM_H
#define BMA456MM_H

#ifdef __cplusplus
extern "C" {
#endif
#include "bma4.h"

/**\name Chip ID of BMA456 sensor */
#define BMA456MM_CHIP_ID                           UINT8_C(0x16)

/**\name Sensor feature size */
#define BMA456MM_FEATURE_SIZE                      UINT8_C(62)

/**\name Feature offset address */
#define BMA456MM_ANY_MOT_OFFSET                    UINT8_C(0x00)
#define BMA456MM_NO_MOT_OFFSET                     UINT8_C(0x04)
#define BMA456MM_ORIENTATION_OFFSET                UINT8_C(0x08)
#define BMA456MM_LOW_G_OFFSET                      UINT8_C(0x0C)
#define BMA456MM_TAP_DETECTOR_OFFSET               UINT8_C(0x12)
#define BMA456MM_AUTO_LOW_POWER_OFFSET             UINT8_C(0x2C)
#define BMA456MM_HIGH_G_OFFSET                     UINT8_C(0x2E)
#define BMA456MM_SIG_MOTION_OFFSET                 UINT8_C(0x34)
#define BMA456MM_CONFIG_ID_OFFSET                  UINT8_C(0x3A)
#define BMA456MM_AXES_REMAP_OFFSET                 UINT8_C(0x3C)

/**\name Read/Write Lengths */
#define BMA456MM_RD_WR_MIN_LEN                     UINT8_C(2)

/*! @name Maximum valid read write length is size of config file array */
#define BMA456MM_RD_WR_MAX_LEN                     ((uint16_t)sizeof(bma456mm_config_file))

/**************************************************************/
/**\name    Re-map Axes */
/**************************************************************/
#define BMA456MM_X_AXIS_MASK                       UINT8_C(0x03)
#define BMA456MM_X_AXIS_SIGN_MASK                  UINT8_C(0x04)
#define BMA456MM_Y_AXIS_MASK                       UINT8_C(0x18)
#define BMA456MM_Y_AXIS_SIGN_MASK                  UINT8_C(0x20)
#define BMA456MM_Z_AXIS_MASK                       UINT8_C(0xC0)
#define BMA456MM_Z_AXIS_SIGN_MASK                  UINT8_C(0x01)

/**************************************************************/
/**\name            Any/no Motion                             */
/**************************************************************/
#define BMA456MM_ANY_MOT_LEN                       UINT8_C(4)
#define BMA456MM_NO_MOT_RD_WR_LEN                  (BMA456MM_ANY_MOT_LEN + BMA456MM_NO_MOT_OFFSET)

/**\name Any/No motion threshold macros */
#define BMA456MM_ANY_NO_MOT_THRES_MSK              UINT16_C(0x07FF)

/**\name Position and mask of interrupt behavior and slope */
#define BMA456MM_ANY_NO_MOTION_INTR_BHVR_EN_POS    UINT8_C(0x03)
#define BMA456MM_ANY_NO_MOTION_INTR_BHVR_EN_MSK    UINT8_C(0x08)

#define BMA456MM_ANY_MOTION_SLOPE_EN_POS           UINT8_C(0x04)
#define BMA456MM_ANY_MOTION_SLOPE_EN_MSK           UINT8_C(0x10)

/**\name Any/No motion duration macros */
#define BMA456MM_ANY_NO_MOT_DUR_MSK                UINT16_C(0x1FFF)

/**\name Any/No motion enable macros */
#define BMA456MM_ANY_NO_MOT_AXIS_EN_POS            UINT8_C(0x0D)
#define BMA456MM_ANY_NO_MOT_AXIS_EN_MSK            UINT16_C(0xE000)

/**************************************************************/
/**\name                Single/Double Tap                     */
/**************************************************************/
/**\name Single tap enable macros */
#define BMA456MM_SINGLE_TAP_EN_MSK                 UINT8_C(0x01)

/**\name Double tap enable macros */
#define BMA456MM_DOUBLE_TAP_EN_MSK                 UINT8_C(0x02)

/**\name Triple tap enable macros */
#define BMA456MM_TRIPLE_TAP_EN_MSK                 UINT8_C(0x04)

/**\name Tap averaging enable macros */
#define BMA456MM_TAP_AVG_EN_MSK                    UINT8_C(0x08)

/**\name Tap output macros */
#define BMA456MM_SINGLE_TAP_OUT_MSK                UINT8_C(0x01)

#define BMA456MM_DOUBLE_TAP_OUT_MSK                UINT8_C(0x02)
#define BMA456MM_DOUBLE_TAP_OUT_POS                UINT8_C(0x01)

#define BMA456MM_TRIPLE_TAP_OUT_MSK                UINT8_C(0x04)
#define BMA456MM_TRIPLE_TAP_OUT_POS                UINT8_C(0x02)

/**************************************************************/
/**\name                Orientation                           */
/**************************************************************/
#define BMA456MM_FEAT_OUT_ADDR                     UINT8_C(0x1E)

#define BMA456MM_ORIENTATION_OUTPUT_MASK           UINT8_C(0x07)

/**\name Orientation enable macros */
#define BMA456MM_ORIENT_EN_POS                     UINT8_C(0)
#define BMA456MM_ORIENT_EN_MSK                     UINT8_C(0x01)

/**\name Orientation upside/down detection macros */
#define BMA456MM_ORIENT_UD_POS                     UINT8_C(1)
#define BMA456MM_ORIENT_UD_MSK                     UINT8_C(0x02)

/**\name Orientation mode macros */
#define BMA456MM_ORIENT_MODE_POS                   UINT8_C(2)
#define BMA456MM_ORIENT_MODE_MSK                   UINT8_C(0x0C)

/**\name Orientation blocking macros */
#define BMA456MM_ORIENT_BLOCK_POS                  UINT8_C(4)
#define BMA456MM_ORIENT_BLOCK_MSK                  UINT8_C(0x30)

/**\name Orientation theta macros */
#define BMA456MM_ORIENT_THETA_POS                  UINT8_C(6)
#define BMA456MM_ORIENT_THETA_MSK                  UINT16_C(0x0FC0)

/**\name Orientation hysteresis macros */
#define BMA456MM_ORIENT_HYST_POS                   UINT8_C(0)
#define BMA456MM_ORIENT_HYST_MSK                   UINT16_C(0x07FF)

/*  Orientation output macros  */
#define BMA456MM_ORIENT_OUT_POS                    UINT8_C(0)
#define BMA456MM_ORIENT_OUT_MSK                    UINT8_C(0x03)
#define BMA456MM_ORIENT_FACEUP_DOWN_POS            UINT8_C(2)
#define BMA456MM_ORIENT_FACEUP_DOWN_MSK            UINT8_C(0x04)

/**\name Orientation output macros */
/* Bit pos 2 reflects the face-up (0) face-down(1) only if ud_en is enabled */
#define BMA456MM_FACE_UP                           UINT8_C(0x00)
#define BMA456MM_FACE_DOWN                         UINT8_C(0x01)

/* Bit pos 0-1 reflects the have the following value */
#define BMA456MM_PORTRAIT_UP_RIGHT                 UINT8_C(0x00)
#define BMA456MM_LANDSCAPE_LEFT                    UINT8_C(0x01)
#define BMA456MM_PORTRAIT_UP_DOWN                  UINT8_C(0x02)
#define BMA456MM_LANDSCAPE_RIGHT                   UINT8_C(0x03)

/**************************************************************/
/**\name                Low-G                                 */
/**************************************************************/
/**\name Low-G enable macros */
#define BMA456MM_LOW_G_FEAT_EN_OFFSET              UINT8_C(0x03)
#define BMA456MM_LOW_G_EN_POS                      UINT8_C(0x04)
#define BMA456MM_LOW_G_EN_MSK                      UINT8_C(0x10)

/**\name Low-g threshold macro */
#define BMA456MM_LOW_G_THRES_MSK                   UINT16_C(0x7FFF)

/**\name Low-g hysteresis macro */
#define BMA456MM_LOW_G_HYST_MSK                    UINT16_C(0x0FFF)

/**\name Low-g duration macro */
#define BMA456MM_LOW_G_DUR_MSK                     UINT16_C(0x0FFF)

/**************************************************************/
/**\name          Auto Low power                              */
/**************************************************************/

#define BMA456MM_AUTO_LOW_POWER_EN_OFFSET          UINT8_C(0x2D)

#define BMA456MM_NO_MOT_AUTO_LOW_POWER_MSK         UINT8_C(0x01)

#define BMA456MM_AUTO_LOW_POWER_TIME_OUT_MSK       UINT8_C(0x02)
#define BMA456MM_AUTO_LOW_POWER_TIME_OUT_POS       UINT8_C(0x1)
#define BMA456MM_TIME_OUT_DUR_MSK                  UINT16_C(0x0FFC)
#define BMA456MM_TIME_OUT_DUR_POS                  UINT16_C(0x02)
#define BMA456MM_AUTO_LOW_POWER_ALP_EN_MSK         UINT16_C(0x10)
#define BMA456MM_AUTO_LOW_POWER_ALP_EN_POS         UINT8_C(0x04)
#define BMA456MM_LOW_POW_ODR_MSK                   UINT8_C(0x60)
#define BMA456MM_LOW_POW_ODR_POS                   UINT8_C(0x05)
#define BMA456MM_PWR_MGT_ENABLE_MSK                UINT8_C(0x80)
#define BMA456MM_PWR_MGT_ENABLE_POS                UINT8_C(0x07)

#define BMA456MM_NO_MOT_AUTO_LOW_POWER_WORD_MSK    UINT16_C(0x01)

#define BMA456MM_AUTO_LOW_POWER_TIME_OUT_WORD_MSK  UINT16_C(0x02)
#define BMA456MM_AUTO_LOW_POWER_TIME_OUT_WORD_POS  UINT8_C(0x1)

#define BMA456MM_TIME_OUT_DUR_WORD_MSK             UINT16_C(0X1FFC)
#define BMA456MM_TIME_OUT_DUR_WORD_POS             UINT8_C(0x02)

#define BMA456MM_LOW_POW_ODR_WORD_MSK              UINT16_C(0x6000)
#define BMA456MM_LOW_POW_ODR_WORD_POS              UINT8_C(0x0D)

#define BMA456MM_PWR_MGT_ENABLE_WORD_MSK           UINT16_C(0x8000)
#define BMA456MM_PWR_MGT_ENABLE_WORD_POS           UINT8_C(0x0F)

#define BMA456MM_AUTO_LOW_POWER_STATE_POS          UINT8_C(0x04)
#define BMA456MM_AUTO_LOW_POWER_STATE_MSK          UINT8_C(0x10)

/**************************************************************/
/**\name    High-g */
/**************************************************************/

/**\name High-g enable macros */
#define BMA456MM_HIGH_G_EN_OFFSET                  UINT8_C(0x31)
#define BMA456MM_HIGH_G_EN_MSK                     UINT8_C(0x80)
#define BMA456MM_HIGH_G_EN_POS                     UINT8_C(15)

/**\name High-g threshold macros */
#define BMA456MM_HIGH_G_THRES_MSK                  UINT16_C(0x7FFF)

/**\name High-g hysteresis macros */
#define BMA456MM_HIGH_G_HYST_MSK                   UINT16_C(0x0FFF)

/**\name High-g duration macros */
#define BMA456MM_HIGH_G_DUR_MSK                    UINT16_C(0x0FFF)

/**\name High-g axis enable macros */
#define BMA456MM_HIGH_G_AXIS_EN_MSK                UINT8_C(0x7000)
#define BMA456MM_HIGH_G_AXIS_EN_POS                UINT8_C(0x0C)

/**\name High-g axis selection macros */
#define BMA456MM_HIGH_G_X_EN                       UINT8_C(0x01)
#define BMA456MM_HIGH_G_Y_EN                       UINT8_C(0x02)
#define BMA456MM_HIGH_G_Z_EN                       UINT8_C(0x04)
#define BMA456MM_HIGH_G_EN_ALL_AXIS                UINT8_C(0x07)
#define BMA456MM_HIGH_G_DIS_ALL_AXIS               UINT8_C(0x00)

/**\name High-g output macros */
#define BMA456MM_HIGH_G_DETECT_X_MSK               UINT8_C(0x01)

#define BMA456MM_HIGH_G_DETECT_Y_MSK               UINT8_C(0x02)
#define BMA456MM_HIGH_G_DETECT_Y_POS               UINT8_C(0x01)

#define BMA456MM_HIGH_G_DETECT_Z_MSK               UINT8_C(0x04)
#define BMA456MM_HIGH_G_DETECT_Z_POS               UINT8_C(0x02)

#define BMA456MM_HIGH_G_DETECT_SIGN_MSK            UINT8_C(0x08)
#define BMA456MM_HIGH_G_DETECT_SIGN_POS            UINT8_C(0x03)

/**************************************************************/
/**\name    Significant motion */
/**************************************************************/
/**\name Significant motion enable macros */
#define BMA456MM_SIG_MOTION_EN_OFFSET              UINT8_C(0x37)
#define BMA456MM_SIG_MOTION_EN_POS                 UINT8_C(1)
#define BMA456MM_SIG_MOTION_EN_MSK                 UINT8_C(0x02)

/**\name Significant motion threshold macros */
#define BMA456MM_SIG_MOTION_THRES_POS              UINT8_C(0)
#define BMA456MM_SIG_MOTION_THRES_MSK              UINT16_C(0x7FFF)

/**\name Significant motion skiptime macros */
#define BMA456MM_SIG_MOTION_SKIPTIME_MSK           UINT16_C(0x01FF)

/**\name Significant motion prooftime macros */
#define BMA456MM_SIG_MOTION_PROOFTIME_POS          UINT8_C(0)
#define BMA456MM_SIG_MOTION_PROOFTIME_MSK          UINT8_C(0x7F)

/**************************************************************/
/**\name                 User macros                          */
/**************************************************************/
/**\name Any-motion/No-motion axis enable macros */
#define BMA456MM_X_AXIS_EN                         UINT8_C(0x01)
#define BMA456MM_Y_AXIS_EN                         UINT8_C(0x02)
#define BMA456MM_Z_AXIS_EN                         UINT8_C(0x04)
#define BMA456MM_EN_ALL_AXIS                       UINT8_C(0x07)
#define BMA456MM_DIS_ALL_AXIS                      UINT8_C(0x00)

/**\name Feature enable macros for the sensor */
#define BMA456MM_LOW_G                             UINT8_C(0x01)
#define BMA456MM_ORIENT                            UINT8_C(0x02)
#define BMA456MM_SINGLE_TAP                        UINT8_C(0x04)
#define BMA456MM_DOUBLE_TAP                        UINT8_C(0x08)
#define BMA456MM_TRIPLE_TAP                        UINT8_C(0x10)
#define BMA456MM_AUTO_LOW_POWER                    UINT8_C(0x20)
#define BMA456MM_HIGH_G                            UINT8_C(0x40)
#define BMA456MM_SIG_MOTION                        UINT8_C(0x80)

/**\name Interrupt status macros */
#define BMA456MM_TAP_OUT_INT                       UINT8_C(0x01)
#define BMA456MM_ORIENT_INT                        UINT8_C(0x02)
#define BMA456MM_LOW_G_INT                         UINT8_C(0x04)
#define BMA456MM_HIGH_G_INT                        UINT8_C(0x08)
#define BMA456MM_SIG_MOT_INT                       UINT8_C(0x10)
#define BMA456MM_ANY_MOT_INT                       UINT8_C(0x20)
#define BMA456MM_NO_MOT_INT                        UINT8_C(0x40)
#define BMA456MM_ERROR_INT                         UINT8_C(0x80)

/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/

/*!
 * @brief Any/No motion configuration
 */
struct bma456mm_any_no_mot_config
{
    /*! Expressed in 50 Hz samples (20 ms) */
    uint16_t duration;

    /*! Threshold value for Any-motion/No-motion detection in
     * 5.11g format
     */
    uint16_t threshold;

    uint8_t intr_bhvr;

    uint8_t slope;

    /*! To enable selected axes */
    uint8_t axes_en;
};

/*!
 * @brief Orientation configuration structure
 */
struct bma456mm_orientation_config
{
    /*! Upside/Downside detection */
    uint8_t upside_down;

    /*! Mode
     * Symmetrical (values 0 or 3), High asymmetrical
     * (value 1) or Low asymmetrical (value 2)
     */
    uint8_t mode;

    /*! Blocking mode */
    uint8_t blocking;

    /*! Coded value of the threshold angle with horizontal
     * used in Blocking modes
     */
    uint8_t theta;

    /*! Acceleration hysteresis for Orientation detection */
    uint16_t hysteresis;
};

/*!
 * @brief Low-G configuration
 */
struct bma456mm_low_g_config
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
 * @brief Auto sleep configuration
 */
struct bma456mm_auto_low_power
{
    /*! Enters auto-sleep, when no-motion is detected */
    uint8_t no_motion;

    /*! Enters auto-sleep, when any-motion is not detected for time_out_dur period */
    uint8_t time_out;

    /*! Duration to enter to auto sleep, when any-motion event is not detected.
     * Range            : 0 ms to 40690 ms
     * Resolution       : 20 ms
     * Default Value    : 2000 ms
     */
    uint16_t time_out_dur;

    /*!       ODR for low power mode
     * ---------------------------------------
     * Value        Name        Description
     * ---------------------------------------
     *   0          odr_1p5        25/16 Hz
     *   1          odr_3p1        25/8 Hz
     *   2          odr_6p25       25/4 Hz
     *   3          odr_12p5       25/2 Hz
     */
    uint8_t lp_odr;

    /*!           Power management
     * --------------------------------------------
     *  Value       Name            Description
     * --------------------------------------------
     *    0        Disable      Disable feature optimized
     *                          acc conf. Uses host desired
     *                          configuration
     *    1        Enable       Enable feature optimized
     *                          acc conf
     */
    uint8_t pwr_mgt;
};

/*!
 * @brief Tap param settings
 */
struct bma456mm_multitap_settings
{
    /*! Reserved parameter */
    uint16_t reserved_1;

    /*! Scaling factor for threshold */
    uint16_t tap_sens_thres;

    /*! Maximum duration after the first tap */
    uint16_t max_gest_dur;

    /*! Reserved parameter */
    uint16_t reserved_4;

    /*! Settling time for high frequency acceleration signal */
    uint16_t tap_shock_dur;

    /*! Reserved parameter */
    uint16_t reserved_6;

    /*! Minimum quite time between the two gesture detection */
    uint16_t quite_time_after_gest;

    /*! Wait for the duration set by max_gest_dur after the first tap */
    uint16_t wait_for_timeout;

    /*! Reserved parameter */
    uint16_t reserved_9;

    /*! Selection of axis from 3D-acceleration signal vector */
    uint16_t axis_sel;

    /*! Reserved parameter */
    uint16_t reserved_11;

    /*! Reserved parameter */
    uint16_t reserved_12;
};

/*!
 * @brief High-g configuration
 */
struct bma456mm_high_g_config
{
    /*! Expressed in 200 Hz samples (5 ms) */
    uint16_t duration;

    /*! Hysteresis value for high-g in 0.74g */
    uint16_t hysteresis;

    /*! Threshold value for Any motion detection in
     * 5.11g format
     */
    uint16_t threshold;

    /*! To enable selected axes */
    uint8_t axes_en;
};

/*!
 * @brief Significant motion configuration
 */
struct bma456mm_sig_motion_config
{
    /*! Holds the threshold in 5.11 g format */
    uint16_t threshold;

    /*! Holds the duration for skip in 50Hz samples (20ms) */
    uint16_t skiptime;

    /*! Holds the duration for proof in 50Hz samples (20ms) */
    uint8_t prooftime;
};

/*!
 * @brief activity, tap output state
 */
struct bma456mm_out_state
{
    /*!
    *-------------------------------|-----------------------------
    *   orientation_output          |        Values
    *-------------------------------|-----------------------------
    *   Bit pos 0-1 reflects        |   BMA456MM_PORTRAIT_UP_RIGHT
    *   orientation output value    |   BMA456MM_LANDSCAPE_LEFT
    *   only if ud_en is enabled    |   BMA456MM_PORTRAIT_UP_DOWN
    *                               |   BMA456MM_LANDSCAPE_RIGHT
    *-------------------------------|-----------------------------
    */
    uint8_t orientation_out;

    /*!
    *-------------------------------|-----------------------------
    *   orientation_faceup_down     |        Values
    *-------------------------------|-----------------------------
    *   Bit pos 2 reflects          |   BMA456MM_FACE_UP
    *   face-up (0) or face-down(1) |   BMA456MM_FACE_DOWN
    *   only if ud_en is enabled    |
    *-------------------------------|-----------------------------
    */
    uint8_t orientation_faceup_down;

    /*! High-g detected on X-axis */
    uint8_t high_g_detect_x;

    /*! High-g detected on Y-axis */
    uint8_t high_g_detect_y;

    /*! High-g detected on Z-axis */
    uint8_t high_g_detect_z;

    /*! Axis direction for which the high-g was detected.
     *  1 for negative axis, 0 for positive axis.
     */
    uint8_t high_g_detect_sign;

    /*! Single tap detected */
    uint8_t s_tap;

    /*! Double tap detected */
    uint8_t d_tap;

    /*! Triple tap detected */
    uint8_t t_tap;
};

/***************************************************************************/

/*!             BMA456MM User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bma456mmApiInit
 * \page bma456mm_api_bma456mm_init bma456mm_init
 * \code
 * int8_t bma456mm_init(struct bma4_dev *dev);
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
int8_t bma456mm_init(struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiConfig Config
 * @brief Configuration APIs
 */

/*!
 * \ingroup bma456mmApiConfig
 * \page bma456mm_api_bma456mm_write_config_file bma456mm_write_config_file
 * \code
 * int8_t bma456mm_write_config_file(struct bma4_dev *dev);
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
int8_t bma456mm_write_config_file(struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApiConfig
 * \page bma456mm_api_bma456mm_get_version_config bma456mm_get_version_config
 * \code
 *int8_t bma456mm_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev);
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
int8_t bma456mm_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApiConfig
 * \page bma456mm_api_bma456mm_get_config_id bma456mm_get_config_id
 * \code
 * int8_t bma456mm_get_config_id(uint16_t *config_id, struct bma4_dev *dev);
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
int8_t bma456mm_get_config_id(uint16_t *config_id, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiMapInt Map / Unmap Interrupt
 * @brief Map / Unmap user provided interrupt to interrupt pin1 or pin2 of the sensor
 */

/*!
 * \ingroup bma456mmApiMapInt
 * \page bma456mm_api_bma456mm_map_interrupt bma456mm_map_interrupt
 * \code
 * int8_t bma456mm_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);
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
 *  - BMA456MM_SINGLE_TAP_INT
 *  - BMA456MM_ORIENT_INT
 *  - BMA456MM_LOW_G_INT
 *  - BMA456MM_DOUBLE_TAP_INT
 *  - BMA456MM_ANY_MOT_INT
 *  - BMA456MM_NO_MOT_INT
 *  - BMA456MM_ERROR_INT
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
int8_t bma456mm_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiIntS Interrupt Status
 * @brief Read interrupt status of the sensor
 */

/*!
 * \ingroup bma456mmApiIntS
 * \page bma456mm_api_bma456mm_read_int_status bma456mm_read_int_status
 * \code
 * int8_t bma456mm_read_int_status(uint16_t *int_status, struct bma4_dev *dev);
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
 *  - BMA456MM_SINGLE_TAP_INT
 *  - BMA456MM_ORIENT_INT
 *  - BMA456MM_LOW_G_INT
 *  - BMA456MM_DOUBLE_TAP_INT
 *  - BMA456MM_ANY_MOT_INT
 *  - BMA456MM_NO_MOT_INT
 *  - BMA456MM_ERROR_INT
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
int8_t bma456mm_read_int_status(uint16_t *int_status, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiFeat Sensor Feature
 * @brief Enables / Disables features of the sensor
 */

/*!
 * \ingroup bma456mmApiFeat
 * \page bma456mm_api_bma456mm_feature_enable bma456mm_feature_enable
 * \code
 * int8_t bma456mm_feature_enable(uint8_t feature, uint8_t enable, struct bma4_dev *dev);
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
 * features of bma456mm sensor
 *
 *   - BMA456MM_LOW_G
 *   - BMA456MM_ORIENT
 *   - BMA456MM_SINGLE_TAP
 *   - BMA456MM_DOUBLE_TAP
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456mm_feature_enable(uint8_t feature, uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiRemap Remap Axes
 * @brief Set / Get x, y and z axis re-mapping in the sensor
 */

/*!
 * \ingroup bma456mmApiRemap
 * \page bma456mm_api_bma456mm_set_remap_axes bma456mm_set_remap_axes
 * \code
 * int8_t bma456mm_set_remap_axes(const struct bma4_remap *remap_data, struct bma4_dev *dev);
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
int8_t bma456mm_set_remap_axes(const struct bma4_remap *remap_data, struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApiRemap
 * \page bma456mm_api_bma456mm_get_remap_axes bma456mm_get_remap_axes
 * \code
 * int8_t bma456mm_get_remap_axes(struct bma4_remap *remap_data, struct bma4_dev *dev);
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
int8_t bma456mm_get_remap_axes(struct bma4_remap *remap_data, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiAnyMot Any motion Feature
 * @brief Functions of Any motion feature of the sensor
 */

/*!
 * \ingroup bma456mmApiAnyMot
 * \page bma456mm_api_bma456mm_set_any_mot_config bma456mm_set_any_mot_config
 * \code
 * int8_t bma456mm_set_any_mot_config(const struct bma456mm_any_no_mot_config *any_mot, struct bma4_dev *dev);
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
 *  0x00     |  BMA456MM_DIS_ALL_AXIS
 *  0x01     |  BMA456MM_X_AXIS_EN
 *  0x02     |  BMA456MM_Y_AXIS_EN
 *  0x04     |  BMA456MM_Z_AXIS_EN
 *  0x07     |  BMA456MM_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456mm_set_any_mot_config(const struct bma456mm_any_no_mot_config *any_mot, struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApiAnyMot
 * \page bma456mm_api_bma456mm_get_any_motion_config bma456mm_get_any_motion_config
 * \code
 * int8_t bma456mm_get_any_motion_config(struct bma456mm_anymotion_config *any_motion, struct bma4_dev *dev);
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
 *  0x00     |  BMA456MM_DIS_ALL_AXIS
 *  0x01     |  BMA456MM_X_AXIS_EN
 *  0x02     |  BMA456MM_Y_AXIS_EN
 *  0x04     |  BMA456MM_Z_AXIS_EN
 *  0x07     |  BMA456MM_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456mm_get_any_mot_config(struct bma456mm_any_no_mot_config *any_mot, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiNomot No-Motion Feature
 * @brief Operations of no-motion feature of the sensor
 */

/*!
 * \ingroup bma456mmApiNomot
 * \page bma456mm_api_bma456mm_set_no_motion_config bma456mm_set_no_motion_config
 * \code
 * int8_t bma456mm_set_no_motion_config(const struct bma456mm_nomotion_config *no_motion, struct bma4_dev *dev);
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
 *  0x00     |  BMA456MM_DIS_ALL_AXIS
 *  0x01     |  BMA456MM_X_AXIS_EN
 *  0x02     |  BMA456MM_Y_AXIS_EN
 *  0x04     |  BMA456MM_Z_AXIS_EN
 *  0x07     |  BMA456MM_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456mm_set_no_mot_config(const struct bma456mm_any_no_mot_config *no_mot, struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApiNomot
 * \page bma456mm_api_bma456mm_get_no_motion_config bma456mm_get_no_motion_config
 * \code
 * int8_t bma456mm_get_no_motion_config(struct bma456mm_nomotion_config *no_motion, struct bma4_dev *dev);
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
 *  0x00     |  BMA456MM_DIS_ALL_AXIS
 *  0x01     |  BMA456MM_X_AXIS_EN
 *  0x02     |  BMA456MM_Y_AXIS_EN
 *  0x04     |  BMA456MM_Z_AXIS_EN
 *  0x07     |  BMA456MM_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456mm_get_no_mot_config(struct bma456mm_any_no_mot_config *no_mot, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiOrientation Orientation Feature
 * @brief Functions of Orientation feature of the sensor
 */

/*!
 * \ingroup bma456mmApiOrientation
 * \page bma456mm_api_bma456mm_set_orientation_config bma456mm_set_orientation_config
 * \code
 * int8_t bma456mm_set_orientation_config(struct bma456mm_orientation_config *orientation, struct bma4_dev *dev);
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
 *                                 |        Symmetrical (values 0 or 3),
 *         mode                    |        High asymmetrical (value 1)
 *                                 |        or Low asymmetrical (value 2).
 * --------------------------------|----------------------------------------
 *                                 |        Sets the blocking mode.
 *         blocking                |        Default value is 3, the most
 *                                 |        restrictive blocking mode.
 * -------------------------------------------------------------------------
 *                                 |        Coded value of the threshold angle
 *                                 |        with horizontal used in blocking
 *        theta                    |        modes, theta = 64 * (tan(angle)^2).
 *                                 |        Default value is 40 which is
 *                                 |        equivalent to 38 degrees angle.
 * -------------------------------------------------------------------------
 *                                 |        Acceleration hysteresis for
 *                                 |        orientation detection.
 *       hysteresis                |        Range is 0 to 1g.
 *                                 |        Default value is 0x80 = 0.0625g.
 * -------------------------------------------------------------------------
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456mm_set_orientation_config(const struct bma456mm_orientation_config *orientation, struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApiOrientation
 * \page bma456mm_api_bma456mm_get_orientation_config bma456mm_get_orientation_config
 * \code
 * int8_t bma456mm_get_orientation_config(struct bma456mm_orientation_config *orientation, struct bma4_dev *dev);
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
 *         blocking                |        Default value is 3, the most
 *                                 |        restrictive blocking mode.
 * -------------------------------------------------------------------------
 *                                 |        Coded value of the threshold angle
 *                                 |        with horizontal used in blocking
 *        theta                    |        modes, theta = 64 * (tan(angle)^2).
 *                                 |        Default value is 40 which is
 *                                 |        equivalent to 38 degrees angle.
 * -------------------------------------------------------------------------
 *                                 |        Acceleration hysteresis for
 *                                 |        orientation detection.
 *       hysteresis                |        Range is 0 to 1g.
 *                                 |        Default value is 0x80 = 0.0625g.
 * -------------------------------------------------------------------------
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456mm_get_orientation_config(struct bma456mm_orientation_config *orientation, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApLowG Low-G Feature
 * @brief Functions of Low-G feature of the sensor
 */

/*!
 * \ingroup bma456mmApLowG
 * \page bma456mm_api_bma456mm_set_low_g_config bma456mm_set_low_g_config
 * \code
 * int8_t bma456mm_set_low_g_config(const struct bma456mm_low_g_config *low_g, struct bma4_dev *dev);
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
int8_t bma456mm_set_low_g_config(const struct bma456mm_low_g_config *low_g, struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApLowG
 * \page bma456mm_api_bma456mm_get_low_g_config bma456mm_get_low_g_config
 * \code
 * int8_t bma456mm_get_low_g_config(struct bma456mm_low_g_config *low_g, struct bma4_dev *dev);
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
int8_t bma456mm_get_low_g_config(struct bma456mm_low_g_config *low_g, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApOP Output state
 * @brief Functions to provide feature output status
 */

/*!
 * \ingroup bma456mmApOP
 * \page bma456mm_api_bma456mm_output_state bma456mm_output_state
 * \code
 * int8_t bma456mm_output_state(struct bma456mm_out_state *out_state, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the output for orientation, tap(single, double, triple) and high-g features.
 *
 * @param[out] out_state : Pointer variable which stores feature output values.
 * @param[in] dev        : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456mm_output_state(struct bma456mm_out_state *out_state, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiTap Tap Feature
 * @brief Tap feature operations
 */

/*!
 * \ingroup bma456mmApiTap
 * \page bma456mm_api_bma456mm_tap_get_parameter bma456mm_tap_get_parameter
 * \code
 * int8_t bma456mm_tap_get_parameter(struct bma456mm_multitap_settings *setting, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the parameter1 to parameter12 settings of the tap
 * feature.
 *
 * @param[out] setting : Pointer to structure variable which stores the
 * parameter1 to parameter12 read from the sensor.
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456mm_tap_get_parameter(struct bma456mm_multitap_settings *setting, struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApiTap
 * \page bma456mm_api_bma456mm_tap_set_parameter bma456mm_tap_set_parameter
 * \code
 * int8_t bma456mm_tap_set_parameter(const struct bma456mm_multitap_settings *setting, struct bma4_dev *dev);
 * \endcode
 * @details This API sets the parameter1 to parameter12 settings of the tap
 * feature in the sensor.
 *
 * @param[in] setting : Pointer to structure variable which stores the
 * parameter1 to parameter12 settings read from the sensor.
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456mm_tap_set_parameter(const struct bma456mm_multitap_settings *setting, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiAutoLP Auto Low power mode
 * @brief Configurations and Status of auto low power mode of the sensor
 */

/*!
 * \ingroup bma456mmApiAutoLP
 * \page bma456mm_api_bma456mm_get_auto_low_power_config bma456mm_get_auto_low_power_config
 * \code
 * int8_t bma456mm_get_auto_low_power_config(struct bma456mm_auto_low_power *auto_low_power, struct bma4_dev *dev);
 * \endcode
 * @details This api gets the auto low power configuration
 *
 * @param auto_low_power[out]  : Pointer gets the auto low power configuration.
 * @param dev[in]   : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456mm_get_auto_low_power_config(struct bma456mm_auto_low_power *auto_low_power, struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApiAutoLP
 * \page bma456mm_api_bma456mm_set_auto_low_power_config bma456mm_set_auto_low_power_config
 * \code
 * int8_t bma456mm_set_auto_low_power_config(const struct bma456mm_auto_low_power *auto_low_power, struct bma4_dev *dev);
 * \endcode
 * @details This api sets the auto low power configuration
 *
 * @param auto_low_power[in]  : Pointer sets the auto low power configuration.
 * @param dev[in]   : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456mm_set_auto_low_power_config(const struct bma456mm_auto_low_power *auto_low_power, struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApiAutoLP
 * \page bma456mm_api_bma456mm_get_auto_low_power_state bma456mm_get_auto_low_power_state
 * \code
 * int8_t bma456mm_get_auto_low_power_state(uint8_t *auto_low_power_state, struct bma4_dev *dev);
 * \endcode
 * @details This api reads the auto low power state output status
 *
 * @param auto_low_power_state[out]  : Pointer reads auto low power state output bit status.
 * @param dev[in]                    : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456mm_get_auto_low_power_state(uint8_t *auto_low_power_state, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApihighg High-g
 * @brief Set/Get Configurations of high-g feature
 */

/*!
 * \ingroup bma456mmApihighg
 * \page bma456mm_api_bma456mm_set_high_g_config bma456mm_set_high_g_config
 * \code
 * int8_t bma456mm_set_high_g_config(struct bma456mm_high_g_config *high_g, struct bma4_dev *dev);
 * \endcode
 * @details This API set the configuration of high_g feature from
 *  the sensor.
 *
 *  @param[in] high_g : Pointer to structure variable used to
 *  store the high_g feature settings read from the sensor.
 *  Structure members are provided in the table below
 *
 *@verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        The acceleration threshold
 *                                 |        above which the high_g motion
 *                                 |        is signed 15 bit, signed int
 *      threshold                  |        holding the threshold in 5.11 g format.
 *                                 |        Default is 3072 = 2.25 g.
 *                                 |        Range is 0 to 24g.
 * --------------------------------|----------------------------------------
 *                                 |        Hysteresis value for high_g feature.
 *      hysterisis                 |        Range is 0 to 3g.
 *                                 |        Default value is 1000 = 0.74g.
 * --------------------------------|----------------------------------------
 *                                 |        12 bit signed character
 *                                 |        (valid values 0...4095) holding
 *      duration                   |        the duration in 200 Hz samples (5 ms)
 *                                 |        for which the threshold has to be
 *                                 |        exceeded; default value 4 = 20 msec.
 *                                 |        Range is 0 to 20sec.
 * --------------------------------|----------------------------------------
 *                                 |        Enables the feature on a per-axis
 *         axis_en                 |        basis.
 * ---------------------------------------------------------------------------
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456mm_set_high_g_config(const struct bma456mm_high_g_config *high_g, struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApihighg
 * \page bma456mm_api_bma456mm_get_high_g_config bma456mm_get_high_g_config
 * \code
 * int8_t bma456mm_get_high_g_config(struct bma456mm_high_g_config *high_g, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the configuration of high_g feature from
 *  the sensor.
 *
 *  @param[out] high_g : Pointer to structure variable used to
 *  store the high_g feature settings read from the sensor.
 *  Structure members are provided in the table below
 *
 *@verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        The acceleration threshold
 *                                 |        above which the high_g motion
 *                                 |        is signed 15 bit, signed int
 *      threshold                  |        holding the threshold in 5.11 g format.
 *                                 |        Default is 3072 = 2.25 g.
 *                                 |        Range is 0 to 24g.
 * --------------------------------|----------------------------------------
 *                                 |        Hysteresis value for high_g feature.
 *      hysterisis                 |        Range is 0 to 3g.
 *                                 |        Default value is 1000 = 0.74g.
 * --------------------------------|----------------------------------------
 *                                 |        12 bit signed character
 *                                 |        (valid values 0...4095) holding
 *      duration                   |        the duration in 200 Hz samples (5 ms)
 *                                 |        for which the threshold has to be
 *                                 |        exceeded; default value 4 = 20 msec.
 *                                 |        Range is 0 to 20sec.
 * --------------------------------|----------------------------------------
 *                                 |        Enables the feature on a per-axis
 *         axis_en                 |        basis.
 * ---------------------------------------------------------------------------
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456mm_get_high_g_config(struct bma456mm_high_g_config *high_g, struct bma4_dev *dev);

/**
 * \ingroup bma456mm
 * \defgroup bma456mmApiSigMot Significant motion Feature
 * @brief Functions of Significant motion feature of the sensor
 */

/*!
 * \ingroup bma456mmApiSigMot
 * \page bma456mm_api_bma456mm_set_sig_motion_config bma456mm_set_sig_motion_config
 * \code
 * int8_t bma456mm_set_sig_motion_config(const struct bma456mm_sig_motion_config *sig_motion, struct bma4_dev *dev);
 * \endcode
 * @details  This API sets the configuration of significant motion
 *  in the sensor.
 *
 *  @param[in]  sig_motion : Pointer to structure variable used to
 *  specify the significant motion feature settings.
 *  structure members are provided in the table below
 *
 *@verbatim
 * -------------------------------------------------------------------------------------
 *    Structure parameters     |        Description
 * -------------------------------------------------------------------------------------
 *                             | Slope threshold value for this feature
 *           threshold         | above which the significant motion is detected.
 *                             | Range is 0 to 16g. Default is 307 = 150mg.
 * -------------------------------------------------------------------------------------
 *                             | Defines the number of consecutive data points
 *                             | for which the feature remains in sleep mode
 *                             | after the first significant motion detection.
 *          skip_time          | The feature checks for significant motion detection
 *                             | again after this sleep duration.
 *                             | It is expressed in 50 Hz samples (20 ms).
 *                             | Range is 0 to 10sec. Default is 150 = 3sec
 * -------------------------------------------------------------------------------------
 *                             | Defines duration of certain number of consecutive
 *                             | data points after sleep time. The second significant
 *           proof_time        | motion must be detected within this duration for
 *                             | the interrupt to get triggered.
 *                             | Range is 0 to 2.5sec. Default value is 50 = 1sec.
 * -------------------------------------------------------------------------------------
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456mm_set_sig_motion_config(const struct bma456mm_sig_motion_config *sig_motion, struct bma4_dev *dev);

/*!
 * \ingroup bma456mmApiSigMot
 * \page bma456mm_api_bma456mm_get_sig_motion_config bma456mm_get_sig_motion_config
 * \code
 * int8_t bma456mm_get_sig_motion_config(struct bma456mm_sig_motion_config *sig_motion, struct bma4_dev *dev);
 * \endcode
 * @details  This API gets the configuration of significant motion feature
 *  from the sensor.
 *
 *  @param[out] sig_motion : Pointer  to structure variable used to
 *  store the significant motion feature settings read from the sensor.
 *  structure members are provided in the table below
 *
 *@verbatim
 * -------------------------------------------------------------------------------------
 *    Structure parameters     |        Description
 * -------------------------------------------------------------------------------------
 *                             | Slope threshold value for this feature
 *           threshold         | above which the significant motion is detected.
 *                             | Range is 0 to 16g. Default is 307 = 150mg.
 * -------------------------------------------------------------------------------------
 *                             | Defines the number of consecutive data points
 *                             | for which the feature remains in sleep mode
 *                             | after the first significant motion detection.
 *          skip_time          | The feature checks for significant motion detection
 *                             | again after this sleep duration.
 *                             | It is expressed in 50 Hz samples (20 ms).
 *                             | Range is 0 to 10sec. Default is 150 = 3sec
 * -------------------------------------------------------------------------------------
 *                             | Defines duration of certain number of consecutive
 *                             | data points after sleep time. The second significant
 *           proof_time        | motion must be detected within this duration for
 *                             | the interrupt to get triggered.
 *                             | Range is 0 to 2.5sec. Default value is 50 = 1sec.
 * -------------------------------------------------------------------------------------
 *@endverbatim
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma456mm_get_sig_motion_config(struct bma456mm_sig_motion_config *sig_motion, struct bma4_dev *dev);

#ifdef __cplusplus
}
#endif /*End of CPP guard */

#endif /*End of header guard macro */

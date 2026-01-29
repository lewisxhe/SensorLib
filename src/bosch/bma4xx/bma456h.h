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
* @file       bma456h.h
* @date       2023-07-05
* @version    V2.29.0
*
*/

/**
 * \ingroup bma4xy
 * \defgroup bma456h BMA456H
 * @brief Sensor driver for BMA456H sensor
 */

#ifndef BMA456H_H
#define BMA456H_H

#ifdef __cplusplus
extern "C" {
#endif
#include "bma4.h"

/**\name Chip ID of BMA456H sensor */
#define BMA456H_CHIP_ID                           UINT8_C(0x16)

/**\ Configuration ID start position of BMA456H sensor */
#define BMA456H_CONFIG_ID_START_ADDR              UINT8_C(90)

/**\name Sensor feature size */
#define BMA456H_FEATURE_SIZE                      UINT8_C(94)
#define BMA456H_FEATURE_SIZE_WORDS                UINT8_C(46)
#define BMA456H_ANY_MOT_LEN                       UINT8_C(4)

/**************************************************************/
/**\name    Feature input offset: Features enable position    */
/**************************************************************/
#define BMA456H_ANY_MOT_OFFSET                    UINT8_C(0x00)
#define BMA456H_NO_MOT_OFFSET                     UINT8_C(0x04)
#define BMA456H_STEP_CNTR_PARAM_OFFSET            UINT8_C(0x08)
#define BMA456H_STEP_CNTR_OFFSET                  UINT8_C(0x3A)
#define BMA456H_TAP_PARAM_OFFSET                  UINT8_C(0x3C)
#define BMA456H_AUTO_LOW_POWER_OFFSET             UINT8_C(0x58)
#define BMA456H_AUTO_LOW_POWER_OFFSET_WORDS       UINT8_C(0x2C)

/**\name Read/Write Lengths */
#define BMA456H_RD_WR_MIN_LEN                     UINT8_C(2)
#define BMA456H_NO_MOT_RD_WR_LEN                  (BMA456H_ANY_MOT_LEN + BMA456H_NO_MOT_OFFSET)

/*! @name Maximum valid read write length is size of config file array */
#define BMA456H_RD_WR_MAX_LEN                     ((uint16_t)sizeof(bma456h_config_file))

/**************************************************************/
/**\name    General settings: Re-map Axes */
/**************************************************************/
#define BMA456H_CONFIG_ID_OFFSET                  UINT8_C(0x5A)
#define BMA456H_AXES_REMAP_OFFSET                 UINT8_C(0x5C)
#define BMA456H_X_AXIS_MASK                       UINT8_C(0x03)
#define BMA456H_X_AXIS_SIGN_MASK                  UINT8_C(0x04)
#define BMA456H_Y_AXIS_MASK                       UINT8_C(0x18)
#define BMA456H_Y_AXIS_SIGN_MASK                  UINT8_C(0x20)
#define BMA456H_Z_AXIS_MASK                       UINT8_C(0xC0)
#define BMA456H_Z_AXIS_SIGN_MASK                  UINT8_C(0x01)

/**************************************************************/
/**\name    Output: Features enable position */
/**************************************************************/
#define BMA456H_NO_MOT_X_EN_POS                   UINT8_C(0)
#define BMA456H_NO_MOT_Y_EN_POS                   UINT8_C(1)
#define BMA456H_NO_MOT_Z_EN_POS                   UINT8_C(2)
#define BMA456H_ANY_MOT_X_EN_POS                  UINT8_C(3)
#define BMA456H_ANY_MOT_Y_EN_POS                  UINT8_C(4)
#define BMA456H_ANY_MOT_Z_EN_POS                  UINT8_C(5)

#define BMA456H_STEP_DETR_EN_POS                  UINT8_C(0)
#define BMA456H_STEP_CNTR_EN_POS                  UINT8_C(1)
#define BMA456H_STEP_ACT_EN_POS                   UINT8_C(2)
#define BMA456H_AUTO_LOW_POWER_EN_POS             UINT8_C(3)
#define BMA456H_SIN_TAP_EN_POS                    UINT8_C(4)
#define BMA456H_DBL_TAP_EN_POS                    UINT8_C(5)
#define BMA456H_TPL_TAP_EN_POS                    UINT8_C(6)
#define BMA456H_TAP_AVERAGE_EN_POS                UINT8_C(7)

/**************************************************************/
/**\name    Output: Features enable mask */
/**************************************************************/
#define BMA456H_NO_MOT_X_EN_MSK                   UINT8_C(0x01)
#define BMA456H_NO_MOT_Y_EN_MSK                   UINT8_C(0x02)
#define BMA456H_NO_MOT_Z_EN_MSK                   UINT8_C(0x04)
#define BMA456H_NO_MOT_ALL_EN_MSK                 UINT8_C(0x07)
#define BMA456H_NO_MOT_ALL_DIS_MSK                UINT8_C(0x00)
#define BMA456H_ANY_MOT_X_EN_MSK                  UINT8_C(0x08)
#define BMA456H_ANY_MOT_Y_EN_MSK                  UINT8_C(0x10)
#define BMA456H_ANY_MOT_Z_EN_MSK                  UINT8_C(0x20)
#define BMA456H_ANY_MOT_ALL_EN_MSK                UINT8_C(0x38)
#define BMA456H_ANY_MOT_ALL_DIS_MSK               UINT8_C(0x00)

#define BMA456H_STEP_DETR_EN_MSK                  UINT8_C(0x01)
#define BMA456H_STEP_CNTR_EN_MSK                  UINT8_C(0x02)
#define BMA456H_STEP_ACT_EN_MSK                   UINT8_C(0x04)
#define BMA456H_AUTO_LOW_POWER_EN_MSK             UINT8_C(0x08)
#define BMA456H_SINGLE_TAP_EN_MSK                 UINT8_C(0x10)
#define BMA456H_DOUBLE_TAP_EN_MSK                 UINT8_C(0x20)
#define BMA456H_TRIPLE_TAP_EN_MSK                 UINT8_C(0x40)
#define BMA456H_TAP_AVERAGE_EN_MSK                UINT8_C(0x80)

/**\name Step counter water-mark macros */
#define BMA456H_STEP_CNTR_WM_MSK                  UINT16_C(0x03FF)

/**\name Step counter reset macros */
#define BMA456H_STEP_CNTR_RST_POS                 UINT8_C(2)
#define BMA456H_STEP_CNTR_RST_MSK                 UINT8_C(0x04)

/**\name Step counter output length */
#define BMA456H_STEP_CNTR_DATA_SIZE               UINT16_C(4)

/**\name Step activity output macros */
#define BMA456H_ACTIVITY_OUT_MSK                  UINT8_C(0x03)
#define BMA456H_ACTIVITY_OUT_POS                  UINT8_C(0x00)

/**\name Tap output macros */
#define BMA456H_SINGLE_TAP_OUT_MSK                UINT8_C(0x04)
#define BMA456H_SINGLE_TAP_OUT_POS                UINT8_C(0x02)

#define BMA456H_DOUBLE_TAP_OUT_MSK                UINT8_C(0x08)
#define BMA456H_DOUBLE_TAP_OUT_POS                UINT8_C(0x03)

#define BMA456H_TRIPLE_TAP_OUT_MSK                UINT8_C(0x10)
#define BMA456H_TRIPLE_TAP_OUT_POS                UINT8_C(0x04)

#define BMA456H_TAP_OUT_MSK                       UINT8_C(0x1C)
#define BMA456H_TAP_OUT_POS                       UINT8_C(0x02)

/**************************************************************/
/**\name    Any/no Motion */
/**************************************************************/
/**\name Any/No motion threshold macros */
#define BMA456H_ANY_NO_MOT_THRES_MSK              UINT16_C(0x07FF)

/**\name Any/No motion duration macros */
#define BMA456H_ANY_NO_MOT_DUR_MSK                UINT16_C(0x1FFF)

/**\name Any/No motion enable macros */
#define BMA456H_ANY_NO_MOT_AXIS_EN_POS            UINT8_C(0x03)
#define BMA456H_ANY_NO_MOT_AXIS_EN_MSK            UINT16_C(0x78)

#define BMA456H_NO_MOT_AUTO_LOW_POWER_MSK         UINT8_C(0x01)

#define BMA456H_AUTO_LOW_POWER_TIME_OUT_MSK       UINT8_C(0x02)
#define BMA456H_AUTO_LOW_POWER_TIME_OUT_POS       UINT8_C(0x1)
#define BMA456H_TIME_OUT_DUR_MSK                  UINT16_C(0x1FFC)
#define BMA456H_TIME_OUT_DUR_POS                  UINT16_C(0x02)
#define BMA456H_LOW_POW_ODR_MSK                   UINT8_C(0x60)
#define BMA456H_LOW_POW_ODR_POS                   UINT8_C(0x05)
#define BMA456H_PWR_MGT_ENABLE_MSK                UINT8_C(0x80)
#define BMA456H_PWR_MGT_ENABLE_POS                UINT8_C(0x07)
#define BMA456H_TIME_OUT_DUR_LSB_MSK              UINT16_C(0X00FC)
#define BMA456H_TIME_OUT_DUR_LSB_POS              UINT8_C(0x02)
#define BMA456H_TIME_OUT_DUR_MSB_MSK              UINT16_C(0X07C0)
#define BMA456H_TIME_OUT_DUR_MSB_POS              UINT8_C(0X06)

#define BMA456H_NO_MOT_AUTO_LOW_POWER_WORD_MSK    UINT16_C(0x01)

#define BMA456H_AUTO_LOW_POWER_TIME_OUT_WORD_MSK  UINT16_C(0x02)
#define BMA456H_AUTO_LOW_POWER_TIME_OUT_WORD_POS  UINT8_C(0x1)

#define BMA456H_TIME_OUT_DUR_WORD_MSK             UINT16_C(0X1FFC)
#define BMA456H_TIME_OUT_DUR_WORD_POS             UINT8_C(0x02)

#define BMA456H_LOW_POW_ODR_WORD_MSK              UINT16_C(0x6000)
#define BMA456H_LOW_POW_ODR_WORD_POS              UINT8_C(0x0D)

#define BMA456H_PWR_MGT_ENABLE_WORD_MSK           UINT16_C(0x8000)
#define BMA456H_PWR_MGT_ENABLE_WORD_POS           UINT8_C(0x0F)

/**************************************************************/
/**\name    User macros */
/**************************************************************/
/**************************************************************/
/**\name    Output: Features for enable/disable */
/**************************************************************/
#define BMA456H_NO_MOTION_X_AXIS_EN               UINT16_C(0x0001)
#define BMA456H_NO_MOTION_Y_AXIS_EN               UINT16_C(0x0002)
#define BMA456H_NO_MOTION_Z_AXIS_EN               UINT16_C(0x0004)
#define BMA456H_NO_MOTION_ALL_AXIS_EN             UINT16_C(0x0007)
#define BMA456H_ANY_MOTION_X_AXIS_EN              UINT16_C(0x0008)
#define BMA456H_ANY_MOTION_Y_AXIS_EN              UINT16_C(0x0010)
#define BMA456H_ANY_MOTION_Z_AXIS_EN              UINT16_C(0x0020)
#define BMA456H_ANY_MOTION_ALL_AXIS_EN            UINT16_C(0x0038)

#define BMA456H_STEP_DETECTOR_EN                  UINT16_C(0x0100)
#define BMA456H_STEP_COUNTER_EN                   UINT16_C(0x0200)
#define BMA456H_STEP_ACTIVITY_EN                  UINT16_C(0x0400)
#define BMA456H_AUTO_LOW_POWER_EN                 UINT16_C(0x0800)
#define BMA456H_SINGLE_TAP_EN                     UINT16_C(0x1000)
#define BMA456H_DOUBLE_TAP_EN                     UINT16_C(0x2000)
#define BMA456H_TRIPLE_TAP_EN                     UINT16_C(0x4000)
#define BMA456H_TAP_AVERAGE_EN                    UINT16_C(0x8000)

/**\name Interrupt status macros */
#define BMA456H_TAP_OUT_INT                       UINT8_C(0x01)
#define BMA456H_STEP_CNTR_INT                     UINT8_C(0x02)
#define BMA456H_ACTIVITY_INT                      UINT8_C(0x04)
#define BMA456H_ANY_MOT_INT                       UINT8_C(0x10)
#define BMA456H_NO_MOT_INT                        UINT8_C(0x20)
#define BMA456H_ERROR_INT                         UINT8_C(0x80)

/**\name Activity recognition macros */
#define BMA456H_USER_STATIONARY                   UINT8_C(0x00)
#define BMA456H_USER_WALKING                      UINT8_C(0x01)
#define BMA456H_USER_RUNNING                      UINT8_C(0x02)
#define BMA456H_UNKNOWN_ACTVTY                    UINT8_C(0x03)

/**\name Address of features enable macros */
#define BMA456H_FEAT_EN_ADDR1                     UINT8_C(0x28)
#define BMA456H_FEAT_EN_ADDR2                     UINT8_C(0x29)

#define BMA456H_FEAT_OUT_ADDR                     UINT8_C(0x27)
#define BMA456H_FEAT_EN_SIZE                      UINT8_C(0x02)

/**\name Position and mask of interrupt behavior and slope */
#define BMA456H_ANY_NO_MOTION_INTR_BHVR_EN_POS    UINT8_C(0x03)
#define BMA456H_ANY_NO_MOTION_INTR_BHVR_EN_MSK    UINT8_C(0x08)

#define BMA456H_ANY_MOTION_SLOPE_EN_POS           UINT8_C(0x04)
#define BMA456H_ANY_MOTION_SLOPE_EN_MSK           UINT8_C(0x10)

#define BMA456H_MULTI_INTR                        UINT8_C(0x00)
#define BMA456H_SINGLE_SHOT                       UINT8_C(0x01)

#define BMA456H_NON_CONSECUTIVE                   UINT8_C(0x00)
#define BMA456H_CONSECUTIVE                       UINT8_C(0x01)

#define BMA456H_AUTO_LOW_POWER_STATE_POS          UINT8_C(0x04)
#define BMA456H_AUTO_LOW_POWER_STATE_MSK          UINT8_C(0x10)

/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/

/*!
 * @brief Any/No motion configuration
 */
struct bma456h_any_no_mot_config
{
    /*! Expressed in 50 Hz samples (20 ms) */
    uint16_t duration;

    /*! Threshold value for Any-motion/No-motion detection in
     * 5.11g format
     */
    uint16_t threshold;

    uint8_t intr_bhvr;

    uint8_t slope;
};

/*!
 * @brief Step counter param settings
 */
struct bma456h_stepcounter_settings
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
 * @brief Auto sleep configuration
 */
struct bma456h_auto_low_power
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
struct bma456h_multitap_settings
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
 * @brief activity, tap output state
 */
struct bma456h_out_state
{
    /*! Output value of activity detection feature. Value after device
     *  initialization is 0b0 i.e User Stationary
     * ----------------------------------------------------------------
     *   Value              Name                        Description
     * ----------------------------------------------------------------
     *      0               Still                    User Stationary
     *      1               Walking                  User walking
     *      2               Running                  User running
     *      3               Unknown                  Unknown state
     */
    uint8_t activity_type;

    /*! Single tap detected */
    uint8_t single_tap;

    /*! Double tap detected */
    uint8_t double_tap;

    /*! Triple tap detected */
    uint8_t triple_tap;
};

/***************************************************************************/

/*!     BMA456H User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bma456h
 * \defgroup bma456hApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bma456hApiInit
 * \page bma456h_api_bma456h_init bma456h_init
 * \code
 * int8_t bma456h_init(struct bma4_dev *dev);
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
int8_t bma456h_init(struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiConfig ConfigFile
 * @brief Write binary configuration in the sensor
 */

/*!
 * \ingroup bma456hApiConfig
 * \page bma456h_api_bma456h_write_config_file bma456h_write_config_file
 * \code
 * int8_t bma456h_write_config_file(struct bma4_dev *dev);
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
int8_t bma456h_write_config_file(struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiConfigId ConfigId
 * @brief Get Configuration ID of the sensor
 */

/*!
 * \ingroup bma456hApiConfig
 * \page bma456h_api_bma456h_get_config_id bma456h_get_config_id
 * \code
 * int8_t bma456h_get_config_id(uint16_t *config_id, struct bma4_dev *dev);
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
int8_t bma456h_get_config_id(uint16_t *config_id, struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiMapInt Map / Unmap Interrupt
 * @brief Map / Unmap user provided interrupt to interrupt pin1 or pin2 of the sensor
 */

/*!
 * \ingroup bma456hApiMapInt
 * \page bma456h_api_bma456h_map_interrupt bma456h_map_interrupt
 * \code
 * int8_t bma456h_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);
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
 *  - BMA456H_TAP_OUT_INT
 *  - BMA456H_STEP_CNTR_INT
 *  - BMA456H_ACTIVITY_INT
 *  - BMA456H_ANY_MOT_INT
 *  - BMA456H_ERROR_INT
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
int8_t bma456h_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiIntS Interrupt Status
 * @brief Read interrupt status of the sensor
 */

/*!
 * \ingroup bma456hApiIntS
 * \page bma456h_api_bma456h_read_int_status bma456h_read_int_status
 * \code
 * int8_t bma456h_read_int_status(uint16_t *int_status, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the bma456h interrupt status from the sensor.
 *
 * @param[out] int_status : Variable to store the interrupt status read from
 * the sensor.
 * @param[in] dev : Structure instance of bma4_dev.
 *
 * @note Below macros are used to check the interrupt status.
 *
 * Feature Interrupts
 *  - BMA456H_STEP_CNTR_INT
 *  - BMA456H_ACTIVITY_INT
 *  - BMA456H_SINGLE_TAP_EN
 *  - BMA456H_DOUBLE_TAP_EN
 *  - BMA456H_TRIPLE_TAP_EN
 *  - BMA456H_ANY_MOT_INT
 *  - BMA456H_ERROR_INT
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
int8_t bma456h_read_int_status(uint16_t *int_status, struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiFeat Sensor Feature
 * @brief Enables / Disables features of the sensor
 */

/*!
 * \ingroup bma456hApiFeat
 * \page bma456h_api_bma456h_feature_enable bma456h_feature_enable
 * \code
 * int8_t bma456h_feature_enable(uint8_t feature, uint8_t enable, struct bma4_dev *dev);
 * \endcode
 * @details This API enables/disables the features of the sensor.
 *
 * @param[in] feature : Variable to specify the features which are to be set in
 * bma456h sensor.
 * @param[in] enable : Variable which specifies whether to enable or disable the
 * features in the bma456h sensor.
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
 * features of bma456h sensor
 *
 *   - BMA456H_NO_MOTION_X_AXIS_EN
 *   - BMA456H_NO_MOTION_Y_AXIS_EN
 *   - BMA456H_NO_MOTION_Z_AXIS_EN
 *   - BMA456H_NO_MOTION_ALL_AXIS_EN
 *   - BMA456H_ANY_MOTION_X_AXIS_EN
 *   - BMA456H_ANY_MOTION_Y_AXIS_EN
 *   - BMA456H_ANY_MOTION_Z_AXIS_EN
 *   - BMA456H_ANY_MOTION_ALL_AXIS_EN
 *   - BMA456H_STEP_DETECTOR_EN
 *   - BMA456H_STEP_COUNTER_EN
 *   - BMA456H_STEP_ACTIVITY_EN
 *   - BMA456H_AUTO_LOW_POWER_EN
 *   - BMA456H_SINGLE_TAP_EN
 *   - BMA456H_DOUBLE_TAP_EN
 *   - BMA456H_TRIPLE_TAP_EN
 *   - BMA456H_TAP_AVERAGE_EN
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456h_feature_enable(uint16_t feature, uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiRemap Remap Axes
 * @brief Set / Get x, y and z axis re-mapping in the sensor
 */

/*!
 * \ingroup bma456hApiRemap
 * \page bma456h_api_bma456h_set_remap_axes bma456h_set_remap_axes
 * \code
 * int8_t bma456h_set_remap_axes(const struct bma4_remap *remap_data, struct bma4_dev *dev);
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
int8_t bma456h_set_remap_axes(const struct bma4_remap *remap_data, struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiRemap
 * \page bma456h_api_bma456h_get_remap_axes bma456h_get_remap_axes
 * \code
 * int8_t bma456h_get_remap_axes(struct bma4_remap *remap_data, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the x, y and z axis remap data from the sensor.
 *
 * @param[out] remap_data : Pointer to store axis remap data which is read
 * from the bma456h sensor.
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456h_get_remap_axes(struct bma4_remap *remap_data, struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiStepC Step counter
 * @brief Operations of step counter feature of the sensor
 */

/*!
 * \ingroup bma456hApiStepC
 * \page bma456h_api_bma456h_step_counter_set_watermark bma456h_step_counter_set_watermark
 * \code
 * int8_t bma456h_step_counter_set_watermark(uint16_t step_counter_wm, struct bma4_dev *dev);
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
int8_t bma456h_step_counter_set_watermark(uint16_t step_counter_wm, struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiStepC
 * \page bma456h_api_bma456h_step_counter_get_watermark bma456h_step_counter_get_watermark
 * \code
 * int8_t bma456h_step_counter_get_watermark(uint16_t *step_counter_wm, struct bma4_dev *dev);
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
int8_t bma456h_step_counter_get_watermark(uint16_t *step_counter_wm, struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiStepC
 * \page bma456h_api_bma456h_reset_step_counter bma456h_reset_step_counter
 * \code
 * int8_t bma456h_reset_step_counter(struct bma4_dev *dev);
 * \endcode
 * @details This API resets the counted steps of step counter.
 *
 * @param[in] dev : structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456h_reset_step_counter(struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiStepC
 * \page bma456h_api_bma456h_step_counter_output bma456h_step_counter_output
 * \code
 * int8_t bma456h_step_counter_output(uint32_t *step_count, struct bma4_dev *dev);
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
int8_t bma456h_step_counter_output(uint32_t *step_count, struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiAct Activity Feature
 * @brief Get output for activity feature of the sensor
 */

/*!
 * \ingroup bma456hApiAct
 * \page bma456h_api_bma456h_output_state bma456h_output_state
 * \code
 * int8_t bma456h_output_state(struct bma456h_out_state *out_state, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the output for activity feature.
 *
 * @param[out] activity : Pointer variable which stores activity output read
 * from the sensor.
 *
 *@verbatim
 *       activity |   State
 *  --------------|------------------------
 *        0x00    | BMA456H_USER_STATIONARY
 *        0x01    | BMA456H_USER_WALKING
 *        0x02    | BMA456H_USER_RUNNING
 *        0x03    | BMA456H_STATE_INVALID
 *@endverbatim
 *
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456h_output_state(struct bma456h_out_state *out_state, struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiStepC
 * \page bma456h_api_bma456h_stepcounter_get_parameter bma456h_stepcounter_get_parameter
 * \code
 * int8_t bma456h_stepcounter_get_parameter(struct bma456h_stepcounter_settings *setting, struct bma4_dev *dev);
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
int8_t bma456h_stepcounter_get_parameter(struct bma456h_stepcounter_settings *setting, struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiStepC
 * \page bma456h_api_bma456h_stepcounter_set_parameter bma456h_stepcounter_set_parameter
 * \code
 * int8_t bma456h_stepcounter_set_parameter(const struct bma456h_stepcounter_settings *setting, struct bma4_dev *dev);
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
int8_t bma456h_stepcounter_set_parameter(const struct bma456h_stepcounter_settings *setting, struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiStepD Step detector
 * @brief Operations of step detector feature of the sensor
 */

/*!
 * \ingroup bma456hApiStepD
 * \page bma456h_api_bma456h_step_detector_enable bma456h_step_detector_enable
 * \code
 * int8_t bma456h_step_detector_enable(uint8_t enable, struct bma4_dev *dev);
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
int8_t bma456h_step_detector_enable(uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiAnyMot Any motion Feature
 * @brief Functions of Any motion feature of the sensor
 */

/*!
 * \ingroup bma456hApiAnyMot
 * \page bma456h_api_bma456h_set_any_motion_config bma456h_set_any_motion_config
 * \code
 * int8_t bma456h_set_any_mot_config(const struct bma456h_any_no_mot_config *any_mot, struct bma4_dev *dev);
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
 *                                 |        Threshold for duration to
 *                                 |        detect any-motion event.
 *                                 |        Range 0 to 163 seconds.
 *         duration                |        Resolution is 20 ms.
 *                                 |        Default value is 100 ms
 *                                 |
 *                                 |
 *                                 |
 * --------------------------------|----------------------------------------
 *                                 |        Threshold for acceleration
 *                                 |        signal slope to detect any-
 *         threshold               |        motion event. Range is 0 to 1g.
 *                                 |        Resolution is 0.4883mg. Default
 *                                 |        value is 100 mg.
 * --------------------------------|-----------------------------------------
 *                                 |        Defines any motion interrupt
 *                                 |        behavior
 *                                 |        Value    Name      Description
 *                                 |
 *                                 |        0     multi_int    Generates
 *        interrupt behavior       |                           interrupt
 *                                 |                           as long as
 *                                 |                           condition
 *                                 |                           is valid
 *                                 |        1     single_shot  Generate one
 *                                 |                           interrupt for
 *                                 |                           every valid
 *                                 |                           condition
 * --------------------------------|-----------------------------------------
 *                                 |        Configuration for acceleration
 *                                 |        slope computation
 *                                 |
 *                                 |         Value    Name     Description
 *                                 |         0     non-        slope between
 *                                 |               consecutive acceleration
 *                                 |                           vector at last
 *          Slope                  |                           event detection
 *                                 |                           to current.
 *                                 |         1     consecutive Computes the
 *                                 |                           slope between
 *                                 |                           consecutive
 *                                 |                           acceleration
 *                                 |                           vector samples
 * ---------------------------------------------------------------------------
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456h_set_any_mot_config(const struct bma456h_any_no_mot_config *any_mot, struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiAnyMot
 * \page bma456h_api_bma456h_get_any_mot_config bma456h_get_any_mot_config
 * \code
 * int8_t bma456h_get_any_mot_config(struct bma456h_any_no_mot_config *any_mot, struct bma4_dev *dev);
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
 *                                 |        Threshold for duration to
 *                                 |        detect any-motion event.
 *                                 |        Range 0 to 163 seconds.
 *         duration                |        Resolution is 20 ms.
 *                                 |        Default value is 100 ms
 *                                 |
 *                                 |
 *                                 |
 * --------------------------------|----------------------------------------
 *                                 |        Threshold for acceleration
 *                                 |        signal slope to detect any-
 *         threshold               |        motion event. Range is 0 to 1g.
 *                                 |        Resolution is 0.4883mg. Default
 *                                 |        value is 100 mg.
 * --------------------------------|-----------------------------------------
 *                                 |        Defines any motion interrupt
 *                                 |        behavior
 *                                 |        Value    Name      Description
 *                                 |
 *                                 |        0     multi_int    Generates
 *        interrupt behavior       |                           interrupt
 *                                 |                           as long as
 *                                 |                           condition
 *                                 |                           is valid
 *                                 |        1     single_shot  Generate one
 *                                 |                           interrupt for
 *                                 |                           every valid
 *                                 |                           condition
 * --------------------------------|-----------------------------------------
 *                                 |        Configuration for acceleration
 *                                 |        slope computation
 *                                 |
 *                                 |         Value    Name     Description
 *                                 |         0     non-        slope between
 *                                 |               consecutive acceleration
 *                                 |                           vector at last
 *          Slope                  |                           event detection
 *                                 |                           to current.
 *                                 |         1     consecutive Computes the
 *                                 |                           slope between
 *                                 |                           consecutive
 *                                 |                           acceleration
 *                                 |                           vector samples
 * ---------------------------------------------------------------------------
 * @endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456h_get_any_mot_config(struct bma456h_any_no_mot_config *any_mot, struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiNoMot No-motion Feature
 * @brief Functions of no-motion feature of the sensor
 */

/*!
 * \ingroup bma456hApiNoMot
 * \page bma456h_api_bma456h_set_no_mot_config bma456h_set_no_mot_config
 * \code
 * int8_t bma456h_set_no_mot_config(const struct bma456h_any_no_mot_config *no_mot, struct bma4_dev *dev);
 * \endcode
 * @details This API sets the configuration of no-motion feature in the sensor
 * This API enables/disables the no-motion feature according to the axis set.
 *
 * @param[in] no_mot           : Pointer to structure variable to configure
 *                                no-motion.
 *
 * @verbatim
 * -------------------------------------------------------------------------
 *         Structure parameters    |        Description
 * --------------------------------|----------------------------------------
 *                                 |        Threshold for acceleration
 *                                 |        signal slope to detect no-
 *         threshold               |        motion event. Range is 0 to 1g.
 *                                 |        Resolution is 0.4883mg. Default
 *                                 |        value is 100 mg.
 * --------------------------------|-----------------------------------------
 *                                 |        Defines any motion interrupt
 *                                 |        behavior
 *                                 |        Value    Name      Description
 *                                 |
 *                                 |        0     multi_int    Generates
 *        interrupt behavior       |                           interrupt
 *                                 |                           as long as
 *                                 |                           condition
 *                                 |                           is valid
 *                                 |        1     single_shot  Generate one
 *                                 |                           interrupt for
 *                                 |                           every valid
 *                                 |                           condition
 * ---------------------------------------------------------------------------
 *                                 |        Threshold for duration to
 *                                 |        detect no-motion event.
 *                                 |        Range 0 to 163 seconds.
 *         duration                |        Resolution is 20 ms.
 *                                 |        Default value is 2000 ms
 *                                 |
 *                                 |
 *                                 |
 * -------------------------------------------------------------------------
 * @endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456h_set_no_mot_config(const struct bma456h_any_no_mot_config *no_mot, struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiNoMot
 * \page bma456h_api_bma456h_get_no_mot_config bma456h_get_no_mot_config
 * \code
 * int8_t bma456h_get_no_mot_config(struct bma456h_any_no_mot_config *no_mot, struct bma4_dev *dev);
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
 *                                 |        Threshold for acceleration
 *                                 |        signal slope to detect no-
 *         threshold               |        motion event. Range is 0 to 1g.
 *                                 |        Resolution is 0.4883mg. Default
 *                                 |        value is 100 mg.
 * --------------------------------|-----------------------------------------
 *                                 |        Defines any motion interrupt
 *                                 |        behavior
 *                                 |        Value    Name      Description
 *                                 |
 *                                 |        0     multi_int    Generates
 *        interrupt behavior       |                           interrupt
 *                                 |                           as long as
 *                                 |                           condition
 *                                 |                           is valid
 *                                 |        1     single_shot  Generate one
 *                                 |                           interrupt for
 *                                 |                           every valid
 *                                 |                           condition
 * ---------------------------------------------------------------------------
 *                                 |        Threshold for duration to
 *                                 |        detect no-motion event.
 *                                 |        Range 0 to 163 seconds.
 *         duration                |        Resolution is 20 ms.
 *                                 |        Default value is 2000 ms
 *                                 |
 *                                 |
 *                                 |
 * -------------------------------------------------------------------------
 * @endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456h_get_no_mot_config(struct bma456h_any_no_mot_config *no_mot, struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiTap Tap Feature
 * @brief Tap feature operations
 */

/*!
 * \ingroup bma456hApiTap
 * \page bma456h_api_bma456h_tap_get_parameter bma456h_tap_get_parameter
 * \code
 * int8_t bma456h_tap_get_parameter(struct bma456h_multitap_settings *setting, struct bma4_dev *dev);
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
int8_t bma456h_tap_get_parameter(struct bma456h_multitap_settings *setting, struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiTap
 * \page bma456h_api_bma456h_tap_set_parameter bma456h_tap_set_parameter
 * \code
 * int8_t bma456h_tap_set_parameter(const struct bma456h_multitap_settings *setting, struct bma4_dev *dev);
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
int8_t bma456h_tap_set_parameter(const struct bma456h_multitap_settings *setting, struct bma4_dev *dev);

/**
 * \ingroup bma456h
 * \defgroup bma456hApiAutoLP Auto Low power mode
 * @brief Configurations and Status of auto low power mode of the sensor
 */

/*!
 * \ingroup bma456hApiAutoLP
 * \page bma456h_api_bma456h_get_auto_low_power_config bma456h_get_auto_low_power_config
 * \code
 * int8_t bma456h_get_auto_low_power_config(struct bma456h_auto_low_power *auto_low_power, struct bma4_dev *dev);
 * \endcode
 * @details This api gets the auto low power configuration
 *
 * @param auto_low_power[out]  : this pointer gets the auto low power configuration.
 * @param dev[in]   : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456h_get_auto_low_power_config(struct bma456h_auto_low_power *auto_low_power, struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiAutoLP
 * \page bma456h_api_bma456h_set_auto_low_power_config bma456h_set_auto_low_power_config
 * \code
 * int8_t bma456h_set_auto_low_power_config(const struct bma456h_auto_low_power *auto_low_power, struct bma4_dev *dev);
 * \endcode
 * @details This api sets the auto low power configuration
 *
 * @param auto_low_power[in]  : this pointer sets the auto low power configuration.
 * @param dev[in]   : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456h_set_auto_low_power_config(const struct bma456h_auto_low_power *auto_low_power, struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiAutoLP
 * \page bma456h_api_bma456h_get_auto_low_power_state bma456h_get_auto_low_power_state
 * \code
 * int8_t bma456h_get_auto_low_power_state(uint8_t *auto_low_power_state, struct bma4_dev *dev);
 * \endcode
 * @details This api reads the auto low power state output status
 *
 * @param auto_low_power_state[out]  : pointer reads auto low power state output bit status.
 * @param dev[in]   : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456h_get_auto_low_power_state(uint8_t *auto_low_power_state, struct bma4_dev *dev);

/*!
 * \ingroup bma456hApiVersionConfig
 * \page bma456h_api_bma456h_get_version_config bma456h_get_version_config
 * \code
 *int8_t bma456h_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev);
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
int8_t bma456h_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev);

#ifdef __cplusplus
}
#endif /*End of CPP guard */

#endif /*End of header guard macro */

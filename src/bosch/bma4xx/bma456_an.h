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
* @file       bma456_an.h
* @date       2023-07-05
* @version    V2.29.0
*
*/

/**
 * \ingroup bma4xy
 * \defgroup bma456_an BMA456_AN
 * @brief Sensor driver for BMA456_AN sensor
 */

#ifndef BMA456_AN_H
#define BMA456_AN_H

#ifdef __cplusplus
extern "C" {
#endif
#include "bma4.h"

/**\name Chip ID of BMA456_AN sensor */
#define BMA456_AN_CHIP_ID_PRIM            UINT8_C(0x16)
#define BMA456_AN_CHIP_ID_SEC             UINT8_C(0x1A)

/**\name Sensor feature size */
#define BMA456_AN_FEATURE_SIZE            UINT8_C(12)
#define BMA456_AN_ANY_MOT_LEN             UINT8_C(4)

/**\name Feature offset address */
#define BMA456_AN_ANY_MOT_OFFSET          UINT8_C(0x00)
#define BMA456_AN_NO_MOT_OFFSET           UINT8_C(0x04)
#define BMA456_AN_CONFIG_ID_OFFSET        UINT8_C(0x08)
#define BMA456_AN_AXES_REMAP_OFFSET       UINT8_C(0x0A)

/**\name Read/Write Lengths */
#define BMA456_AN_RD_WR_MIN_LEN           UINT8_C(2)

/*! @name Maximum valid read write length is size of config file array */
#define BMA456_AN_RD_WR_MAX_LEN           ((uint16_t)sizeof(bma456_an_config_file))

#define BMA456_AN_NO_MOT_RD_WR_LEN        (BMA456_AN_ANY_MOT_LEN + BMA456_AN_NO_MOT_OFFSET)

/**************************************************************/
/**\name    Re-map Axes */
/**************************************************************/
#define BMA456_AN_X_AXIS_MASK             UINT8_C(0x03)
#define BMA456_AN_X_AXIS_SIGN_MASK        UINT8_C(0x04)
#define BMA456_AN_Y_AXIS_MASK             UINT8_C(0x18)
#define BMA456_AN_Y_AXIS_SIGN_MASK        UINT8_C(0x20)
#define BMA456_AN_Z_AXIS_MASK             UINT8_C(0xC0)
#define BMA456_AN_Z_AXIS_SIGN_MASK        UINT8_C(0x01)

/**************************************************************/
/**\name    Any/no Motion */
/**************************************************************/
/**\name Any/No motion threshold macros */
#define BMA456_AN_ANY_NO_MOT_THRES_MSK    UINT16_C(0x07FF)

/**\name Any/No motion duration macros */
#define BMA456_AN_ANY_NO_MOT_DUR_MSK      UINT16_C(0x1FFF)

/**\name Any/No motion enable macros */
#define BMA456_AN_ANY_NO_MOT_AXIS_EN_POS  UINT8_C(0x0D)
#define BMA456_AN_ANY_NO_MOT_AXIS_EN_MSK  UINT16_C(0xE000)

/**************************************************************/
/**\name    User macros */
/**************************************************************/
/**\name Any-motion/No-motion axis enable macros */
#define BMA456_AN_X_AXIS_EN               UINT8_C(0x01)
#define BMA456_AN_Y_AXIS_EN               UINT8_C(0x02)
#define BMA456_AN_Z_AXIS_EN               UINT8_C(0x04)
#define BMA456_AN_EN_ALL_AXIS             UINT8_C(0x07)
#define BMA456_AN_DIS_ALL_AXIS            UINT8_C(0x00)

/**\name Interrupt status macros */
#define BMA456_AN_ANY_MOT_INT             UINT8_C(0x20)
#define BMA456_AN_NO_MOT_INT              UINT8_C(0x40)
#define BMA456_AN_ERROR_INT               UINT8_C(0x80)

/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/

/*!
 * @brief Any/No motion configuration
 */
struct bma456_an_any_no_mot_config
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

/***************************************************************************/

/*!     BMA456_AN User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bma456_an
 * \defgroup bma456_anApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bma456_anApiInit
 * \page bma456_an_api_bma456_an_init bma456_an_init
 * \code
 * int8_t bma456_an_init(struct bma4_dev *dev);
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
int8_t bma456_an_init(struct bma4_dev *dev);

/**
 * \ingroup bma456_an
 * \defgroup bma456_anApiConfig ConfigFile
 * @brief Write binary configuration in the sensor
 */

/*!
 * \ingroup bma456_anApiConfig
 * \page bma456_an_api_bma456_an_write_config_file bma456_an_write_config_file
 * \code
 * int8_t bma456_an_write_config_file(struct bma4_dev *dev);
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
int8_t bma456_an_write_config_file(struct bma4_dev *dev);

/*!
 * \ingroup bma456_anApiConfig
 * \page bma456_an_api_bma456_an_get_version_config bma456_an_get_version_config
 * \code
 *int8_t bma456_an_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev);
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
int8_t bma456_an_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev);

/**
 * \ingroup bma456_an
 * \defgroup bma456_anApiMapInt Map / Unmap Interrupt
 * @brief Map / Unmap user provided interrupt to interrupt pin1 or pin2 of the sensor
 */

/*!
 * \ingroup bma456_anApiMapInt
 * \page bma456_an_api_bma456_an_map_interrupt bma456_an_map_interrupt
 * \code
 * int8_t bma456_an_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);
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
 *  - BMA456_AN_ANY_MOT_INT
 *  - BMA456_AN_NO_MOT_INT
 *  - BMA456_AN_ERROR_INT
 *
 * Hardware Interrupts
 *  - BMA4_FIFO_FULL_INT
 *  - BMA4_FIFO_WM_INT
 *  - BMA4_ACCEL_DATA_RDY_INT
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_an_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma456_an
 * \defgroup bma456_anApiIntS Interrupt Status
 * @brief Read interrupt status of the sensor
 */

/*!
 * \ingroup bma456_anApiIntS
 * \page bma456_an_api_bma456_an_read_int_status bma456_an_read_int_status
 * \code
 * int8_t bma456_an_read_int_status(uint16_t *int_status, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the bma456_an interrupt status from the sensor.
 *
 * @param[out] int_status : Variable to store the interrupt status read from
 * the sensor.
 * @param[in] dev : Structure instance of bma4_dev.
 *
 * @note Below macros are used to check the interrupt status.
 *
 * Feature Interrupts
 *  - BMA456_AN_ANY_MOT_INT
 *  - BMA456_AN_NO_MOT_INT
 *  - BMA456_AN_ERROR_INT
 *
 * Hardware Interrupts
 *  - BMA4_FIFO_FULL_INT
 *  - BMA4_FIFO_WM_INT
 *  - BMA4_ACCEL_DATA_RDY_INT
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_an_read_int_status(uint16_t *int_status, struct bma4_dev *dev);

/**
 * \ingroup bma456_an
 * \defgroup bma456_anApiRemap Remap Axes
 * @brief Set / Get x, y and z axis re-mapping in the sensor
 */

/*!
 * \ingroup bma456_anApiRemap
 * \page bma456_an_api_bma456_an_set_remap_axes bma456_an_set_remap_axes
 * \code
 * int8_t bma456_an_set_remap_axes(const struct bma4_remap *remap_data, struct bma4_dev *dev);
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
int8_t bma456_an_set_remap_axes(const struct bma4_remap *remap_data, struct bma4_dev *dev);

/*!
 * \ingroup bma456_anApiRemap
 * \page bma456_an_api_bma456_an_get_remap_axes bma456_an_get_remap_axes
 * \code
 * int8_t bma456_an_get_remap_axes(struct bma4_remap *remap_data, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the x, y and z axis remap data from the sensor.
 *
 * @param[out] remap_data : Pointer to store axis remap data which is read
 * from the bma456_an sensor.
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_an_get_remap_axes(struct bma4_remap *remap_data, struct bma4_dev *dev);

/**
 * \ingroup bma456_an
 * \defgroup bma456_anApiAnyMot Any motion Feature
 * @brief Functions of Any motion feature of the sensor
 */

/*!
 * \ingroup bma456_anApiAnyMot
 * \page bma456_an_api_bma456_an_set_any_mot_config bma456_an_set_any_mot_config
 * \code
 * int8_t bma456_an_set_any_mot_config(const struct bma456_an_any_no_mot_config *any_motion, struct bma4_dev *dev);
 * \endcode
 * @details This API sets the configuration of any-motion feature in the sensor
 * This API enables/disables the any-motion feature according to the axis set.
 *
 * @param[in] any_motion           : Pointer to structure variable to configure
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
 *  0x00     |  BMA456_AN_DIS_ALL_AXIS
 *  0x01     |  BMA456_AN_X_AXIS_EN
 *  0x02     |  BMA456_AN_Y_AXIS_EN
 *  0x04     |  BMA456_AN_Z_AXIS_EN
 *  0x07     |  BMA456_AN_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_an_set_any_mot_config(const struct bma456_an_any_no_mot_config *any_motion, struct bma4_dev *dev);

/*!
 * \ingroup bma456_anApiNomot
 * \page bma456_an_api_bma456_an_get_any_motion_config bma456_an_get_any_motion_config
 * \code
 * int8_t bma456_an_get_any_motion_config(struct bma456_an_anymotion_config *any_motion, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the configuration of any-motion feature from the
 * sensor.
 *
 * @param[out] any_motion        : Pointer to structure variable to configure
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
 *  0x00     |  BMA456_AN_DIS_ALL_AXIS
 *  0x01     |  BMA456_AN_X_AXIS_EN
 *  0x02     |  BMA456_AN_Y_AXIS_EN
 *  0x04     |  BMA456_AN_Z_AXIS_EN
 *  0x07     |  BMA456_AN_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_an_get_any_mot_config(struct bma456_an_any_no_mot_config *any_motion, struct bma4_dev *dev);

/**
 * \ingroup bma456_an
 * \defgroup bma456_anApiNomot No-Motion Feature
 * @brief Operations of no-motion feature of the sensor
 */

/*!
 * \ingroup bma456_anApiNomot
 * \page bma456_an_api_bma456_an_set_no_motion_config bma456_an_set_no_motion_config
 * \code
 * int8_t bma456_an_set_no_motion_config(const struct bma456_an_nomotion_config *no_motion, struct bma4_dev *dev);
 * \endcode
 * @details This API sets the configuration of no-motion feature in the sensor
 * This API enables/disables the no-motion feature according to the axis set.
 *
 * @param[in] no_motion             : Pointer to structure variable to configure
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
 *  0x00     |  BMA456_AN_DIS_ALL_AXIS
 *  0x01     |  BMA456_AN_X_AXIS_EN
 *  0x02     |  BMA456_AN_Y_AXIS_EN
 *  0x04     |  BMA456_AN_Z_AXIS_EN
 *  0x07     |  BMA456_AN_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_an_set_no_mot_config(const struct bma456_an_any_no_mot_config *no_motion, struct bma4_dev *dev);

/*!
 * \ingroup bma456_anApiAnyMot
 * \page bma456_an_api_bma456_an_get_no_motion_config bma456_an_get_no_motion_config
 * \code
 * int8_t bma456_an_get_no_motion_config(struct bma456_an_nomotion_config *no_motion, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the configuration of no-motion feature from the
 * sensor.
 *
 * @param[out] no_motion      : Pointer to structure variable to configure
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
 *  0x00     |  BMA456_AN_DIS_ALL_AXIS
 *  0x01     |  BMA456_AN_X_AXIS_EN
 *  0x02     |  BMA456_AN_Y_AXIS_EN
 *  0x04     |  BMA456_AN_Z_AXIS_EN
 *  0x07     |  BMA456_AN_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
int8_t bma456_an_get_no_mot_config(struct bma456_an_any_no_mot_config *no_motion, struct bma4_dev *dev);

#ifdef __cplusplus
}
#endif /*End of CPP guard */

#endif /*End of header guard macro */

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
* @file       bma422_an.h
* @date       2023-07-05
* @version    V2.29.0
*
*/

/**
 * \ingroup bma4xy
 * \defgroup bma422_an BMA422_AN
 * @brief Sensor driver for BMA422_AN sensor
 */

#ifndef BMA422_AN_H_
#define BMA422_AN_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************/

/*!             Header files
 ****************************************************************************/
#include "bma4.h"

/***************************************************************************/

/*!               Macro definitions
 ****************************************************************************/

/*! @name Chip ID of BMA422_AN sensor */
#define BMA422_AN_CHIP_ID                 UINT8_C(0x12)

/*! @name Sensor feature size */
#define BMA422_AN_FEATURE_SIZE            UINT8_C(0x0C)

/*! @name Data Lengths */
#define BMA422_AN_RD_WR_MIN_LEN           UINT8_C(0x02)

/*! @name Maximum valid read write length is size of config file array */
#define BMA422_AN_RD_WR_MAX_LEN           ((uint16_t)sizeof(bma422_an_config_file))

/*! @name Feature start address */
#define BMA422_AN_ANY_MOT_START_ADDR      UINT8_C(0x00)
#define BMA422_AN_NO_MOT_START_ADDR       UINT8_C(0x04)
#define BMA422_AN_CONFIG_ID_START_ADDR    UINT8_C(0x08)
#define BMA422_AN_AXES_REMAP_START_ADDR   UINT8_C(0x0A)

/*! @name Mask definitions for axes re-mapping */
#define BMA422_AN_X_AXIS_MSK              UINT8_C(0x03)
#define BMA422_AN_X_AXIS_SIGN_MSK         UINT8_C(0x04)
#define BMA422_AN_Y_AXIS_MSK              UINT8_C(0x18)
#define BMA422_AN_Y_AXIS_SIGN_MSK         UINT8_C(0x20)
#define BMA422_AN_Z_AXIS_MSK              UINT8_C(0xC0)
#define BMA422_AN_Z_AXIS_SIGN_MSK         UINT8_C(0x01)

/*! @name Bit position for axes re-mapping */
#define BMA422_AN_X_AXIS_SIGN_POS         UINT8_C(0x02)
#define BMA422_AN_Y_AXIS_POS              UINT8_C(0x03)
#define BMA422_AN_Y_AXIS_SIGN_POS         UINT8_C(0x05)
#define BMA422_AN_Z_AXIS_POS              UINT8_C(0x06)

/*! @name Any/No-motion threshold macros */
#define BMA422_AN_ANY_NO_MOT_THRES_MSK    UINT16_C(0x07FF)

/*! @name Any/No-motion enable macros */
#define BMA422_AN_ANY_NO_MOT_AXIS_EN_MSK  UINT16_C(0xE000)
#define BMA422_AN_ANY_NO_MOT_AXIS_EN_POS  UINT8_C(0x0D)

/*! @name Any/No-motion duration macros */
#define BMA422_AN_ANY_NO_MOT_DUR_MSK      UINT16_C(0x1FFF)

/*! @name Any/No-motion axis enable macros */
#define BMA422_AN_X_AXIS_EN               UINT8_C(0x01)
#define BMA422_AN_Y_AXIS_EN               UINT8_C(0x02)
#define BMA422_AN_Z_AXIS_EN               UINT8_C(0x04)
#define BMA422_AN_EN_ALL_AXIS             UINT8_C(0x07)
#define BMA422_AN_DIS_ALL_AXIS            UINT8_C(0x00)

/*! @name Interrupt status macros */
#define BMA422_AN_ANY_MOT_INT             UINT8_C(0x20)
#define BMA422_AN_NO_MOT_INT              UINT8_C(0x40)
#define BMA422_AN_ERROR_INT               UINT8_C(0x80)

/******************************************************************************/
/*!  @name         Structure Declarations                                     */
/******************************************************************************/
/*! @name Structure to define any/no-motion configuration */
struct bma422_an_any_no_mot_config
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

/*!                 BMA422_AN User Interface function prototypes
 ****************************************************************************/

/**
 * \ingroup bma422_an
 * \defgroup bma422_anApiInit Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bma422_anApiInit
 * \page bma422_an_api_bma422_an_init bma422_an_init
 * \code
 * int8_t bma422_an_init(struct bma4_dev *dev);
 * \endcode
 * @details This API is the entry point. Call this API before using all other APIs
 * This API reads the chip-id of the sensor and sets the resolution, feature
 * length and the type of variant.
 *
 * @param[in,out] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_init(struct bma4_dev *dev);

/**
 * \ingroup bma422_an
 * \defgroup bma422_anApiConfig ConfigFile
 * @brief Write binary configuration in the sensor
 */

/*!
 * \ingroup bma422_anApiConfig
 * \page bma422_an_api_bma422_an_write_config_file bma422_an_write_config_file
 * \code
 * int8_t bma422_an_write_config_file(struct bma4_dev *dev);
 * \endcode
 * @details This API is used to upload the configuration file to enable the
 * features of the sensor.
 *
 * @param[in, out] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_write_config_file(struct bma4_dev *dev);

/**
 * \ingroup bma422_an
 * \defgroup bma422_anApiConfigId ConfigId
 * @brief Get Configuration ID of the sensor
 */

/*!
 * \ingroup bma422_anApiConfig
 * \page bma422_an_api_bma422_an_get_config_id bma422_an_get_config_id
 * \code
 * int8_t bma422_an_get_config_id(uint16_t *config_id, struct bma4_dev *dev);
 * \endcode
 * @details This API is used to get the configuration id of the sensor.
 *
 * @param[out] config_id    : Pointer variable used to store the configuration
 *                            id.
 * @param[in] dev           : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_get_config_id(uint16_t *config_id, struct bma4_dev *dev);

/**
 * \ingroup bma422_an
 * \defgroup bma422_anApiMapInt Map / Unmap Interrupt
 * @brief Map / Unmap user provided interrupt to interrupt pin1 or pin2 of the sensor
 */

/*!
 * \ingroup bma422_anApiMapInt
 * \page bma422_an_api_bma422_an_map_interrupt bma422_an_map_interrupt
 * \code
 * int8_t bma422_an_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);
 * \endcode
 * @details This API maps/un-maps the user provided interrupt to either interrupt
 * pin1 or pin2 in the sensor.
 *
 * @param[in] int_line  : Variable to select either interrupt pin1 or pin2.
 *
 *@verbatim
 *  int_line    |   Macros
 *  ------------|-------------------
 *      0       | BMA4_INTR1_MAP
 *      1       | BMA4_INTR2_MAP
 *@endverbatim
 *
 * @param[in] int_map   : Variable to specify the interrupts.
 * @param[in] enable    : Variable to specify mapping or un-mapping of
 *                        interrupts.
 *
 *@verbatim
 *  enable  |   Macros
 *----------|-------------------
 *   0x01   |  BMA4_EN
 *   0x00   |  BMA4_DIS
 *@endverbatim
 *
 * @param[in] dev      : Structure instance of bma4_dev.
 *
 * @note Below macros specify the feature interrupts.
 *          - BMA422_AN_ANY_MOT_INT
 *          - BMA422_AN_NO_MOT_INT
 *          - BMA422_AN_ERROR_INT
 *
 * @note Below macros specify the hardware interrupts.
 *          - BMA4_FIFO_FULL_INT
 *          - BMA4_FIFO_WM_INT
 *          - BMA4_DATA_RDY_INT
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev);

/**
 * \ingroup bma422_an
 * \defgroup bma422_anApiIntS Interrupt Status
 * @brief Read interrupt status of the sensor
 */

/*!
 * \ingroup bma422_anApiIntS
 * \page bma422_an_api_bma422_an_read_int_status bma422_an_read_int_status
 * \code
 * int8_t bma422_an_read_int_status(uint16_t *int_status, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the bma422_an interrupt status from the sensor.
 *
 * @param[out] int_status   : Variable to store the interrupt status
 *                            read from the sensor.
 * @param[in] dev           : Structure instance of bma4_dev.
 *
 * @note Below macros specify the feature interrupts.
 *          - BMA422_AN_ANY_MOT_INT
 *          - BMA422_AN_NO_MOT_INT
 *          - BMA422_AN_ERROR_INT
 *
 * @note Below macros specify the hardware interrupts.
 *          - BMA4_FIFO_FULL_INT
 *          - BMA4_FIFO_WM_INT
 *          - BMA4_DATA_RDY_INT
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_read_int_status(uint16_t *int_status, struct bma4_dev *dev);

/**
 * \ingroup bma422_an
 * \defgroup bma422_anApiRemap Remap Axes
 * @brief Set / Get x, y and z axis re-mapping in the sensor
 */

/*!
 * \ingroup bma422_anApiRemap
 * \page bma422_an_api_bma422_an_set_remap_axes bma422_an_set_remap_axes
 * \code
 * int8_t bma422_an_set_remap_axes(const struct bma4_remap *remap_data, struct bma4_dev *dev);
 * \endcode
 * @details This API performs x, y and z axis re-mapping in the sensor.
 *
 * @param[in] remap_data    : Pointer to store axes re-mapping data.
 * @param[in] dev           : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_set_remap_axes(const struct bma4_remap *remap_data, struct bma4_dev *dev);

/*!
 * \ingroup bma422_anApiRemap
 * \page bma422_an_api_bma422_an_get_remap_axes bma422_an_get_remap_axes
 * \code
 * int8_t bma422_an_get_remap_axes(struct bma4_remap *remap_data, struct bma4_dev *dev);
 * \endcode
 * @details This API reads the x, y and z axis re-mapped data from the sensor.
 *
 * @param[out] remap_data   : Pointer to store the read axes re-mapped data.
 * @param[in] dev           : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_get_remap_axes(struct bma4_remap *remap_data, struct bma4_dev *dev);

/**
 * \ingroup bma422_an
 * \defgroup bma422_anApiAnyMot Any motion Feature
 * @brief Functions of Any motion feature of the sensor
 */

/*!
 * \ingroup bma422_anApiAnyMot
 * \page bma422_an_api_bma422_an_set_any_motion_config bma422_an_set_any_motion_config
 * \code
 * int8_t bma422_an_set_any_motion_config(const struct bma422_an_anymotion_config *any_motion, struct bma4_dev
 **dev);
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
 *  0x00     |  BMA422_AN_DIS_ALL_AXIS
 *  0x01     |  BMA422_AN_X_AXIS_EN
 *  0x02     |  BMA422_AN_Y_AXIS_EN
 *  0x04     |  BMA422_AN_Z_AXIS_EN
 *  0x07     |  BMA422_AN_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_set_any_mot_config(const struct bma422_an_any_no_mot_config *any_mot, struct bma4_dev *dev);

/*!
 * \ingroup bma422_anApiAnyMot
 * \page bma422_an_api_bma422_an_get_any_motion_config bma422_an_get_any_motion_config
 * \code
 * int8_t bma422_an_get_any_motion_config(struct bma422_an_anymotion_config *any_motion, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the configuration of any-motion feature from the
 * sensor.
 *
 * @param[out] any_mot        : Pointer to structure variable to configure
 *                              any-motion.
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
 *  0x00     |  BMA422_AN_DIS_ALL_AXIS
 *  0x01     |  BMA422_AN_X_AXIS_EN
 *  0x02     |  BMA422_AN_Y_AXIS_EN
 *  0x04     |  BMA422_AN_Z_AXIS_EN
 *  0x07     |  BMA422_AN_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_get_any_mot_config(struct bma422_an_any_no_mot_config *any_mot, struct bma4_dev *dev);

/**
 * \ingroup bma422_an
 * \defgroup bma422_anApiNomot No-Motion Feature
 * @brief Operations of no-motion feature of the sensor
 */

/*!
 * \ingroup bma422_anApiNomot
 * \page bma422_an_api_bma422_an_set_no_motion_config bma422_an_set_no_motion_config
 * \code
 * int8_t bma422_an_set_no_motion_config(const struct bma422_an_nomotion_config *no_motion, struct bma4_dev
 **dev);
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
 *  0x00     |  BMA422_AN_DIS_ALL_AXIS
 *  0x01     |  BMA422_AN_X_AXIS_EN
 *  0x02     |  BMA422_AN_Y_AXIS_EN
 *  0x04     |  BMA422_AN_Z_AXIS_EN
 *  0x07     |  BMA422_AN_EN_ALL_AXIS
 *@endverbatim
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_set_no_mot_config(const struct bma422_an_any_no_mot_config *no_mot, struct bma4_dev *dev);

/*!
 * \ingroup bma422_anApiNomot
 * \page bma422_an_api_bma422_an_get_no_motion_config bma422_an_get_no_motion_config
 * \code
 * int8_t bma422_an_get_no_motion_config(struct bma422_an_nomotion_config *no_motion, struct bma4_dev *dev);
 * \endcode
 * @details This API gets the configuration of no-motion feature from the
 * sensor.
 *
 * @param[out] no_mot        : Pointer to structure variable to configure
 *                              no-motion.
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
 *  Value    |  axis_en
 *  ---------|-------------------------
 *  0x00     |  BMA422_AN_DIS_ALL_AXIS
 *  0x01     |  BMA422_AN_X_AXIS_EN
 *  0x02     |  BMA422_AN_Y_AXIS_EN
 *  0x04     |  BMA422_AN_Z_AXIS_EN
 *  0x07     |  BMA422_AN_EN_ALL_AXIS
 *
 * @param[in] dev               : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_get_no_mot_config(struct bma422_an_any_no_mot_config *no_mot, struct bma4_dev *dev);

/**
 * \ingroup bma422_an
 * \defgroup bma422_anApiVersionConfig Version Config
 * @brief Get version configuration of the sensor
 */

/*!
 * \ingroup bma422_anApiVersionConfig
 * \page bma422_an_api_bma422_an_get_version_config bma422_an_get_version_config
 * \code
 * int8_t bma422_an_get_version_config(uint16_t *config_major,
 *                                                          uint16_t *config_minor,
 *                                                          struct bma4_dev *dev);
 * \endcode
 * @details This API is used to get the config file major and minor information.
 * @param[in] dev   : Structure instance of bma4_dev.
 * @param[out] config_major    : pointer to data buffer to store the config major.
 * @param[out] config_minor    : pointer to data buffer to store the config minor.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
int8_t bma422_an_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev);

#ifdef __cplusplus
}
#endif /*End of CPP guard */

#endif /*End of header guard macro */

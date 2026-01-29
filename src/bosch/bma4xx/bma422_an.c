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
* @file       bma422_an.c
* @date       2023-07-05
* @version    V2.29.0
*
*/

/***************************************************************************/

/*!             Header files
 ****************************************************************************/
#include "bma422_an.h"

/***************************************************************************/

/*!              Global Variable
 ****************************************************************************/

/*! @name  Global array that stores the configuration file of BMA422_AN */
const uint8_t bma422_an_config_file[] = {
    0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00,
    0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0xc8, 0x2e, 0x00, 0x2e, 0x80, 0x2e, 0x53, 0x01, 0x80, 0x2e, 0xf8, 0x01, 0xb0, 0xf0,
    0x10, 0x30, 0x21, 0x2e, 0x16, 0xf0, 0x80, 0x2e, 0xf9, 0x00, 0x1f, 0x50, 0x1d, 0x52, 0x01, 0x42, 0x3b, 0x80, 0x41,
    0x30, 0x01, 0x42, 0x3c, 0x80, 0x00, 0x2e, 0x01, 0x40, 0x01, 0x42, 0x21, 0x2e, 0xff, 0xaf, 0xb8, 0x2e, 0x97, 0x20,
    0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18,
    0x00, 0x80, 0x2e, 0x18, 0x00, 0x80, 0x2e, 0x18, 0x00, 0xfd, 0x2d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x2e, 0x55, 0xf0, 0xc0, 0x2e, 0x21, 0x2e, 0x55, 0xf0, 0x80, 0x2e, 0x18, 0x00, 0xaa, 0x00, 0x05, 0x00, 0xaa,
    0x00, 0x05, 0x00, 0x41, 0x44, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x52, 0x0b, 0x50, 0x60, 0x42, 0x00, 0x3f, 0x0d, 0x54, 0x42, 0x42, 0x69, 0x82,
    0x0f, 0x54, 0x42, 0x42, 0x42, 0x82, 0x4b, 0x30, 0x42, 0x40, 0x10, 0x08, 0x50, 0x42, 0x7e, 0x80, 0x4b, 0x42, 0x61,
    0x30, 0x01, 0x42, 0x10, 0x50, 0x03, 0x2e, 0x40, 0xf0, 0x52, 0x90, 0x08, 0x80, 0xf0, 0x7f, 0x31, 0x30, 0x1f, 0x2f,
    0x03, 0x30, 0x17, 0x52, 0x11, 0x54, 0x54, 0x33, 0x06, 0x30, 0x13, 0x50, 0x55, 0x32, 0x1d, 0x1a, 0xe3, 0x22, 0x18,
    0x1a, 0x15, 0x58, 0xe3, 0x22, 0x04, 0x30, 0xd5, 0x40, 0xb5, 0x0d, 0xe1, 0xbe, 0x6f, 0xbb, 0x80, 0x91, 0xa9, 0x0d,
    0x01, 0x89, 0xb5, 0x23, 0x10, 0xa1, 0xf7, 0x2f, 0xda, 0x0e, 0x54, 0x33, 0xeb, 0x2f, 0x01, 0x2e, 0x25, 0x00, 0x70,
    0x1a, 0x00, 0x30, 0x21, 0x30, 0x48, 0x22, 0x40, 0xb2, 0x06, 0x2f, 0x23, 0x2e, 0x59, 0xf0, 0x98, 0x2e, 0x39, 0x00,
    0x00, 0x2e, 0x00, 0x2e, 0xd0, 0x2e, 0xf0, 0x6f, 0x01, 0x31, 0x20, 0x26, 0x01, 0x42, 0x10, 0x30, 0x21, 0x2e, 0x59,
    0xf0, 0x98, 0x2e, 0x39, 0x00, 0x01, 0x2e, 0x92, 0xf0, 0x0d, 0xbc, 0x0d, 0xb8, 0x03, 0xb2, 0x00, 0x30, 0x02, 0x2f,
    0x21, 0x2e, 0x58, 0xf0, 0x03, 0x2d, 0x10, 0x30, 0x21, 0x2e, 0x58, 0xf0, 0x98, 0x2e, 0x39, 0x00, 0x01, 0x50, 0x03,
    0x52, 0x98, 0x2e, 0x7a, 0x01, 0x01, 0x50, 0x05, 0x84, 0x13, 0x30, 0x05, 0x50, 0x64, 0x30, 0x07, 0x52, 0x93, 0x42,
    0x84, 0x42, 0x98, 0x2e, 0x7a, 0x01, 0x05, 0x50, 0x06, 0x80, 0x71, 0x30, 0x01, 0x42, 0x00, 0x2e, 0x00, 0x2e, 0xd0,
    0x2e, 0x98, 0x2e, 0xdf, 0x00, 0x01, 0x2e, 0x35, 0x00, 0x00, 0xb2, 0x0d, 0x2f, 0x00, 0x30, 0x21, 0x2e, 0x35, 0x00,
    0x09, 0x50, 0x98, 0x2e, 0x13, 0x01, 0x09, 0x50, 0x01, 0x52, 0x98, 0x2e, 0x8b, 0x01, 0x09, 0x50, 0x05, 0x52, 0x98,
    0x2e, 0x8b, 0x01, 0x98, 0x2e, 0x39, 0x00, 0xe6, 0x2d, 0x07, 0x2e, 0x00, 0xf0, 0x03, 0x2e, 0x00, 0xf0, 0x21, 0x50,
    0x9c, 0xbe, 0x02, 0x40, 0x1d, 0x52, 0xf4, 0x33, 0xdc, 0xba, 0xd9, 0x0f, 0x94, 0x08, 0x06, 0x2f, 0x49, 0xaf, 0x04,
    0x2f, 0x51, 0x0a, 0x02, 0x34, 0x47, 0xa3, 0x8a, 0x0a, 0x91, 0x22, 0x3c, 0x80, 0x25, 0x2e, 0x59, 0xf0, 0x01, 0x40,
    0x01, 0x42, 0xb8, 0x2e, 0x1a, 0x24, 0x26, 0x00, 0x80, 0x2e, 0x58, 0x00, 0x80, 0x2e, 0x18, 0x00, 0xfd, 0x2d, 0x46,
    0x00, 0x40, 0x00, 0x4f, 0x00, 0x42, 0x00, 0x36, 0x00, 0xaf, 0x00, 0xff, 0x00, 0xec, 0x00, 0xff, 0xb7, 0x00, 0x02,
    0x00, 0xb0, 0x05, 0x80, 0xc9, 0xf0, 0x88, 0x00, 0x80, 0x00, 0x5e, 0xf0, 0x59, 0xf0, 0x89, 0xf0, 0x39, 0x00, 0x60,
    0x50, 0x03, 0x2e, 0x45, 0x00, 0xe0, 0x7f, 0xf1, 0x7f, 0xdb, 0x7f, 0x30, 0x30, 0x1b, 0x54, 0x0a, 0x1a, 0x28, 0x2f,
    0x1a, 0x25, 0x7a, 0x82, 0x00, 0x30, 0x43, 0x30, 0x32, 0x30, 0x05, 0x30, 0x04, 0x30, 0xf6, 0x6f, 0xf2, 0x09, 0xfc,
    0x13, 0xc2, 0xab, 0xb3, 0x09, 0xef, 0x23, 0x80, 0xb3, 0xe6, 0x6f, 0xb7, 0x01, 0x00, 0x2e, 0x8b, 0x41, 0x4b, 0x42,
    0x03, 0x2f, 0x46, 0x40, 0x86, 0x17, 0x81, 0x8d, 0x46, 0x42, 0x41, 0x8b, 0x23, 0xbd, 0xb3, 0xbd, 0x03, 0x89, 0x41,
    0x82, 0x07, 0x0c, 0x43, 0xa3, 0xe6, 0x2f, 0xe1, 0x6f, 0xa2, 0x6f, 0x52, 0x42, 0x00, 0x2e, 0xb2, 0x6f, 0x52, 0x42,
    0x00, 0x2e, 0xc2, 0x6f, 0x42, 0x42, 0x03, 0xb2, 0x03, 0x2e, 0x59, 0xf0, 0xf3, 0x3d, 0x02, 0x32, 0x0b, 0x08, 0x8a,
    0x0a, 0xdb, 0x6f, 0x02, 0x22, 0xa0, 0x5f, 0x21, 0x2e, 0x59, 0xf0, 0xb8, 0x2e, 0x60, 0x50, 0xc3, 0x7f, 0xd4, 0x7f,
    0xe7, 0x7f, 0xf6, 0x7f, 0xb2, 0x7f, 0xa5, 0x7f, 0x36, 0x30, 0x07, 0x2e, 0x01, 0xf0, 0xbe, 0xbd, 0xbe, 0xbb, 0x23,
    0x58, 0x77, 0x05, 0x09, 0x56, 0x25, 0x54, 0x27, 0x41, 0x06, 0x41, 0xf8, 0xbf, 0xbe, 0x0b, 0xb5, 0x11, 0xd6, 0x42,
    0x03, 0x89, 0x5a, 0x0e, 0xf6, 0x2f, 0x12, 0x30, 0x25, 0x2e, 0x35, 0x00, 0x02, 0x31, 0x25, 0x2e, 0xb8, 0xf0, 0xd4,
    0x6f, 0xc3, 0x6f, 0xe7, 0x6f, 0xb2, 0x6f, 0xa5, 0x6f, 0xf6, 0x6f, 0xa0, 0x5f, 0xc8, 0x2e, 0x09, 0x86, 0x02, 0x30,
    0x12, 0x42, 0x43, 0x0e, 0xfc, 0x2f, 0x37, 0x80, 0x13, 0x30, 0x13, 0x42, 0x12, 0x42, 0x12, 0x42, 0x12, 0x42, 0x02,
    0x42, 0x03, 0x80, 0x41, 0x84, 0x11, 0x42, 0x02, 0x42, 0xb8, 0x2e, 0x46, 0x84, 0x80, 0x50, 0xa3, 0x40, 0x83, 0x88,
    0x82, 0x40, 0x04, 0x41, 0xc3, 0x7f, 0x42, 0x8a, 0x06, 0x41, 0x6d, 0xbb, 0xf6, 0x7f, 0x80, 0xb3, 0xd5, 0x7f, 0xe0,
    0x7f, 0x59, 0x2f, 0x31, 0x25, 0x55, 0x40, 0x41, 0x91, 0xb1, 0x7f, 0x0f, 0x2f, 0x01, 0x30, 0xc1, 0x42, 0x00, 0x2e,
    0xd2, 0x6f, 0x13, 0x40, 0x93, 0x42, 0x00, 0x2e, 0x13, 0x40, 0x93, 0x42, 0x00, 0x2e, 0x00, 0x40, 0x80, 0x42, 0xbd,
    0x80, 0xc0, 0x2e, 0x01, 0x42, 0x80, 0x5f, 0xc7, 0x86, 0x01, 0x30, 0xc5, 0x40, 0xfb, 0x86, 0x45, 0x41, 0x04, 0x41,
    0x43, 0xbe, 0xd5, 0xbe, 0x43, 0xba, 0xd5, 0xba, 0x84, 0x7f, 0x95, 0x7f, 0xa1, 0x7f, 0x14, 0x30, 0x61, 0x15, 0xf5,
    0x09, 0x15, 0x40, 0xc0, 0xb3, 0x0b, 0x2f, 0xc6, 0x40, 0xae, 0x05, 0x07, 0x30, 0xfe, 0x05, 0x80, 0xa9, 0xb7, 0x23,
    0x97, 0x6f, 0x77, 0x0f, 0xa6, 0x6f, 0xe6, 0x23, 0xf6, 0x6f, 0xa7, 0x7f, 0x80, 0x90, 0x00, 0x2f, 0xc5, 0x42, 0x41,
    0x82, 0xc1, 0x86, 0x43, 0xa2, 0xe7, 0x2f, 0xa1, 0x6f, 0xb0, 0x6f, 0x0a, 0x1a, 0x02, 0x2f, 0x01, 0x30, 0x1b, 0x2c,
    0x01, 0x42, 0x01, 0x40, 0x4c, 0x28, 0x82, 0x6f, 0x01, 0x42, 0x4a, 0x0e, 0x13, 0x2f, 0xc0, 0x6f, 0x00, 0xb2, 0x03,
    0x2f, 0x3f, 0x80, 0x20, 0x14, 0x21, 0x2e, 0x5e, 0xf0, 0xe1, 0x6f, 0xd0, 0x6f, 0x52, 0x40, 0x12, 0x42, 0x00, 0x2e,
    0x52, 0x40, 0x12, 0x42, 0x00, 0x2e, 0x41, 0x40, 0x03, 0x2c, 0x01, 0x42, 0x10, 0x30, 0x40, 0x42, 0x80, 0x5f, 0xb8,
    0x2e, 0x10, 0x24, 0x0e, 0x02, 0x11, 0x24, 0x00, 0x0c, 0x12, 0x24, 0x80, 0x2e, 0x13, 0x24, 0x18, 0x00, 0x12, 0x42,
    0x13, 0x42, 0x41, 0x1a, 0xfb, 0x2f, 0x10, 0x24, 0x50, 0x39, 0x11, 0x24, 0x21, 0x2e, 0x21, 0x2e, 0x10, 0x00, 0x23,
    0x2e, 0x11, 0x00, 0x80, 0x2e, 0x10, 0x00
};

/***************************************************************************/

/*!                   Static Function Declarations
 ****************************************************************************/

/*!
 * @brief This internal API is used to update variant information by reading from GPIO register.
 *
 *  @param[in, out] dev : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t update_variant(struct bma4_dev *dev);

/***************************************************************************/

/*!                      User Interface Definitions
 ****************************************************************************/

/*!
 * @brief This API is the entry point. Call this API before using all other APIs
 * This API reads the chip-id of the sensor and sets the resolution, feature
 * length and the type of variant.
 */
int8_t bma422_an_init(struct bma4_dev *dev)
{
    /* Initialize BMA4 sensor */
    int8_t rslt = bma4_init(dev);

    /* Structure to define the default values for axes re-mapping */
    struct bma4_axes_remap axes_remap = {
        .x_axis = BMA4_MAP_X_AXIS, .x_axis_sign = BMA4_MAP_POSITIVE, .y_axis = BMA4_MAP_Y_AXIS,
        .y_axis_sign = BMA4_MAP_POSITIVE, .z_axis = BMA4_MAP_Z_AXIS, .z_axis_sign = BMA4_MAP_POSITIVE
    };

    if (rslt == BMA4_OK)
    {
        if (dev->chip_id == BMA422_AN_CHIP_ID)
        {
            /* Resolution of BMA422_AN sensor is 12 bit */
            dev->resolution = BMA4_12_BIT_RESOLUTION;

            /* Size of configuration file */
            dev->feature_len = BMA422_AN_FEATURE_SIZE;

            dev->config_size = sizeof(bma422_an_config_file);

            /* Set the default values for axis
             *  re-mapping in the device structure
             */
            dev->remap = axes_remap;
        }
        else
        {
            rslt = BMA4_E_INVALID_SENSOR;
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to upload the configuration file to enable the
 * features of the sensor.
 */
int8_t bma422_an_write_config_file(struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMA4_OK;

    /* Check for NULL pointer */
    if (dev != NULL)
    {
        /* Validate read/write length */
        if ((dev->read_write_len >= BMA422_AN_RD_WR_MIN_LEN) && (dev->read_write_len <= BMA422_AN_RD_WR_MAX_LEN))
        {
            /* Check if even or odd  */
            if ((dev->read_write_len % 2) != 0)
            {
                dev->read_write_len = dev->read_write_len - 1;
            }

            /* Assign stream data */
            dev->config_file_ptr = bma422_an_config_file;

            /* Load the configuration file */
            rslt = bma4_write_config_file(dev);

            if (rslt == BMA4_OK)
            {
                /* Update variant */
                rslt = update_variant(dev);
            }
        }
        else
        {
            rslt = BMA4_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is used to get the configuration id of the sensor.
 */
int8_t bma422_an_get_config_id(uint16_t *config_id, struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMA4_OK;

    /* Initialize configuration file */
    uint8_t feature_config[BMA422_AN_FEATURE_SIZE] = { 0 };

    /* Initialize index to get configuration id */
    uint8_t index = BMA422_AN_CONFIG_ID_START_ADDR;

    /* Variable to define MSB of configuration id */
    uint16_t config_id_lsb = 0;

    /* Variable to define LSB of configuration id */
    uint16_t config_id_msb = 0;

    /* Check for NULL pointer */
    if ((dev != NULL) && (config_id != NULL))
    {
        /* Read the configuration file */
        rslt = bma4_read_regs(BMA4_FEATURE_CONFIG_ADDR, feature_config, BMA422_AN_FEATURE_SIZE, dev);

        /* Get configuration id */
        if (rslt == BMA4_OK)
        {
            config_id_lsb = (uint16_t)feature_config[index];
            config_id_msb = ((uint16_t)feature_config[index + 1]) << 8;
            *config_id = config_id_lsb | config_id_msb;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API maps/un-maps the user provided interrupt to either interrupt
 * pin1 or pin2 in the sensor.
 */
int8_t bma422_an_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMA4_OK;

    /* Check for NULL pointer */
    if (dev != NULL)
    {
        /* Validate the interrupt line */
        if (int_line <= 1)
        {
            /* Map/Un-map the interrupt */
            rslt = bma4_map_interrupt(int_line, int_map, enable, dev);
        }
        else
        {
            rslt = BMA4_E_INT_LINE_INVALID;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the bma422_an interrupt status from the sensor.
 */
int8_t bma422_an_read_int_status(uint16_t *int_status, struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMA4_OK;

    /* Check for NULL pointer */
    if ((dev != NULL) && (int_status != NULL))
    {
        /* Read the interrupt status */
        rslt = bma4_read_int_status(int_status, dev);
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API performs x, y and z axis re-mapping in the sensor.
 */
int8_t bma422_an_set_remap_axes(const struct bma4_remap *remap_axes, struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMA4_OK;

    /* Initialize configuration file */
    uint8_t feature_config[BMA422_AN_FEATURE_SIZE] = { 0 };

    /* Initialize index to set re-mapped data */
    uint8_t index = BMA422_AN_AXES_REMAP_START_ADDR;

    if (remap_axes != NULL)
    {
        rslt = bma4_set_remap_axes(remap_axes, feature_config, index, BMA422_AN_FEATURE_SIZE, dev);
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the x, y and z axis re-mapped data from the sensor.
 */
int8_t bma422_an_get_remap_axes(struct bma4_remap *remap_axes, struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMA4_OK;

    /* Initialize configuration file */
    uint8_t feature_config[BMA422_AN_FEATURE_SIZE] = { 0 };

    /* Initialize index to get re-mapped data */
    uint8_t index = BMA422_AN_AXES_REMAP_START_ADDR;

    if (remap_axes != NULL)
    {
        rslt = bma4_get_remap_axes(remap_axes, feature_config, index, BMA422_AN_FEATURE_SIZE, dev);
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the configuration of any-motion feature in the sensor.
 * This API enables/disables the any-motion feature according to the axis set.
 */
int8_t bma422_an_set_any_mot_config(const struct bma422_an_any_no_mot_config *any_mot, struct bma4_dev *dev)
{
    /* Initialize configuration file */
    uint8_t feature_config[BMA422_AN_FEATURE_SIZE] = { 0 };

    /* Update index to configure any-motion axes */
    uint8_t index = BMA422_AN_ANY_MOT_START_ADDR;

    /* Variable to define LSB */
    uint16_t lsb = 0;

    /* Variable to define MSB */
    uint16_t msb = 0;

    /* Variable to define LSB and MSB */
    uint16_t lsb_msb = 0;

    int8_t rslt;

    if ((dev != NULL) && (any_mot != NULL))
    {
        rslt = bma4_set_advance_power_save(BMA4_DISABLE, dev);

        /* Wait for sensor time synchronization. Refer the data-sheet for
         * more information
         */
        dev->delay_us(450, dev->intf_ptr);

        if (rslt == BMA4_OK)
        {

            /* Get any-motion configuration from the sensor */
            rslt = bma4_read_regs(BMA4_FEATURE_CONFIG_ADDR, feature_config, BMA422_AN_FEATURE_SIZE, dev);
            if (rslt == BMA4_OK)
            {
                /* Set threshold value in feature configuration array */
                feature_config[index++] = BMA4_GET_LSB(any_mot->threshold);
                feature_config[index++] = BMA4_GET_MSB(any_mot->threshold);

                /* Extract the word where duration and axes enable
                 * resides
                 */
                lsb = feature_config[index];
                msb = feature_config[index + 1] << 8;
                lsb_msb = lsb | msb;

                /* Set the duration in the same word */
                lsb_msb = BMA4_SET_BITS_POS_0(lsb_msb, BMA422_AN_ANY_NO_MOT_DUR, any_mot->duration);

                /* Set the axes in the same word */
                lsb_msb = BMA4_SET_BITSLICE(lsb_msb, BMA422_AN_ANY_NO_MOT_AXIS_EN, any_mot->axes_en);

                /* Assign the word with set duration and axes enable
                 * value back to feature configuration array
                 */
                feature_config[index++] = BMA4_GET_LSB(lsb_msb);
                feature_config[index] = BMA4_GET_MSB(lsb_msb);

                /* Set any-motion configuration to the sensor */
                rslt = bma4_write_regs(BMA4_FEATURE_CONFIG_ADDR, feature_config, BMA422_AN_FEATURE_SIZE, dev);
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the configuration of any-motion feature from the
 * sensor.
 */
int8_t bma422_an_get_any_mot_config(struct bma422_an_any_no_mot_config *any_mot, struct bma4_dev *dev)
{
    /* Initialize configuration file */
    uint8_t feature_config[BMA422_AN_FEATURE_SIZE] = { 0 };

    /* Update index to configure any-motion axes */
    uint8_t index = BMA422_AN_ANY_MOT_START_ADDR;

    /* Variable to define LSB */
    uint16_t lsb = 0;

    /* Variable to define MSB */
    uint16_t msb = 0;

    /* Variable to define LSB and MSB */
    uint16_t lsb_msb = 0;

    int8_t rslt = 0;

    if ((dev != NULL) && (any_mot != NULL))
    {
        rslt = bma4_set_advance_power_save(BMA4_DISABLE, dev);

        if (rslt == BMA4_OK)
        {
            /* Wait for sensor time synchronization. Refer the data-sheet for
             * more information
             */
            dev->delay_us(450, dev->intf_ptr);

            /* Get any-motion configuration from the sensor */
            rslt = bma4_read_regs(BMA4_FEATURE_CONFIG_ADDR, feature_config, BMA422_AN_FEATURE_SIZE, dev);

            if (rslt == BMA4_OK)
            {
                /* Get word to calculate threshold and any-motion
                 * select
                 */
                lsb = (uint16_t)feature_config[index++];
                msb = ((uint16_t)feature_config[index++] << 8);
                lsb_msb = lsb | msb;

                /* Extract threshold value */
                any_mot->threshold = lsb_msb & BMA422_AN_ANY_NO_MOT_THRES_MSK;

                /* Get word to calculate duration and axes enable */
                lsb = (uint16_t)feature_config[index++];
                msb = ((uint16_t)feature_config[index] << 8);
                lsb_msb = lsb | msb;

                /* Extract duration value */
                any_mot->duration = lsb_msb & BMA422_AN_ANY_NO_MOT_DUR_MSK;

                /* Extract axes enable value */
                any_mot->axes_en =
                    (uint8_t)((lsb_msb & BMA422_AN_ANY_NO_MOT_AXIS_EN_MSK) >> BMA422_AN_ANY_NO_MOT_AXIS_EN_POS);
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API sets the configuration of no-motion feature in the sensor.
 * This API enables/disables the no-motion feature according to the axis set.
 */
int8_t bma422_an_set_no_mot_config(const struct bma422_an_any_no_mot_config *no_mot, struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMA4_OK;

    /* Initialize configuration file */
    uint8_t feature_config[BMA422_AN_FEATURE_SIZE] = { 0 };

    /* Update index to configure no-motion axes */
    uint8_t index = BMA422_AN_NO_MOT_START_ADDR;

    /* Variable to define LSB */
    uint16_t lsb = 0;

    /* Variable to define MSB */
    uint16_t msb = 0;

    /* Variable to define LSB and MSB */
    uint16_t lsb_msb = 0;

    if ((dev != NULL) && (no_mot != NULL))
    {
        /* Get no-motion configuration from the sensor */
        rslt = bma4_read_regs(BMA4_FEATURE_CONFIG_ADDR, feature_config, BMA422_AN_FEATURE_SIZE, dev);
        if (rslt == BMA4_OK)
        {
            /* Set threshold value in feature configuration array */
            feature_config[index++] = BMA4_GET_LSB(no_mot->threshold);
            feature_config[index++] = BMA4_GET_MSB(no_mot->threshold);

            /* Extract the word where duration and axes enable
             * resides
             */
            lsb = feature_config[index];
            msb = feature_config[index + 1] << 8;
            lsb_msb = lsb | msb;

            /* Set the duration in the same word */
            lsb_msb = BMA4_SET_BITS_POS_0(lsb_msb, BMA422_AN_ANY_NO_MOT_DUR, no_mot->duration);

            /* Set the axes in the same word */
            lsb_msb = BMA4_SET_BITSLICE(lsb_msb, BMA422_AN_ANY_NO_MOT_AXIS_EN, no_mot->axes_en);

            /* Assign the word with set duration and axes enable
             * value back to feature configuration array
             */
            feature_config[index++] = BMA4_GET_LSB(lsb_msb);
            feature_config[index] = BMA4_GET_MSB(lsb_msb);

            /* Set no-motion configuration to the sensor */
            rslt = bma4_write_regs(BMA4_FEATURE_CONFIG_ADDR, feature_config, BMA422_AN_FEATURE_SIZE, dev);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the configuration of no-motion feature from the
 * sensor.
 */
int8_t bma422_an_get_no_mot_config(struct bma422_an_any_no_mot_config *no_mot, struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt = BMA4_OK;

    /* Initialize configuration file */
    uint8_t feature_config[BMA422_AN_FEATURE_SIZE] = { 0 };

    /* Update index to configure no-motion axes */
    uint8_t index = BMA422_AN_NO_MOT_START_ADDR;

    /* Variable to define LSB */
    uint16_t lsb = 0;

    /* Variable to define MSB */
    uint16_t msb = 0;

    /* Variable to define LSB and MSB */
    uint16_t lsb_msb = 0;

    if ((dev != NULL) && (no_mot != NULL))
    {
        /* Get no-motion configuration from the sensor */
        rslt = bma4_read_regs(BMA4_FEATURE_CONFIG_ADDR, feature_config, BMA422_AN_FEATURE_SIZE, dev);
        if (rslt == BMA4_OK)
        {
            /* Get word to calculate threshold and no-motion
             * select
             */
            lsb = (uint16_t)feature_config[index++];
            msb = ((uint16_t)feature_config[index++] << 8);
            lsb_msb = lsb | msb;

            /* Extract threshold value */
            no_mot->threshold = lsb_msb & BMA422_AN_ANY_NO_MOT_THRES_MSK;

            /* Get word to calculate duration and axes enable */
            lsb = (uint16_t)feature_config[index++];
            msb = ((uint16_t)feature_config[index] << 8);
            lsb_msb = lsb | msb;

            /* Extract duration value */
            no_mot->duration = lsb_msb & BMA422_AN_ANY_NO_MOT_DUR_MSK;

            /* Extract axes enable value */
            no_mot->axes_en =
                (uint8_t)((lsb_msb & BMA422_AN_ANY_NO_MOT_AXIS_EN_MSK) >> BMA422_AN_ANY_NO_MOT_AXIS_EN_POS);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API is used to get the config file major and minor information.
 */
int8_t bma422_an_get_version_config(uint16_t *config_major, uint16_t *config_minor, struct bma4_dev *dev)
{
    /* Initialize configuration file */
    uint8_t feature_config[BMA422_AN_FEATURE_SIZE] = { 0 };

    /* Update index to config file version */
    uint8_t index = BMA422_AN_CONFIG_ID_START_ADDR;

    /* Variable to define LSB */
    uint8_t lsb = 0;

    /* Variable to define MSB */
    uint8_t msb = 0;

    /* Variable to define LSB and MSB */
    uint16_t lsb_msb = 0;

    /* Result of api are returned to this variable */
    int8_t rslt = BMA4_OK;

    if ((dev != NULL) && (config_major != NULL) && (config_minor != NULL))
    {
        rslt = bma4_set_advance_power_save(BMA4_DISABLE, dev);

        /* Wait for sensor time synchronization. Refer the data-sheet for
         * more information
         */
        dev->delay_us(450, dev->intf_ptr);

        if (rslt == BMA4_OK)
        {
            /* Get config file identification from the sensor */
            rslt = bma4_read_regs(BMA4_FEATURE_CONFIG_ADDR, feature_config, BMA422_AN_FEATURE_SIZE, dev);

            if (rslt == BMA4_OK)
            {
                /* Get word to calculate config file identification */
                lsb = feature_config[index++];
                msb = feature_config[index++];
                lsb_msb = (uint16_t)(msb << 8 | lsb);

                /* Get major and minor version */
                *config_major = BMA4_GET_BITSLICE(lsb_msb, BMA4_CONFIG_MAJOR);
                *config_minor = BMA4_GET_BITS_POS_0(lsb, BMA4_CONFIG_MINOR);
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/***************************************************************************/

/*!                      Static Function Definitions
 ****************************************************************************/

/*!
 * @brief This internal API is used to update variant information by reading from GPIO register.
 */
static int8_t update_variant(struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    if (dev->chip_id == BMA422_AN_CHIP_ID)
    {
        /* Reads the mems_id output from the gpio register */
        rslt = bma4_read_regs(BMA4_MEMS_ID_ADDR, &data, 1, dev);

        if (rslt == BMA4_OK)
        {
            if (data == BMA4_MEMS_SPN_281)
            {
                dev->variant = BMA42X_VARIANT;
            }
            else if (data == BMA4_MEMS_SPN_013)
            {
                dev->variant = BMA42X_B_VARIANT;
            }
            else
            {
                rslt = BMA4_E_INVALID_MEMS_ID;
            }
        }
    }
    else
    {
        rslt = BMA4_E_INVALID_SENSOR;
    }

    return rslt;
}

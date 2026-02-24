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
* @file       bhi360_event_data.c
* @date       2025-03-28
* @version    v2.2.0
*
*/

#include "bhi360_event_data.h"
// #include "bhi360_hif.h"
#include "../bhi260x/bhy2_hif.h"

/**
 * @brief Function to parse FIFO frame data into 3 axes vector
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] vector   : Reference to the data buffer to store vector
 */
void bhi360_event_data_parse_xyz(const uint8_t *data, struct bhi360_event_data_xyz *vector)
{
    vector->x = BHY2_LE2S16(data);
    vector->y = BHY2_LE2S16(data + 2);
    vector->z = BHY2_LE2S16(data + 4);
}

/**
 * @brief Function to parse FIFO frame data into quaternion
 * @param[in] data          : Reference to the data buffer storing data from the FIFO
 * @param[out] quaternion   : Reference to the data buffer to store quaternion
 */
void bhi360_event_data_parse_quaternion(const uint8_t *data, struct bhi360_event_data_quaternion *quaternion)
{
    quaternion->x = BHY2_LE2S16(data);
    quaternion->y = BHY2_LE2S16(data + 2);
    quaternion->z = BHY2_LE2S16(data + 4);
    quaternion->w = BHY2_LE2S16(data + 6);
    quaternion->accuracy = BHY2_LE2U16(data + 8);
}

/**
 * @brief Function to parse FIFO frame data into orientation
 * @param[in] data          : Reference to the data buffer storing data from the FIFO
 * @param[out] orientation  : Reference to the data buffer to store orientation
 */
void bhi360_event_data_parse_orientation(const uint8_t *data, struct bhi360_event_data_orientation *orientation)
{
    orientation->heading = BHY2_LE2S16(data);
    orientation->pitch = BHY2_LE2S16(data + 2);
    orientation->roll = BHY2_LE2S16(data + 4);
}

/*!
 * @brief Parsing the FIFO data to Head Orientation Quaternion output format
 *
 * @param[in]  payload  Reference to FIFO data
 * @param[out] data     Reference to store Head Orientation [Quaternion] data
 *
 * @return  API error codes
 *
 */
void bhi360_event_data_head_orientation_quat_parsing(const uint8_t *payload,
                                                     bhi360_event_data_head_orientation_quat *data)
{
    uint8_t i = 0;

    if ((payload != NULL) && (data != NULL))
    {
        data->x = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->y = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->z = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->w = BHY2_LE2S16(&payload[i]);
    }
}

/*!
 * @brief Parsing the FIFO data to Head Orientation Euler output format
 *
 * @param[in]  payload  Reference to FIFO data
 * @param[out] data     Reference to store Head Orientation [Euler] data
 *
 * @return  API error codes
 *
 */
void bhi360_event_data_head_orientation_eul_parsing(const uint8_t *payload,
                                                    bhi360_event_data_head_orientation_eul *data)
{
    uint8_t i = 0;

    if ((payload != NULL) && (data != NULL))
    {
        data->heading = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->pitch = BHY2_LE2S16(&payload[i]);
        i += 2;
        data->roll = BHY2_LE2S16(&payload[i]);
    }
}

/*!
 * @brief Parsing the fifo data to Multi Tap output format
 *
 * @param[in]  data    Multi Tap data
 * @param[out] output  Reference to store parameter data
 *
 * @return  API error codes
 *
 */
int8_t bhi360_event_data_multi_tap_parsing(const uint8_t *data, uint8_t* output)
{
    int8_t rslt = BHY2_OK;

    if ((data == NULL) || (output == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        *output = *data;
    }

    return rslt;
}

/*!
 * @brief To parse Wrist Gesture Detection Data
 *
 * @param[in] data  : Gesture Detection Data
 * @param[out] Wrist : Gesture Detection Output Structure
 *
 * @return API error codes
 *
 */
int8_t bhi360_event_data_wrist_gesture_detect_parsing(const uint8_t *data,
                                                      bhi360_event_data_wrist_gesture_detect_t *output)
{
    int8_t rslt = BHY2_OK;

    if ((data == NULL) || (output == NULL))
    {
        rslt = BHY2_E_NULL_PTR;
    }
    else
    {
        output->wrist_gesture = (enum bhi360_event_data_wrist_gesture_activity)*(data);
    }

    return rslt;
}

/**
 * @brief Parses the payload data and extracts the air quality information.
 *
 * @param[in] payload : Pointer to the payload data.
 * @param[out] air_quality_data : Pointer to the `bhi360_event_data_iaq_output_t` structure where the extracted air quality
 *                         data will be stored.
 */
void bhi360_event_data_parse_air_quality(const uint8_t *payload, bhi360_event_data_iaq_output_t *air_quality_data)
{
    uint8_t i = 0;

    if ((payload != NULL) && (air_quality_data != NULL))
    {
        air_quality_data->iaq = BHY2_LE2U16(payload + i);
        i += 2;
        air_quality_data->siaq = BHY2_LE2U16(payload + i);
        i += 2;
        air_quality_data->voc = BHY2_LE2U16(payload + i);
        i += 2;
        air_quality_data->co2 = BHY2_LE2U24(payload + i);
        i += 3;
        air_quality_data->iaq_accuracy = payload[i];
        i += 1;
        air_quality_data->comp_temperature = BHY2_LE2S16(payload + i);
        i += 2;
        air_quality_data->comp_humidity = BHY2_LE2U16(payload + i);
        i += 2;
        air_quality_data->raw_gas = BHY2_LE2U32(payload + i);
    }
}

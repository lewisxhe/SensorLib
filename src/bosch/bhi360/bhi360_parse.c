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
* SERVICES; LOSS OF USE, log_i, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bhi360_parse.c
* @date       2025-03-28
* @version    v2.2.0
*
*/

#include "bhi360.h"
#include "bhi360_parse.h"

#ifndef log_i
#define log_i(...)
#endif

#ifndef log_e
#define log_e(...)
#endif

/*SCALE FACTOR from BHI3 data sheet, IAQ data format*/
#define SCALE_IAQ_VOC                100.0
#define SCALE_IAQ_TEMP               256.0
#define SCALE_IAQ_HUMI               500.0

#define MAXIMUM_VIRTUAL_SENSOR_LIST  UINT16_C(256)

static uint16_t count[MAXIMUM_VIRTUAL_SENSOR_LIST] = { 0 };
static bool enable_ds[MAXIMUM_VIRTUAL_SENSOR_LIST] = { false };
static int16_t odr_ds[MAXIMUM_VIRTUAL_SENSOR_LIST] = { 0 };

/**
* @brief Function to convert time in tick to seconds and nanoseconds
* @param[in] time_ticks : Time in ticks
* @param[out] s         : Second part of time
* @param[out] ns        : Nanosecond part of time
* @param[out] tns       : Total time in nanoseconds
*/
static void time_to_s_ns(uint64_t time_ticks, uint32_t *s, uint32_t *ns, uint64_t *tns)
{
    *tns = time_ticks * 15625; /* timestamp is now in nanoseconds */
    *s = (uint32_t)(*tns / UINT64_C(1000000000));
    *ns = (uint32_t)(*tns - ((*s) * UINT64_C(1000000000)));
}

/**
* @brief Function to parse sensor status meta event
* @param[in] event_text    : Event text
* @param[in] s             : Second part of time
* @param[in] ns            : Nanosecond part of time
* @param[in] byte1         : Byte 1 in meta event
* @param[in] byte2         : Byte 2 in meta event
* @param[in] parse_table   : Pointer to parse table
*/
static void parse_meta_event_sensor_status(const char *event_text,
                                           uint32_t s,
                                           uint32_t ns,
                                           uint8_t byte1,
                                           uint8_t byte2,
                                           struct bhi360_parse_ref *parse_table)
{
    struct bhi360_parse_sensor_details *sensor_details;

    log_i("%s; T: %lu.%09lu; Accuracy for sensor id %u changed to %u", event_text, s, ns, byte1, byte2);
    sensor_details = bhi360_parse_get_sensor_details(byte1, parse_table);

    /*lint -e774 */
    if (parse_table && sensor_details)
    {
        sensor_details->accuracy = byte2;
    }
    else
    {
        log_i("Parse slot not defined for %u", byte1);
    }
}

/**
* @brief Function to parse meta event
* @param[in] callback_info : Pointer to callback information
* @param[in] event_text    : Event text
* @param[in] s             : Second part of time
* @param[in] ns            : Nanosecond part of time
* @param[in] parse_table   : Pointer to parse table
*/
static void parse_meta_event_type(const struct bhi360_fifo_parse_data_info *callback_info,
                                  const char *event_text,
                                  uint32_t s,
                                  uint32_t ns,
                                  struct bhi360_parse_ref *parse_table)
{
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];

    switch (meta_event_type)
    {
        case BHI360_META_EVENT_FLUSH_COMPLETE:
            log_i("%s; T: %lu.%09lu; Flush complete for sensor id %u", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_SAMPLE_RATE_CHANGED:
            log_i("%s; T: %lu.%09lu; Sample rate changed for sensor id %u", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_POWER_MODE_CHANGED:
            log_i("%s; T: %lu.%09lu; Power mode changed for sensor id %u", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_ALGORITHM_EVENTS:
            log_i("%s; T: %lu.%09lu; Algorithm event", event_text, s, ns);
            break;
        case BHI360_META_EVENT_SENSOR_STATUS:
            parse_meta_event_sensor_status(event_text, s, ns, byte1, byte2, parse_table);
            break;
        case BHI360_META_EVENT_BSX_DO_STEPS_MAIN:
            log_i("%s; T: %lu.%09lu; BSX event (do steps main)", event_text, s, ns);
            break;
        case BHI360_META_EVENT_BSX_DO_STEPS_CALIB:
            log_i("%s; T: %lu.%09lu; BSX event (do steps calib)", event_text, s, ns);
            break;
        case BHI360_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
            log_i("%s; T: %lu.%09lu; BSX event (get output signal)", event_text, s, ns);
            break;
        case BHI360_META_EVENT_SENSOR_ERROR:
            log_i("%s; T: %lu.%09lu; Sensor id %u reported error 0x%02X", event_text, s, ns, byte1, byte2);
            break;
        case BHI360_META_EVENT_FIFO_OVERFLOW:
            log_i("%s; T: %lu.%09lu; FIFO overflow", event_text, s, ns);
            break;
        case BHI360_META_EVENT_DYNAMIC_RANGE_CHANGED:
            log_i("%s; T: %lu.%09lu; Dynamic range changed for sensor id %u", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_FIFO_WATERMARK:
            log_i("%s; T: %lu.%09lu; FIFO watermark reached", event_text, s, ns);
            break;
        case BHI360_META_EVENT_INITIALIZED:
            log_i("%s; T: %lu.%09lu; Firmware initialized. Firmware version %u", event_text, s, ns,
                 ((uint16_t)byte2 << 8) | byte1);
            break;
        case BHI360_META_TRANSFER_CAUSE:
            log_i("%s; T: %lu.%09lu; Transfer cause for sensor id %u", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_SENSOR_FRAMEWORK:
            log_i("%s; T: %lu.%09lu; Sensor framework event for sensor id %u", event_text, s, ns, byte1);
            break;
        case BHI360_META_EVENT_RESET:
            log_i("%s; T: %lu.%09lu; Reset event. Cause : %u", event_text, s, ns, byte2);
            break;
        case BHI360_META_EVENT_SPACER:
            break;
        default:
            log_i("%s; T: %lu.%09lu; Unknown meta event with id: %u", event_text, s, ns, meta_event_type);
            break;
    }
}

/**
* @brief Function to log data
* @param[in] sid           : Sensor ID
* @param[in] tns           : Time in nanoseconds
* @param[in] event_size    : Event size
* @param[in] event_payload : Event payload
* @param[in] logdev        : Device instance for log
*/
static void log_data(uint8_t sid,
                     uint64_t tns,
                     uint8_t event_size,
                     const uint8_t *event_payload,
                     struct bhi360_logbin_dev *logdev)
{
    if (logdev && logdev->logfile)
    {
#if !defined(PC) && defined(MCU_APP30)
        coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
#endif

#if defined(MCU_APP31)
        coines_set_pin_config(COINES_APP31_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif

        bhi360_logbin_add_data(sid, tns, event_size, event_payload, logdev);

#if !defined(PC) && defined(MCU_APP30)
        coines_set_pin_config(COINES_APP30_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_HIGH);
#endif

#if defined(MCU_APP31)
        coines_set_pin_config(COINES_APP31_LED_G, COINES_PIN_DIRECTION_OUT, COINES_PIN_VALUE_LOW);
#endif
    }
}

/**
* @brief Function to stream hex data
* @param[in] sid           : Sensor ID
* @param[in] ts            : Time in seconds
* @param[in] tns           : Time in nanoseconds
* @param[in] event_size    : Event size
* @param[in] event_payload : Event payload
*/
static void stream_hex_data(uint8_t sid, uint32_t ts, uint32_t tns, uint8_t event_size, const uint8_t *event_payload)
{
    /* Print sensor ID */
    // HEX("%02x%08x%08x", sid, ts, tns);

    for (uint16_t i = 0; i < event_size; i++)
    {
        /* Output raw data in hex */
        log_i("%02x", event_payload[i]);
    }

    log_i("");
}

/**
* @brief Function to print activity in string
* @param[in] activity : Activity value
*/
static void print_activity(uint16_t activity)
{
    if (activity & BHI360_STILL_ACTIVITY_ENDED)
    {
        log_i(" Still activity ended,");
    }

    if (activity & BHI360_WALKING_ACTIVITY_ENDED)
    {
        log_i(" Walking activity ended,");
    }

    if (activity & BHI360_RUNNING_ACTIVITY_ENDED)
    {
        log_i(" Running activity ended,");
    }

    if (activity & BHI360_ON_BICYCLE_ACTIVITY_ENDED)
    {
        log_i(" On bicycle activity ended,");
    }

    if (activity & BHI360_IN_VEHICLE_ACTIVITY_ENDED)
    {
        log_i(" In vehicle ended,");
    }

    if (activity & BHI360_TILTING_ACTIVITY_ENDED)
    {
        log_i(" Tilting activity ended,");
    }

    if (activity & BHI360_STILL_ACTIVITY_STARTED)
    {
        log_i(" Still activity started,");
    }

    if (activity & BHI360_WALKING_ACTIVITY_STARTED)
    {
        log_i(" Walking activity started,");
    }

    if (activity & BHI360_RUNNING_ACTIVITY_STARTED)
    {
        log_i(" Running activity started,");
    }

    if (activity & BHI360_ON_BICYCLE_ACTIVITY_STARTED)
    {
        log_i(" On bicycle activity started,");
    }

    if (activity & BHI360_IN_VEHICLE_ACTIVITY_STARTED)
    {
        log_i(" In vehicle activity started,");
    }

    if (activity & BHI360_TILTING_ACTIVITY_STARTED)
    {
        log_i(" Tilting activity started,");
    }
}

/**
* @brief Function to get sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct bhi360_parse_sensor_details *bhi360_parse_get_sensor_details(uint8_t id, struct bhi360_parse_ref *ref)
{
    uint8_t i;

    for (i = 0; i < BHI360_MAX_SIMUL_SENSORS; i++)
    {
        if (ref->sensor[i].id == id)
        {
            return &ref->sensor[i];
        }
    }

    return NULL;
}

/**
* @brief Function to add sensor details
* @param[in] id  : Sensor ID
* @param[in] ref : Parse reference
* @return Sensor details on success, or NULL on failure
*/
struct bhi360_parse_sensor_details *bhi360_parse_add_sensor_details(uint8_t id, struct bhi360_parse_ref *ref)
{
    uint8_t i = 0;

    struct bhi360_parse_sensor_details *sensor_details;

    sensor_details = bhi360_parse_get_sensor_details(id, ref);
    if (sensor_details)
    {

        /* Slot for the sensor ID is already used */
        return sensor_details;
    }
    else
    {
        /* Find a new slot */
        for (i = 0; i < BHI360_MAX_SIMUL_SENSORS; i++)
        {
            if (ref->sensor[i].id == 0)
            {
                log_i("Using slot %u for SID %u", i, id);
                ref->sensor[i].id = id;

                return &ref->sensor[i];
            }
        }
    }

    return NULL;
}

/**
* @brief Function to check stream log flags
* @param[in] callback_info  : Pointer to callback information
* @param[in] parse_flag     : Stream log flags
*/
static void check_stream_log_flags(const struct bhi360_fifo_parse_data_info *callback_info, uint8_t parse_flag)
{
    if ((parse_flag & PARSE_FLAG_STREAM) && (count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
    {
        if (count[callback_info->sensor_id] == odr_ds[callback_info->sensor_id])
        {
            count[callback_info->sensor_id] = 0;
        }
    }
}

/**
* @brief Function to print log for 3-axis format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] sensor_details : Pointer to sensor details
*/
static void print_log_3axis_s16(const struct bhi360_fifo_parse_data_info *callback_info,
                                struct bhi360_event_data_xyz data,
                                float scaling_factor,
                                uint32_t s,
                                uint32_t ns,
                                const struct bhi360_parse_sensor_details *sensor_details)
{
    log_i("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f; acc: %u",
         callback_info->sensor_id,
         s,
         ns,
         data.x * scaling_factor,
         data.y * scaling_factor,
         data.z * scaling_factor,
         sensor_details->accuracy);
}

/**
* @brief Function to stream and log for 3-axis format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] sensor_details : Pointer to sensor details
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_3axis_s16(bool flag,
                                     const struct bhi360_fifo_parse_data_info *callback_info,
                                     struct bhi360_event_data_xyz data,
                                     uint32_t s,
                                     uint32_t ns,
                                     uint64_t tns,
                                     struct bhi360_parse_ref *parse_table,
                                     uint8_t parse_flag,
                                     const struct bhi360_parse_sensor_details *sensor_details,
                                     float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_3axis_s16(callback_info, data, scaling_factor, s, ns, sensor_details);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_3axis_s16(callback_info, data, scaling_factor, s, ns, sensor_details);
            }
        }
    }

    if (parse_flag & PARSE_FLAG_HEXSTREAM)
    {
        stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for euler format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] sensor_details : Pointer to sensor details
*/
static void print_log_euler(const struct bhi360_fifo_parse_data_info *callback_info,
                            struct bhi360_event_data_orientation data,
                            float scaling_factor,
                            uint32_t s,
                            uint32_t ns,
                            const struct bhi360_parse_sensor_details *sensor_details)
{
    log_i("SID: %u; T: %lu.%09lu; h: %f, p: %f, r: %f; acc: %u",
         callback_info->sensor_id,
         s,
         ns,
         data.heading * scaling_factor,
         data.pitch * scaling_factor,
         data.roll * scaling_factor,
         sensor_details->accuracy);
}

/**
* @brief Function to stream and log for euler format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] sensor_details : Pointer to sensor details
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_euler(bool flag,
                                 const struct bhi360_fifo_parse_data_info *callback_info,
                                 struct bhi360_event_data_orientation data,
                                 uint32_t s,
                                 uint32_t ns,
                                 uint64_t tns,
                                 struct bhi360_parse_ref *parse_table,
                                 uint8_t parse_flag,
                                 const struct bhi360_parse_sensor_details *sensor_details,
                                 float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_euler(callback_info, data, scaling_factor, s, ns, sensor_details);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_euler(callback_info, data, scaling_factor, s, ns, sensor_details);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for quaternion format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_quaternion(const struct bhi360_fifo_parse_data_info *callback_info,
                                 struct bhi360_event_data_quaternion data,
                                 uint32_t s,
                                 uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f; acc: %f",
         callback_info->sensor_id,
         s,
         ns,
         data.x / 16384.0f,
         data.y / 16384.0f,
         data.z / 16384.0f,
         data.w / 16384.0f,
         ((data.accuracy * 180.0f) / 16384.0f) / 3.141592653589793f);
}

/**
* @brief Function to stream and log for quaternion format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_quaternion(bool flag,
                                      const struct bhi360_fifo_parse_data_info *callback_info,
                                      struct bhi360_event_data_quaternion data,
                                      uint32_t s,
                                      uint32_t ns,
                                      uint64_t tns,
                                      struct bhi360_parse_ref *parse_table,
                                      uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_quaternion(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_quaternion(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for 16-bit signed format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_s16_as_float(const struct bhi360_fifo_parse_data_info *callback_info,
                                   int16_t data,
                                   float scaling_factor,
                                   uint32_t s,
                                   uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; %f", callback_info->sensor_id, s, ns, data * scaling_factor);
}

/**
* @brief Function to stream and log for 16-bit signed format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_s16_as_float(bool flag,
                                        const struct bhi360_fifo_parse_data_info *callback_info,
                                        int16_t data,
                                        uint32_t s,
                                        uint32_t ns,
                                        uint64_t tns,
                                        struct bhi360_parse_ref *parse_table,
                                        uint8_t parse_flag,
                                        float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_s16_as_float(callback_info, data, scaling_factor, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_s16_as_float(callback_info, data, scaling_factor, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for 32-bit scalar format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_scalar_u32(const struct bhi360_fifo_parse_data_info *callback_info,
                                 uint32_t data,
                                 uint32_t s,
                                 uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; %lu", callback_info->sensor_id, s, ns, data);
}

/**
* @brief Function to stream and log for 32-bit scalar format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_scalar_u32(bool flag,
                                      const struct bhi360_fifo_parse_data_info *callback_info,
                                      uint32_t data,
                                      uint32_t s,
                                      uint32_t ns,
                                      uint64_t tns,
                                      struct bhi360_parse_ref *parse_table,
                                      uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_scalar_u32(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_scalar_u32(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for scalar event format
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_scalar_event(const struct bhi360_fifo_parse_data_info *callback_info, uint32_t s, uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu;", callback_info->sensor_id, s, ns);
}

/**
* @brief Function to stream and log for scalar event format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_scalar_event(bool flag,
                                        const struct bhi360_fifo_parse_data_info *callback_info,
                                        uint32_t s,
                                        uint32_t ns,
                                        uint64_t tns,
                                        struct bhi360_parse_ref *parse_table,
                                        uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_scalar_event(callback_info, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_scalar_event(callback_info, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for activity format
* @param[in] callback_info  : Pointer to callback information
* @param[in] activity       : Activity value
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_activity(const struct bhi360_fifo_parse_data_info *callback_info,
                               uint16_t activity,
                               uint32_t s,
                               uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; ", callback_info->sensor_id, s, ns);

    print_activity(activity);

    log_i("");
}

/**
* @brief Function to stream and log for activity format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] activity       : Activity value
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_activity(bool flag,
                                    const struct bhi360_fifo_parse_data_info *callback_info,
                                    uint16_t activity,
                                    uint32_t s,
                                    uint32_t ns,
                                    uint64_t tns,
                                    struct bhi360_parse_ref *parse_table,
                                    uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_activity(callback_info, activity, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_activity(callback_info, activity, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for 24-bit unsigned format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] scaling_factor : Scaling factor
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_u24_as_float(const struct bhi360_fifo_parse_data_info *callback_info,
                                   uint32_t data,
                                   float scaling_factor,
                                   uint32_t s,
                                   uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; %f", callback_info->sensor_id, s, ns, (float)data * scaling_factor);
}

/**
* @brief Function to stream and log for 24-bit unsigned format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
* @param[in] scaling_factor : Scaling factor
*/
static void stream_and_log_u24_as_float(bool flag,
                                        const struct bhi360_fifo_parse_data_info *callback_info,
                                        uint32_t data,
                                        uint32_t s,
                                        uint32_t ns,
                                        uint64_t tns,
                                        struct bhi360_parse_ref *parse_table,
                                        uint8_t parse_flag,
                                        float scaling_factor)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_u24_as_float(callback_info, data, scaling_factor, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_u24_as_float(callback_info, data, scaling_factor, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for 8-bit unsigned scalar format
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_scalar_u8(const struct bhi360_fifo_parse_data_info *callback_info,
                                uint8_t data,
                                uint32_t s,
                                uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; %u", callback_info->sensor_id, s, ns, data);
}

/**
* @brief Function to stream and log for 8-bit unsigned scalar format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to stream and log
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_scalar_u8(bool flag,
                                 const struct bhi360_fifo_parse_data_info *callback_info,
                                 uint8_t data,
                                 uint32_t s,
                                 uint32_t ns,
                                 uint64_t tns,
                                 struct bhi360_parse_ref *parse_table,
                                 uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_scalar_u8(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_scalar_u8(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for generic format
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_generic(const struct bhi360_fifo_parse_data_info *callback_info, uint32_t s, uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; D: ", callback_info->sensor_id, s, ns);

    for (uint8_t i = 0; i < (callback_info->data_size - 1); i++)
    {
        log_i("%02X", callback_info->data_ptr[i]);
    }

    log_i("");
}

/**
* @brief Function to stream and log for generic format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_generic(bool flag,
                                   const struct bhi360_fifo_parse_data_info *callback_info,
                                   uint32_t s,
                                   uint32_t ns,
                                   uint64_t tns,
                                   struct bhi360_parse_ref *parse_table,
                                   uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_generic(callback_info, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_generic(callback_info, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to print log for device orientation format
* @param[in] callback_info  : Pointer to callback information
* @param[in] ori            : Pointer to device orientation
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_device_ori(const struct bhi360_fifo_parse_data_info *callback_info,
                                 char *ori,
                                 uint32_t s,
                                 uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; %s", callback_info->sensor_id, s, ns, ori);
}

/**
* @brief Function to stream and log for device orientation format
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] ori            : Pointer to device orientation
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_device_ori(bool flag,
                                      const struct bhi360_fifo_parse_data_info *callback_info,
                                      char *ori,
                                      uint32_t s,
                                      uint32_t ns,
                                      uint64_t tns,
                                      struct bhi360_parse_ref *parse_table,
                                      uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_device_ori(callback_info, ori, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_device_ori(callback_info, ori, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse meta event (wake-up and non-wake-up)
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_meta_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint32_t s, ns;
    uint64_t tns;
    const char *event_text;

    if (!callback_info)
    {
        return;
    }

    if (callback_info->sensor_id == BHI360_SYS_ID_META_EVENT)
    {
        event_text = "[META EVENT]";
    }
    else if (callback_info->sensor_id == BHI360_SYS_ID_META_EVENT_WU)
    {
        event_text = "[META EVENT WAKE UP]";
    }
    else
    {
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    parse_meta_event_type(callback_info, event_text, s, ns, parse_table);
}

/**
* @brief Function to parse 3-axis format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_3axis_s16(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_xyz data;
    uint32_t s, ns;
    uint64_t tns;
    uint8_t parse_flag;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_parse_xyz(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;

        stream_and_log_3axis_s16(flag,
                                 callback_info,
                                 data,
                                 s,
                                 ns,
                                 tns,
                                 parse_table,
                                 parse_flag,
                                 sensor_details,
                                 scaling_factor);

        check_stream_log_flags(callback_info, parse_flag);

    }
    else
    {
        flag = false;

        stream_and_log_3axis_s16(flag,
                                 callback_info,
                                 data,
                                 s,
                                 ns,
                                 tns,
                                 parse_table,
                                 parse_flag,
                                 sensor_details,
                                 scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse euler format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_euler(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_orientation data;
    uint32_t s, ns;
    uint64_t tns;
    uint8_t parse_flag;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");
        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_parse_orientation(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_euler(flag,
                             callback_info,
                             data,
                             s,
                             ns,
                             tns,
                             parse_table,
                             parse_flag,
                             sensor_details,
                             scaling_factor);

        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_euler(flag,
                             callback_info,
                             data,
                             s,
                             ns,
                             tns,
                             parse_table,
                             parse_flag,
                             sensor_details,
                             scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse quaternion format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_quaternion(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    struct bhi360_event_data_quaternion data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_parse_quaternion(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_quaternion(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_quaternion(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse 16-bit signed format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_s16_as_float(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    int16_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not define for %u", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    data = BHI360_LE2S16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_s16_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_s16_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse 32-bit scalar format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_u32(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    data = BHI360_LE2U32(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_scalar_u32(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_scalar_u32(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse scalar event format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_event(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_scalar_event(flag, callback_info, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_scalar_event(flag, callback_info, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse activity format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_activity(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint16_t activity;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    activity = BHI360_LE2U16(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_activity(flag, callback_info, activity, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_activity(flag, callback_info, activity, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse 24-bit unsigned format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_u24_as_float(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    float scaling_factor;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    scaling_factor = sensor_details->scaling_factor;
    parse_flag = sensor_details->parse_flag;

    data = BHI360_LE2U24(callback_info->data_ptr);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_u24_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_u24_as_float(flag, callback_info, data, s, ns, tns, parse_table, parse_flag, scaling_factor);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse 8-bit unsigned scalar format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_scalar_u8(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    data = callback_info->data_ptr[0];
    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_scalar_u8(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_scalar_u8(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse generic format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_generic(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_generic(flag, callback_info, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_generic(flag, callback_info, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse device orientation format
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_device_ori(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    char *ori;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    switch (callback_info->data_ptr[0])
    {
        case 0:
            ori = "Portrait upright";
            break;
        case 1:
            ori = "Landscape left";
            break;
        case 2:
            ori = "Portrait upside down";
            break;
        case 3:
            ori = "Landscape right";
            break;
        default:
            ori = "Unknown orientation";
            break;
    }

    parse_flag = sensor_details->parse_flag;

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_device_ori(flag, callback_info, ori, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_device_ori(flag, callback_info, ori, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to parse debug message
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_debug_message(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    (void)callback_ref;

    uint32_t s, ns;
    uint64_t tns;
    uint8_t msg_length;
    uint8_t debug_msg[17] = { 0 }; /* Max payload size is 16 bytes, adds a trailing zero if the payload is full */

    if (!callback_info)
    {
        log_e("Null reference");

        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    msg_length = callback_info->data_ptr[0];

    memcpy(debug_msg, &callback_info->data_ptr[1], msg_length);
    debug_msg[msg_length] = '\0'; /* Terminate the string */

    log_i("[DEBUG MSG]; T: %lu.%09lu; %s", s, ns, debug_msg);
}

/**
* @brief Function to print log for Air quality
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for Air quality
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_air_quality(const struct bhi360_fifo_parse_data_info *callback_info,
                                  bhi360_event_data_iaq_output_t data,
                                  uint32_t s,
                                  uint32_t ns)
{
    log_i(
        "SID: %u; T: %lu.%09lu; IAQ: %u, SIAQ: %u, VOC: %.2f ppm, CO2: %u ppm,  ACCU: %u, TEMP: %.3f C, HUMI: %.3f%%, GAS: %u Ohm",
        callback_info->sensor_id,
        s,
        ns,
        data.iaq,
        data.siaq,
        data.voc / SCALE_IAQ_VOC,
        data.co2,
        data.iaq_accuracy,
        data.comp_temperature / SCALE_IAQ_TEMP,
        data.comp_humidity / SCALE_IAQ_HUMI,
        data.raw_gas);
}

/**
* @brief Function to stream and log for Air quality
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for Air quality
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_air_quality(bool flag,
                                       const struct bhi360_fifo_parse_data_info *callback_info,
                                       bhi360_event_data_iaq_output_t data,
                                       uint32_t s,
                                       uint32_t ns,
                                       uint64_t tns,
                                       struct bhi360_parse_ref *parse_table,
                                       uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_air_quality(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_air_quality(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Air quality
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_air_quality(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint8_t parse_flag;
    uint32_t s = 0, ns = 0;
    uint64_t tns = 0;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    struct bhi360_parse_sensor_details *sensor_details;
    bhi360_event_data_iaq_output_t air_quality = { 0 };
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference\r");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_e("Parse slot not defined");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    bhi360_event_data_parse_air_quality(callback_info->data_ptr, &air_quality);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;

        stream_and_log_air_quality(flag, callback_info, air_quality, s, ns, tns, parse_table, parse_flag);

        check_stream_log_flags(callback_info, parse_flag);

    }
    else
    {
        flag = false;

        stream_and_log_air_quality(flag, callback_info, air_quality, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to print log for Multi-tap
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for Multi-tap
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_multitap(const struct bhi360_fifo_parse_data_info *callback_info,
                               bhi360_event_data_multi_tap data,
                               uint32_t s,
                               uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; %s; ",
         callback_info->sensor_id,
         s,
         ns,
         bhi360_event_data_multi_tap_string_out[data]);
}

/**
* @brief Function to stream and log for Multi-tap
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for Multi-tap
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_multitap(bool flag,
                                    const struct bhi360_fifo_parse_data_info *callback_info,
                                    bhi360_event_data_multi_tap data,
                                    uint32_t s,
                                    uint32_t ns,
                                    uint64_t tns,
                                    struct bhi360_parse_ref *parse_table,
                                    uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_multitap(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_multitap(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Multi-tap
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_multitap(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;

    bhi360_event_data_multi_tap multitap_data = BHI360_NO_TAP;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference\r");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_e("Parse slot not defined");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    (void)bhi360_event_data_multi_tap_parsing(callback_info->data_ptr, (uint8_t *)&multitap_data);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;

        stream_and_log_multitap(flag, callback_info, multitap_data, s, ns, tns, parse_table, parse_flag);

        check_stream_log_flags(callback_info, parse_flag);

    }
    else
    {
        flag = false;

        stream_and_log_multitap(flag, callback_info, multitap_data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to print log for Wrist Gesture Detector
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for Wrist Gesture Detector
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_wrist_gesture_detect(const struct bhi360_fifo_parse_data_info *callback_info,
                                           bhi360_event_data_wrist_gesture_detect_t data,
                                           uint32_t s,
                                           uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; wrist_gesture: %s; ",
         callback_info->sensor_id,
         s,
         ns,
         bhi360_event_data_wrist_gesture_detect_output[data.wrist_gesture]);
}

/**
* @brief Function to stream and log for Wrist Gesture Detector
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data for Wrist Gesture Detector
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_wrist_gesture_detect(bool flag,
                                                const struct bhi360_fifo_parse_data_info *callback_info,
                                                bhi360_event_data_wrist_gesture_detect_t data,
                                                uint32_t s,
                                                uint32_t ns,
                                                uint64_t tns,
                                                struct bhi360_parse_ref *parse_table,
                                                uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_wrist_gesture_detect(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_wrist_gesture_detect(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Wrist Gesture Detector
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_wrist_gesture_detect(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;

    bhi360_event_data_wrist_gesture_detect_t wrist_gesture_detect_data;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference\r");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_e("Parse slot not defined");

        return;
    }

    parse_flag = sensor_details->parse_flag;
    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    (void)bhi360_event_data_wrist_gesture_detect_parsing(callback_info->data_ptr, &wrist_gesture_detect_data);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;

        stream_and_log_wrist_gesture_detect(flag,
                                            callback_info,
                                            wrist_gesture_detect_data,
                                            s,
                                            ns,
                                            tns,
                                            parse_table,
                                            parse_flag);

        check_stream_log_flags(callback_info, parse_flag);

    }
    else
    {
        flag = false;

        stream_and_log_wrist_gesture_detect(flag,
                                            callback_info,
                                            wrist_gesture_detect_data,
                                            s,
                                            ns,
                                            tns,
                                            parse_table,
                                            parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to print log for hmc
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data head orientation euler
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_hmc(const struct bhi360_fifo_parse_data_info *callback_info,
                          bhi360_event_data_head_orientation_quat data,
                          uint32_t s,
                          uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f",
         callback_info->sensor_id,
         s,
         ns,
         data.x / 16384.0f,
         data.y / 16384.0f,
         data.z / 16384.0f,
         data.w / 16384.0f);
}

/**
* @brief Function to stream and log for hmc
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data head orientation hmc
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_hmc(bool flag,
                               const struct bhi360_fifo_parse_data_info *callback_info,
                               bhi360_event_data_head_orientation_quat data,
                               uint32_t s,
                               uint32_t ns,
                               uint64_t tns,
                               struct bhi360_parse_ref *parse_table,
                               uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_hmc(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_hmc(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Head Misalignment Calibration
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_hmc(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhi360_event_data_head_orientation_quat data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_head_orientation_quat_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_hmc(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_hmc(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to print log for oc
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data to print
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_oc(const struct bhi360_fifo_parse_data_info *callback_info,
                         bhi360_event_data_head_orientation_quat data,
                         uint32_t s,
                         uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; x: %f, y: %f, z: %f, w: %f",
         callback_info->sensor_id,
         s,
         ns,
         data.x / 16384.0f,
         data.y / 16384.0f,
         data.z / 16384.0f,
         data.w / 16384.0f);
}

/**
* @brief Function to stream and log for oc
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data head orientation quaternion
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_oc(bool flag,
                              const struct bhi360_fifo_parse_data_info *callback_info,
                              bhi360_event_data_head_orientation_quat data,
                              uint32_t s,
                              uint32_t ns,
                              uint64_t tns,
                              struct bhi360_parse_ref *parse_table,
                              uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_oc(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_oc(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Head Orientation Quaternion
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_oc(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhi360_event_data_head_orientation_quat data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_head_orientation_quat_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_oc(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_oc(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to print log for ec
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data head orientation euler
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
*/
static void print_log_ec(const struct bhi360_fifo_parse_data_info *callback_info,
                         bhi360_event_data_head_orientation_eul data,
                         uint32_t s,
                         uint32_t ns)
{
    log_i("SID: %u; T: %lu.%09lu; h: %f, p: %f, r: %f",
         callback_info->sensor_id,
         s,
         ns,
         (data.heading * 360.0f) / 32768.0f,
         (data.pitch * 360.0f) / 32768.0f,
         (data.roll * 360.0f) / 32768.0f);
}

/**
* @brief Function to stream and log for ec
* @param[in] flag           : Flag for enabling/disabling downsampling check
* @param[in] callback_info  : Pointer to callback information
* @param[in] data           : Data head orientation euler
* @param[in] s              : Second part of time
* @param[in] ns             : Nanosecond part of time
* @param[in] tns            : Total time in nanoseconds
* @param[in] parse_table    : Pointer to parse table
* @param[in] parse_flag     : Parse flag
*/
static void stream_and_log_ec(bool flag,
                              const struct bhi360_fifo_parse_data_info *callback_info,
                              bhi360_event_data_head_orientation_eul data,
                              uint32_t s,
                              uint32_t ns,
                              uint64_t tns,
                              struct bhi360_parse_ref *parse_table,
                              uint8_t parse_flag)
{
    if (parse_flag & PARSE_FLAG_STREAM)
    {
        if (flag)
        {
            if ((count[callback_info->sensor_id] % odr_ds[callback_info->sensor_id] == 0))
            {
                print_log_ec(callback_info, data, s, ns);
            }
        }
        else
        {
            if (odr_ds[callback_info->sensor_id] != 0)
            {
                print_log_ec(callback_info, data, s, ns);
            }
        }
    }
    else
    {
        if (parse_flag & PARSE_FLAG_HEXSTREAM)
        {
            stream_hex_data(callback_info->sensor_id, s, ns, callback_info->data_size - 1, callback_info->data_ptr);
        }
    }

    if (parse_flag & PARSE_FLAG_LOG)
    {
        log_data(callback_info->sensor_id,
                 tns,
                 callback_info->data_size - 1,
                 callback_info->data_ptr,
                 &parse_table->logdev);
    }
}

/**
* @brief Function to parse for Head Orientation Euler
* @param[in] callback_info : Pointer to callback information
* @param[in] callback_ref  : Pointer to callback reference
*/
void bhi360_parse_ec(const struct bhi360_fifo_parse_data_info *callback_info, void *callback_ref)
{
    bhi360_event_data_head_orientation_eul data;
    uint32_t s, ns;
    uint64_t tns;
    struct bhi360_parse_ref *parse_table = (struct bhi360_parse_ref *)callback_ref;
    uint8_t parse_flag;
    struct bhi360_parse_sensor_details *sensor_details;
    bool flag;

    if (!parse_table || !callback_info)
    {
        log_e("Null reference");

        return;
    }

    sensor_details = bhi360_parse_get_sensor_details(callback_info->sensor_id, parse_table);
    if (!sensor_details)
    {
        log_i("Parse slot not defined for %u", callback_info->sensor_id);

        return;
    }

    parse_flag = sensor_details->parse_flag;

    bhi360_event_data_head_orientation_eul_parsing(callback_info->data_ptr, &data);

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    if (enable_ds[callback_info->sensor_id] == true)
    {
        flag = true;
        stream_and_log_ec(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
        check_stream_log_flags(callback_info, parse_flag);
    }
    else
    {
        flag = false;
        stream_and_log_ec(flag, callback_info, data, s, ns, tns, parse_table, parse_flag);
    }

    count[callback_info->sensor_id]++;
}

/**
* @brief Function to set down sampling flag
* @param[in] sen_id : Virtual sensor ID
* @param[in] enable : Down sampling value
*/
void bhi360_set_downsampling_flag(uint8_t sen_id, bool enable)
{
    enable_ds[sen_id] = enable;
}

/**
* @brief Function to get down sampling flag
* @param[in] sen_id  : Virtual sensor ID
* @return Down sampling value
*/
bool bhi360_get_downsampling_flag(uint8_t sen_id)
{
    return enable_ds[sen_id];
}

/**
* @brief Function to set down sampling ratio
* @param[in] sen_id : Virtual sensor ID
* @param[in] enable : Down sampling ratio
*/
void bhi360_set_downsampling_odr(uint8_t sen_id, int16_t odr)
{
    odr_ds[sen_id] = odr;
}

/**
 * @brief Function to parse FIFO frame data into temperature
 * @param[in] data          : Reference to the data buffer storing data from the FIFO
 * @param[out] temperature  : Reference to the data buffer to store temperature in degree C
 */
void bhi360_parse_temperature_celsius(const uint8_t *data, bhi360_float *temperature)
{
    /* 1 LSB = 1/100 degC */
    float scale_factor = (float)1 / 100;

    *temperature = BHI360_LE2S16(data) * scale_factor;
}

/**
 * @brief Function to parse FIFO frame data into humidity
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] humidity : Reference to the data buffer to store humidity in %
 */
void bhi360_parse_humidity(const uint8_t *data, bhi360_float *humidity)
{
    float scale_factor = (float)1;

    *humidity = data[0] * scale_factor;
}

/**
 * @brief Function to parse FIFO frame data into barometric pressure
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] pressure : Reference to the data buffer to store pressure in Pascals
 */
void bhi360_parse_pressure(const uint8_t *data, bhi360_float *pressure)
{
    /* 1 LSB = 1/128 Pa */
    float scale_factor = (float)1 / 128;

    *pressure = (float)BHI360_LE2U24(data) * scale_factor;
}

/**
 * @brief Function to parse FIFO frame data into altitude
 * @param[in] data      : Reference to the data buffer storing data from the FIFO
 * @param[out] altitude : Reference to the data buffer to store altitude
 */
void bhi360_parse_altitude(const uint8_t *data, bhi360_float *altitude)
{
    *altitude = (float)(data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24));
}

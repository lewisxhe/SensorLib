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
* @file       bhi360_logbin.c
* @date       2025-03-28
* @version    v2.2.0
*
*/

#include "bhi360_logbin.h"

/**
* @brief Function to start logging meta data
* @param[in] dev    : Device instance for binary log
*/
void bhi360_logbin_start_meta(struct bhi360_logbin_dev *dev)
{
    if (dev && dev->logfile)
    {
        fprintf(dev->logfile, "%s\n", LOGBIN_VERSION);
        dev->last_time_ns = 0;
    }
}

/**
* @brief Function to add meta data to log file
* @param[in] sensor_id     : Sensor ID
* @param[in] name          : Sensor name
* @param[in] event_size    : Event size
* @param[in] parse_format  : Sensor parse format
* @param[in] axis_names    : Sensor axis name
* @param[in] scaling       : Sensor scaling factor
* @param[in] dev           : Device instance for binary log
*/
void bhi360_logbin_add_meta(uint8_t sensor_id,
                            char *name,
                            uint8_t event_size,
                            char *parse_format,
                            char *axis_names,
                            float scaling,
                            const struct bhi360_logbin_dev *dev)
{
    if (dev && dev->logfile)
    {
        fprintf(dev->logfile, "%u: %s: %u: %s: %s: %f\n", sensor_id, name, event_size, parse_format, axis_names,
                scaling);
    }
}

/**
* @brief Function to end logging meta data
* @param[in] dev    : Device instance for binary log
*/
void bhi360_logbin_end_meta(struct bhi360_logbin_dev *dev)
{
    if (dev && dev->logfile)
    {
        fprintf(dev->logfile, "\n");
        dev->last_time_ns = 0;
    }
}

/**
* @brief Function to add data to log file
* @param[in] sensor_id     : Sensor ID
* @param[in] time_ns       : Timestamp (ns)
* @param[in] event_size    : Event size
* @param[in] event_payload : Event payload
* @param[in] dev           : Device instance for binary log
*/
void bhi360_logbin_add_data(uint8_t sensor_id,
                            uint64_t time_ns,
                            uint8_t event_size,
                            const uint8_t *event_payload,
                            struct bhi360_logbin_dev *dev)
{
    const uint8_t time_ns_id = LOGBIN_META_ID_TIME_NS;

    if (dev && dev->logfile)
    {
        if (time_ns > dev->last_time_ns)
        {
            fwrite(&time_ns_id, 1, 1, dev->logfile);
            fwrite(&time_ns, 1, LOGBIN_TIME_NS_SIZE, dev->logfile);
            dev->last_time_ns = time_ns;
        }

        fwrite(&sensor_id, 1, 1, dev->logfile);
        fwrite(event_payload, 1, event_size, dev->logfile);
    }
}

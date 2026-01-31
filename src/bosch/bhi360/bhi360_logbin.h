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
* @file       bhi360_logbin.h
* @date       2025-03-28
* @version    v2.2.0
*
*/

#ifndef _BHI360_LOGBIN_H_
#define _BHI360_LOGBIN_H_

/* Start of CPP Guard */
#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus */

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define LOGBIN_VERSION          "1.0"

#define LOGBIN_TIME_NS_SIZE     UINT8_C(8)
#define LOGBIN_LABEL_SIZE       UINT8_C(16)

#define LOGBIN_META_ID_START    UINT8_C(0xF0)
#define LOGBIN_META_ID_TIME_NS  (LOGBIN_META_ID_START + UINT8_C(1)) /* Unsigned 64bit timestamp in nanoseconds */
#define LOGBIN_META_ID_LABEL    (LOGBIN_META_ID_START + UINT8_C(8)) /* String of 16 characters */

struct bhi360_logbin_dev
{
    char logfilename[100];
    FILE *logfile;
    uint64_t last_time_ns;
};

/**
* @brief Function to start logging meta data
* @param[in] dev    : Device instance for binary log
*/
void bhi360_logbin_start_meta(struct bhi360_logbin_dev *dev);

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
                            char * axis_names,
                            float scaling,
                            const struct bhi360_logbin_dev *dev);

/**
* @brief Function to end logging meta data
* @param[in] dev    : Device instance for binary log
*/
void bhi360_logbin_end_meta(struct bhi360_logbin_dev *dev);

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
                            struct bhi360_logbin_dev *dev);

/* End of CPP Guard */
#ifdef __cplusplus
}
#endif /*__cplusplus */

#endif /* _BHI360_LOGBIN_H_ */

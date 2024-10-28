/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      BoschParse.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-30
 *
 */
#include "BoschParse.h"

#ifdef USE_STD_VECTOR
std::vector<ParseCallBackList_t> BoschParse::bhyParseEventVector;
#else
ParseCallBackList_t *BoschParse::BoschParse_bhyParseEventVector = NULL;
uint32_t BoschParse::BoschParse_bhyParseEventVectorSize = 0;
uint32_t BoschParse::BoschParse_bhyParseEventVectorCapacity = 0;
#endif

BhyEventCb BoschParse::_event_callback = NULL;
BhyDebugMessageCallback BoschParse::_debug_callback = NULL;

uint8_t ParseCallBackList::current_id = 1;

#ifndef USE_STD_VECTOR
void BoschParse::expandParseEventVector()
{
    uint32_t newCapacity = BoschParse_bhyParseEventVectorCapacity == 0 ? 10 : 2 * BoschParse_bhyParseEventVectorCapacity;
    ParseCallBackList_t *newVector = (ParseCallBackList_t *)realloc(BoschParse_bhyParseEventVector, newCapacity * sizeof(ParseCallBackList_t));
    if (newVector == NULL) {
        printf("Memory allocation failed for sensor result vector.\n");
        return;
    }
    BoschParse_bhyParseEventVector = newVector;
    BoschParse_bhyParseEventVectorCapacity = newCapacity;
}
#endif

void BoschParse::parseData(const struct bhy2_fifo_parse_data_info *fifo, void *user_data)
{
    int8_t size = fifo->data_size - 1;
#ifdef LOG_PORT
    LOG_PORT.print("\n");
    LOG_PORT.print("[");
    LOG_PORT.print(__func__);
    LOG_PORT.print("]");
    LOG_PORT.print(" Sensor: ");
    LOG_PORT.print(fifo->sensor_id);
    LOG_PORT.print(" name: ");
    LOG_PORT.print(get_sensor_name(fifo->sensor_id));
    LOG_PORT.print(" size: ");
    LOG_PORT.print(fifo->data_size);
    LOG_PORT.print("  value:");
    for (uint8_t i = 0; i < size; i++) {
        LOG_PORT.printf("%04x", fifo->data_ptr[i]);
        LOG_PORT.print(" ");
    }
    LOG_PORT.println();
#endif

#ifdef USE_STD_VECTOR
    for (uint32_t i = 0; i < bhyParseEventVector.size(); i++) {
        ParseCallBackList_t entry = bhyParseEventVector[i];
        if (entry.cb ) {
            if (entry.id == fifo->sensor_id) {
                if (entry.cb) {
                    entry.cb(fifo->sensor_id, fifo->data_ptr, size, fifo->time_stamp);
                }
            }
        }
    }
#else
    for (uint32_t i = 0; i < BoschParse_bhyParseEventVectorSize; i++) {
        ParseCallBackList_t entry = BoschParse_bhyParseEventVector[i];
        if (entry.cb != NULL) {
            if (entry.id == fifo->sensor_id) {
                if (entry.cb != NULL) {
                    entry.cb(fifo->sensor_id, fifo->data_ptr, size, fifo->time_stamp);
                }
            }
        }
    }
#endif
}

void BoschParse::parseMetaEvent(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data)
{
    (void)user_data;

    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    const char *event_text;
    // Remove warning
    (void)byte1;
    (void)byte2;
    (void)event_text;
    uint8_t *accuracy = (uint8_t *)user_data;

#if defined(ARDUINO_ARCH_NRF52)
    /*
    *
    * Unknown error, NRF52 Arduino version 1.6.1
    * If you remove the serial output here, there will never be an interrupt.
    * Suspect SDK problem.
    * Temporarily add event id here to print ten times, then close
    * */
    static uint8_t cnt = 10;
    if (cnt) {
        cnt--;
        Serial.print("parseMetaEvent:"); Serial.println(meta_event_type);
    }
#endif

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT) {
        event_text = "[META EVENT]";
    } else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU) {
        event_text = "[META EVENT WAKE UP]";
    } else {
        return;
    }

    switch (meta_event_type) {
    case BHY2_META_EVENT_FLUSH_COMPLETE:
        log_i("%s Flush complete for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
        log_i("%s Sample rate changed for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_POWER_MODE_CHANGED:
        log_i("%s Power mode changed for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_ALGORITHM_EVENTS:
        log_i("%s Algorithm event", event_text);
        break;
    case BHY2_META_EVENT_SENSOR_STATUS:
        log_i("%s Accuracy for sensor id %u changed to %u", event_text, byte1, byte2);
        if (accuracy) {
            *accuracy = byte2;
        }
        break;
    case BHY2_META_EVENT_BSX_DO_STEPS_MAIN:
        log_i("%s BSX event (do steps main)", event_text);
        break;
    case BHY2_META_EVENT_BSX_DO_STEPS_CALIB:
        log_i("%s BSX event (do steps calib)", event_text);
        break;
    case BHY2_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
        log_i("%s BSX event (get output signal)", event_text);
        break;
    case BHY2_META_EVENT_SENSOR_ERROR:
        log_i("%s Sensor id %u reported error 0x%02X", event_text, byte1, byte2);
        break;
    case BHY2_META_EVENT_FIFO_OVERFLOW:
        log_i("%s FIFO overflow", event_text);
        break;
    case BHY2_META_EVENT_DYNAMIC_RANGE_CHANGED:
        log_i("%s Dynamic range changed for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_FIFO_WATERMARK:
        log_i("%s FIFO watermark reached", event_text);
        break;
    case BHY2_META_EVENT_INITIALIZED:
        log_i("%s Firmware initialized. Firmware version %u", event_text, ((uint16_t)byte2 << 8) | byte1);
        break;
    case BHY2_META_TRANSFER_CAUSE:
        log_i("%s Transfer cause for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_SENSOR_FRAMEWORK:
        log_i("%s Sensor framework event for sensor id %u", event_text, byte1);
        break;
    case BHY2_META_EVENT_RESET:
        log_i("%s Reset event", event_text);
        break;
    case BHY2_META_EVENT_SPACER:
        return;
    default:
        log_i("%s Unknown meta event with id: %u", event_text, meta_event_type);
        break;
    }

    if (_event_callback) {
        _event_callback(meta_event_type, byte1, byte2);
    }
}


void BoschParse::parseDebugMessage(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref)
{
    uint32_t s, ns;
    uint64_t tns;
    uint8_t msg_length = 0;
    uint8_t debug_msg[17] = { 0 }; /* Max payload size is 16 bytes, adds a trailing zero if the payload is full */

    if (!callback_info) {
        log_i("Null reference");
        return;
    }

    time_to_s_ns(*callback_info->time_stamp, &s, &ns, &tns);

    msg_length = callback_info->data_ptr[0];

    memcpy(debug_msg, &callback_info->data_ptr[1], msg_length);
    debug_msg[msg_length] = '\0'; /* Terminate the string */

    // log_i("[DEBUG MSG]; T: %lu.%09lu; %s", s, ns, debug_msg);
    log_i("[DEBUG MSG]: %s", debug_msg);

    if (_debug_callback) {
        _debug_callback((const char *)debug_msg);
    }
}
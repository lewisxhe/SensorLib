/**
 *
 * @license MIT License
 *
 * Copyright (c) 2026 lewis he
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
 * @file      BoschSensorBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-04
 */
#pragma once

#include "../sensor/SensorDefs.hpp"
#include <functional>
#include "BoschSensorID.hpp"
#include "BoschParseCallbackManager.hpp"

using SensorDataParseCallback = void (*)(uint8_t sensor_id, const uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data);

enum class BoschSensorType {
    BOSCH_SENSORTEC_UNKNOWN,
    BOSCH_SENSORTEC_BHI260,
    BOSCH_SENSORTEC_BHI360,
};

class BoschSensorBase
{
protected:
    BoschSensorBase() = default;
    ~BoschSensorBase() = default;
    BoschSensorBase(const BoschSensorBase &) = delete;
    BoschSensorBase &operator=(const BoschSensorBase &) = delete;
public:

    const char *getModelName()
    {
        BoschSensorType mode = getModel();
        switch (mode) {
        case BoschSensorType::BOSCH_SENSORTEC_BHI260:   return "BHI260";
        case BoschSensorType::BOSCH_SENSORTEC_BHI360:   return "BHI360";
        default: break;
        }
        return "UNKNOWN";
    }
    virtual BoschSensorType getModel() = 0;
    virtual float getScaling(uint8_t sensor_id) = 0;
    virtual bool configure(uint8_t sensor_id, float sample_rate, uint32_t report_latency_ms) = 0;
    virtual bool configureRange(uint8_t sensor_id, uint16_t range) = 0;
    virtual SensorConfig getConfigure(uint8_t sensor_id) = 0;
    virtual bool onResultEvent(uint8_t sensor_id, SensorDataParseCallback callback, void *user_data = (void *)nullptr) = 0;
    virtual bool removeResultEvent(uint8_t sensor_id, SensorDataParseCallback callback) = 0;
};

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
 * @file      BoschSensorInfo.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-18
 *
 */

#include "BoschSensorInfo.hpp"
#include <cstring>
#include <algorithm>

BoschSensorInfo::BoschSensorInfo() 
    : kernel_version(0), 
      user_version(0), 
      rom_version(0),
      product_id(0), 
      host_status(0), 
      feat_status(0), 
      boot_status(0), 
      sensor_error(0), 
      dev(nullptr) {
    initializeSensorInfo();
}

BoschSensorInfo::~BoschSensorInfo() = default;

BoschSensorInfo::BoschSensorInfo(BoschSensorInfo&& other) noexcept
    : kernel_version(other.kernel_version),
      user_version(other.user_version),
      rom_version(other.rom_version),
      product_id(other.product_id),
      host_status(other.host_status),
      feat_status(other.feat_status),
      boot_status(other.boot_status),
      sensor_error(other.sensor_error),
      dev(other.dev),
      info(std::move(other.info)) {
    other.clear();
}

BoschSensorInfo& BoschSensorInfo::operator=(BoschSensorInfo&& other) noexcept {
    if (this != &other) {
        kernel_version = other.kernel_version;
        user_version = other.user_version;
        rom_version = other.rom_version;
        product_id = other.product_id;
        host_status = other.host_status;
        feat_status = other.feat_status;
        boot_status = other.boot_status;
        sensor_error = other.sensor_error;
        dev = other.dev;
        info = std::move(other.info);
        
        other.clear();
    }
    return *this;
}

void BoschSensorInfo::initializeSensorInfo() {
    info = std::unique_ptr<bhy2_sensor_info[]>(new bhy2_sensor_info[BHY2_SENSOR_ID_MAX]());
}

void BoschSensorInfo::clear() {
    kernel_version = 0;
    user_version = 0;
    rom_version = 0;
    product_id = 0;
    host_status = 0;
    feat_status = 0;
    boot_status = 0;
    sensor_error = 0;
    dev = nullptr;
    info.reset();
}

bool BoschSensorInfo::isSensorAvailable(uint8_t sensorId) const {
    if (!dev || !info) {
        return false;
    }
    return bhy2_is_sensor_available(sensorId, dev) != 0;
}

std::vector<uint8_t> BoschSensorInfo::getAvailableSensors() const {
    std::vector<uint8_t> available;
    
    if (!dev || !info) {
        return available;
    }
    
    for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; ++i) {
        if (bhy2_is_sensor_available(i, dev)) {
            available.push_back(i);
        }
    }
    
    return available;
}

size_t BoschSensorInfo::getAvailableSensorCount() const {
    if (!dev || !info) {
        return 0;
    }
    
    size_t count = 0;
    for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; ++i) {
        if (bhy2_is_sensor_available(i, dev)) {
            ++count;
        }
    }
    return count;
}

const char* BoschSensorInfo::getSensorName(uint8_t sensorId) {
    return BoschSensorUtils::get_sensor_name(sensorId);
}

bool BoschSensorInfo::updateFromDevice() {
    if (!dev) {
        return false;
    }
    
    bool success = true;
    for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; ++i) {
        if (bhy2_get_sensor_info(i, &info[i], dev) != BHY2_OK) {
            success = false;
        }
    }
    
    return success;
}

const char* BoschSensorInfo::getErrorText(uint8_t error) {
    return BoschSensorUtils::get_sensor_error_text(error);
}
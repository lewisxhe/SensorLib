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
 * @file      SensorBHI360.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-06
 *
 */
#include "SensorBHI360.hpp"

uint16_t SensorBHI360::getConfirmationIDImpl()
{
    return BHI360_CHIP_ID;
}

bool SensorBHI360::getStepCounterConfig(StepCounterConfig &config)
{
    int8_t rslt = bhi360_phy_sensor_ctrl_param_get_step_counter_config(&config, getDev());
    if (rslt != BHY2_OK) {
        log_e("Failed to get Step Counter configuration, error code: %d", rslt);
        return false;
    }
    return true;
}

bool SensorBHI360::setStepCounterConfig(const StepCounterConfig &config)
{
    int8_t rslt = bhi360_phy_sensor_ctrl_param_set_step_counter_config(&config, getDev());
    if (rslt != BHY2_OK) {
        log_e("Failed to set Step Counter configuration, error code: %d", rslt);
        return false;
    }
    return true;
}

bool SensorBHI360::getMultiTapDetectorConfig(MultiTapDetectorConfig &config)
{
    int8_t rslt = bhi360_multi_tap_param_detector_get_config(&config, getDev());
    if (rslt != BHY2_OK) {
        log_e("Failed to get Multi Tap Detector configuration, error code: %d", rslt);
        return false;
    }
    return true;
}

bool SensorBHI360::setMultiTapDetectorConfig(const MultiTapDetectorConfig &config)
{
    int8_t rslt = bhi360_multi_tap_param_detector_set_config(&config, getDev());
    if (rslt != BHY2_OK) {
        log_e("Failed to set Multi Tap Detector configuration, error code: %d", rslt);
        return false;
    }
    return true;
}

bool SensorBHI360::getMultiTapParamConfig(MultiTapDataType (&config)[8])
{
    int8_t rslt = bhi360_multi_tap_param_get_config(config, getDev());
    if (rslt != BHY2_OK) {
        log_e("Failed to get Multi Tap Param configuration, error code: %d", rslt);
        return false;
    }
    return true;
}

bool SensorBHI360::setMultiTapParamConfig(const MultiTapDataType &config)
{
    int8_t rslt = bhi360_multi_tap_param_set_config(&config, getDev());
    if (rslt != BHY2_OK) {
        log_e("Failed to set Multi Tap Param configuration, error code: %d", rslt);
        return false;
    }
    return true;
}


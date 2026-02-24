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
 * @file      SensorBHI360.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-06
 * @note      Most source code references come from the https://github.com/boschsensortec/BHY2-Sensor-API
 *            Simplification for Arduino
 */
#pragma once
#include "bosch/BoschSensorBase.hpp"
#include "bosch/bhi36x/bhi360_event_data.h"
#include "bosch/bhi36x/bhi360_multi_tap_param.h"
#include "bosch/bhi36x/bhi360_phy_sensor_ctrl_param.h"
#include "bosch/bhi36x/bhi360_head_orientation_param.h"
#include "bosch/bhi36x/bhi360_bsx_algo_param.h"
#include "bosch/bhi36x/bhi360_bsec_param.h"

// The BHI360 I2C address can be either 0x28 or 0x29, depending on the state of the HSDO pin.
// HSDO pin set low
#define BHI360_SLAVE_ADDRESS_L          0x28
// HSDO pin set high
#define BHI360_SLAVE_ADDRESS_H          0x29

using StepCounterConfig = bhi360_phy_sensor_ctrl_param_step_counter;
class SensorBHI360 final: public BoschSensorBase
{
public:
    ~SensorBHI360() = default;
    SensorBHI360() = default;


    /**
     * @brief Get the current configuration of the Step Counter algorithm.
     * @param config Reference to a StepCounterConfig structure to store the retrieved configuration.
     * @return true if the configuration was successfully retrieved, false otherwise.
     */
    bool getStepCounterConfig(StepCounterConfig &config);
    
    /**
     * @brief Set the configuration for the Step Counter algorithm.
     * @param config Reference to a StepCounterConfig structure containing the desired configuration.
     * @return true if the configuration was successfully set, false otherwise.
     */
    bool setStepCounterConfig(const StepCounterConfig &config);

protected:
    /**
     * @brief  Get the confirmation ID for the sensor.
     * @note   This ID is used to confirm the sensor's identity during communication.
     * @retval The confirmation ID.
     */
    uint16_t getConfirmationIDImpl() override;
};

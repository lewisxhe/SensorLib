/**
 *
 * @license MIT License
 *
 * Copyright (c) 2023 lewis he
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
 * @file      SensorBHI260AP.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-06
 * @note      Most source code references come from the https://github.com/boschsensortec/BHY2-Sensor-API
 *            Simplification for Arduino
 */
#pragma once
#include "bosch/BoschSensorBase.hpp"

// The BHI260 I2C address can be either 0x28 or 0x29, depending on the state of the HSDO pin.
// HSDO pin set low
#define BHI260AP_SLAVE_ADDRESS_L          0x28
// HSDO pin set high
#define BHI260AP_SLAVE_ADDRESS_H          0x29

class SensorBHI260AP final: public BoschSensorBase
{
public:

    // The pin names are named according to the sensor manual.
    enum BHI260AP_GPIO {
        MCSB1 = 1,
        RESV1 = 2,
        RESV2 = 3,
        MCSB2 = 4,  //It may be connected to the BMM150 sensor, select according to the actual situation
        MCSB3 = 5,
        MCSB4 = 6,

        QSPI_CLK = 8, // If BHI260 carries external flash, it is not available
        QSPI_CSN = 9, // If BHI260 carries external flash, it is not available
        QSPI_D0 = 10, // If BHI260 carries external flash, it is not available
        QSPI_D1 = 11, // If BHI260 carries external flash, it is not available
        QSPI_D2 = 12, // If BHI260 carries external flash, it is not available
        QSPI_D3 = 13, // If BHI260 carries external flash, it is not available

        M2SCX = 14,
        M2SDX = 15,
        M2SDI = 16,
        M3SCL = 17, //It may be connected to the BMM150 sensor, select according to the actual situation
        M3SDA = 18, //It may be connected to the BMM150 sensor, select according to the actual situation
        JTAG_CLK = 19,
        JTAG_DIO = 20,

        M1SCX = 127, // Invalid Pin
        M1SDX = 128, // Invalid Pin
        M1SDI = 129, // Invalid Pin
        RESV3 = 130, // Invalid Pin
    };

    ~SensorBHI260AP() = default;
    SensorBHI260AP() = default;

    /**
     * @brief  setBootFromFlash
     * @note   Set whether to start from external flash
     * @param  boot_from_flash: true boot form flash or boot form ram
     * @retval None
     */
    void setBootFromFlash(bool boot_from_flash);

    /**
     * @brief  digitalRead
     * @note   Read GPIO level, only for custom firmware
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @param  pullup: true is set pullup or input mode
     * @retval 1 is high ,0 is low
     */
    uint8_t digitalRead(uint8_t pin, bool pullup = false);

    /**
     * @brief  digitalWrite
     * @note   Write GPIO level, only for custom firmware
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @param  level: 1 is high ,0 is low
     * @retval None
     */
    void digitalWrite(uint8_t pin, uint8_t level);

    /**
     * @brief  disableGpio
     * @note   Disable GPIO function
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @retval None
     */
    void disableGpio(uint8_t pin);

protected:
    /**
     * @brief  Get the confirmation ID for the sensor.
     * @note   This ID is used to confirm the sensor's identity during communication.
     * @retval The confirmation ID.
     */
    uint16_t getConfirmationIDImpl() override;
};

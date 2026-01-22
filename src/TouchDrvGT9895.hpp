/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      TouchDrvGT9895.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-21
 *
 */
#pragma once

#include "REG/GT9895Constants.h"
#include "TouchDrvInterface.hpp"

class TouchDrvGT9895 :  public TouchDrvInterface, public GT9895Constants
{
public:
    /**
     * @brief  Constructor for the touch driver
     * @retval None
     */
    TouchDrvGT9895() = default;

    /**
     * @brief  Destructor for the touch driver
     * @retval None
     */
    ~TouchDrvGT9895() = default;

    /**
     * @brief  Reset the touch driver
     * @note   This function will reset the touch driver by toggling the reset pin.
     * @retval None
     */
    void reset() override;

    /**
    * @brief Puts the touch driver to sleep
    * @note This function puts the touch driver into sleep mode. 
    *       If the device does not have a reset pin connected, it cannot be woken up after being put 
    *       into sleep mode and must be powered on again.
    * @retval None
    */
    void sleep() override;

    /**
     * @brief  Wake up the touch driver
     * @note   This function will wake up the touch driver from sleep mode.
     * @retval None
     */
    void wakeup() override;

    /**
     * @brief  Get the touch point coordinates
     * @note   This function will retrieve the touch point coordinates from the touch driver.
     * @param  *x_array: Pointer to the array to store the X coordinates
     * @param  *y_array: Pointer to the array to store the Y coordinates
     * @param  size: Number of touch points to retrieve
     * @retval None
     */
    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t size = 1) override;

    /**
     * @brief  Check if the touch point is pressed
     * @note   This function will check if the touch point is currently pressed.
     * @retval True if the touch point is pressed, false otherwise.
     */
    bool isPressed() override;

    /**
     * @brief  Get the model name
     * @note   This function will retrieve the model name from the touch driver.
     * @retval The model name.
     */
    const char *getModelName() override;

private:
    bool initImpl(uint8_t addr) override;
    int is_risk_data(const uint8_t *data, int size);
    int checksum_cmp(const uint8_t *data, int size, int mode);
    int readVersion(ChipFirmwareVersion *version);
    int convertChipInfo(ChipInfo *info, const uint8_t *data);
    void printChipInfo(ChipInfo *ic_info);
    int readChipInfo(ChipInfo *ic_info);
    void clearStatus();
    int getTouchData( uint8_t *pre_buf, uint32_t pre_buf_len);

protected:
    ChipTsEvent          _ts_event;
    ChipFirmwareVersion  _version;
    ChipInfo             _ic_info;
};



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
 * @file      TouchDrvHI8561.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-06
 *
 */
#pragma once

#include "TouchDrvInterface.hpp"

static constexpr uint8_t HI8561_SLAVE_ADDRESS = (0x68);

/**
* @brief HI8561 Touchscreen Display Chip Touch Class
* This class provides the touch reading functionality for the HI8561.
* Note: This class cannot be used independently; the screen must be
* initialized first before calling this class for initialization.
* This is a characteristic of most touchscreen display chips.
*/
class TouchDrvHI8561 :  public TouchDrvInterface
{
public:
    /**
    * @brief  Constructor for the touch driver
    * @retval None
    */
    TouchDrvHI8561() = default;

    /**
    * @brief  Destructor for the touch driver
    * @retval None
    */
    ~TouchDrvHI8561() = default;

    /**
    * @brief Puts the touch driver to sleep
    * @note The chip integrates touch and display functions, and a sleep mode cannot be set.
    * @retval None
    */
    void sleep() override;

    /**
    * @brief  Get the touch points
    * @note   This function will retrieve the touch points from the touch driver.
    *         TouchPoints is configured with a 5-point touch point buffer by default,
    *         so it can return touch data from a maximum of 5 points. Although the HI8561
    *         supports 10-point touch
    * @retval A reference to the touch points.
    */
    const TouchPoints &getTouchPoints() override;

    /**
    * @brief  Get the model name
    * @note   This function will retrieve the model name from the touch driver.
    * @retval The model name.
    */
    const char *getModelName() override;

private:

    /**
     * @brief  Initialize the touch driver
     * @note   This function will initialize the touch driver by setting up the I2C communication.
     * @param  No use
     * @retval True if the initialization was successful, false otherwise.
     */
    bool initImpl(uint8_t) override;

    bool inline makePacketThenRead(uint32_t addr, uint8_t *data, uint16_t len)
    {
        uint8_t addrBuf[6];
        addrBuf[0] = 0xF3;
        addrBuf[1] = (uint8_t)((addr & 0xFF000000) >> 24);
        addrBuf[2] = (uint8_t)((addr & 0x00FF0000) >> 16);
        addrBuf[3] = (uint8_t)((addr & 0x0000FF00) >> 8);
        addrBuf[4] = (uint8_t)((addr & 0x000000FF) >> 0);
        addrBuf[5] = 0x03;
        return writeThenRead(addrBuf, sizeof(addrBuf), data, len) == 0;
    }

protected:
    static constexpr uint32_t REG_SECTION_INFO = 0x200110D8;
    static constexpr uint32_t REG_INFO = 0x20011120;
    static constexpr uint32_t REG_POINT = 0x20011123;
    static constexpr uint8_t BYTES_OFFSET = 3;
    static constexpr uint8_t MAX_FINGER_NUM = (10);
    static constexpr uint8_t BYTES_PER_POINT = (5);
};

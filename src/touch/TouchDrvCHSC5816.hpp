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
 * @file      TouchDrvCHSC5816.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-12
 *
 */
#pragma once

#include "TouchDrvInterface.hpp"

static constexpr uint8_t  CHSC5816_SLAVE_ADDRESS = (0x2E);

class TouchDrvCHSC5816 :  public TouchDrvInterface
{
public:
    /**
    * @brief  Constructor for the touch driver
    * @retval None
    */
    TouchDrvCHSC5816() = default;

    /**
    * @brief  Destructor for the touch driver
    * @retval None
    */
    ~TouchDrvCHSC5816() = default;

    /**
    * @brief Puts the touch driver to sleep
    * @note This function puts the touch driver into sleep mode.
    *       If the device does not have a reset pin connected, it cannot be woken up after being put
    *       into sleep mode and must be powered on again.
    * @retval None
    */
    void sleep() override;

    /**
     * @brief  Get the touch points
     * @note   This function retrieves the touch points from the touch driver.
     * @retval The touch points.
     */
    const TouchPoints &getTouchPoints() override;

    /**
    * @brief  Get the model name
    * @note   This function will retrieve the model name from the touch driver.
    * @retval The model name.
    */
    const char *getModelName() override;

private:

    bool initImpl(uint8_t) override;

protected:
    static constexpr uint32_t REG_BOOT_STATE  = 0x20000018;
    static constexpr uint32_t REG_IMG_HEAD    = 0x20000014;
    static constexpr uint32_t REG_POINT_EVENT = 0x2000002C;
    static constexpr uint32_t REG_POINT       = 0x2000002E;
    static constexpr uint8_t  MAX_FINGER_NUM  = (1);
    static constexpr uint8_t  BYTES_PER_POINT = (5);
    static constexpr uint8_t  POINT_BUFFER_SIZE = (MAX_FINGER_NUM * BYTES_PER_POINT + 3);
    static constexpr uint8_t  COMMAND_SIZE = (4);
};

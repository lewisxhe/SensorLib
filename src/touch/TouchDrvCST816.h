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
 * @file      TouchDrvCST816.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-06
 */
#pragma once

#include "TouchDrvInterface.hpp"

static constexpr uint8_t  CST816_SLAVE_ADDRESS  = (0x15);

class TouchDrvCST816 : public TouchDrvInterface
{
public:
    using TouchDrvInterface::getPoint;
    /**
     * @brief  Constructor for the touch driver
     * @retval None
     */
    TouchDrvCST816() = default;

    /**
     * @brief  Destructor for the touch driver
     * @retval None
     */
    ~TouchDrvCST816() = default;

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
     * @note   This function will retrieve the touch points from the touch driver.
     * @retval A reference to the touch points.
     */
    const TouchPoints &getTouchPoints() override;

    /**
    * @brief  Get the model name
    * @note   This function will retrieve the model name from the touch driver.
    * @retval The model name.
    */
    const char *getModelName() override;

    /**
     * @brief  Disable auto sleep mode
     * @note   This function will disable the auto sleep mode of the touch driver.
     *
     * @retval None
     */
    void disableAutoSleep() ;

    /**
     * @brief  Enable auto sleep mode
     * @note   This function will enable the auto sleep mode of the touch driver.
     * @retval None
     */
    void enableAutoSleep() ;

private:
    bool initImpl(uint8_t) override;

    void beforeBegin() override
    {
        _pinsCfg.rstHoldTimeMs = 30;
        _pinsCfg.rstReleaseTimeMs = 50;
    }

protected:
    static constexpr uint8_t  CST8xx_REG_STATUS         = (0x00);
    static constexpr uint8_t  CST8xx_REG_XPOS_HIGH      = (0x03);
    static constexpr uint8_t  CST8xx_REG_XPOS_LOW       = (0x04);
    static constexpr uint8_t  CST8xx_REG_YPOS_HIGH      = (0x05);
    static constexpr uint8_t  CST8xx_REG_YPOS_LOW       = (0x06);
    static constexpr uint8_t  CST8xx_REG_DIS_AUTOSLEEP  = (0xFE);
    static constexpr uint8_t  CST8xx_REG_CHIP_ID        = (0xA7);
    static constexpr uint8_t  CST8xx_REG_FW_VERSION     = (0xA9);
    static constexpr uint8_t  CST8xx_REG_SLEEP          = (0xE5);
    static constexpr uint8_t  CST816S_CHIP_ID           = (0xB4);
    static constexpr uint8_t  CST816T_CHIP_ID           = (0xB5);
    static constexpr uint8_t  CST716_CHIP_ID            = (0x20);
    static constexpr uint8_t  CST820_CHIP_ID            = (0xB7);
    static constexpr uint8_t  CST816D_CHIP_ID           = (0xB6);
    static constexpr uint8_t  MAX_FINGER_NUM            = (1);
};

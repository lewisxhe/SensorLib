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
 * @file      TouchDrvCST3530.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-21
 */
#pragma once

#include "TouchDrvInterface.hpp"

// The device address is not fixed and is determined by the manufacturer.
static constexpr uint8_t   CS3530_SLAVE_ADDRESS = (0x1A);


class TouchDrvCST3530 : public TouchDrvInterface
{
public:
    using TouchDrvInterface::getPoint;
    /**
    * @brief  Constructor for the touch driver
    * @retval None
    */
    TouchDrvCST3530() = default;

    /**
     * @brief  Destructor for the touch driver
     * @retval None
     */
    ~TouchDrvCST3530() = default;

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

private:

    bool initImpl(uint8_t) override;

    void beforeBegin() override
    {
        _pinsCfg.rstHoldTimeMs = 30;
        _pinsCfg.rstReleaseTimeMs = 50;
    }

    bool writeCommand(uint32_t cmd, uint8_t *read_buffer = nullptr, size_t read_size = 0)
    {
        uint8_t write_buffer[4];
        write_buffer[0] = (cmd >> 24) & 0xFF;
        write_buffer[1] = (cmd >> 16) & 0xFF;
        write_buffer[2] = (cmd >> 8) & 0xFF;
        write_buffer[3] = (cmd >> 0) & 0xFF;
        if (read_buffer) {
            return 0 == writeThenRead(write_buffer, arraySize(write_buffer), read_buffer, read_size);
        } else {
            return 0 == writeBuff(write_buffer, arraySize(write_buffer));
        }
    }

    uint16_t check_sum_16(int val, uint8_t* buf, uint16_t len);

protected:
    static constexpr uint32_t  READ_COMMAND                = (0xD0070000);
    static constexpr uint32_t  INFO_COMMAND                = (0xD0030000);
    static constexpr uint32_t  SLEEP_COMMAND               = (0xD00022AB);
    static constexpr uint32_t  CLEAR_COMMAND               = (0xD00002AB);
    static constexpr uint8_t   MAX_FINGER_NUM              = (5);
    static constexpr uint8_t   MAX_READ_BYTES              = (32);
};

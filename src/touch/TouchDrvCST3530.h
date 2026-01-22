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
#define CS3530_SLAVE_ADDRESS      (0x1A)


class TouchDrvCST3530 : public TouchDrvInterface
{
private:
    static constexpr uint32_t  READ_COMMAND                = (0xD0070000);
    static constexpr uint32_t  INFO_COMMAND                = (0xD0030000);
    static constexpr uint32_t  SLEEP_COMMAND               = (0xD00022AB);
    static constexpr uint32_t  CLEAR_COMMAND               = (0xD00002AB);
    static constexpr uint8_t   MAX_FINGER_NUM              = (5);
    static constexpr uint8_t   MAX_READ_BYTES              = (32);

public:

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
    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point) override;

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
    const char *getModelName();

private:

    bool initImpl(uint8_t addr) override;

    bool writeCommand(uint32_t cmd, uint8_t *read_buffer = nullptr, size_t read_size = 0)
    {
        uint8_t write_buffer[4];
        write_buffer[0] = (cmd >> 24) & 0xFF;
        write_buffer[1] = (cmd >> 16) & 0xFF;
        write_buffer[2] = (cmd >> 8) & 0xFF;
        write_buffer[3] = (cmd >> 0) & 0xFF;
        if (read_buffer) {
            return 0 == comm->writeThenRead(write_buffer, arraySize(write_buffer), read_buffer, read_size);
        } else {
            return 0 == comm->writeBuffer(write_buffer, arraySize(write_buffer));
        }
    }

    uint16_t check_sum_16(int val, uint8_t* buf, uint16_t len);
};

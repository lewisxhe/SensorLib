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

#include "TouchDrvInterface.hpp"

static constexpr uint8_t  GT9895_SLAVE_ADDRESS_H = (0x14);
static constexpr uint8_t  GT9895_SLAVE_ADDRESS_L = (0x5D);

class TouchDrvGT9895 :  public TouchDrvInterface
{
public:
    using TouchDrvInterface::getPoint;
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
    *         TouchPoints is configured with a 5-point touch point buffer by default,
    *         so it can return touch data from a maximum of 5 points. Although the GT9895
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

    enum CheckSumMode {
        CHECKSUM_MODE_U8_LE,
        CHECKSUM_MODE_U16_LE,
    };

    bool initImpl(uint8_t) override;

    /**
     * @brief  Compare the checksum of the data
     * @note   This function will compare the checksum of the data with the expected value.
     * @param  *data: Pointer to the data buffer
     * @param  size: Size of the data buffer
     * @param  mode: Checksum mode
     * @retval 0 if the checksum is valid, -1 if it is invalid
     */
    int calculateChecksum(const uint8_t *data, int size, CheckSumMode mode);

    /**
     * @brief  Get chip pid
     * @note   This function will retrieve the chip pid from the touch driver.
     * @retval The chip pid.
     */
    uint32_t getChipPID();

    /**
     * @brief  Clear the touch driver status
     * @note   This function will clear the touch driver status.
     * @retval None
     */
    void clearStatus();

protected:

    static constexpr uint8_t MAX_FINGER_NUM             = (10);
    static constexpr uint8_t IRQ_EVENT_HEAD_LEN         = (8);
    static constexpr uint8_t BYTES_PER_POINT            = (8);
    static constexpr uint8_t COORDS_DATA_CHECKSUM_SIZE  = (2);

    static constexpr uint8_t POINT_TYPE_STYLUS_HOVER    = (1);
    static constexpr uint8_t POINT_TYPE_STYLUS          = (3);

    static constexpr uint32_t REG_FW_VERSION = (0x010014);
    static constexpr uint32_t REG_INFO       = (0x010070);
    static constexpr uint32_t REG_CMD        = (0x010174);
    static constexpr uint32_t REG_POINT      = (0x010308);
    static constexpr uint32_t REG_NUM_POINT  = (0x01030A);
    
    static constexpr uint32_t CHIP_PID       = (0x9895);
};

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
 * @file      TouchDrvFT6X36.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-01
 *
 */
#pragma once

#include "TouchDrvInterface.hpp"

static constexpr uint8_t FT3267_SLAVE_ADDRESS = (0x38);
static constexpr uint8_t FT5206_SLAVE_ADDRESS = (0x38);
static constexpr uint8_t FT6X36_SLAVE_ADDRESS = (0x38);

class TouchDrvFT6X36 :  public TouchDrvInterface
{
public:
    using TouchDrvInterface::getPoint;
    enum GesTrue {
        NO_GESTURE,
        MOVE_UP,
        MOVE_LEFT,
        MOVE_DOWN,
        MOVE_RIGHT,
        ZOOM_IN,
        ZOOM_OUT,
    } ;

    enum EventFlag {
        EVENT_PUT_DOWN,
        EVENT_PUT_UP,
        EVENT_CONTACT,
        EVENT_NONE,
    } ;

    enum PowerMode {
        PMODE_ACTIVE = 0,         // ~4mA
        PMODE_MONITOR = 1,        // ~3mA
        PMODE_DEEP_SLEEP = 3,     // ~100uA  The reset pin must be pulled down to wake up
    } ;

    /**
    * @brief  Constructor for the touch driver
    * @retval None
    */
    TouchDrvFT6X36() = default;

    /**
    * @brief  Destructor for the touch driver
    * @retval None
    */
    ~TouchDrvFT6X36() = default;

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
     * @brief  Get the device mode
     * @note   This function will retrieve the current device mode from the touch driver.
     * @retval The device mode.
     */
    uint8_t getDeviceMode(void);

    // Obtaining gestures depends on whether the built-in firmware of the chip has this function
    uint8_t getGesture();


    /**
     * @brief  Set the device mode
     * @note   This function will set the current device mode for the touch driver.
     * @param  value: The device mode to set.
     * @retval None
     */
    void setDeviceMode(uint8_t value);

    /**
     * @brief  Get the touch threshold
     * @note   This function will retrieve the current touch threshold from the touch driver.
     * @retval The touch threshold.
     */
    uint8_t getThreshold(void);


    /**
     * @brief  Set the touch threshold
     * @note   This function will set the touch threshold for the touch driver.
     * @param  value: The touch threshold to set.
     * @retval None
     */
    void setThreshold(uint8_t value);

    /**
     * @brief  Get the monitor time
     * @note   This function will retrieve the current monitor time from the touch driver.
     * @retval The monitor time.
     */
    uint8_t getMonitorTime(void);

    /**
     * @brief  Set the monitor time
     * @note   This function will set the monitor time for the touch driver.
     * @param  sec: The monitor time to set.
     * @retval None
     */
    void setMonitorTime(uint8_t sec);


    /**
     * @brief  Get the library version
     * @note   This function will retrieve the current library version from the touch driver.
     * @retval The library version.
     */
    uint16_t getLibraryVersion();

    /**
     * @brief  Polling mode for touch interrupt
     * @note   This function will configure the touch driver to use polling mode for touch interrupts.
     * @retval None
     */
    void interruptPolling(void);

    /**
     * @brief  Trigger mode for touch interrupt
     * @note   This function will configure the touch driver to use trigger mode for touch interrupts.
     * @retval None
     */
    void interruptTrigger(void);

    /**
     * @brief  Set the touch interrupt mode
     * @note   This function will configure the touch driver to use the specified interrupt mode.
     * @param  mode: The interrupt mode to set.
     * @retval None
     */
    void setPowerMode(PowerMode mode);

    /**
     * @brief  Get the vendor ID
     * @note   This function will retrieve the current vendor ID from the touch driver.
     * @retval The vendor ID.
     */
    uint8_t getVendorID(void);

    /**
     * @brief  Get the error code
     * @note   This function will retrieve the current error code from the touch driver.
     * @retval The error code.
     */
    uint8_t getErrorCode(void);

private:
    bool initImpl(uint8_t) override;

    void beforeBegin() override
    {
        _pinsCfg.rstHoldTimeMs = 30;
        _pinsCfg.rstReleaseTimeMs = 160;
    }


    static constexpr uint8_t BYTES_PER_POINT = (6);
    static constexpr uint8_t MAX_FINGER_NUM = (5);

    static constexpr uint8_t FT_VEND_ID1                 = (0x11);
    static constexpr uint8_t FT_VEND_ID2                 = (0xCD);
    static constexpr uint8_t FT_VEND_ID3                 = (0x51);

    static constexpr uint8_t FT3267_CHIP_ID              = (0x33);
    static constexpr uint8_t FT3068_CHIP_ID              = (0xA0);
    static constexpr uint8_t FT5336_CHIP_ID              = (0x14);
    static constexpr uint8_t FT6206_CHIP_ID              = (0x06);
    static constexpr uint8_t FT6236_CHIP_ID              = (0x36);
    static constexpr uint8_t FT6336U_CHIP_ID             = (0x64);

    static constexpr uint8_t FT6X36_REG_MODE             = (0x00);
    static constexpr uint8_t FT6X36_REG_GEST             = (0x01);
    static constexpr uint8_t FT6X36_REG_STATUS           = (0x02);
    static constexpr uint8_t FT6X36_REG_TOUCH1_XH        = (0x03);
    static constexpr uint8_t FT6X36_REG_TOUCH1_XL        = (0x04);
    static constexpr uint8_t FT6X36_REG_TOUCH1_YH        = (0x05);
    static constexpr uint8_t FT6X36_REG_TOUCH1_YL        = (0x06);
    static constexpr uint8_t FT6X36_REG_THRESHOLD        = (0x80);
    static constexpr uint8_t FT6X36_REG_MONITOR_TIME     = (0x87);
    static constexpr uint8_t FT6X36_REG_PERIOD_ACTIVE    = (0x88);
    static constexpr uint8_t FT6X36_REG_PERIOD_MONITOR   = (0x89);
    static constexpr uint8_t FT6X36_REG_AUTO_CLB_MODE    = (0xA0);
    static constexpr uint8_t FT6X36_REG_LIB_VERSION_H    = (0xA1);
    static constexpr uint8_t FT6X36_REG_LIB_VERSION_L    = (0xA2);
    static constexpr uint8_t FT6X36_REG_INT_STATUS       = (0xA4);
    static constexpr uint8_t FT6X36_REG_POWER_MODE       = (0xA5);
    static constexpr uint8_t FT6X36_REG_FIRM_VERS        = (0xA6);
    static constexpr uint8_t FT6X36_REG_CHIP_ID          = (0xA3);
    static constexpr uint8_t FT6X36_REG_VENDOR1_ID       = (0xA8);
    static constexpr uint8_t FT6X36_REG_ERROR_STATUS     = (0xA9);
};

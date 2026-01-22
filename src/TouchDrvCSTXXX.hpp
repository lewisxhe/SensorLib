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
 * @file      TouchDrvCSTXXX.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-24
 * @date      last 2025-01-20
 *
 */
#pragma once

#include "touch/TouchDrvCST226.h"
#include "touch/TouchDrvCST816.h"
#include "touch/TouchDrvCST92xx.h"
#include "touch/TouchDrvCST3530.h"

#define CSTXXX_SLAVE_ADDRESS        (0x15)
#define CST328_SLAVE_ADDRESS        (0x1A)

enum TouchDrvType {
    TouchDrv_CST226,
    TouchDrv_CST8XX,
    TouchDrv_CST92XX,
    TouchDrv_CST3530,
    TouchDrv_UNKNOWN,
};

class TouchDrvCSTXXX : public TouchDrvInterface
{
public:
    /**
    * @brief  Constructor for the touch driver
    * @retval None
    */
    TouchDrvCSTXXX();

    /**
    * @brief  Destructor for the touch driver
    * @retval None
    */
    ~TouchDrvCSTXXX() = default;

// *INDENT-OFF*
    /**
     * @brief  Set the touch driver model
     * @note   This function sets the touch driver model to be used.
     * @param  model: The touch driver model to set [TouchDrv_CST226, TouchDrv_CST8XX, TouchDrv_CST92XX, TouchDrv_CST3530].
     * @retval None
     */
    void setTouchDrvModel(TouchDrvType model);

#if defined(ARDUINO)
    /**
     * @brief  Initialize the touch driver for Arduino
     * @note   This function initializes the touch driver with the specified I2C parameters.
     * @param  &wire: The I2C wire object to use for communication.
     * @param  addr: The I2C address of the touch driver.
     * @param  sda: The SDA pin number.
     * @param  scl: The SCL pin number.
     * @retval True if successful, false otherwise.
     */
    bool begin(TwoWire &wire, uint8_t addr, int sda, int scl) override;
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    /**
     * @brief  Initialize the touch driver for ESP32 using legacy I2C
     * @note   This function initializes the touch driver with the specified I2C parameters.
     * @param  port_num: The I2C port number to use for communication.
     * @param  addr: The I2C address of the touch driver.
     * @param  sda: The SDA pin number.
     * @param  scl: The SCL pin number.
     * @retval True if successful, false otherwise.
     */
    bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1) override;
#else
    /**
     * @brief  Initialize the touch driver for ESP32 using new I2C
     * @note   This function initializes the touch driver with the specified I2C parameters.
     * @param  handle: The I2C master bus handle to use for communication.
     * @param  addr: The I2C address of the touch driver.
     * @retval True if successful, false otherwise.
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr) override;
#endif  // ESP_PLATFORM
#endif  // ARDUINO

    /**
     * @brief  Initialize the touch driver for custom communication
     * @note   This function initializes the touch driver with the specified custom callbacks.
     * @param  callback: The custom callback function for communication.
     * @param  hal_callback: The custom HAL callback function for hardware abstraction.
     * @param  addr: The I2C address of the touch driver.
     * @retval True if successful, false otherwise.
     */
    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr) override;

    /**
     * @brief  Sets the GPIO callbacks for the touch driver
     * @note   This function sets the callbacks for the GPIO operations in the touch driver.
     * @param  mode_cb: The callback for the mode change
     * @param  write_cb: The callback for the write operation
     * @param  read_cb: The callback for the read operation
     * @retval None
     */
    void setGpioCallback(CustomMode mode_cb, CustomWrite write_cb, CustomRead read_cb);

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
     * @retval Return the number of touch points retrieved
     */
    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point = 1) override;

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

    /**
    * @brief  Get the chip ID
    * @note   This function will retrieve the chip ID from the touch driver.
    * @retval The chip ID.
    */
    uint32_t getChipID() override;

    /**
     * @brief  Set the center button coordinate
     * @note   This function will set the center button coordinate for the touch driver.
     * @param  x: The X coordinate of the center button
     * @param  y: The Y coordinate of the center button
     * @retval None
     */
    void setCenterButtonCoordinate(int16_t x, int16_t y) override;

    /**
     * @brief  Set the home button callback
     * @note   This function will set the callback function for the home button.
     * @param  cb: The callback function to set
     * @param  *user_data: Pointer to user data to pass to the callback function
     * @retval None
     */
    void setHomeButtonCallback(TouchDrvInterface::HomeButtonCallback callback, void *user_data = NULL) override;

    /**
     * @brief  Disable automatic sleep
     * @note   This function will disable automatic sleep for the touch driver,only for CST328
     * @retval None
     */
    void disableAutoSleep();

    /**
     * @brief  Enable automatic sleep
     * @note   This function will enable automatic sleep for the touch driver,only for CST328
     * @retval None
     */
    void enableAutoSleep();

    /**
     * @brief  Set the swap XY flag
     * @note   This function will set the swap XY flag for the touch driver.
     * @param  swap: True to swap X and Y coordinates, false otherwise.
     * @retval None
     */
    void setSwapXY(bool swap) override;

    /**
     * @brief  Set the mirror XY flag
     * @note   This function will set the mirror XY flag for the touch driver.
     * @param  mirrorX: True to mirror X coordinates, false otherwise.
     * @param  mirrorY: True to mirror Y coordinates, false otherwise.
     * @retval None
     */
    void setMirrorXY(bool mirrorX, bool mirrorY) override;

    /**
     * @brief  Set the maximum coordinates
     * @note   This function will set the maximum coordinates for the touch driver.
     * @param  x: The maximum X coordinate
     * @param  y: The maximum Y coordinate
     * @retval None
     */
    void setMaxCoordinates(uint16_t x, uint16_t y) override;

    /**
     * @brief  Gets the number of supported touch points
     * @note   This function retrieves the number of touch points supported by the touch driver.
     * @retval The number of supported touch points.
     */
    uint8_t getSupportTouchPoint() override;

    /**
     * @brief  Gets the touch point coordinates
     * @note   This function retrieves the touch point coordinates from the touch driver.
     * @param  &x: The X coordinate of the touch point
     * @param  &y: The Y coordinate of the touch point
     * @retval None
     */
    void getResolution(int16_t &x, int16_t &y) override;

        /**
     * @brief  Gets the touch point resolution
     * @note   This function retrieves the touch point resolution from the touch driver.
     * @retval None
     */
    int16_t getResolutionX() override;

    /**
     * @brief  Gets the touch point resolution
     * @note   This function retrieves the touch point resolution from the touch driver.
     * @retval None
     */
    int16_t getResolutionY() override;

// *INDENT-ON*

private:

    bool initImpl(uint8_t addr) override
    {
        return false;
    }

    using DriverCreator = std::unique_ptr<TouchDrvInterface> (*)();

    static constexpr uint8_t driverCreatorMaxNum = 4;
    static DriverCreator driverCreators[driverCreatorMaxNum];

    std::unique_ptr<TouchDrvInterface> createDriver(TouchDrvType type)
    {
        if (/*type >= TouchDrv_UNKNOWN &&*/
            type < sizeof(driverCreators) / sizeof(driverCreators[0])) {
            return driverCreators[type]();
        }
        return nullptr;
    }

    void setupDriver();

    SensorHalCustom::CustomWrite        _writePtr;
    SensorHalCustom::CustomRead         _readPtr;
    SensorHalCustom::CustomMode         _modePtr;
    TouchDrvType                        _touchType;
    std::unique_ptr<TouchDrvInterface>  _drv;
};

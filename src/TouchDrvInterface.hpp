/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted,free of charge,to any person obtaining a copy
 * of this software and associated documentation files (the "Software"),to deal
 * in the Software without restriction,including without limitation the rights
 * to use,copy,modify,merge,publish,distribute,sublicense,and/or sell
 * copies of the Software,and to permit persons to whom the Software is
 * furnished to do so,subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS",WITHOUT WARRANTY OF ANY KIND,EXPRESS OR
 * IMPLIED,INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,DAMAGES OR OTHER
 * LIABILITY,WHETHER IN AN ACTION OF CONTRACT,TORT OR OTHERWISE,ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      TouchDrvInterface.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-17
 *
 */
#pragma once

#include "SensorPlatform.hpp"
#include "touch/TouchPoints.hpp"

class TouchDrvInterface : public SensorHalCustom
{
public:
    using HomeButtonCallback = void(*)(void *user_data);

    /**
     * @brief  Constructor for the touch driver
     * @retval None
     */
    TouchDrvInterface() : comm(nullptr), hal(nullptr), _halModeCallback(nullptr),
        _halWriteCallback(nullptr), _halReadCallback(nullptr),
        _resX(0), _resY(0), _xMax(0), _yMax(0),
        _swapXY(false), _mirrorX(false), _mirrorY(false), _rst(-1), _irq(-1),
        _chipID(0x00), _HButtonCallback(nullptr), _userData(nullptr), _maxTouchPoints(0),
        _center_btn_x(-1), _center_btn_y(-1)
    {
    }

    /**
    * @brief  Destructor for the touch driver
    * @retval None
    */
    virtual ~TouchDrvInterface()
    {
        if (comm) {
            comm->deinit();
        }
    }

// *INDENT-OFF*
#if defined(ARDUINO)
    /**
     * @brief  Initializes the touch driver with the specified I2C port and address
     * @note   This function initializes the touch driver with the provided I2C port and address.
     * @param  &wire: The I2C wire object
     * @param  addr: The I2C address of the touch driver
     * @param  sda: The SDA pin number
     * @param  scl: The SCL pin number
     * @retval True if the initialization was successful, false otherwise.
     */
    virtual bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
            return false;
        }
        hal->setCustomMode(_halModeCallback);
        hal->setCustomWrite(_halWriteCallback);
        hal->setCustomRead(_halReadCallback);
        return initImpl(addr);
    }

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)

    /**
     * @brief  Initializes the touch driver with the specified I2C port and address
     * @note   This function initializes the touch driver with the provided I2C port and address.
     * @param  port_num: The I2C port number
     * @param  addr: The I2C address of the touch driver
     * @param  sda: The SDA pin number
     * @param  scl: The SCL pin number
     * @retval True if the initialization was successful, false otherwise.
     */
    virtual bool begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, addr, sda, scl)) {
            return false;
        }
        hal->setCustomMode(_halModeCallback);
        hal->setCustomWrite(_halWriteCallback);
        hal->setCustomRead(_halReadCallback);
        return initImpl(addr);
    }

#else

    /**
     * @brief  Initializes the touch driver with the specified I2C handle and address
     * @note   This function initializes the touch driver with the provided I2C handle and address.
     * @param  handle: The I2C master bus handle
     * @param  addr: The I2C address of the touch driver
     * @retval True if the initialization was successful, false otherwise.
     */
    virtual bool begin(i2c_master_bus_handle_t handle, uint8_t addr)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr)) {
            return false;
        }
        hal->setCustomMode(_halModeCallback);
        hal->setCustomWrite(_halWriteCallback);
        hal->setCustomRead(_halReadCallback);
        return initImpl(addr);
    }

#endif  //USEING_I2C_LEGACY
#endif  //ARDUINO

    /**
    * @brief  Initializes the touch driver with custom callbacks
    * @note   This function initializes the touch driver with the provided custom callbacks.
    * @param  callback: The custom callback for communication
    * @param  hal_callback: The custom callback for hardware abstraction layer
    * @param  addr: The I2C address of the touch driver
    * @retval True if the initialization was successful, false otherwise.
    */
    virtual bool begin(SensorCommCustom::CustomCallback callback,
                                SensorCommCustomHal::CustomHalCallback hal_callback,
                                uint8_t addr)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
                callback, hal_callback, addr, comm, hal)) {
            return false;
        }
        return initImpl(addr);
    }

    /**
     * @brief  Sets the GPIO callbacks for the touch driver
     * @note   This function sets the callbacks for the GPIO operations in the touch driver.
     * @param  mode_cb: The callback for the mode change
     * @param  write_cb: The callback for the write operation
     * @param  read_cb: The callback for the read operation
     * @retval None
     */
    virtual void setGpioCallback(CustomMode mode_cb, CustomWrite write_cb, CustomRead read_cb) {
        _halModeCallback  = mode_cb;
        _halWriteCallback = write_cb;
        _halReadCallback  = read_cb;
    }

    /**
     * @brief  Gets the number of supported touch points
     * @note   This function retrieves the number of touch points supported by the touch driver.
     * @retval The number of supported touch points.
     */
    virtual uint8_t getSupportTouchPoint() {
        return _maxTouchPoints;
    }

    /**
     * @brief  Gets the touch point coordinates
     * @note   This function retrieves the touch point coordinates from the touch driver.
     * @param  &x: The X coordinate of the touch point
     * @param  &y: The Y coordinate of the touch point
     * @retval None
     */
    virtual void getResolution(int16_t &x, int16_t &y) {
        x = _resX;
        y = _resY;
    }

    /**
     * @brief  Gets the touch point resolution
     * @note   This function retrieves the touch point resolution from the touch driver.
     * @retval None
     */
    virtual int16_t getResolutionX() {
        return _resX;
    }

    /**
     * @brief  Gets the touch point resolution
     * @note   This function retrieves the touch point resolution from the touch driver.
     * @retval None
     */
    virtual int16_t getResolutionY() {
        return _resY;
    }

    /**
     * @brief  Initializes the touch driver
     * @note   This function initializes the touch driver and prepares it for use.
     *         This function is virtual and needs to be implemented in subclasses.
     * @param  addr: The I2C address of the touch driver
     * @retval True if the initialization was successful, false otherwise.
     */
    virtual bool initImpl(uint8_t addr) = 0;

    /**
     * @brief  Resets the touch driver
     * @note   This function resets the touch driver to its initial state.
     *         This function is virtual and needs to be implemented in subclasses.
     * @retval None
     */
    virtual void reset() = 0;

    /**
     * @brief  Puts the touch driver to sleep
     * @note   This function puts the touch driver into sleep mode.
     *         This function is virtual and needs to be implemented in subclasses.
     *         If the device does not have a reset pin connected, it cannot be woken up after being put
     *         into sleep mode and must be powered on again.
     * @retval None
     */
    virtual void sleep() = 0;

    /**
    * @brief Wake up the touch driver
    * @note This function will wake the touch driver from sleep mode.
    *       This function is virtual and needs to be implemented in subclasses.
    * @retval None
    */
    virtual void wakeup() = 0;

    /**
    * @brief  Get the touch point coordinates
    * @note   This function will retrieve the touch point coordinates from the touch driver.
    *         This function is virtual and needs to be implemented in subclasses.
    * @param  *x_array: Pointer to the array to store the X coordinates
    * @param  *y_array: Pointer to the array to store the Y coordinates
    * @param  size: Number of touch points to retrieve
    * @retval Number of touch points retrieved
    */
    virtual uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point) __attribute__((deprecated("use getTouchPoints instead of getPoint")))
    {
        TouchPoints data = getTouchPoints();

        if (x_array == nullptr || y_array == nullptr) {
            return data.getPointCount();
        }
        if (data.hasPoints()) {
            uint8_t pointsToCopy = (get_point < data.getPointCount()) ? get_point : data.getPointCount();
            for (int i = 0; i < pointsToCopy; i++) {
               const TouchPoint &pt = data.getPoint(i);
                x_array[i] = pt.x;
                y_array[i] = pt.y;
            }
        }
        return data.getPointCount();
    }

    /**
     * @brief  Get the touch points
     * @note   This function retrieves the touch points from the touch driver.
     * @retval A reference to the touch points.
     */
    virtual const TouchPoints& getTouchPoints() = 0;

    /**
    * @brief  Check if the touch point is pressed
    * @note   This function will check if the touch point is currently pressed.
    * @retval True if the touch point is pressed, false otherwise.
    */
    virtual bool isPressed() = 0;

    /**
     * @brief  Get the model name
     * @note   This function will retrieve the model name from the touch driver.
     *         This function is virtual and needs to be implemented in subclasses.
     * @retval The model name.
     */
    virtual const char *getModelName() = 0;

    /**
    * @brief  Get the chip ID
    * @note   This function will retrieve the chip ID from the touch driver.
    * @retval The chip ID.
    */
    virtual uint32_t getChipID() {
        return _chipID;
    }

    /**
     * @brief  Set the reset and interrupt pins
     * @note   This function will set the reset and interrupt pins for the touch driver.
     * @param  rst: The reset pin number
     * @param  irq: The interrupt pin number
     * @retval None
     */
    virtual void setPins(int rst, int irq) {
        _rst = rst;
        _irq = irq;
    }

    /**
     * @brief  Set the swap XY flag
     * @note   This function will set the swap XY flag for the touch driver.
     * @param  swap: True to swap X and Y coordinates, false otherwise.
     * @retval None
     */
    virtual void setSwapXY(bool swap) {
        _swapXY = swap;
    }

    /**
     * @brief  Set the mirror XY flag
     * @note   This function will set the mirror XY flag for the touch driver.
     * @param  mirrorX: True to mirror X coordinates, false otherwise.
     * @param  mirrorY: True to mirror Y coordinates, false otherwise.
     * @retval None
     */
    virtual void setMirrorXY(bool mirrorX, bool mirrorY) {
        _mirrorX = mirrorX;
        _mirrorY = mirrorY;
    }

    /**
     * @brief  Set the maximum coordinates
     * @note   This function will set the maximum coordinates for the touch driver.
     * @param  x: The maximum X coordinate
     * @param  y: The maximum Y coordinate
     * @retval None
     */
    virtual void setMaxCoordinates(uint16_t x, uint16_t y) {
        _xMax = x;
        _yMax = y;
    }

    /**
     * @brief Applies coordinate transformations to all points in a TouchPoints object.
     *
     * Transformations are applied in the following order:
     * 1. Swap X and Y (if _swapXY is true)
     * 2. Mirror X (if _mirrorX is true) using _xMax as maximum X value
     * 3. Mirror Y (if _mirrorY is true) using _yMax as maximum Y value
     *
     * @param points The TouchPoints object to transform (modified in-place).
     */
    virtual void updateXY(TouchPoints &points) const
    {
        for (uint8_t i = 0; i < points.getPointCount(); ++i) {
            TouchPoint &pt = points.getPoint(i);

            // Swap XY
            if (_swapXY) {
                uint16_t tmp = pt.x;
                pt.x = pt.y;
                pt.y = tmp;
            }

            // Mirror X
            if (_mirrorX && _xMax > 0) {
                pt.x = _xMax - pt.x;
            }

            // Mirror Y
            if (_mirrorY && _yMax > 0) {
                pt.y = _yMax - pt.y;
            }
        }
    }

    /**
     * @brief  Set the home button callback
     * @note   This function will set the callback function for the home button.
     * @param  cb: The callback function to set
     * @param  *user_data: Pointer to user data to pass to the callback function
     * @retval None
     */
    virtual void setHomeButtonCallback(HomeButtonCallback cb, void *user_data) {
        _HButtonCallback = cb;
        _userData = user_data;
    }

    /**
     * @brief  Set the center button coordinate
     * @note   This function will set the center button coordinate for the touch driver.
     * @param  x: The X coordinate of the center button
     * @param  y: The Y coordinate of the center button
     * @retval None
     */
    virtual void setCenterButtonCoordinate(int16_t x, int16_t y) {
        _center_btn_x = x;
        _center_btn_y = y;
    }

// *INDENT-ON*

protected:
    std::unique_ptr <SensorCommBase> comm;
    std::unique_ptr <SensorHal> hal;
    CustomMode  _halModeCallback;
    CustomWrite _halWriteCallback;
    CustomRead  _halReadCallback;

    uint16_t _resX, _resY, _xMax, _yMax;
    bool _swapXY, _mirrorX, _mirrorY;
    int _rst;
    int _irq;
    uint32_t _chipID;
    HomeButtonCallback _HButtonCallback;
    void *_userData;
    uint8_t _maxTouchPoints;
    int16_t _center_btn_x;
    int16_t _center_btn_y;
};

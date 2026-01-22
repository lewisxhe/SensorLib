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

typedef struct {
    uint8_t  pos_id;
    uint8_t  event;
    uint16_t pos_x;
    uint16_t pos_y;
    uint8_t  pres_z;
} TouchPoint_t;

class TouchData
{
public:
    TouchData() {}
    ~TouchData() {};
    uint8_t available;
    uint8_t id[5];
    int16_t x[5];
    int16_t y[5];
    uint8_t  status[5];
    uint8_t  pressure[5];

    uint16_t getX(uint8_t index = 0)
    {
        return x[index];
    }
    uint16_t getY(uint8_t index = 0)
    {
        return y[index];
    }
    uint16_t getPressure(uint8_t index = 0)
    {
        return pressure[index];
    }
    uint16_t getStatus(uint8_t index = 0)
    {
        return status[index];
    }
};



class TouchDrvInterface : public SensorHalCustom
{
public:
    using HomeButtonCallback = void(*)(void *user_data);

    /**
     * @brief  Constructor for the touch driver
     * @retval None
     */
    TouchDrvInterface() : comm(nullptr), hal(nullptr),
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
    ~TouchDrvInterface()
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
    virtual bool begin(TwoWire &wire, uint8_t addr, int sda, int scl)
    {
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) {
            return false;
        }
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
        SensorHalCustom::setCustomMode(mode_cb);
        SensorHalCustom::setCustomWrite(write_cb);
        SensorHalCustom::setCustomRead(read_cb);
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
    * @retval None
    */
    virtual uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point) = 0;

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
     * @brief  Update the touch point coordinates
     * @note   This function will update the touch point coordinates in the driver.
     * @param  pointNum: The touch point number to update
     * @param  *xBuffer: Pointer to the buffer containing the new X coordinates
     * @param  *yBuffer: Pointer to the buffer containing the new Y coordinates
     * @retval None
     */
    virtual void updateXY(uint8_t pointNum, int16_t *xBuffer, int16_t *yBuffer) {
        if (!pointNum){
            return;
        }
        for (int i = 0; i < pointNum; ++i) {
            if (_swapXY) {
                uint16_t tmp = xBuffer[i];
                xBuffer[i] = yBuffer[i];
                yBuffer[i] = tmp;
            }
            if (_mirrorX && _xMax ) {
                xBuffer[i] = _xMax - xBuffer[i];
            }
            if (_mirrorY && _yMax) {
                yBuffer[i] = _yMax - yBuffer[i];
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
    std::unique_ptr < SensorCommBase > comm;
    std::unique_ptr < SensorHal > hal;

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

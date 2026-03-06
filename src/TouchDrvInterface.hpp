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

/**
 * @brief Abstract base class for touch screen drivers.
 *
 * This class provides a common interface for initializing, configuring,
 * and reading touch data from various touch controller chips.
 * It also handles coordinate transformations (swap, mirror, scaling) and
 * optional home button callbacks.
 */
class TouchDrvInterface : public SensorHalCustom
{
public:
    /**
     * @brief Callback type for home button events.
     * @param user_data Pointer to user-defined data passed during callback registration.
     */
    using HomeButtonCallback = void(*)(void *user_data);

    /**
     * @brief Construct a new TouchDrvInterface object.
     *
     * Initializes member variables to default values.
     * Does not allocate any hardware resources.
     */
    TouchDrvInterface() : comm(nullptr), hal(nullptr), _halModeCallback(nullptr),
        _halWriteCallback(nullptr), _halReadCallback(nullptr),
        _resX(0), _resY(0), _xMax(0), _yMax(0),
        _swapXY(false), _mirrorX(false), _mirrorY(false), _rst(-1), _irq(-1),
        _chipID(0x00), _HButtonCallback(nullptr), _userData(nullptr), _maxTouchPoints(0),
        _center_btn_x(-1), _center_btn_y(-1), _scaleX(1.0f), _scaleY(1.0f), _scalingEnabled(false)
    {
    }

    /**
     * @brief Destroy the TouchDrvInterface object.
     *
     * Deinitializes the communication interface if it was created.
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
     * @brief Initialize the touch driver using Arduino I2C (TwoWire).
     *
     * @param wire Reference to the Arduino TwoWire I2C object.
     * @param addr I2C slave address of the touch controller.
     * @param sda SDA pin number (optional, default -1 means use default Wire pins).
     * @param scl SCL pin number (optional, default -1 means use default Wire pins).
     * @retval true  Initialization successful.
     * @retval false Initialization failed.
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
     * @brief Initialize the touch driver using ESP-IDF legacy I2C driver.
     *
     * @param port_num I2C port number (e.g., I2C_NUM_0).
     * @param addr I2C slave address of the touch controller.
     * @param sda SDA pin number.
     * @param scl SCL pin number.
     * @retval true  Initialization successful.
     * @retval false Initialization failed.
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
     * @brief Initialize the touch driver using ESP-IDF I2C master bus handle (new driver).
     *
     * @param handle I2C master bus handle obtained from `i2c_new_master_bus()`.
     * @param addr I2C slave address of the touch controller.
     * @retval true  Initialization successful.
     * @retval false Initialization failed.
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
     * @brief Initialize the touch driver with custom communication callbacks.
     *
     * Use this method when you need to provide your own low-level I2C/SPI-like
     * communication functions (e.g., for non-standard buses or simulated interfaces).
     *
     * @param callback Custom callback for basic communication operations.
     * @param hal_callback Custom callback for hardware abstraction layer (GPIO) operations.
     * @param addr I2C slave address (ignored if not applicable).
     * @retval true  Initialization successful.
     * @retval false Initialization failed.
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
     * @brief Set custom GPIO callbacks for pin control (e.g., reset, interrupt).
     *
     * These callbacks will be used by the HAL to manipulate GPIO pins.
     * Must be called before `begin()` if custom GPIO handling is required.
     *
     * @param mode_cb Callback to set pin mode (input/output).
     * @param write_cb Callback to write a digital value to a pin.
     * @param read_cb Callback to read a digital value from a pin.
     */
    virtual void setGpioCallback(CustomMode mode_cb, CustomWrite write_cb, CustomRead read_cb) {
        _halModeCallback  = mode_cb;
        _halWriteCallback = write_cb;
        _halReadCallback  = read_cb;
    }

    /**
     * @brief Get the maximum number of touch points supported by the hardware.
     *
     * @return uint8_t Maximum number of simultaneous touches.
     */
    virtual uint8_t getSupportTouchPoint() {
        return _maxTouchPoints;
    }

    /**
     * @brief Get the raw resolution (maximum coordinate values) of the touch panel.
     *
     * @param[out] x Reference to store the maximum X coordinate.
     * @param[out] y Reference to store the maximum Y coordinate.
     */
    virtual void getResolution(int16_t &x, int16_t &y) {
        x = _resX;
        y = _resY;
    }

    /**
     * @brief Get the raw maximum X coordinate of the touch panel.
     *
     * @return int16_t Maximum X coordinate.
     */
    virtual int16_t getResolutionX() {
        return _resX;
    }

    /**
     * @brief Get the raw maximum Y coordinate of the touch panel.
     *
     * @return int16_t Maximum Y coordinate.
     */
    virtual int16_t getResolutionY() {
        return _resY;
    }

    /**
     * @brief Device-specific initialization implementation (pure virtual).
     *
     * This method must be implemented by derived classes to perform
     * hardware-specific initialization, such as reading chip ID,
     * configuring registers, and setting up default behavior.
     *
     * @param addr I2C address (or other bus address) of the device.
     * @retval true  Initialization successful.
     * @retval false Initialization failed.
     */
    virtual bool initImpl(uint8_t addr) = 0;

    /**
     * @brief Reset the touch controller (pure virtual).
     *
     * Typically toggles the reset pin (if connected) or sends a software reset command.
     */
    virtual void reset() = 0;

    /**
     * @brief Put the touch controller into low-power sleep mode (pure virtual).
     *
     * @note If the device does not have a hardware reset pin connected,
     *       it may not be possible to wake it from sleep without power cycling.
     */
    virtual void sleep() = 0;

    /**
     * @brief Wake the touch controller from sleep mode (pure virtual).
     */
    virtual void wakeup() = 0;

    /**
     * @brief Retrieve touch point coordinates (deprecated).
     *
     * @deprecated Use `getTouchPoints()` instead, which returns a richer TouchPoints object.
     *
     * @param[out] x_array Pointer to array to store X coordinates.
     * @param[out] y_array Pointer to array to store Y coordinates.
     * @param get_point Number of points to retrieve (max).
     * @return uint8_t Actual number of touch points detected.
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
     * @brief Get the current touch points (pure virtual).
     *
     * @return const TouchPoints& Reference to an object containing all detected touch points,
     *         their coordinates, and additional metadata.
     */
    virtual const TouchPoints& getTouchPoints() = 0;

    /**
     * @brief Check if any touch point is currently pressed.
     *
     * @retval true  At least one touch point detected.
     * @retval false No touch detected.
     */
    virtual bool isPressed() = 0;

    /**
     * @brief Get the human-readable model name of the touch controller.
     *
     * @return const char* String describing the chip model (e.g., "FT6336").
     */
    virtual const char *getModelName() = 0;

    /**
     * @brief Get the chip identifier (e.g., ID register value).
     *
     * @return uint32_t Chip ID.
     */
    virtual uint32_t getChipID() {
        return _chipID;
    }

    /**
     * @brief Set the hardware reset and interrupt pin numbers.
     *
     * These pins will be used by the driver for hardware control.
     * Must be called before `begin()` if the pins are connected.
     *
     * @param rst Reset pin number (negative value if not used).
     * @param irq Interrupt pin number (negative value if not used).
     */
    virtual void setPins(int rst, int irq) {
        _rst = rst;
        _irq = irq;
    }

    /**
     * @brief Enable or disable swapping of X and Y coordinates.
     *
     * @param swap true to swap X and Y, false to keep original order.
     */
    virtual void setSwapXY(bool swap) {
        _swapXY = swap;
    }

    /**
     * @brief Enable or disable mirroring of X and/or Y coordinates.
     *
     * Mirroring is applied after scaling, using the maximum coordinates
     * set via `setMaxCoordinates()` or `setTargetResolution()`.
     *
     * @param mirrorX true to mirror X (flip horizontally).
     * @param mirrorY true to mirror Y (flip vertically).
     */
    virtual void setMirrorXY(bool mirrorX, bool mirrorY) {
        _mirrorX = mirrorX;
        _mirrorY = mirrorY;
    }

    /**
     * @brief Set the maximum coordinate values used for mirroring.
     *
     * These values define the reference range when mirroring is enabled.
     * If you also use scaling, consider using `setTargetResolution()` instead,
     * which automatically sets these maxima.
     *
     * @param x Maximum X coordinate (e.g., display width).
     * @param y Maximum Y coordinate (e.g., display height).
     */
    virtual void setMaxCoordinates(uint16_t x, uint16_t y) {
        _xMax = x;
        _yMax = y;
    }

    /**
     * @brief Set the raw resolution of the touch panel (physical maximum coordinates).
     *
     * This should be called if the touch controller does not provide its resolution automatically.
     * These values are used to compute scaling factors when `setTargetResolution()` is called.
     *
     * @param width  Maximum X coordinate reported by the touch controller.
     * @param height Maximum Y coordinate reported by the touch controller.
     */
    virtual void setResolution(uint16_t width, uint16_t height) {
        _resX = width;
        _resY = height;
    }

    /**
     * @brief Set the target display resolution to map touch coordinates to screen pixels.
     *
     * This function calculates scaling factors based on the previously set raw resolution
     * (via `setResolution()`). If raw resolution is not set (both zero), scaling is disabled.
     * It also stores the target dimensions as maximum coordinates for mirroring.
     *
     * @param width  Target display width in pixels.
     * @param height Target display height in pixels.
     */
    virtual void setTargetResolution(uint16_t width, uint16_t height) {
        if (_resX == 0 || _resY == 0) {
            _scaleX = _scaleY = 1.0f;
            _scalingEnabled = false;
        } else {
            _scaleX = (float)width / _resX;
            _scaleY = (float)height / _resY;
            _scalingEnabled = true;
        }
        _xMax = width;
        _yMax = height;
    }

    /**
     * @brief Apply coordinate transformations (swap, scale, mirror) to all touch points.
     *
     * Transformations are applied in the following order:
     * 1. Swap X and Y (if `_swapXY` is true).
     * 2. Scale from raw to target resolution (if scaling enabled).
     * 3. Mirror X (if `_mirrorX` true) using `_xMax` as the maximum X value.
     * 4. Mirror Y (if `_mirrorY` true) using `_yMax` as the maximum Y value.
     * 5. Clamp coordinates to `[0, _xMax]` and `[0, _yMax]`.
     *
     * @param points The TouchPoints object to transform (modified in-place).
     */
    virtual void updateXY(TouchPoints &points) const
    {
        uint8_t count = points.getPointCount();
        if (count == 0){
            return;
        }
        for (uint8_t i = 0; i < count; ++i) {
            TouchPoint &pt = points.getPoint(i);

            // Swap XY
            if (_swapXY) {
                uint16_t tmp = pt.x;
                pt.x = pt.y;
                pt.y = tmp;
            }

            // Apply scaling if target resolution set
            if (_scalingEnabled) {
                pt.x = static_cast<uint16_t>(pt.x * _scaleX + 0.5f);
                pt.y = static_cast<uint16_t>(pt.y * _scaleY + 0.5f);
            }

            // Mirror X
            if (_mirrorX && _xMax > 0) {
                pt.x = _xMax - pt.x;
            }

            // Mirror Y
            if (_mirrorY && _yMax > 0) {
                pt.y = _yMax - pt.y;
            }

            // Clamp to display bounds
            if(_xMax != 0) {
                if (pt.x > _xMax) {
                    pt.x = _xMax;
                }
            }
            if(_yMax != 0) {
                if (pt.y > _yMax) {
                    pt.y = _yMax;
                }
            }
        }
    }

    /**
     * @brief Register a callback for home button events.
     *
     * The callback will be invoked when a touch event occurs near the center button area
     * (coordinates set via `setCenterButtonCoordinate()`).
     *
     * @param cb Function pointer to the callback.
     * @param user_data Pointer to user-defined data passed to the callback.
     */
    virtual void setHomeButtonCallback(HomeButtonCallback cb, void *user_data) {
        _HButtonCallback = cb;
        _userData = user_data;
    }

    /**
     * @brief Set the expected screen coordinates of a hardware home button.
     *
     * If a touch occurs within a small region around these coordinates,
     * and a home button callback is registered, the callback will be triggered.
     *
     * @param x X coordinate of the button center.
     * @param y Y coordinate of the button center.
     */
    virtual void setCenterButtonCoordinate(int16_t x, int16_t y) {
        _center_btn_x = x;
        _center_btn_y = y;
    }

// *INDENT-ON*

protected:
    std::unique_ptr<SensorCommBase> comm;        ///< Communication interface (I2C/SPI/etc.)
    std::unique_ptr<SensorHal> hal;              ///< Hardware abstraction layer for GPIO
    CustomMode  _halModeCallback;                ///< Custom GPIO mode callback
    CustomWrite _halWriteCallback;               ///< Custom GPIO write callback
    CustomRead  _halReadCallback;                ///< Custom GPIO read callback

    uint16_t _resX, _resY;        ///< Raw touch panel resolution (max X, max Y)
    uint16_t _xMax, _yMax;        ///< Maximum coordinates used for mirroring/clamping
    bool _swapXY;                 ///< Flag to swap X and Y
    bool _mirrorX, _mirrorY;      ///< Flags to mirror X and Y axes
    int _rst;                     ///< Reset pin number (-1 if not used)
    int _irq;                     ///< Interrupt pin number (-1 if not used)
    uint32_t _chipID;              ///< Chip identification value
    HomeButtonCallback _HButtonCallback; ///< Home button callback function
    void *_userData;               ///< User data for home button callback
    uint8_t _maxTouchPoints;       ///< Maximum supported touch points
    int16_t _center_btn_x;         ///< X coordinate of home button center
    int16_t _center_btn_y;         ///< Y coordinate of home button center
    float _scaleX, _scaleY;        ///< Scaling factors for X and Y
    bool _scalingEnabled;           ///< Flag indicating whether scaling is active
};

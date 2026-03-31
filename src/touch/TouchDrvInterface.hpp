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

#include "platform/comm/I2CDeviceWithHal.hpp"
#include "TouchPoints.hpp"

/**
 * @brief Abstract base class for touch screen drivers.
 *
 * This class provides a common interface for initializing, configuring,
 * and reading touch data from various touch controller chips.
 * It also handles coordinate transformations (swap, mirror, scaling) and
 * optional home button callbacks.
 */
class TouchDrvInterface : public I2CDeviceWithHal
{
public:

    struct TouchPinsCfg {
        int rstPin;             // Reset pin number, -1 if not used
        int irqPin;             // Interrupt pin number, -1 if not used
        int rstActiveLevel;     // Active level for reset pin (HIGH or LOW)
        int irqTriggerLevel;    // Trigger level for interrupt pin (HIGH or LOW)
        int rstHoldTimeMs;      // Minimum hold time for reset signal in milliseconds
        int rstReleaseTimeMs;   // Minimum release time after reset in milliseconds
    };

    struct TouchConfig {
        bool swapXY;            // Whether to swap X and Y coordinates
        bool mirrorX;           // Whether to mirror X coordinates
        bool mirrorY;           // Whether to mirror Y coordinates
        bool scalingEnabled;    // Whether scaling is enabled
        uint16_t xMax;          // Maximum X coordinate value from the hardware
        uint16_t yMax;          // Maximum Y coordinate value from the hardware
        uint16_t resolutionX;   // Physical resolution of the touch panel (used for scaling)
        uint16_t resolutionY;   // Physical resolution of the touch panel (used for scaling)
        float scaleX;           // Scaling factor for X coordinates
        float scaleY;           // Scaling factor for Y coordinates
    };

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
    TouchDrvInterface() : _halModeCallback(nullptr),
        _halWriteCallback(nullptr), _halReadCallback(nullptr),
        _chipID(0x00), _HButtonCallback(nullptr), _userData(nullptr), _maxTouchPoints(0),
        _center_btn_x(-1), _center_btn_y(-1),
        _lastPulse(0),  _pinsCfg({-1, -1, LOW, LOW, 10, 10}),
               _touchConfig({false, false, false, false, 0, 0, 0, 0, 1.0f, 1.0f})
    {
    }

    /**
     * @brief Destroy the TouchDrvInterface object.
     *
     * Deinitializes the communication interface if it was created.
     */
    virtual ~TouchDrvInterface() = default;

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
        return I2CDeviceWithHal::begin(wire, addr, sda, scl);
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
        return I2CDeviceWithHal::begin(port_num, addr, sda, scl);
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
        return I2CDeviceWithHal::begin(handle, addr);
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
        return I2CDeviceWithHal::begin(callback, hal_callback, addr);
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
    virtual void setGpioCallback(SensorHalCustom::CustomMode mode_cb, 
                                SensorHalCustom::CustomWrite write_cb, 
                                SensorHalCustom::CustomRead read_cb) {
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
    virtual void getResolution(uint16_t &x, uint16_t &y) {
        x = _touchConfig.resolutionX;
        y = _touchConfig.resolutionY;
    }

    /**
     * @brief Get the raw maximum X coordinate of the touch panel.
     *
     * @return uint16_t Maximum X coordinate.
     */
    virtual uint16_t getResolutionX() {
        return _touchConfig.resolutionX;
    }

    /**
     * @brief Get the raw maximum Y coordinate of the touch panel.
     *
     * @return uint16_t Maximum Y coordinate.
     */
    virtual uint16_t getResolutionY() {
        return _touchConfig.resolutionY;
    }

    /**
     * @brief Device-specific initialization implementation (pure virtual).
     *
     * This method must be implemented by derived classes to perform
     * hardware-specific initialization, such as reading chip ID,
     * configuring registers, and setting up default behavior.
     *
     * @param param Reserved parameters for future use (e.g., I2C address, configuration flags).
     * @retval true  Initialization successful.
     * @retval false Initialization failed.
     */
    virtual bool initImpl(uint8_t param) = 0;

    /**
     * @brief Reset the touch controller.
     *
     * Typically toggles the reset pin (if connected) or sends a software reset command.
     */
    virtual void reset()
    {
        if (_pinsCfg.rstPin != -1) {
            // Perform hardware reset using the reset pin
            hal->digitalWrite(_pinsCfg.rstPin, _pinsCfg.rstActiveLevel);
            hal->delay(_pinsCfg.rstHoldTimeMs);
            hal->digitalWrite(_pinsCfg.rstPin, !_pinsCfg.rstActiveLevel);
            hal->delay(_pinsCfg.rstReleaseTimeMs);
        }
        // If no reset pin is available, derived classes can override this method to implement a software reset if supported.
    }

    /**
     * @brief Put the touch controller into low-power sleep mode (pure virtual).
     *
     * @note If the device does not have a hardware reset pin connected,
     *       it may not be possible to wake it from sleep without power cycling.
     */
    virtual void sleep() = 0;

    /**
     * @brief Wake the touch controller from sleep mode (pure virtual).
     * @note Most touch devices cannot be woken up by commands when they are in deep sleep; 
     *       they are usually woken up by pulling the reset button low. 
     *       This method can be overridden by subclasses, and the default is to call the reset method.
     */
    virtual void wakeup()
    {
        reset();
    }

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
     * @note  The `isPressed` method is suitable for screens with an interrupt pin connected, using the interrupt level 
     * to detect whether a touch is pressed. For devices without an interrupt pin connected, simply call `getTouchPoints`.
     * @param filter_ms Optional debounce/filter time in milliseconds. If > 0, 
     * the function will only return true if a touch has been detected continuously for at least this duration.
     * @retval true  At least one touch point detected.
     * @retval false No touch detected.
     */
    virtual bool isPressed(uint32_t filter_ms = 0)
    {
        if (_pinsCfg.irqPin != -1) {
            if (hal->digitalRead(_pinsCfg.irqPin) == _pinsCfg.irqTriggerLevel) {
                if(filter_ms == 0){
                    return true;
                }
                uint32_t now = hal->millis();
                if (now - _lastPulse > filter_ms) {
                    _lastPulse = now;
                    return true;
                }
            }
            return false;
        }
        return getTouchPoints().hasPoints();
    }

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
        _pinsCfg.irqPin = irq;
        _pinsCfg.rstPin = rst;
    }

    /**
     * @brief Get the current touch pins configuration.
     *
     * @return const TouchPinsCfg& Reference to the touch pins configuration.
     */
    virtual const TouchPinsCfg& getTouchPinsCfg() const { return _pinsCfg; }

    /**
     * @brief Enable or disable swapping of X and Y coordinates.
     *
     * @param swap true to swap X and Y, false to keep original order.
     */
    virtual void setSwapXY(bool swap) {
        _touchConfig.swapXY = swap;
    }

    /**
     * @brief Check if X and Y coordinates are swapped.
     *
     * @return true if coordinates are swapped, false otherwise.
     */
    virtual bool isSwapXY() const { return _touchConfig.swapXY; }

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
        _touchConfig.mirrorX = mirrorX;
        _touchConfig.mirrorY = mirrorY;
    }

    /**
     * @brief Check if X coordinates are mirrored.
     *
     * @return true if X is mirrored, false otherwise.
     */
    virtual bool isMirrorX() const { return _touchConfig.mirrorX; }

    /**
     * @brief Check if Y coordinates are mirrored.
     *
     * @return true if Y is mirrored, false otherwise.
     */
    virtual bool isMirrorY() const { return _touchConfig.mirrorY; }

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
        _touchConfig.xMax = x;
        _touchConfig.yMax = y;
    }

    /**
     * @brief  Get the maximum coordinates used for mirroring.
     * @param  &x: Reference to store the maximum X coordinate.
     * @param  &y: Reference to store the maximum Y coordinate.
     * @retval None
     */
    virtual void getMaxCoordinates(uint16_t &x, uint16_t &y) {
        x = _touchConfig.xMax;
        y = _touchConfig.yMax;
    }

    /**
     * @brief  Get the maximum X coordinate (e.g., display width).
     * @note   
     * @retval Maximum X coordinate.
     */
    virtual uint16_t getMaxX() { return _touchConfig.xMax; }

    /**
     * @brief  Get the maximum Y coordinate (e.g., display height).
     * @retval Maximum Y coordinate.
     */
    virtual uint16_t getMaxY() { return _touchConfig.yMax; }

    /**
     * @brief  Check if coordinate scaling is enabled.
     * @retval true if scaling is enabled, false otherwise.
     */
    virtual bool isScalingEnabled() { return _touchConfig.scalingEnabled; }

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
        _touchConfig.resolutionX = width;
        _touchConfig.resolutionY = height;
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
        if (_touchConfig.resolutionX == 0 || _touchConfig.resolutionY == 0) {
            _touchConfig.scaleX = _touchConfig.scaleY = 1.0f;
            _touchConfig.scalingEnabled = false;
        } else {
            _touchConfig.scaleX = (float)width / _touchConfig.resolutionX;
            _touchConfig.scaleY = (float)height / _touchConfig.resolutionY;
            _touchConfig.scalingEnabled = true;
        }
        _touchConfig.xMax = width;
        _touchConfig.yMax = height;
    }

    /**
     * @brief Apply coordinate transformations (swap, scale, mirror) to all touch points.
     *
     * Transformations are applied in the following order:
     * 1. Swap X and Y (if `_touchConfig.swapXY` is true).
     * 2. Scale from raw to target resolution (if `_touchConfig.scalingEnabled` is true).
     * 3. Mirror X (if `_touchConfig.mirrorX` true) using `_touchConfig.xMax` as the maximum X value.
     * 4. Mirror Y (if `_touchConfig.mirrorY` true) using `_touchConfig.yMax` as the maximum Y value.
     * 5. Clamp coordinates to `[0, _touchConfig.xMax]` and `[0, _touchConfig.yMax]`.
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
            if (_touchConfig.swapXY) {
                uint16_t tmp = pt.x;
                pt.x = pt.y;
                pt.y = tmp;
            }

            // Apply scaling if target resolution set
            if (_touchConfig.scalingEnabled) {
                pt.x = static_cast<uint16_t>(pt.x * _touchConfig.scaleX + 0.5f);
                pt.y = static_cast<uint16_t>(pt.y * _touchConfig.scaleY + 0.5f);
            }

            // Mirror X
            if (_touchConfig.mirrorX && _touchConfig.xMax > 0) {
                pt.x = _touchConfig.xMax - pt.x;
            }

            // Mirror Y
            if (_touchConfig.mirrorY && _touchConfig.yMax > 0) {
                pt.y = _touchConfig.yMax - pt.y;
            }

            // Clamp to display bounds
            if(_touchConfig.xMax != 0) {
                if (pt.x > _touchConfig.xMax) {
                    pt.x = _touchConfig.xMax;
                }
            }
            if(_touchConfig.yMax != 0) {
                if (pt.y > _touchConfig.yMax) {
                    pt.y = _touchConfig.yMax;
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
     * @note This feature is hardware-dependent and not all touch panels support it. 
     *       It's only available on specific hardware. 
     *       Please refer to the sample program for your specific model for details.
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
    
    SensorHalCustom::CustomMode  _halModeCallback;                ///< Custom GPIO mode callback
    SensorHalCustom::CustomWrite _halWriteCallback;               ///< Custom GPIO write callback
    SensorHalCustom::CustomRead  _halReadCallback;                ///< Custom GPIO read callback

    uint32_t _chipID;              ///< Chip identification value
    HomeButtonCallback _HButtonCallback; ///< Home button callback function
    void *_userData;               ///< User data for home button callback
    uint8_t _maxTouchPoints;       ///< Maximum supported touch points
    int16_t _center_btn_x;         ///< X coordinate of home button center
    int16_t _center_btn_y;         ///< Y coordinate of home button center
    uint32_t _lastPulse;           ///< Timestamp of the last touch event
    TouchPinsCfg _pinsCfg;        ///< Configuration for reset and interrupt pins
    TouchConfig _touchConfig;        ///< Configuration for coordinate transformations
    TouchPoints _touchPoints;      ///< Touch points data

private:
    /**
     * @brief Called after communication with the touch controller is ready.
     *
     * This function sets up custom GPIO callbacks and initializes GPIO pins.
     */
    virtual void afterCommReady() override
    {
        if (_halModeCallback) {
            hal->setCustomMode(_halModeCallback);
        }
        if (_halWriteCallback) {
            hal->setCustomWrite(_halWriteCallback);
        }
        if (_halReadCallback) {
            hal->setCustomRead(_halReadCallback);
        }
        initGpioPins();
    }

    /**
     * @brief Initialize GPIO pins for reset and interrupt.
     *
     * Configures the reset pin as OUTPUT and the interrupt pin as INPUT based on the current configuration.
     * Also performs an initial reset of the touch controller if a reset pin is configured.
     */
    virtual void initGpioPins()
    {
        if (_pinsCfg.rstPin != -1) {
            hal->pinMode(_pinsCfg.rstPin, OUTPUT);
        }
        if (_pinsCfg.irqPin != -1) {
            hal->pinMode(_pinsCfg.irqPin, INPUT);
        }
        reset();
    }
};

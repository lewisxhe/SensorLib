/**
 *
 * @license MIT License
 *
 * Copyright (c) 2026 lewis he
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
 * @file      I2CDeviceWithHal.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-10
 *
 */
#pragma once
#include "DeviceBeginCommon.hpp"

class I2CDeviceWithHal : public DeviceBeginCommon
{
public:
#if defined(ARDUINO)
    /**
     * @brief  Initialization using the Arduino Wire Interface
     * @note   This function sets up the I2C communication parameters for the sensor.
     * @param  &wire: Reference to the TwoWire I2C interface.
     * @param  addr: I2C address of the sensor.
     * @param  sda: SDA pin number, default is -1 (use board default)
     * @param  scl: SCL pin number, default is -1 (use board default)
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1)
    {
        beforeBegin();
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, addr, sda, scl)) return false;
        _addr = addr; _iface = COMM_I2C;
        afterCommReady();
        if (!initImpl(_addr)) {
            return fail();
        }
        afterInitSuccess(_addr);
        return true;
    }

#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)
    /**
     * @brief  Initialization using the ESP-IDF I2C Legacy Interface
     * @param  port_num: I2C port number.
     * @param  addr: I2C address of the sensor.
     * @param  sda: SDA pin number, default is -1 (use board default)
     * @param  scl: SCL pin number, default is -1 (use board default)
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(i2c_port_t port, uint8_t addr, int sda = -1, int scl = -1)
    {
        beforeBegin();
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port, addr, sda, scl)) return false;
        _addr = addr; _iface = COMM_I2C;
        afterCommReady();
        if (!initImpl(_addr)) {
            return fail();
        }
        afterInitSuccess(_addr);
        return true;
    }
#else
    /**
     * @brief  Initialization using the ESP-IDF I2C LL Interface idf version > 5.0.0
     * @param  handle: I2C master bus handle.
     * @param  addr: I2C address of the sensor.
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr)
    {
        beforeBegin();
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, addr)) return false;
        _addr = addr; _iface = COMM_I2C;
        afterCommReady();
        if (!initImpl(_addr)) {
            return fail();
        }
        afterInitSuccess(_addr);
        return true;
    }
#endif  //USEING_I2C_LEGACY
#endif  //ARDUINO
    /**
     * @brief Initialize the sensor using custom callback interface.
     * @note Suitable for other platforms not covered by standard implementations.
     * @param interface Communication interface type (COMM_SPI or COMM_I2C).
     * @param callback Register read/write callback function.
     * @param hal_callback Platform digital IO and delay callback function.
     * @param addr Device address
     * @return True if initialization is successful, false otherwise.
     */
    bool begin(SensorCommCustom::CustomCallback cb,
               SensorCommCustomHal::CustomHalCallback hal_cb,
               uint8_t addr)
    {
        beforeBegin();
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM, cb, hal_cb, addr, comm, hal))
            return false;
        _addr = addr; _iface = COMM_CUSTOM;
        afterCommReady();
        if (!initImpl(_addr)) {
            return fail();
        }
        afterInitSuccess(_addr);
        return true;
    }
protected:
    /**
    * @brief Ensure the communication interface is valid.
    *  @return True if valid, false otherwise.
    */
    bool ensureValid() const override
    {
        if (!comm || !hal) return false;
        return true;
    }
};
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
 * @file      SPIDeviceWithHal.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-10
 *
 */
#pragma once
#include "DeviceBeginCommon.hpp"

class SPIDeviceWithHal : public DeviceBeginCommon
{
public:
#if defined(ARDUINO)
    /**
    * @brief Initialize using Arduino SPI interface.
    * @param spi Reference to SPIClass object.
    * @param csPin SPI chip select pin number.
    * @param mosi SPI MOSI pin number (optional, uses default if -1).
    * @param miso SPI MISO pin number (optional, uses default if -1).
    * @param sck SPI clock pin number (optional, uses default if -1).
    * @return True if initialization is successful, false otherwise.
    */
    bool begin(SPIClass &spi, uint8_t csPin, int mosi = -1, int miso = -1, int sck = -1)
    {
        beforeBegin();
        if (!beginCommon<SensorCommSPI, HalArduino>(comm, hal, spi, csPin, mosi, miso, sck)) {
            return fail();
        }
        _addr = 0;
        _iface = COMM_SPI;
        afterCommReady();
        if (!initImpl(_iface)) {
            return fail();
        }
        afterInitSuccess(_iface);
        return true;
    }
#elif defined(ESP_PLATFORM)
    /**
     * @brief  Initialization using the ESP-IDF I2C LL Interface idf version > 5.0.0
     * @param  handle: I2C master bus handle.
     * @param  addr: I2C address of the sensor.
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(spi_host_device_t host, spi_device_handle_t handle,
               uint8_t csPin, int mosi = -1, int miso = -1, int sck = -1)
    {
        beforeBegin();
        if (!beginCommon<SensorCommSPI, HalEspIDF>(comm, hal, host, handle, csPin, mosi, miso, sck)) {
            return fail();
        }
        _addr = 0;
        _iface = COMM_SPI;
        afterCommReady();
        if (!initImpl(_iface)) {
            return fail();
        }
        afterInitSuccess(_iface);
        return true;
    }
#endif
    /**
     * @brief Initialize the sensor using custom callback interface.
     * @note Suitable for other platforms not covered by standard implementations.
     * @param interface Communication interface type (COMM_SPI or COMM_I2C).
     * @param callback Register read/write callback function.
     * @param hal_callback Platform digital IO and delay callback function.
     * @return True if initialization is successful, false otherwise.
     */
    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback)
    {
        beforeBegin();
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(
                    COMM_CUSTOM, callback, hal_callback, 0x00, comm, hal)) {
            return fail();
        }
        _addr = 0x00;
        _iface = COMM_CUSTOM;
        afterCommReady();
        if (!initImpl(_iface)) {
            return fail();
        }
        afterInitSuccess(_iface);
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

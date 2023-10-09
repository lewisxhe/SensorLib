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
 * @file      SensorBMM150.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-09
 * @note      Most source code references come from the https://github.com/boschsensortec/BMM150-Sensor-API
 *            Simplification for Arduino
 */

#include "bosch/common/bosch_interfaces.h"
#include "bosch/BMM150/bmm150.h"
#include "bosch/SensorBhy2Define.h"

#if !defined(ARDUINO)
#error "Currently only supports Arduino"
#endif

/*! @name I2C ADDRESS       */
#define BMM150_DEFAULT_I2C_ADDRESS                UINT8_C(0x10)
#define BMM150_I2C_ADDRESS_CSB_LOW_SDO_HIGH       UINT8_C(0x11)
#define BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW       UINT8_C(0x12)
#define BMM150_I2C_ADDRESS_CSB_HIGH_SDO_HIGH      UINT8_C(0x13)


class SensorBMM150
{
    friend class BoschParse;
public:
    SensorBMM150(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BMM150_DEFAULT_I2C_ADDRESS)
    {
        __handler.u.i2c_dev.scl = scl;
        __handler.u.i2c_dev.sda = sda;
        __handler.u.i2c_dev.addr = addr;
        __handler.u.i2c_dev.wire = &w;
        __handler.intf = SENSORLIB_I2C_INTERFACE;
    }

    SensorBMM150(int cs, int mosi = -1, int miso = -1, int sck = -1, PLATFORM_SPI_TYPE &spi = SPI )
    {
        __handler.u.spi_dev.cs = cs;
        __handler.u.spi_dev.miso = miso;
        __handler.u.spi_dev.mosi = mosi;
        __handler.u.spi_dev.sck = sck;
        __handler.u.spi_dev.spi = &spi;
        __handler.intf = SENSORLIB_SPI_INTERFACE;
    }

    ~SensorBMM150()
    {
        deinit();
    }

    SensorBMM150()
    {
        memset(&__handler, 0, sizeof(__handler));
    }

    void setPins(int rst, int irq)
    {
        __handler.irq = irq;
        __handler.rst = rst;

    }

    bool init(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BMM150_DEFAULT_I2C_ADDRESS)
    {
        __handler.u.i2c_dev.scl = scl;
        __handler.u.i2c_dev.sda = sda;
        __handler.u.i2c_dev.addr = addr;
        __handler.u.i2c_dev.wire = &w;
        __handler.intf = SENSORLIB_I2C_INTERFACE;
        return initImpl();
    }

    bool init( PLATFORM_SPI_TYPE &spi, int cs, int mosi = MOSI, int miso = MISO, int sck = SCK)
    {
        __handler.u.spi_dev.cs = cs;
        __handler.u.spi_dev.miso = miso;
        __handler.u.spi_dev.mosi = mosi;
        __handler.u.spi_dev.sck = sck;
        __handler.u.spi_dev.spi = &spi;
        __handler.intf = SENSORLIB_SPI_INTERFACE;
        return initImpl();
    }

    bool init()
    {
        return initImpl();
    }

    void deinit()
    {
        if (dev) {
            free(dev);
            dev = NULL;
        }
        if (__handler.irq != SENSOR_PIN_NONE) {
            detachInterrupt(__handler.irq);
        }
    }

    void reset()
    {
        if (__handler.rst != SENSOR_PIN_NONE) {
            digitalWrite(__handler.rst, HIGH);
            delay(5);
            digitalWrite(__handler.rst, LOW);
            delay(10);
            digitalWrite(__handler.rst, HIGH);
            delay(5);
        }
    }


private:

    static void IRAM_ATTR handleISR(void *available)
    {
        *(bool *)(available) = true;
    }


    bool initImpl()
    {
        int8_t rslt;
        uint8_t product_id = 0;

        if (__handler.rst != SENSOR_PIN_NONE) {
            pinMode(__handler.rst, OUTPUT);
        }

        reset();

        dev = (struct bmm150_dev *)malloc(sizeof(struct bmm150_dev));
        BHY2_RLST_CHECK(!dev, " Device handler malloc failed!", false);

        switch (__handler.intf) {
        case SENSORLIB_I2C_INTERFACE:
            BHY2_RLST_CHECK(!__handler.u.i2c_dev.wire, "Wire ptr NULL", false);
            if (!SensorInterfaces::setup_interfaces(__handler)) {
                log_e("setup_interfaces failed");
                free(dev);
                return false;
            }
            dev->intf = BMM150_I2C_INTF;
            dev->read = SensorInterfaces::bhy2_i2c_read;
            dev->write = SensorInterfaces::bhy2_i2c_write;
            dev->intf_ptr = &__handler;
            break;

        case SENSORLIB_SPI_INTERFACE:
            BHY2_RLST_CHECK(!__handler.u.spi_dev.spi, "SPI ptr NULL", false);
            if (!SensorInterfaces::setup_interfaces(__handler)) {
                log_e("setup_interfaces failed");
                free(dev);
                return false;
            }
            dev->intf = BMM150_I2C_INTF;
            dev->read = SensorInterfaces::bhy2_spi_read;
            dev->write = SensorInterfaces::bhy2_spi_write;
            dev->intf_ptr = &__handler;
            break;
        default:
            free(dev);
            return false;
        }

        dev->delay_us = SensorInterfaces::bhy2_delay_us;
        __error_code = bmm150_init(dev);
        if (__error_code != BMM150_OK) {
            log_e("bmm150 init failed!");
            free(dev);
            return false;
        }

        __error_code = bmm150_soft_reset(dev);
        if (__error_code != BMM150_OK) {
            log_e("reset failed!");
            free(dev);
            return false;
        }

        if (__handler.irq != SENSOR_PIN_NONE) {
#if defined(ARDUINO_ARCH_RP2040)
            attachInterruptParam((pin_size_t)(__handler.irq), handleISR, (PinStatus )RISING, (void *)&__data_available);
#else
            attachInterruptArg(__handler.irq, handleISR, (void *)&__data_available, RISING);
#endif
        }

        return __error_code == BMM150_OK;
    }

protected:
    struct bmm150_dev *dev = NULL;
    SensorLibConfigure __handler;
    int8_t           __error_code;
    volatile bool    __data_available;
};







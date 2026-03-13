/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
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
 * @file      SensorCommEspIDF_SPI.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-18
 *
 */
#pragma once

#include "../SensorCommBase.hpp"

#if !defined(ARDUINO)  && defined(ESP_PLATFORM)
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"



class SensorCommSPI : public SensorCommBase
{
public:
    SensorCommSPI(spi_host_device_t host, spi_device_handle_t &spi, uint8_t csPin, SensorHal *hal) : host(host), spi(spi), csPin(csPin), hal(hal) {}
    SensorCommSPI(spi_host_device_t host, spi_device_handle_t &spi, uint8_t csPin, int mosi, int miso, int sck, SensorHal *hal) : host(host), spi(spi),
        csPin(csPin), hal(hal), mosi(mosi), miso(miso), sck(sck) {}

    bool init() override
    {
        if (!hal) {
            setError(SENSOR_ERR_INVALID_ARG);
            return false;
        }
        if (mosi != -1 && miso != -1 && sck != -1) {
            spi_bus_config_t buscfg ;
            buscfg.miso_io_num = miso,
            buscfg.mosi_io_num = mosi,
            buscfg.sclk_io_num = sck,
            buscfg.quadwp_io_num = -1,
            buscfg.quadhd_io_num = -1,
            buscfg.data4_io_num = -1,
            buscfg.data5_io_num = -1,
            buscfg.data6_io_num = -1,
            buscfg.data7_io_num = -1,
            buscfg.max_transfer_sz = 128;
            buscfg.flags = 0;
            buscfg.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;
            buscfg.intr_flags = 0;

            spi_device_interface_config_t devcfg ;
            memset(&devcfg, 0, sizeof(devcfg));
            devcfg.address_bits = 8;
            devcfg.clock_speed_hz = spiSetting.clock;
            devcfg.mode = spiSetting.dataMode;
            devcfg.spics_io_num = -1;
            devcfg.queue_size = 2;
            devcfg.pre_cb = NULL;

            esp_err_t ret;
            ret = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
            if (ret != ESP_OK) {
                setError(SENSOR_ERR_COMM_INIT);
                return false;
            }

            ret = spi_bus_add_device(host, &devcfg, &spi);
            if (ret != ESP_OK) {
                setError(SENSOR_ERR_COMM_INIT);
                return false;
            }
        }

        hal->pinMode(csPin, OUTPUT);
        hal->digitalWrite(csPin, HIGH);

        return true;
    }

    void deinit() override
    {
        spi_bus_remove_device(spi);
        // spi_bus_free(host);
    }

    int writeRegister(const uint8_t reg, uint8_t *buf, size_t len) override
    {
        if (hal == nullptr) {
            return SENSOR_ERR_INVALID_ARG;
        }

        hal->digitalWrite(csPin, LOW);

        spi_transaction_t trans;
        memset(&trans, 0, sizeof(trans));
        trans.flags = 0;
        trans.cmd = 0;
        trans.addr = reg & WRITE_MASK;
        trans.length = len * 8;
        trans.rxlength = 0;
        trans.user = NULL;
        trans.tx_buffer = buf;
        trans.rx_buffer = NULL;
        esp_err_t ret = spi_device_transmit(spi, &trans);

        hal->digitalWrite(csPin, HIGH);

        return ret == ESP_OK ? SENSOR_OK : SENSOR_ERR_COMM_NACK;
    }

    int readBuffer(uint8_t *buf, size_t len) override
    {
        if (!buf || len == 0 || hal == nullptr) {
            return SENSOR_ERR_INVALID_ARG;
        }

        hal->digitalWrite(csPin, LOW);
        spi_transaction_t trans;
        memset(&trans, 0, sizeof(trans));
        trans.addr = READ_MASK;
        trans.length = len * 8;
        trans.rxlength = len * 8;
        trans.rx_buffer = buf;
        esp_err_t ret = spi_device_transmit(spi, &trans);
        hal->digitalWrite(csPin, HIGH);
        return ret == ESP_OK ? SENSOR_OK : SENSOR_ERR_COMM_NACK;
    }


    int writeBuffer(uint8_t *buf, size_t len)
    {
        if (!buf || len == 0 || hal == nullptr) {
            return SENSOR_ERR_INVALID_ARG;
        }

        hal->digitalWrite(csPin, LOW);

        spi_transaction_t trans;
        memset(&trans, 0, sizeof(trans));
        trans.length = 8 * len;
        trans.tx_buffer = buf;
        esp_err_t ret = spi_device_polling_transmit(spi, &trans);

        hal->digitalWrite(csPin, HIGH);
        return ret == ESP_OK ? 0 : -1;
    }

    int readRegister(const uint8_t reg, uint8_t *buf, size_t len) override
    {

        if (!buf || len == 0 || hal == nullptr) {
            return SENSOR_ERR_INVALID_ARG;
        }

        hal->digitalWrite(csPin, LOW);

        spi_transaction_t trans;
        memset(&trans, 0, sizeof(trans));
        trans.addr = reg | READ_MASK;
        trans.length = len * 8;
        trans.rxlength = len * 8;
        trans.rx_buffer = buf;
        esp_err_t ret = spi_device_transmit(spi, &trans);

        hal->digitalWrite(csPin, HIGH);
        return ret == ESP_OK ? SENSOR_OK : SENSOR_ERR_COMM_NACK;
    }

    int writeThenRead(const uint8_t *write_buffer, size_t write_len, uint8_t *read_buffer, size_t read_len) override
    {
        if (!write_buffer || write_len == 0 || !read_buffer || read_len == 0 || hal == nullptr) {
            return SENSOR_ERR_INVALID_ARG;
        }

        hal->digitalWrite(csPin, LOW);

        esp_err_t ret;
        spi_transaction_t trans;
        memset(&trans, 0, sizeof(trans)); // Zero out the trans

        auto perform_transmission = [&](const uint8_t *tx_buf, size_t len, uint8_t *rx_buf) {
            trans.length = 8 * len;
            trans.tx_buffer = tx_buf;
            trans.rx_buffer = rx_buf;
            return spi_device_polling_transmit(spi, &trans);
        };

        ret = perform_transmission(write_buffer, write_len, nullptr);
        if (ret != ESP_OK) {
            hal->digitalWrite(csPin, HIGH);
            return -1;
        }

        ret = perform_transmission(nullptr, read_len, read_buffer);

        hal->digitalWrite(csPin, HIGH);

        return ret == ESP_OK ? SENSOR_OK : SENSOR_ERR_COMM_NACK;
    }


    void setParams(const CommParamsBase &params) override
    {
#if defined(__cpp_rtti)
        const auto pdat = dynamic_cast<const SPIParam *>(&params);
#else
        const auto pdat = static_cast<const SPIParam *>(&params);
#endif
        pdat->getSetting();

        /*Remove SPI BUS and re-initialize according to the new settings*/
        // spi_bus_remove_device(spi);
        // spi_bus_free(host);
    }

private:
    static constexpr const uint8_t READ_MASK = 0x80;
    static constexpr const uint8_t WRITE_MASK = 0x7F;
    spi_host_device_t host;
    spi_device_handle_t spi;
    uint8_t csPin;
    SensorHal *hal;
    int mosi, miso, sck;
    SPISettings spiSetting;
};

#endif //*ESP_PLATFORM

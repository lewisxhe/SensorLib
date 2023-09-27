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
 * @file      SensorBHI260AP.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-06
 * @note      Most source code references come from the https://github.com/boschsensortec/BHY2-Sensor-API
 *            Simplification for Arduino
 */


#include "SensorCommon.tpp"
#include "bosch/common/common.h"

#define BHI260AP_SLAVE_ADDRESS          0x28
#define BHY_PROCESS_BUFFER_SZIE         512

class SensorBHI260AP
{
public:

#if defined(ARDUINO)
    SensorBHI260AP(TwoWire &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BHI260AP_SLAVE_ADDRESS)
    {
        __handler.u.i2c_dev.scl = scl;
        __handler.u.i2c_dev.sda = sda;
        __handler.u.i2c_dev.addr = addr;
        __handler.u.i2c_dev.wire = &w;
        __handler.intf = BHY2_I2C_INTERFACE;
    }
#endif

    SensorBHI260AP(int cs, int mosi = -1, int miso = -1, int sck = -1, SPIClass &spi = SPI)
    {
        __handler.u.spi_dev.cs = cs;
        __handler.u.spi_dev.miso = miso;
        __handler.u.spi_dev.mosi = mosi;
        __handler.u.spi_dev.sck = sck;
        __handler.u.spi_dev.spi = &spi;
        __handler.intf = BHY2_SPI_INTERFACE;
    }

    ~SensorBHI260AP()
    {
        deinit();
    }

    SensorBHI260AP()
    {
        memset(&__handler, 0, sizeof(__handler));
    }

    void setPins(int rst, int irq)
    {
        __handler.irq = irq;
        __handler.rst = rst;
    }

#if defined(ARDUINO)
    bool init(TwoWire &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BHI260AP_SLAVE_ADDRESS)
    {
        __handler.u.i2c_dev.scl = scl;
        __handler.u.i2c_dev.sda = sda;
        __handler.u.i2c_dev.addr = addr;
        __handler.u.i2c_dev.wire = &w;
        __handler.intf = BHY2_I2C_INTERFACE;
        return initImpl();
    }

    bool init(SPIClass &spi, int cs, int mosi = MOSI, int miso = MISO, int sck = SCK)
    {
        __handler.u.spi_dev.cs = cs;
        __handler.u.spi_dev.miso = miso;
        __handler.u.spi_dev.mosi = mosi;
        __handler.u.spi_dev.sck = sck;
        __handler.u.spi_dev.spi = &spi;
        __handler.intf = BHY2_SPI_INTERFACE;
        return initImpl();
    }
#endif

    bool init()
    {
        return initImpl();
    }

    void deinit()
    {
        if (processBuffer) {
            free(processBuffer);
        }
        processBuffer = NULL;
        // end();
    }

    void reset()
    {
        bhy2_soft_reset(bhy2);
    }

    void update()
    {
        if (!processBuffer)
            return;
        if (__handler.irq != -1) {
            if ( digitalRead(__handler.irq) == HIGH) {
                bhy2_get_and_process_fifo(processBuffer, BHY2_WORK_BUFFER_SIZE, bhy2);
            }
        }
    }

    bool enablePowerSave()
    {
        return true;
    }

    bool disablePowerSave()
    {

        return true;
    }

    void disableInterruptCtrl()
    {
    }

    void enableInterruptCtrl()
    {
    }

    bhy2_dev *getHandler()
    {
        return &__handler.bhy2;
    }

    void printSensors(Stream &port)
    {
        bool presentBuff[256];

        for (uint16_t i = 0; i < sizeof(bhy2->present_buff); i++) {
            for (uint8_t j = 0; j < 8; j++) {
                presentBuff[i * 8 + j] = ((bhy2->present_buff[i] >> j) & 0x01);
            }
        }

        port.println("Present sensors: ");
        for (int i = 0; i < (int)sizeof(presentBuff); i++) {
            if (presentBuff[i]) {
                port.print(i);
                port.print(" - ");
                port.print(get_sensor_name(i));
                port.println();
            }
        }
    }


    bool setInterruptCtrl(uint8_t data)
    {
        __error_code = bhy2_set_host_interrupt_ctrl(data, bhy2);
        if (__error_code != BHY2_OK) {
            return false;
        }
    }

    uint8_t getInterruptCtrl()
    {
        uint8_t data;
        __error_code = bhy2_get_host_interrupt_ctrl(&data, bhy2);
        if (__error_code != BHY2_OK) {
            return 0;
        }
        return data;
    }

    void printInterruptCtrl(Stream &steram)
    {
        uint8_t data;
        __error_code = bhy2_get_host_interrupt_ctrl(&data, bhy2);
        if (__error_code != BHY2_OK) {
            return ;
        }
        steram.printf("Host interrupt control\r\n");
        steram.printf("-- Wake up FIFO %s.\r\n", (data & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
        steram.printf("-- Non wake up FIFO %s.\r\n", (data & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
        steram.printf("-- Status FIFO %s.\r\n", (data & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
        steram.printf("-- Debugging %s.\r\n", (data & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
        steram.printf("-- Fault %s.\r\n", (data & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
        steram.printf("-- Interrupt is %s.\r\n", (data & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
        steram.printf("-- Interrupt is %s triggered.\r\n", (data & BHY2_ICTL_EDGE) ? "pulse" : "level");
        steram.printf("-- Interrupt pin drive is %s.\r\n", (data & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");
    }

    bool isReady()
    {
        uint8_t  boot_status;
        __error_code = bhy2_get_boot_status(&boot_status, bhy2);
        if (__error_code != BHY2_OK) {
            return false;
        }
        return boot_status & BHY2_BST_HOST_INTERFACE_READY;
    }

    uint16_t getKernelVersion()
    {
        uint16_t version = 0;
        __error_code = bhy2_get_kernel_version(&version, bhy2);
        if ((__error_code != BHY2_OK) && (version == 0)) {
            return 0;
        }
        log_i("Boot successful. Kernel version %u.\r\n", version);
        return version;
    }

    enum EventID {
        EVENT_ID_PADDING = (0),
        EVENT_ID_TS_SMALL_DELTA = (251),
        EVENT_ID_TS_LARGE_DELTA = (252),
        EVENT_ID_TS_FULL = (253),
        EVENT_ID_META_EVENT = (254),
        EVENT_ID_TS_SMALL_DELTA_WU = (245),
        EVENT_ID_TS_LARGE_DELTA_WU = (246),
        EVENT_ID_TS_FULL_WU = (247),
        EVENT_ID_META_EVENT_WU = (248),
        EVENT_ID_FILLER = (255),
        EVENT_ID_DEBUG_MSG = (250),
        EVENT_ID_BHY2_LOG_UPDATE_SUB = (243),
        EVENT_ID_BHY2_LOG_DOSTEP = (244),
    };

    bool registerEventCallback(EventID event_id, bhy2_fifo_parse_callback_t callback, void *private_data)
    {
        __error_code = bhy2_register_fifo_parse_callback(event_id, callback, private_data, bhy2);
        return __error_code == BHY2_OK;
    }


    void setProcessBufferSize(uint32_t size)
    {
        processBufferSize = size;
    }


    bool uploadFirmware(const uint8_t *firmware, uint32_t length, bool write2Flash = false)
    {
        uint8_t sensor_error;
        uint8_t boot_status;
        __error_code = bhy2_get_boot_status(&boot_status, bhy2);
        if (__error_code != BHY2_OK) {
            return false;
        }
        if (write2Flash) {
            if (boot_status & BHY2_BST_FLASH_DETECTED) {
                uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
                uint32_t end_addr = start_addr + length;
                log_i("Flash detected. Erasing flash to upload firmware\r\n");
                __error_code = bhy2_erase_flash(start_addr, end_addr, bhy2);
                if (__error_code != BHY2_OK) {
                    return false;
                }
            } else {
                log_e("Flash not detected\r\n");
                return false;
            }
            printf("Loading firmware into FLASH.\r\n");
            __error_code = bhy2_upload_firmware_to_flash(firmware, length, bhy2);
            if (__error_code != BHY2_OK) {
                return false;
            }
        } else {
            log_i("Loading firmware into RAM.\r\n");
            __error_code = bhy2_upload_firmware_to_ram(firmware, length, bhy2);
        }

        log_i("Loading firmware into RAM Done\r\n");
        __error_code = bhy2_get_error_value(&sensor_error, bhy2);
        if (__error_code != BHY2_OK) {
            return false;
        }
        if (sensor_error != BHY2_OK) {
            __error_code = bhy2_get_error_value(&sensor_error, bhy2);
            log_e("%s\r\n", get_sensor_error_text(sensor_error));
            return false;
        }


        if (write2Flash) {
            log_i("Booting from FLASH.\r\n");
            __error_code = bhy2_boot_from_flash(bhy2);
        } else {
            log_i("Booting from RAM.\r\n");
            __error_code = bhy2_boot_from_ram(bhy2);
        }

        __error_code = bhy2_get_error_value(&sensor_error, bhy2);
        if (sensor_error) {
            log_e("%s\r\n", get_sensor_error_text(sensor_error));
            return false;
        }
        return sensor_error == BHY2_OK;
    }



private:

    bool initImpl()
    {
        int8_t rslt;
        uint8_t product_id = 0;

        switch (__handler.intf) {
        case BHY2_I2C_INTERFACE:

            if (!__handler.u.i2c_dev.wire) {
                log_e("Wire ptr NULL");
                return false;
            }

            setup_interfaces(true, __handler); /* Perform a power on reset */

            rslt = bhy2_init(BHY2_I2C_INTERFACE, bhy2_i2c_read,  bhy2_i2c_write, bhy2_delay_us, BHY2_RD_WR_LEN, &__handler, &__handler.bhy2);

            rslt |= bhy2_set_host_intf_ctrl(BHY2_I2C_INTERFACE, &__handler.bhy2);
            break;

        case BHY2_SPI_INTERFACE:

            if (!__handler.u.spi_dev.spi) {
                log_e("SPI ptr NULL");
                return false;
            }

            setup_interfaces(true, __handler); /* Perform a power on reset */

            rslt = bhy2_init(BHY2_SPI_INTERFACE,  bhy2_spi_read,  bhy2_spi_write, bhy2_delay_us, BHY2_RD_WR_LEN, &__handler, &__handler.bhy2);

            rslt |= bhy2_set_host_intf_ctrl(BHY2_SPI_INTERFACE, &__handler.bhy2);
            break;
        default:
            return false;
        }



        if (rslt != BHY2_OK) {
            return false;
        }

        bhy2 = &__handler.bhy2;

        rslt = bhy2_soft_reset(bhy2);
        if (rslt != BHY2_OK) {
            log_e("reset bhy2 failed!");
            return false;
        }

        rslt = bhy2_get_product_id(&product_id, bhy2);

        /* Check for a valid product ID */
        if (product_id != BHY2_PRODUCT_ID) {
            log_e("Product ID read %X. Expected %X\r\n", product_id, BHY2_PRODUCT_ID);
            return false;
        } else {
            log_i("BHI260/BHA260 found. Product ID read %X\r\n", product_id);
        }


        bhy2_update_virtual_sensor_list(bhy2);

        bhy2_get_virt_sensor_list(bhy2);

#if     defined(ESP32) && defined(BOARD_HAS_PSRAM)
        processBuffer = (uint8_t *)ps_malloc(processBufferSize);
#else
        processBuffer = (uint8_t *)malloc(processBufferSize);
#endif

        return processBuffer == NULL;
    }

protected:
    struct bhy2_dev             *bhy2 = NULL;
    bhy_config_t                __handler;
    uint8_t __error_code;
    uint8_t *processBuffer = NULL;
    size_t processBufferSize = BHY_PROCESS_BUFFER_SZIE;
};







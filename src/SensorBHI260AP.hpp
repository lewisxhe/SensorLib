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

#include "bosch/BoschParse.h"
#include "bosch/SensorBhy2Define.h"
#include "bosch/firmware/BHI260AP.fw.h"

class SensorBHI260AP
{
    friend class BoschParse;
public:

#if defined(ARDUINO)
    SensorBHI260AP(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BHI260AP_SLAVE_ADDRESS)
    {
        __handler.u.i2c_dev.scl = scl;
        __handler.u.i2c_dev.sda = sda;
        __handler.u.i2c_dev.addr = addr;
        __handler.u.i2c_dev.wire = &w;
        __handler.intf = BHY2_I2C_INTERFACE;
    }
#endif

    SensorBHI260AP(int cs, int mosi = -1, int miso = -1, int sck = -1,
                   PLATFORM_SPI_TYPE &spi = SPI
                  )
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
    bool init(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BHI260AP_SLAVE_ADDRESS)
    {
        __handler.u.i2c_dev.scl = scl;
        __handler.u.i2c_dev.sda = sda;
        __handler.u.i2c_dev.addr = addr;
        __handler.u.i2c_dev.wire = &w;
        __handler.intf = BHY2_I2C_INTERFACE;
        return initImpl();
    }
#endif

    bool init(
        PLATFORM_SPI_TYPE &spi,
        int cs, int mosi = MOSI, int miso = MISO, int sck = SCK)
    {
        __handler.u.spi_dev.cs = cs;
        __handler.u.spi_dev.miso = miso;
        __handler.u.spi_dev.mosi = mosi;
        __handler.u.spi_dev.sck = sck;
        __handler.u.spi_dev.spi = &spi;
        __handler.intf = BHY2_SPI_INTERFACE;
        return initImpl();
    }

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

        if (__handler.irq != SENSOR_PIN_NONE) {
            detachInterrupt(__handler.irq);
        }
        // end();
    }

    void reset()
    {
        if (__handler.rst != SENSOR_PIN_NONE) {
            digitalWrite(__handler.rst, HIGH);
            delay(5);
            digitalWrite(__handler.rst, HIGH);
            delay(10);
            digitalWrite(__handler.rst, HIGH);
            delay(5);
        }
    }

    void update()
    {
        if (!processBuffer) {
            return;
        }
        if (__handler.irq != SENSOR_PIN_NONE) {
            if (__data_available) {
                bhy2_get_and_process_fifo(processBuffer, processBufferSize, bhy2);
            }
        } else {
            bhy2_get_and_process_fifo(processBuffer, processBufferSize, bhy2);
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
        uint8_t  boot_status = 0;
        __error_code = bhy2_get_boot_status(&boot_status, bhy2);
        log_i("boot_status:0x%x", boot_status);
        if (__error_code != BHY2_OK) {
            return false;
        }
        return boot_status & BHY2_BST_HOST_INTERFACE_READY == false;
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



    void onEvent(BhySensorEvent event_id, BhyEventCb callback)
    {
        SensorEventCbList_t newEventHandler;
        newEventHandler.cb = callback;
        newEventHandler.event = event_id;
        BoschParse::bhyEventVector.push_back(newEventHandler);
    }

    void removeEvent(BhySensorEvent event_id, BhyEventCb callback)
    {
        if (!callback) {
            return;
        }
        for (uint32_t i = 0; i < BoschParse::bhyEventVector.size(); i++) {
            SensorEventCbList_t entry = BoschParse::bhyEventVector[i];
            if (entry.cb == callback && entry.event == event_id) {
                BoschParse::bhyEventVector.erase(BoschParse::bhyEventVector.begin() + i);
            }
        }
    }


    void onResultEvent(BhySensorID sensor_id, BhyParseDataCallback callback)
    {
        ParseCallBackList_t newEventHandler;
        newEventHandler.cb = callback;
        newEventHandler.id = sensor_id;
        BoschParse::bhyParseEventVector.push_back(newEventHandler);
    }

    void removeResultEvent(BhySensorID sensor_id, BhyParseDataCallback callback)
    {
        if (!callback) {
            return;
        }
        for (uint32_t i = 0; i < BoschParse::bhyParseEventVector.size(); i++) {
            ParseCallBackList_t entry = BoschParse::bhyParseEventVector[i];
            if (entry.cb == callback && entry.id == sensor_id) {
                BoschParse::bhyParseEventVector.erase(BoschParse::bhyParseEventVector.begin() + i);
            }
        }
    }

    void setProcessBufferSize(uint32_t size)
    {
        processBufferSize = size;
    }


    bool uploadFirmware(const uint8_t *firmware, uint32_t length, bool write2Flash = false)
    {
        uint8_t sensor_error;
        uint8_t boot_status;

        log_i("Upload Firmware ...");

        __error_code = bhy2_get_boot_status(&boot_status, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);

        if (write2Flash) {
            if (boot_status & BHY2_BST_FLASH_DETECTED) {
                uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
                uint32_t end_addr = start_addr + length;
                log_i("Flash detected. Erasing flash to upload firmware\r\n");
                __error_code = bhy2_erase_flash(start_addr, end_addr, bhy2);
                BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_erase_flash failed!", false);
            } else {
                log_e("Flash not detected\r\n");
                return false;
            }
            printf("Loading firmware into FLASH.\r\n");
            __error_code = bhy2_upload_firmware_to_flash(firmware, length, bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_upload_firmware_to_flash failed!", false);
        } else {
            log_i("Loading firmware into RAM.\r\n");
            log_i("upload size = %u", length);
            __error_code = bhy2_upload_firmware_to_ram(firmware, length, bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_upload_firmware_to_ram failed!", false);
        }

        log_i("Loading firmware into RAM Done\r\n");
        __error_code = bhy2_get_error_value(&sensor_error, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_error_value failed!", false);
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
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2 boot failed!", false);

        __error_code = bhy2_get_error_value(&sensor_error, bhy2);
        if (sensor_error) {
            log_e("%s\r\n", get_sensor_error_text(sensor_error));
            return false;
        }
        return sensor_error == BHY2_OK;
    }

    String getError()
    {
        String err = get_api_error(__error_code);
        err += " Code:" + String(__error_code);
        return err;
    }

    bool configure(uint8_t sensor_id, float sample_rate, uint32_t report_latency_ms)
    {
        __error_code = bhy2_set_virt_sensor_cfg(sensor_id, sample_rate, report_latency_ms, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_set_virt_sensor_cfg failed!", false);
        log_i("Enable %s at %.2fHz.\r\n", get_sensor_name(sensor_id), sample_rate);
        return true;
    }

    float getScaling(uint8_t sensor_id)
    {
        return get_sensor_default_scaling(sensor_id);
    }

    void setFirmware(const uint8_t *image, size_t image_len, bool write_flash)
    {
        __firmware = image;
        __firmware_size = image_len;
        __write_flash = write_flash;
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

        switch (__handler.intf) {
        case BHY2_I2C_INTERFACE:
            BHY2_RLST_CHECK(!__handler.u.i2c_dev.wire, "Wire ptr NULL", false);
            if (!SensorInterfaces::setup_interfaces(true, __handler)) {
                log_e("setup_interfaces failed");
                return false;
            }
            __error_code = bhy2_init(BHY2_I2C_INTERFACE,
                                     SensorInterfaces::bhy2_i2c_read,
                                     SensorInterfaces::bhy2_i2c_write,
                                     SensorInterfaces::bhy2_delay_us,
                                     BHY2_RD_WR_LEN, &__handler, &__handler.bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_init failed!", false);
            // __error_code = bhy2_set_host_intf_ctrl(BHY2_I2C_INTERFACE, &__handler.bhy2);
            // BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_set_host_intf_ctrl failed!", false);
            break;

        case BHY2_SPI_INTERFACE:
            BHY2_RLST_CHECK(!__handler.u.spi_dev.spi, "SPI ptr NULL", false);
            if (!SensorInterfaces::setup_interfaces(true, __handler)) {
                log_e("setup_interfaces failed");
                return false;
            }
            __error_code = bhy2_init(BHY2_SPI_INTERFACE,
                                     SensorInterfaces::bhy2_spi_read,
                                     SensorInterfaces::bhy2_spi_write,
                                     SensorInterfaces::bhy2_delay_us,
                                     BHY2_RD_WR_LEN,
                                     &__handler,
                                     &__handler.bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_init failed!", false);
            // __error_code = bhy2_set_host_intf_ctrl(BHY2_SPI_INTERFACE, &__handler.bhy2);
            // BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_set_host_intf_ctrl failed!", false);
            break;
        default:
            return false;
        }


        bhy2 = &__handler.bhy2;

        __error_code = bhy2_soft_reset(bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "reset bhy2 failed!", false);

        __error_code = bhy2_get_product_id(&product_id, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_product_id failed!", false);


        /* Check for a valid product ID */
        if (product_id != BHY2_PRODUCT_ID) {
            log_e("Product ID read %X. Expected %X\r\n", product_id, BHY2_PRODUCT_ID);
            return false;
        } else {
            log_i("BHI260/BHA260 found. Product ID read %X\r\n", product_id);
        }

        if (!__firmware) {
            // Default write to ram
            setFirmware(bhy2_firmware_image, sizeof(bhy2_firmware_image), false);
        }

        if (!isReady()) {
            if (!uploadFirmware(__firmware, __firmware_size, __write_flash)) {
                log_e("uploadFirmware failed!");
                return false;
            }
        }

        uint16_t version = getKernelVersion();
        BHY2_RLST_CHECK(!version, "getKernelVersion failed!", false);
        log_i("Boot successful. Kernel version %u.\r\n", version);

        //Set event callback
        __error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, BoschParse::parseMetaEvent, NULL, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_register_fifo_parse_callback failed!", false);


        __error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, BoschParse::parseMetaEvent, NULL, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_register_fifo_parse_callback failed!", false);


        // All sensors' data are handled in the same generic way
        for (uint8_t i = 1; i < BHY2_SENSOR_ID_MAX; i++) {
            bhy2_register_fifo_parse_callback(i, BoschParse::parseData, NULL, bhy2);
        }

        //Set process buffer
#if     defined(ESP32) && defined(BOARD_HAS_PSRAM)
        processBuffer = (uint8_t *)ps_malloc(processBufferSize);
#else
        processBuffer = (uint8_t *)malloc(processBufferSize);
#endif
        BHY2_RLST_CHECK(!processBuffer, "process buffer malloc failed!", false);

        __error_code = bhy2_get_and_process_fifo(processBuffer, processBufferSize, bhy2);
        if (__error_code != BHY2_OK) {
            log_e("bhy2_get_and_process_fifo failed");
            free(processBuffer);
            return false;
        }

        /* Update the callback table to enable parsing of sensor hintr_ctrl */
        bhy2_update_virtual_sensor_list(bhy2);

        bhy2_get_virt_sensor_list(bhy2);


        if (__handler.irq != SENSOR_PIN_NONE) {
#if defined(ARDUINO_ARCH_RP2040)
            attachInterruptParam((pin_size_t)(__handler.irq), handleISR, (PinStatus )RISING, (void *)&__data_available);
#else
            // attachInterrupt(__handler.irq, std::bind(&SensorBHI260AP::handleISR, this), RISING);
            attachInterruptArg(__handler.irq, handleISR, (void *)&__data_available, RISING);
#endif
        }

        return __error_code == BHY2_OK;
    }

protected:
    struct bhy2_dev  *bhy2 = NULL;
    bhy_config_t     __handler;
    int8_t           __error_code;
    volatile bool    __data_available;
    uint8_t          *processBuffer = NULL;
    size_t           processBufferSize = BHY_PROCESS_BUFFER_SZIE;
    const uint8_t    *__firmware;
    size_t          __firmware_size;
    bool            __write_flash;
};







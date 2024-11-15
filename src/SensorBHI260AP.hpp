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
#pragma once

#if defined(ARDUINO)

#include "bosch/BoschParse.h"
#include "bosch/SensorBhy2Define.h"
#include "bosch/firmware/BHI260AP.fw.h"




class SensorBHI260AP
{
    friend class BoschParse;
public:

    // The pin names are named according to the sensor manual.
    enum BHI260AP_GPIO {
        MCSB1 = 1,
        RESV1 = 2,
        RESV2 = 3,
        MCSB2 = 4,  //It may be connected to the BMM150 sensor, select according to the actual situation
        MCSB3 = 5,
        MCSB4 = 6,

        QSPI_CLK = 8, // If BHI260 carries external flash, it is not available
        QSPI_CSN = 9, // If BHI260 carries external flash, it is not available
        QSPI_D0 = 10, // If BHI260 carries external flash, it is not available
        QSPI_D1 = 11, // If BHI260 carries external flash, it is not available
        QSPI_D2 = 12, // If BHI260 carries external flash, it is not available
        QSPI_D3 = 13, // If BHI260 carries external flash, it is not available

        M2SCX = 14,
        M2SDX = 15,
        M2SDI = 16,
        M3SCL = 17, //It may be connected to the BMM150 sensor, select according to the actual situation
        M3SDA = 18, //It may be connected to the BMM150 sensor, select according to the actual situation
        JTAG_CLK = 19,
        JTAG_DIO = 20,

        M1SCX = 127, // Invalid Pin
        M1SDX = 128, // Invalid Pin
        M1SDI = 129, // Invalid Pin
        RESV3 = 130, // Invalid Pin
    };

    SensorBHI260AP(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BHI260AP_SLAVE_ADDRESS_L)
    {
        __handler.u.i2c_dev.scl = scl;
        __handler.u.i2c_dev.sda = sda;
        __handler.u.i2c_dev.addr = addr;
        __handler.u.i2c_dev.wire = &w;
        __handler.intf = SENSORLIB_I2C_INTERFACE;
    }

    SensorBHI260AP(int cs, int mosi = -1, int miso = -1, int sck = -1,
                   PLATFORM_SPI_TYPE &spi = SPI
                  )
    {
        __handler.u.spi_dev.cs = cs;
        __handler.u.spi_dev.miso = miso;
        __handler.u.spi_dev.mosi = mosi;
        __handler.u.spi_dev.sck = sck;
        __handler.u.spi_dev.spi = &spi;
        __handler.intf = SENSORLIB_SPI_INTERFACE;
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

    bool init(PLATFORM_WIRE_TYPE &w, int sda = DEFAULT_SDA, int scl = DEFAULT_SCL, uint8_t addr = BHI260AP_SLAVE_ADDRESS_L)
    {
        __handler.u.i2c_dev.scl = scl;
        __handler.u.i2c_dev.sda = sda;
        __handler.u.i2c_dev.addr = addr;
        __handler.u.i2c_dev.wire = &w;
        __handler.intf = SENSORLIB_I2C_INTERFACE;
        return initImpl();
    }

    bool init(
        PLATFORM_SPI_TYPE &spi,
        int cs, int mosi = MOSI, int miso = MISO, int sck = SCK)
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
        if (__pro_buf) {
            free(__pro_buf);
        }
        __pro_buf = NULL;

        if (bhy2) {
            free(bhy2);
            bhy2 = NULL;
        }

        if (__handler.irq != SENSOR_PIN_NONE) {
            detachInterrupt(__handler.irq);
        }
        // end();
    }

    /**
     * @brief  reset
     * @note   Reset sensor
     * @retval None
     */
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

    /**
     * @brief  update
     * @note   Update sensor fifo data
     * @retval None
     */
    void update()
    {
        if (!__pro_buf) {
            return;
        }
        if (__handler.irq != SENSOR_PIN_NONE) {
            if (__data_available) {
                bhy2_get_and_process_fifo(__pro_buf, __pro_buf_size, bhy2);
                __data_available = false;
            }
        } else {
            bhy2_get_and_process_fifo(__pro_buf, __pro_buf_size, bhy2);
        }
    }

    /**
     * @brief  setBootFormFlash
     * @note   Set whether to start from external flash
     * @param  boot_from_flash: true boot form flash or boot form ram
     * @retval None
     */
    void setBootFormFlash(bool boot_from_flash)
    {
        __boot_from_flash = boot_from_flash;
    }

    /**
     * @brief  getHandler
     * @note   Get the native BHI API handle
     * @retval handle
     */
    bhy2_dev *getHandler()
    {
        return bhy2;
    }

    /**
     * @brief  printSensors
     * @note   Output available sensor IDs to serial
     * @param  &port: Serial or other
     * @retval None
     */
    void printSensors(Stream &port)
    {
        uint8_t cnt = 0;
        bool presentBuff[256];

        for (uint16_t i = 0; i < sizeof(bhy2->present_buff); i++) {
            for (uint8_t j = 0; j < 8; j++) {
                presentBuff[i * 8 + j] = ((bhy2->present_buff[i] >> j) & 0x01);
            }
        }

        port.println("Present sensors: ");
        for (int i = 0; i < (int)sizeof(presentBuff); i++) {
            if (presentBuff[i]) {
                cnt++;
                port.print(i);
                port.print(" - ");
                port.print(get_sensor_name(i));
                port.println();
            }
        }
        port.printf("Total %u Sensor online .\n", cnt);
    }

    /**
     * @brief  printInfo
     * @note   Print sensor information
     * @param  &stream: Serial or other
     * @retval true is success , false is failed
     */
    bool printInfo(Stream &stream)
    {
        uint16_t kernel_version = 0, user_version = 0;
        uint16_t rom_version = 0;
        uint8_t product_id = 0;
        uint8_t host_status = 0, feat_status = 0;
        uint8_t boot_status = 0;
        uint8_t sensor_error;
        struct bhy2_sensor_info info;

        /* Get product_id */
        __error_code = (bhy2_get_product_id(&product_id, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_product_id failed!", false);

        /* Get Kernel version */
        __error_code = (bhy2_get_kernel_version(&kernel_version, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_kernel_version failed!", false);

        /* Get User version */
        __error_code = (bhy2_get_user_version(&user_version, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_user_version failed!", false);

        /* Get ROM version */
        __error_code = (bhy2_get_rom_version(&rom_version, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_rom_version failed!", false);

        __error_code = (bhy2_get_host_status(&host_status, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_host_status failed!", false);

        __error_code = (bhy2_get_feature_status(&feat_status, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_feature_status failed!", false);

        stream.printf("Product ID     : %02x\n", product_id);
        stream.printf("Kernel version : %04u\n", kernel_version);
        stream.printf("User version   : %04u\n", user_version);
        stream.printf("ROM version    : %04u\n", rom_version);
        stream.printf("Power state    : %s\n", (host_status & BHY2_HST_POWER_STATE) ? "sleeping" : "active");
        stream.printf("Host interface : %s\n", (host_status & BHY2_HST_HOST_PROTOCOL) ? "SPI" : "I2C");
        stream.printf("Feature status : 0x%02x\n", feat_status);

        /* Read boot status */
        __error_code = (bhy2_get_boot_status(&boot_status, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);

        stream.printf("Boot Status : 0x%02x: ", boot_status);

        if (boot_status & BHY2_BST_FLASH_DETECTED) {
            stream.println("\tFlash detected. ");
        }

        if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
            stream.println("\tFlash verify done. ");
        }

        if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR) {
            stream.println("Flash verification failed. ");
        }

        if (boot_status & BHY2_BST_NO_FLASH) {
            stream.println("\tNo flash installed. ");
        }

        if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
            stream.println("\tHost interface ready. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) {
            stream.println("\tFirmware verification done. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR) {
            stream.println("\tFirmware verification error. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_IDLE) {
            stream.println("\tFirmware halted. ");
        }

        /* Read error value */
        __error_code = (bhy2_get_error_value(&sensor_error, bhy2));
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_error_value failed!", false);
        if (sensor_error) {
            log_e("%s", get_sensor_error_text(sensor_error));
        }


        if (feat_status & BHY2_FEAT_STATUS_OPEN_RTOS_MSK) {

            bhy2_update_virtual_sensor_list(bhy2);

            /* Get present virtual sensor */
            bhy2_get_virt_sensor_list(bhy2);

            stream.printf("Virtual sensor list.\r\n");
            stream.printf("Sensor ID |                          Sensor Name |  ID | Ver |  Min rate |  Max rate |\r\n");
            stream.printf("----------+--------------------------------------+-----+-----+-----------+-----------|\r\n");
            for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++) {
                if (bhy2_is_sensor_available(i, bhy2)) {
                    if (i < BHY2_SENSOR_ID_CUSTOM_START) {
                        stream.printf(" %8u | %36s ", i, get_sensor_name(i));
                    }
                    __error_code = (bhy2_get_sensor_info(i, &info, bhy2));
                    BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_sensor_info failed!", false);
                    stream.printf("| %3u | %3u | %9.4f | %9.4f |\r\n",
                                  info.driver_id,
                                  info.driver_version,
                                  info.min_rate.f_val,
                                  info.max_rate.f_val);
                }
            }
        }
        return true;
    }

    /**
     * @brief  setInterruptCtrl
     * @note   Set the interrupt control mask
     * @param  data:
     *               BHY2_ICTL_DISABLE_FIFO_W
     *               BHY2_ICTL_DISABLE_FIFO_NW
     *               BHY2_ICTL_DISABLE_STATUS_FIFO
     *               BHY2_ICTL_DISABLE_DEBUG
     *               BHY2_ICTL_DISABLE_FAULT
     *               BHY2_ICTL_ACTIVE_LOW
     *               BHY2_ICTL_EDGE
     *               BHY2_ICTL_OPEN_DRAIN
     *
     * @retval true is success , false is failed
     */
    bool setInterruptCtrl(uint8_t data)
    {
        __error_code = bhy2_set_host_interrupt_ctrl(data, bhy2);
        if (__error_code != BHY2_OK) {
            return false;
        }
    }

    /**
     * @brief  getInterruptCtrl
     * @note   Get interrupt control mask
     * @retval interrupt mask
     */
    uint8_t getInterruptCtrl()
    {
        uint8_t data;
        __error_code = bhy2_get_host_interrupt_ctrl(&data, bhy2);
        if (__error_code != BHY2_OK) {
            return 0;
        }
        return data;
    }

    /**
     * @brief  printInterruptCtrl
     * @note   Print sensor interrupt control methods to a stream
     * @param  &stream: Serial or other
     * @retval None
     */
    void printInterruptCtrl(Stream &stream)
    {
        uint8_t data;
        __error_code = bhy2_get_host_interrupt_ctrl(&data, bhy2);
        if (__error_code != BHY2_OK) {
            return ;
        }
        stream.printf("Host interrupt control\r\n");
        stream.printf("-- Wake up FIFO %s.\r\n", (data & BHY2_ICTL_DISABLE_FIFO_W) ? "disabled" : "enabled");
        stream.printf("-- Non wake up FIFO %s.\r\n", (data & BHY2_ICTL_DISABLE_FIFO_NW) ? "disabled" : "enabled");
        stream.printf("-- Status FIFO %s.\r\n", (data & BHY2_ICTL_DISABLE_STATUS_FIFO) ? "disabled" : "enabled");
        stream.printf("-- Debugging %s.\r\n", (data & BHY2_ICTL_DISABLE_DEBUG) ? "disabled" : "enabled");
        stream.printf("-- Fault %s.\r\n", (data & BHY2_ICTL_DISABLE_FAULT) ? "disabled" : "enabled");
        stream.printf("-- Interrupt is %s.\r\n", (data & BHY2_ICTL_ACTIVE_LOW) ? "active low" : "active high");
        stream.printf("-- Interrupt is %s triggered.\r\n", (data & BHY2_ICTL_EDGE) ? "pulse" : "level");
        stream.printf("-- Interrupt pin drive is %s.\r\n", (data & BHY2_ICTL_OPEN_DRAIN) ? "open drain" : "push-pull");
    }

    /**
     * @brief  isReady
     * @note   Query whether the sensor is ready
     * @retval 1 OK , 0 Not ready
     */
    bool isReady()
    {
        uint8_t  boot_status = 0;
        __error_code = bhy2_get_boot_status(&boot_status, bhy2);
        log_i("boot_status:0x%x", boot_status);
        if (__error_code != BHY2_OK) {
            return false;
        }
        return (boot_status & BHY2_BST_HOST_INTERFACE_READY);
    }

    /**
     * @brief  getKernelVersion
     * @note   Get the sensor firmware kernel version
     * @retval 2 bytes
     */
    uint16_t getKernelVersion()
    {
        uint16_t version = 0;
        __error_code = bhy2_get_kernel_version(&version, bhy2);
        if ((__error_code != BHY2_OK) && (version == 0)) {
            return 0;
        }
        log_d("Boot successful. Kernel version %u.", version);
        return version;
    }


    /**
     * @brief  onEvent
     * @note   Registered sensor event callback function
     * @param  callback: Callback Function
     * @retval None
     */
    void onEvent(BhyEventCb callback)
    {
        BoschParse::_event_callback = callback;
    }

    /**
     * @brief  removeEvent
     * @note   Remove sensor event callback function
     * @retval None
     */
    void removeEvent()
    {
        BoschParse::_event_callback = NULL;
    }

    /**
     * @brief  onResultEvent
     * @note   Registered sensor result callback function , The same sensor ID can register multiple event callbacks.
     *         Please note that you should not register the same event callback repeatedly.
     * @param  sensor_id: Sensor ID , see enum BhySensorID
     * @param  callback: Callback Function
     * @retval None
     */
    void onResultEvent(BhySensorID sensor_id, BhyParseDataCallback callback)
    {
#ifdef USE_STD_VECTOR
        ParseCallBackList_t newEventHandler;
        newEventHandler.cb = callback;
        newEventHandler.id = sensor_id;
        BoschParse::bhyParseEventVector.push_back(newEventHandler);
#else
        if (BoschParse::BoschParse_bhyParseEventVectorSize == BoschParse::BoschParse_bhyParseEventVectorCapacity) {
            BoschParse::expandParseEventVector();
        }
        ParseCallBackList_t newEventHandler;
        newEventHandler.cb = callback;
        newEventHandler.id = sensor_id;
        BoschParse::BoschParse_bhyParseEventVector[BoschParse::BoschParse_bhyParseEventVectorSize++] = newEventHandler;
#endif
    }

    /**
     * @brief  removeResultEvent
     * @note   Remove the registered result callback function
     * @param  sensor_id: Sensor ID , see enum BhySensorID
     * @param  callback: Callback Function
     * @retval None
     */
    void removeResultEvent(BhySensorID sensor_id, BhyParseDataCallback callback)
    {
        if (!callback) {
            return;
        }
#ifdef USE_STD_VECTOR
        for (uint32_t i = 0; i < BoschParse::bhyParseEventVector.size(); i++) {
            ParseCallBackList_t entry = BoschParse::bhyParseEventVector[i];
            if (entry.cb == callback && entry.id == sensor_id) {
                BoschParse::bhyParseEventVector.erase(BoschParse::bhyParseEventVector.begin() + i);
            }
        }
#else
        for (uint32_t i = 0; i < BoschParse::BoschParse_bhyParseEventVectorSize; i++) {
            ParseCallBackList_t entry = BoschParse::BoschParse_bhyParseEventVector[i];
            if (entry.cb == callback && entry.id == sensor_id) {
                for (uint32_t j = i; j < BoschParse::BoschParse_bhyParseEventVectorSize - 1; j++) {
                    BoschParse::BoschParse_bhyParseEventVector[j] = BoschParse::BoschParse_bhyParseEventVector[j + 1];
                }
                BoschParse::BoschParse_bhyParseEventVectorSize--;
                break;
            }
        }
#endif
    }

    /**
     * @brief  setProcessBufferSize
     * @note   The default value is 512Bytes
     * @param  size: The default value is 512Bytes
     * @retval None
     */
    void setProcessBufferSize(uint32_t size)
    {
        __pro_buf_size = size;
    }

    /**
     * @brief  uploadFirmware
     * @note   Update BHI sensor firmware
     * @param  *firmware: Firmware data address
     * @param  length: Firmware data length
     * @param  write2Flash: 1 is written to external flash, 0 is written to RAM
     * @retval true success or failed
     */
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
                log_i("Flash detected. Erasing flash to upload firmware");
                __error_code = bhy2_erase_flash(start_addr, end_addr, bhy2);
                BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_erase_flash failed!", false);
            } else {
                log_e("Flash not detected");
                return false;
            }
            log_i("Loading firmware into FLASH.");
            __error_code = bhy2_upload_firmware_to_flash(firmware, length, bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_upload_firmware_to_flash failed!", false);
            log_i("Loading firmware into FLASH Done");
        } else {
            log_i("Loading firmware into RAM.");
            log_i("upload size = %lu", length);
            __error_code = bhy2_upload_firmware_to_ram(firmware, length, bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_upload_firmware_to_ram failed!", false);
            log_i("Loading firmware into RAM Done");
        }

        __error_code = bhy2_get_error_value(&sensor_error, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_error_value failed!", false);
        if (sensor_error != BHY2_OK) {
            __error_code = bhy2_get_error_value(&sensor_error, bhy2);
            log_e("%s", get_sensor_error_text(sensor_error));
            return false;
        }


        if (write2Flash) {
            log_i("Booting from FLASH.");
            __error_code = bhy2_boot_from_flash(bhy2);
        } else {
            log_i("Booting from RAM.");
            __error_code = bhy2_boot_from_ram(bhy2);
        }
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2 boot failed!", false);

        __error_code = bhy2_get_error_value(&sensor_error, bhy2);
        if (sensor_error) {
            log_e("%s", get_sensor_error_text(sensor_error));
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

    /**
     * @brief  configure
     * @note   Sensor Configuration
     * @param  sensor_id: Sensor ID , see enum BhySensorID
     * @param  sample_rate: Data output rate, unit: HZ
     * @param  report_latency_ms: Report interval in milliseconds
     * @return bool true-> Success false-> failure
     */
    bool configure(uint8_t sensor_id, float sample_rate, uint32_t report_latency_ms)
    {
        if (!bhy2_is_sensor_available(sensor_id, bhy2)) {
            log_e("Sensor not present"); return false;
        }
        __error_code = bhy2_set_virt_sensor_cfg(sensor_id, sample_rate, report_latency_ms, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_set_virt_sensor_cfg failed!", false);
        log_i("Enable %s at %.2fHz.", get_sensor_name(sensor_id), sample_rate);
        return true;
    }

    /**
     * @brief  configureRange
     * @note   Set range of the sensor
     * @param  sensor_id: Sensor ID , see enum BhySensorID
     * @param  range:     Range for selected SensorID. See Table 79 in BHY260 datasheet 109 page
     * @retval  bool true-> Success false-> failure
     */
    bool configureRange(uint8_t sensor_id, uint16_t range)
    {
        __error_code = bhy2_set_virt_sensor_range(sensor_id, range, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_set_virt_sensor_range failed!", false);
        return true;
    }


    /**
     * @brief  getConfigure
     * @note   Get sensor configuration
     * @param  sensor_id: Sensor ID , see enum BhySensorID
     * @retval  struct bhy2_virt_sensor_conf
     */
    struct bhy2_virt_sensor_conf getConfigure(uint8_t sensor_id)
    {
        bhy2_virt_sensor_conf conf;
        bhy2_get_virt_sensor_cfg(sensor_id, &conf, bhy2);
        log_i("range:%u sample_rate:%f latency:%lu sensitivity:%u\n", conf.range, conf.sample_rate, conf.latency, conf.sensitivity);
        return conf;
    }

    /**
     * @brief  getScaling
     * @note   Get sensor scale factor
     * @param  sensor_id: Sensor ID , see enum BhySensorID
     * @retval scale factor
     */
    float getScaling(uint8_t sensor_id)
    {
        return get_sensor_default_scaling(sensor_id);
    }

    /**
     * @brief  setFirmware
     * @note   Set the firmware
     * @param  *image: firmware data address
     * @param  image_len: firmware length
     * @param  write_flash: true : write to flash otherwise ram
     * @param  force_update: true, rewrite to flash or ram regardless of whether there is firmware, false, do not write if firmware is detected
     * @retval None
     */
    void setFirmware(const uint8_t *image, size_t image_len, bool write_flash, bool force_update = false)
    {
        __firmware = image;
        __firmware_size = image_len;
        __write_flash = write_flash;
        __force_update = force_update;
    }

    /**
     * @brief  getSensorName
     * @note   Get sensor name
     * @param  sensor_id: Sensor ID , see enum BhySensorID
     * @retval sensor name
     */
    static const char *getSensorName(uint8_t sensor_id)
    {
        return get_sensor_name(sensor_id);
    }

    // Get an accuracy report
    uint8_t getAccuracy()
    {
        return __accuracy;
    }

    /**
     * @brief  digitalRead
     * @note   Read GPIO level, only for custom firmware
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @param  pullup: true is set pullup or input mode
     * @retval 1 is high ,0 is low
     */
    uint8_t digitalRead(uint8_t pin, bool pullup = false)
    {
        if (pin > JTAG_DIO)return 0;
        uint32_t pin_mask = pin   | BHY2_GPIO_SET;
        if (pullup) {
            pin_mask |= (BHY2_INPUT_PULLUP << 8);
        } else {
            pin_mask |= (BHY2_INPUT << 8);
        }
        bhy2_set_virt_sensor_cfg(SENSOR_ID_GPIO_EXP, (float)pin_mask, 0, bhy2);
        pin_mask = pin /*GetCmd*/;
        bhy2_set_virt_sensor_cfg(SENSOR_ID_GPIO_EXP, (float)pin_mask, 0, bhy2);
        bhy2_virt_sensor_conf conf;
        bhy2_get_virt_sensor_cfg(SENSOR_ID_GPIO_EXP, &conf, bhy2);
        uint8_t level = conf.sample_rate;
        return level;
    }

    /**
     * @brief  digitalWrite
     * @note   Write GPIO level, only for custom firmware
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @param  level: 1 is high ,0 is low
     * @retval None
     */
    void digitalWrite(uint8_t pin, uint8_t level)
    {
        if (pin > JTAG_DIO)return;
        uint32_t pin_mask = pin  | (BHY2_OUTPUT << 8) | (level << 6) | BHY2_GPIO_SET ;
        bhy2_set_virt_sensor_cfg(SENSOR_ID_GPIO_EXP, (float)pin_mask, 0, bhy2);
    }

    /**
     * @brief  disableGpio
     * @note   Disable GPIO function
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @retval None
     */
    void disableGpio(uint8_t pin)
    {
        if (pin > JTAG_DIO)return;
        uint32_t pin_mask = pin  | (BHY2_OPEN_DRAIN << 8) | BHY2_GPIO_SET;
        bhy2_set_virt_sensor_cfg(SENSOR_ID_GPIO_EXP, (float)pin_mask, 0, bhy2);
    }

    /**
     * @brief  setDebug
     * @note   Whether to enable chip debug output
     * @param  enable: true Enable message debug , false disable debug , Requires firmware support, the default firmware will not output any messages
     * @param  &serial: Stream
     * @retval None
     */
    void setDebug(bool enable)
    {
        uint8_t data = 0;
        bhy2_get_host_interrupt_ctrl(&data, bhy2);
        if (enable) {
            data &= ~BHY2_ICTL_DISABLE_DEBUG;           /* Enable debug interrupts */
        } else {
            data |= BHY2_ICTL_DISABLE_DEBUG;            /* Disable debug interrupts */
        }
        bhy2_set_host_interrupt_ctrl(data, bhy2);
        bhy2_register_fifo_parse_callback(BHY2_SYS_ID_DEBUG_MSG, enable ? BoschParse::parseDebugMessage : NULL, NULL, bhy2);
    }

    /**
     * @brief setDebugCallback
     * @param  cb: Sensor debug output callback function , Requires firmware support, the default firmware will not output any messages
     * @retval None
     */
    void setDebugCallback(BhyDebugMessageCallback cb)
    {
        BoschParse::_debug_callback = cb;
    }

private:

#if 0
    void get_phy_sensor_info(uint8_t sens_id)
    {
        Stream &stream = Serial;
        int8_t assert_rslt = BHY2_OK;
        uint16_t param_id = 0;
        struct bhy2_phys_sensor_info psi;

        memset(&psi, 0, sizeof(psi));

        if (!bhy2)return;

        // sens_id = (uint8_t)atoi((char *)&payload[0]);
        param_id = (uint16_t)(0x0120 | sens_id);

        if (param_id >= 0x0121 && param_id <= 0x0160) {
            BHY2_ASSERT(bhy2_get_phys_sensor_info(sens_id, &psi, bhy2));
            if (assert_rslt != BHY2_OK) {
                return;
            }

            stream.printf("Field Name            hex                    | Value (dec)\r\n");
            stream.printf("----------------------------------------------------------\r\n");
            stream.printf("Physical Sensor ID    %02X                     | %d\r\n", psi.sensor_type, psi.sensor_type);
            stream.printf("Driver ID             %02X                     | %d\r\n", psi.driver_id, psi.driver_id);
            stream.printf("Driver Version        %02X                     | %d\r\n", psi.driver_version, psi.driver_version);
            stream.printf("Current Consumption   %02X                     | %0.3fmA\r\n",
                          psi.power_current,
                          psi.power_current / 10.f);
            stream.printf("Dynamic Range         %04X                   | %d\r\n", psi.curr_range.u16_val, psi.curr_range.u16_val);

            const char *irq_status[2] = { "Disabled", "Enabled" };
            const char *master_intf[5] = { "None", "SPI0", "I2C0", "SPI1", "I2C1" };
            const char *power_mode[8] = {
                "Sensor Not Present", "Power Down", "Suspend", "Self-Test", "Interrupt Motion", "One Shot",
                "Low Power Active", "Active"
            };

            stream.printf("Flags                 %02X                     | IRQ status       : %s\r\n", psi.flags,
                          irq_status[psi.flags & 0x01]);
            stream.printf("                                             | Master interface : %s\r\n",
                          master_intf[(psi.flags >> 1) & 0x0F]);
            stream.printf("                                             | Power mode       : %s\r\n",
                          power_mode[(psi.flags >> 5) & 0x07]);
            stream.printf("Slave Address         %02X                     | %d\r\n", psi.slave_address, psi.slave_address);
            stream.printf("GPIO Assignment       %02X                     | %d\r\n", psi.gpio_assignment, psi.gpio_assignment);
            stream.printf("Current Rate          %08X               | %.3fHz\r\n", psi.curr_rate.u32_val, psi.curr_rate.f_val);
            stream.printf("Number of axes        %02X                     | %d\r\n", psi.num_axis, psi.num_axis);

#define INT4_TO_INT8(INT4)  ((int8_t)(((INT4) > 1) ? -1 : (INT4)))
            struct bhy2_orient_matrix ort_mtx = { 0 };
            ort_mtx.c[0] = INT4_TO_INT8(psi.orientation_matrix[0] & 0x0F);
            ort_mtx.c[1] = INT4_TO_INT8(psi.orientation_matrix[0] >> 8);
            ort_mtx.c[2] = INT4_TO_INT8(psi.orientation_matrix[1] & 0x0F);
            ort_mtx.c[3] = INT4_TO_INT8(psi.orientation_matrix[1] >> 8);
            ort_mtx.c[4] = INT4_TO_INT8(psi.orientation_matrix[2] & 0x0F);
            ort_mtx.c[5] = INT4_TO_INT8(psi.orientation_matrix[2] >> 8);
            ort_mtx.c[6] = INT4_TO_INT8(psi.orientation_matrix[3] & 0x0F);
            ort_mtx.c[7] = INT4_TO_INT8(psi.orientation_matrix[3] >> 8);
            ort_mtx.c[8] = INT4_TO_INT8(psi.orientation_matrix[4] & 0x0F);

            stream.printf("Orientation Matrix    %02X%02X%02X%02X%02X             | %+02d %+02d %+02d |\r\n",
                          psi.orientation_matrix[0],
                          psi.orientation_matrix[1],
                          psi.orientation_matrix[2],
                          psi.orientation_matrix[3],
                          psi.orientation_matrix[4],
                          ort_mtx.c[0],
                          ort_mtx.c[1],
                          ort_mtx.c[2]);
            stream.printf("                                             | %+02d %+02d %+02d |\r\n",
                          ort_mtx.c[3],
                          ort_mtx.c[4],
                          ort_mtx.c[5]);
            stream.printf("                                             | %+02d %+02d %+02d |\r\n",
                          ort_mtx.c[6],
                          ort_mtx.c[7],
                          ort_mtx.c[8]);
            stream.printf("Reserved              %02X                     | %d\r\n", psi.reserved, psi.reserved);
            stream.printf("\r\n");

        }
    }
#endif


    bool bootFromFlash()
    {
        int8_t rslt;
        uint8_t boot_status, feat_status;
        uint8_t error_val = 0;
        uint16_t tries = 300; /* Wait for up to little over 3s */

        log_d("Waiting for firmware verification to complete");
        do {
            __error_code = bhy2_get_boot_status(&boot_status, bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);
            if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
                break;
            }
            delay(10);
        } while (tries--);

        __error_code = bhy2_get_boot_status(&boot_status, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);
        print_boot_status(boot_status);

        if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {

            if (boot_status & BHY2_BST_FLASH_DETECTED) {

                /* If no firmware is running, boot from Flash */
                log_d("Booting from flash");
                rslt = bhy2_boot_from_flash(bhy2);
                if (rslt != BHY2_OK) {
                    log_e("%s. Booting from flash failed.\r\n", get_api_error(rslt));
                    __error_code = bhy2_get_regs(BHY2_REG_ERROR_VALUE, &error_val, 1, bhy2);
                    BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_regs failed!", false);
                    if (error_val) {
                        log_e("%s\r\n", get_sensor_error_text(error_val));
                    }
                    return false;
                }

                __error_code = bhy2_get_boot_status(&boot_status, bhy2);
                BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_boot_status failed!", false);
                print_boot_status(boot_status);

                if (!(boot_status & BHY2_BST_HOST_INTERFACE_READY)) {
                    /* hub is not ready, need reset hub */
                    log_d("Host interface is not ready, triggering a reset");
                    __error_code = bhy2_soft_reset(bhy2);
                    BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_soft_reset failed!", false);
                }

                __error_code = (bhy2_get_feature_status(&feat_status, bhy2));
                BHY2_RLST_CHECK(__error_code != BHY2_OK, "Reading Feature status failed, booting from flash failed!", false);

            } else {
                log_e("Can't detect external flash");
                return false;
            }
        } else {
            log_e("Host interface is not ready");
            return false;
        }

        log_d("Booting from flash successful");
        return true;
    }


    void print_boot_status(uint8_t boot_status)
    {
        log_d("Boot Status : 0x%02x: ", boot_status);
        if (boot_status & BHY2_BST_FLASH_DETECTED) {
            log_d("Flash detected. ");
        }

        if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
            log_d("Flash verify done. ");
        }

        if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR) {
            log_d("Flash verification failed. ");
        }

        if (boot_status & BHY2_BST_NO_FLASH) {
            log_d("No flash installed. ");
        }

        if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
            log_d("Host interface ready. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) {
            log_d("Firmware verification done. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR) {
            log_d("Firmware verification error. ");
        }

        if (boot_status & BHY2_BST_HOST_FW_IDLE) {
            log_d("Firmware halted. ");
        }
    }

    static void handleISR()
    {
        __data_available = true;
    }

    bool initImpl()
    {
        uint8_t product_id = 0;

        if (__handler.rst != SENSOR_PIN_NONE) {
            pinMode(__handler.rst, OUTPUT);
        }

        reset();

        bhy2 = (struct bhy2_dev *)malloc(sizeof(struct bhy2_dev ));
        BHY2_RLST_CHECK(!bhy2, " Device handler malloc failed!", false);

        switch (__handler.intf) {
        case BHY2_I2C_INTERFACE:
            // esp32s3 test I2C maximum read and write is 64 bytes
            __max_rw_length = 64;
            BHY2_RLST_CHECK(!__handler.u.i2c_dev.wire, "Wire ptr NULL", false);
            if (!SensorInterfaces::setup_interfaces(__handler)) {
                log_e("setup_interfaces failed");
                return false;
            }
            __error_code = bhy2_init(BHY2_I2C_INTERFACE,
                                     SensorInterfaces::bhy2_i2c_read,
                                     SensorInterfaces::bhy2_i2c_write,
                                     SensorInterfaces::bhy2_delay_us,
                                     __max_rw_length, &__handler, bhy2);
            BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_init failed!", false);
            // __error_code = bhy2_set_host_intf_ctrl(BHY2_I2C_INTERFACE, bhy2);
            // BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_set_host_intf_ctrl failed!", false);
            break;

        case BHY2_SPI_INTERFACE:
            // esp32s3 test SPI maximum read and write is 256 bytes
            __max_rw_length = 256;
            BHY2_RLST_CHECK(!__handler.u.spi_dev.spi, "SPI ptr NULL", false);
            if (!SensorInterfaces::setup_interfaces(__handler)) {
                log_e("setup_interfaces failed");
                return false;
            }
            __error_code = bhy2_init(BHY2_SPI_INTERFACE,
                                     SensorInterfaces::bhy2_spi_read,
                                     SensorInterfaces::bhy2_spi_write,
                                     SensorInterfaces::bhy2_delay_us,
                                     __max_rw_length,
                                     &__handler,
                                     bhy2);
            break;
        default:
            return false;
        }

        __error_code = bhy2_soft_reset(bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "reset bhy2 failed!", false);

        __error_code = bhy2_get_product_id(&product_id, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_get_product_id failed!", false);


        /* Check for a valid product ID */
        if (product_id != BHY2_PRODUCT_ID) {
            log_e("Product ID read %X. Expected %X", product_id, BHY2_PRODUCT_ID);
            return false;
        } else {
            log_i("BHI260/BHA260 found. Product ID read %X", product_id);
        }

        // Set default interrupt configure
        uint8_t data = 0, data_exp;
        bhy2_get_host_interrupt_ctrl(&data, bhy2);
        data &= ~BHY2_ICTL_DISABLE_STATUS_FIFO;     /* Enable status interrupts */
        // data &= ~BHY2_ICTL_DISABLE_DEBUG;           /* Enable debug interrupts */
        data |= BHY2_ICTL_DISABLE_DEBUG;           /* Disable debug interrupts */
        data &= ~BHY2_ICTL_EDGE;                    /* Level */
        data &= ~BHY2_ICTL_ACTIVE_LOW;              /* Active high */
        data &= ~BHY2_ICTL_OPEN_DRAIN;              /* Push-pull */
        data_exp = data;
        bhy2_set_host_interrupt_ctrl(data, bhy2);
        bhy2_get_host_interrupt_ctrl(&data, bhy2);
        if (data != data_exp) {
            log_d("Expected Host Interrupt Control (0x07) to have value 0x%x but instead read 0x%x\r\n", data_exp, data);
        }
        /* Config status channel */
        bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, bhy2);
        bhy2_get_host_intf_ctrl(&data, bhy2);
        if (!(data & BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL)) {
            log_d("Expected Host Interface Control (0x06) to have bit 0x%x to be set\r\n", BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL);
        }

        if (!__firmware) {
            // Default write to ram
            setFirmware(bhy2_firmware_image, sizeof(bhy2_firmware_image), false);
        }

        if (__boot_from_flash) {
            if (!bootFromFlash() || __force_update) {
                if (__force_update) {
                    log_i("Force update firmware.");
                }
                //** If the boot from flash fails, re-upload the firmware to flash
                __error_code = bhy2_soft_reset(bhy2);
                BHY2_RLST_CHECK(__error_code != BHY2_OK, "reset bhy2 failed!", false);

                if (!uploadFirmware(__firmware, __firmware_size, __write_flash)) {
                    log_e("uploadFirmware failed!");
                    return false;
                }
            }
        } else {
            // ** Upload firmware to RAM
            if (!uploadFirmware(__firmware, __firmware_size, __write_flash)) {
                log_e("uploadFirmware failed!");
                return false;
            }
        }

        uint16_t version = getKernelVersion();
        BHY2_RLST_CHECK(!version, "getKernelVersion failed!", false);
        log_i("Boot successful. Kernel version %u.", version);

        //Set event callback
        __error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, BoschParse::parseMetaEvent, (void *)&__accuracy, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_register_fifo_parse_callback failed!", false);

        __error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, BoschParse::parseMetaEvent, (void *)&__accuracy, bhy2);
        BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_register_fifo_parse_callback failed!", false);

        // __error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_DEBUG_MSG, BoschParse::parseDebugMessage, NULL, bhy2);
        // BHY2_RLST_CHECK(__error_code != BHY2_OK, "bhy2_register_fifo_parse_callback parseDebugMessage failed!", false);

        //Set process buffer
#if     (defined(ESP32) || defined(ARDUINO_ARCH_ESP32)) && defined(BOARD_HAS_PSRAM)
        __pro_buf = (uint8_t *)ps_malloc(__pro_buf_size);
#else
        __pro_buf = (uint8_t *)malloc(__pro_buf_size);
#endif
        BHY2_RLST_CHECK(!__pro_buf, "process buffer malloc failed!", false);

        __error_code = bhy2_get_and_process_fifo(__pro_buf, __pro_buf_size, bhy2);
        if (__error_code != BHY2_OK) {
            log_e("bhy2_get_and_process_fifo failed");
            free(__pro_buf);
            return false;
        }

        /* Update the callback table to enable parsing of sensor hintr_ctrl */
        bhy2_update_virtual_sensor_list(bhy2);

        /* Get present virtual sensor */
        bhy2_get_virt_sensor_list(bhy2);

        // Only register valid sensor IDs
        for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++) {
            if (bhy2_is_sensor_available(i, bhy2)) {
                bhy2_register_fifo_parse_callback(i, BoschParse::parseData, NULL, bhy2);
            }
        }

        if (__handler.irq != SENSOR_PIN_NONE) {
#if defined(ARDUINO_ARCH_RP2040)
            attachInterrupt((pin_size_t)(__handler.irq), handleISR, (PinStatus )RISING);
#elif defined(ARDUINO_ARCH_NRF52) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_STM32)
            attachInterrupt(__handler.irq, handleISR, RISING);
#else
#error "Interrupt registration not implemented"
#endif
        }

        return __error_code == BHY2_OK;
    }

protected:
    struct bhy2_dev  *bhy2 = NULL;
    SensorLibConfigure __handler;
    int8_t           __error_code = 0;
    static volatile bool __data_available;
    uint8_t          *__pro_buf = NULL;
    size_t           __pro_buf_size = BHY_PROCESS_BUFFER_SIZE;
    const uint8_t    *__firmware = NULL;
    size_t          __firmware_size = 0;
    bool            __write_flash = false;
    bool            __boot_from_flash = false;
    bool            __force_update = false;
    uint16_t        __max_rw_length = 32;
    uint8_t         __accuracy = 0;      /* Accuracy is reported as a meta event. */
};


#endif /*defined(ARDUINO)*/





/**
 *
 * @license MIT License
 *
 * Copyright (c) 2023 lewis he
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
 * @file      TouchDrvCST92xx.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-07
 */
#pragma once

#include "TouchDrvInterface.hpp"

#define CST92XX_SLAVE_ADDRESS      (0x5A)

class TouchDrvCST92xx : public TouchDrvInterface
{
public:
    using TouchDrvInterface::getPoint;
    enum CST92_RunMode {
        MODE_NORMAL = (0x00),
        MODE_LOW_POWER = (0X01),
        MODE_DEEP_SLEEP = (0X02),
        MODE_WAKEUP = (0x03),
        MODE_DEBUG_DIFF = (0x04),
        MODE_DEBUG_RAWDATA = (0X05),
        MODE_FACTORY = (0x06),
        MODE_DEBUG_INFO = (0x07),
        MODE_UPDATE_FW = (0x08),
        MODE_FACTORY_HIGH_DRV = (0x10),
        MODE_FACTORY_LOW_DRV = (0x11),
        MODE_FACTORY_SHORT = (0x12),
        MODE_LPSCAN = (0x13),
    };

    /**
    * @brief  Constructor for the touch driver
    * @retval None
    */
    TouchDrvCST92xx();

    /**
    * @brief  Destructor for the touch driver
    * @retval None
    */
    ~TouchDrvCST92xx() = default;

    /**
    * @brief  Reset the touch driver
    * @note   This function will reset the touch driver by toggling the reset pin.
    * @retval None
    */
    void reset() override;

    /**
    * @brief Puts the touch driver to sleep
    * @note This function puts the touch driver into sleep mode.
    *       If the device does not have a reset pin connected, it cannot be woken up after being put
    *       into sleep mode and must be powered on again.
    * @retval None
    */
    void sleep() override;

    /**
     * @brief  Wake up the touch driver
     * @note   This function will wake up the touch driver from sleep mode.
     * @retval None
     */
    void wakeup() override;

    /**
     * @brief  Get the touch points
     * @note   This function will retrieve the touch points from the touch driver.
     * @retval A reference to the touch points.
     */
    const TouchPoints &getTouchPoints() override;

    /**
    * @brief  Check if the touch point is pressed
    * @note   This function will check if the touch point is currently pressed.
    * @retval True if the touch point is pressed, false otherwise.
    */
    bool isPressed() override;

    /**
    * @brief  Get the model name
    * @note   This function will retrieve the model name from the touch driver.
    * @retval The model name.
    */
    const char *getModelName() override;

    /**
     * @brief  Set the cover screen callback
     * @note   This function will set the callback function for the cover screen.
     * @param  cb: The callback function to be set
     * @param  *user_data: Pointer to user data to be passed to the callback function
     * @retval None
     */
    void setCoverScreenCallback(HomeButtonCallback cb, void *user_data = NULL);

private:
    bool initImpl(uint8_t addr) override;
    bool setMode(uint8_t mode);
    bool enterBootloader();
    bool getAttribute();
    uint32_t readWordFromMem(uint8_t type, uint16_t mem_addr);
    uint32_t getChipType();

protected:
    int _slave_addr;
    uint16_t chipType;

    static constexpr uint16_t  CST9220_CHIP_ID             = (0x9220);
    static constexpr uint16_t  CST9217_CHIP_ID             = (0x9217);
    
    static constexpr uint16_t  REG_READ                    = (0xD000);
    static constexpr uint16_t  REG_DEBUG_MODE              = (0xD101);
    static constexpr uint16_t  REG_SLEEP_MODE              = (0xD105);
    static constexpr uint16_t  REG_DIS_LOW_POWER_SCAN_MODE = (0xD106);
    static constexpr uint16_t  REG_NORMAL_MODE             = (0xD109);
    static constexpr uint16_t  REG_RAW_MODE                = (0xD10A);
    static constexpr uint16_t  REG_DIFF_MODE               = (0xD10D);
    static constexpr uint16_t  REG_BASE_LINE_MODE          = (0xD10E);
    static constexpr uint16_t  REG_LOW_POWER_MODE          = (0xD10F);
    static constexpr uint16_t  REG_FACTORY_MODE            = (0xD114);
    
    static constexpr uint8_t   CST92XX_BOOT_ADDRESS        = (0x5A);
    static constexpr uint8_t   CST92XX_ACK                 = (0xAB);
    static constexpr uint32_t  CST92XX_MEM_SIZE            = (0x007F80);// 31KB

    static constexpr uint8_t   MAX_FINGER_NUM              = (2);
    static constexpr uint8_t   PROGRAM_PAGE_SIZE           = (128);

#if 0  /*DISABLE UPDATE FIRMWARE*/

    struct {
        bool firmware_info_ok;
        uint32_t firmware_ic_type;
        uint32_t firmware_version;
        uint32_t firmware_checksum;
        uint32_t firmware_project_id;
        uint8_t tx_num;
        uint8_t rx_num;
        uint8_t key_num;
    } IC_firmware;

    struct {
        bool ok;
        uint8_t *head_data;
        uint8_t *data;
        uint32_t checksum;
        uint32_t version;
        uint32_t project_id;
        uint32_t chip_type;
    } bin_data;
    uint32_t get_u32_from_ptr(const void *ptr);
    bool getFirmwareInfo(void);
    int16_t eraseMem(void);
    int16_t writeSRAM(uint8_t *buf, uint16_t len);
    int16_t writeMemPage(uint16_t addr, uint8_t *buf, uint16_t len) ;
    int16_t writeMemAll(void) ;
    int16_t calculateVerifyChecksum(void) ;
    int16_t upgradeFirmware(void);
    uint32_t verifyFirmware(uint8_t *pdata, uint16_t order);
    int16_t parseFirmware(void);
    int16_t upgradeFirmwareJudge(void);
    int16_t getFirmwareAddress(uint8_t data_seq, uint16_t data_len) ;
    int16_t updateFirmware(void);
#endif
};

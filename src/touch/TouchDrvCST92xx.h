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

#include "REG/CST9xxConstants.h"
#include "TouchDrvInterface.hpp"

#define CST92XX_SLAVE_ADDRESS      (0x5A)

class TouchDrvCST92xx : public TouchDrvInterface, public CST92xxConstants
{
public:
    enum CST92_RunMode {
        CST92_MODE_NORMAL = (0x00),
        CST92_MODE_LOW_POWER = (0X01),
        CST92_MODE_DEEP_SLEEP = (0X02),
        CST92_MODE_WAKEUP = (0x03),
        CST92_MODE_DEBUG_DIFF = (0x04),
        CST92_MODE_DEBUG_RAWDATA = (0X05),
        CST92_MODE_FACTORY = (0x06),
        CST92_MODE_DEBUG_INFO = (0x07),
        CST92_MODE_UPDATE_FW = (0x08),
        CST92_MODE_FACTORY_HIGHDRV = (0x10),
        CST92_MODE_FACTORY_LOWDRV = (0x11),
        CST92_MODE_FACTORY_SHORT = (0x12),
        CST92_MODE_LPSCAN = (0x13),
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
     * @brief  Get the touch point coordinates
     * @note   This function will retrieve the touch point coordinates from the touch driver.
     * @param  *x_array: Pointer to the array to store the X coordinates
     * @param  *y_array: Pointer to the array to store the Y coordinates
     * @param  size: Number of touch points to retrieve
     * @retval Return the number of touch points retrieved
     */
    uint8_t getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point = 1) override;

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
    void parseFingerData(uint8_t *data,  cst9xx_point_t *point);
    uint32_t readWordFromMem(uint8_t type, uint16_t mem_addr);
    uint32_t get_u32_from_ptr(const void *ptr);
    uint32_t getChipType();

protected:
    int _slave_addr;
    uint16_t chipType;

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

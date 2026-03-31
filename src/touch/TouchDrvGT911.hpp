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
 * @file      TouchDrvGT911.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-04-12
 *
 */
#pragma once

#include "TouchDrvInterface.hpp"

//! Dangerous operation! If power is lost during the writing process, all parameters will be lost due to the absence of the original parameters, resulting in touch screen errors.
// #define ENABLE_GT911_CONFIG


#if defined(ARDUINO_ARCH_NRF52)
// NRF52840 I2C BUFFER : 64 Bytes ,
#warning "NRF Platform I2C Buffer expansion is not implemented , GT911 requires at least 188 bytes to read all configurations"
#endif

static constexpr uint8_t   GT911_SLAVE_ADDRESS_H =  (0x14);
static constexpr uint8_t   GT911_SLAVE_ADDRESS_L =  (0x5D);
static constexpr uint8_t   GT911_SLAVE_ADDRESS_UNKNOWN =  (0xFF);

class TouchDrvGT911 :  public TouchDrvInterface
{
public:
    using TouchDrvInterface::getPoint;

    enum InterruptMode {
        RISING_EDGE = 0x00,
        FALLING_EDGE = 0x01,
        LOW_LEVEL_QUERY = 0x02,
        HIGH_LEVEL_QUERY = 0x03
    };

    /**
    * @brief  Constructor for the touch driver
    * @retval None
    */
    TouchDrvGT911() = default;

    /**
    * @brief  Destructor for the touch driver
    * @retval None
    */
    ~TouchDrvGT911() = default;

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
    * @brief  Get the model name
    * @note   This function will retrieve the model name from the touch driver.
    * @retval The model name.
    */
    const char *getModelName() override;

    /**
    * @brief  Set the interrupt mode
    * @note   This function will set the interrupt mode for the touch driver.
    * @param  mode: The interrupt mode to set , see InterruptMode.
    * @retval True if the operation was successful, false otherwise.
    */
    bool setInterruptMode(InterruptMode mode);

    /**
     * @brief  Get the interrupt mode
     * @note   This function will retrieve the current interrupt mode from the touch driver.
     * @retval The interrupt mode.
     *          0x0: Rising edge trigger
     *          0x1: Falling edge trigger
     *          0x2: Low level query
     *          0x3: High level query
     */
    InterruptMode getInterruptMode();

    /**
     * @brief  Get the chip ID
     * @note   This function will retrieve the chip ID from the touch driver.
     * @retval The chip ID.
     */
    uint32_t getChipID() override;

    /**
     * @brief  Get the firmware version
     * @note   This function will retrieve the firmware version from the touch driver.
     * @retval The firmware version.
     */
    uint16_t getFwVersion();

    /**
     * @brief  Get the configuration version
     * @note   This function will retrieve the configuration version from the touch driver.
     * @retval The configuration version.
     */
    uint8_t getConfigVersion();

    /**
     * @brief  Get the refresh rate
     * @note   This function will retrieve the current refresh rate from the touch driver.
     * @param  rate_ms: Pointer to store the refresh rate in milliseconds , 5 ~ 15 ms.
     * @retval None
     */
    void updateRefreshRate(uint8_t rate_ms);

    /**
     * @brief  Get the refresh rate
     * @note   This function will retrieve the current refresh rate from the touch driver.
     * @retval The refresh rate in milliseconds.
     */
    uint8_t getRefreshRate();

    /**
     * @brief  Get the vendor ID
     * @note   This function will retrieve the vendor ID from the touch driver.
     * @retval The vendor ID.
     */
    int getVendorID();

#ifdef ENABLE_GT911_CONFIG
    /**
     * @brief  Set the home button callback
     * @note   This function will set the callback function for the home button.
     * @param  *config_buffer: Pointer to the configuration buffer.
     * @param  buffer_size: Size of the configuration buffer.
     * @retval None
     */
    bool writeConfig(const uint8_t *config_buffer, size_t buffer_size);
#endif

    /**
     * @brief  Load the configuration data
     * @note   This function will load the configuration data from the touch driver.
     *         Memory allocation is automatically managed within the class and does not require manual deallocation.
     * @param  *output_size: Pointer to store the size of the output data.
     * @param  print_out: Flag to indicate whether to print the output.
     * @retval The pointer to the configuration data buffer.
     */
    uint8_t *loadConfig(size_t *output_size, bool print_out = false);

    /**
     * @brief  Reload the configuration data
     * @note   This function will reload the configuration data from the touch driver.
     * @retval True if the operation was successful, false otherwise.
     */
    bool reloadConfig();

    /**
     * @brief  Set the maximum touch points
     * @note   This function will set the maximum number of touch points.
     * @param  num: The maximum number of touch points (1-5).
     * @retval None
     */
    void setMaxTouchPoint(uint8_t num);


private:
    /**
     * @brief  Read a register from the GT911 touch driver
     * @note   This function will read a register from the touch driver.
     * @param  cmd: The register address to read.
     * @retval The value read from the register.
     */
    uint8_t readGT911(uint16_t cmd);
    /**
     * @brief  Write a register to the GT911 touch driver
     * @note   This function will write a register to the touch driver.
     * @param  cmd: The register address to write.
     * @param  value: The value to write to the register.
     * @retval None
     */
    int writeGT911(uint16_t cmd, uint8_t value);

    /**
     * @brief  Write a command to the GT911 touch driver
     * @note   This function will write a command to the touch driver.
     * @param  command: The command to write.
     * @retval None
     */
    void writeCommand(uint8_t command);

    /**
     * @brief  Clear the touch buffer
     * @note   This function will clear the touch buffer.
     * @retval None
     */
    void clearBuffer();

    /**
     * @brief  Probe the I2C address of the GT911 touch driver
     * @note   This function will try to communicate with the touch driver at the specified I2C address.
     * @retval True if the device is found, false otherwise.
     */
    bool probeAddress();

    bool initImpl(uint8_t) override;

    bool autoProbe();



    static constexpr uint8_t MAX_FINGER_NUM = (5);
    static constexpr uint8_t BYTES_PER_POINT = (8);

protected:
    // Real-time command (Write only)
    static constexpr uint16_t  GT911_COMMAND                 = (0x8040);
    static constexpr uint16_t  GT911_ESD_CHECK               = (0x8041);
    static constexpr uint16_t  GT911_COMMAND_CHECK           = (0x8046);
    static constexpr uint8_t   IRQ_TRIGGER_MASK              = (0x03);

    // Configuration information (R/W)
    static constexpr uint16_t  GT911_CONFIG_START            = (0x8047);
    static constexpr uint16_t  GT911_CONFIG_VERSION          = (0x8047);
    static constexpr uint16_t  GT911_X_OUTPUT_MAX_LOW        = (0x8048);
    static constexpr uint16_t  GT911_X_OUTPUT_MAX_HIGH       = (0x8049);
    static constexpr uint16_t  GT911_Y_OUTPUT_MAX_LOW        = (0x804A);
    static constexpr uint16_t  GT911_Y_OUTPUT_MAX_HIGH       = (0x804B);
    static constexpr uint16_t  GT911_TOUCH_NUMBER            = (0x804C);
    static constexpr uint16_t  GT911_MODULE_SWITCH_1         = (0x804D);
    static constexpr uint16_t  GT911_MODULE_SWITCH_2         = (0x804E);
    static constexpr uint16_t  GT911_SHAKE_COUNT             = (0x804F);
    static constexpr uint16_t  GT911_FILTER                  = (0x8050);
    static constexpr uint16_t  GT911_LARGE_TOUCH             = (0x8051);
    static constexpr uint16_t  GT911_NOISE_REDUCTION         = (0x8052);
    static constexpr uint16_t  GT911_SCREEN_TOUCH_LEVEL      = (0x8053);
    static constexpr uint16_t  GT911_SCREEN_RELEASE_LEVEL    = (0x8054);
    static constexpr uint16_t  GT911_LOW_POWER_CONTROL       = (0x8055);
    static constexpr uint16_t  GT911_REFRESH_RATE            = (0x8056);
    static constexpr uint16_t  GT911_X_THRESHOLD             = (0x8057);
    static constexpr uint16_t  GT911_Y_THRESHOLD             = (0x8058);
    static constexpr uint16_t  GT911_X_SPEED_LIMIT           = (0x8059); // Reserve
    static constexpr uint16_t  GT911_Y_SPEED_LIMIT           = (0x805A); // Reserve
    static constexpr uint16_t  GT911_SPACE_TOP_BOTTOM        = (0x805B);
    static constexpr uint16_t  GT911_SPACE_LEFT_RIGHT        = (0x805C);
    static constexpr uint16_t  GT911_MINI_FILTER             = (0x805D);
    static constexpr uint16_t  GT911_STRETCH_R0              = (0x805E);
    static constexpr uint16_t  GT911_STRETCH_R1              = (0x805F);
    static constexpr uint16_t  GT911_STRETCH_R2              = (0x8060);
    static constexpr uint16_t  GT911_STRETCH_RM              = (0x8061);
    static constexpr uint16_t  GT911_DRV_GROUPA_NUM          = (0x8062);
    static constexpr uint16_t  GT911_DRV_GROUPB_NUM          = (0x8063);
    static constexpr uint16_t  GT911_SENSOR_NUM              = (0x8064);
    static constexpr uint16_t  GT911_FREQ_A_FACTOR           = (0x8065);
    static constexpr uint16_t  GT911_FREQ_B_FACTOR           = (0x8066);
    static constexpr uint16_t  GT911_PANEL_BIT_FREQ_L        = (0x8067);
    static constexpr uint16_t  GT911_PANEL_BIT_FREQ_H        = (0x8068);
    static constexpr uint16_t  GT911_PANEL_SENSOR_TIME_L     = (0x8069); // Reserve
    static constexpr uint16_t  GT911_PANEL_SENSOR_TIME_H     = (0x806A);
    static constexpr uint16_t  GT911_PANEL_TX_GAIN           = (0x806B);
    static constexpr uint16_t  GT911_PANEL_RX_GAIN           = (0x806C);
    static constexpr uint16_t  GT911_PANEL_DUMP_SHIFT        = (0x806D);
    static constexpr uint16_t  GT911_DRV_FRAME_CONTROL       = (0x806E);
    static constexpr uint16_t  GT911_CHARGING_LEVEL_UP       = (0x806F);
    static constexpr uint16_t  GT911_MODULE_SWITCH3          = (0x8070);
    static constexpr uint16_t  GT911_GESTURE_DIS             = (0X8071);
    static constexpr uint16_t  GT911_GESTURE_LONG_PRESS_TIME = (0x8072);
    static constexpr uint16_t  GT911_X_Y_SLOPE_ADJUST        = (0X8073);
    static constexpr uint16_t  GT911_GESTURE_CONTROL         = (0X8074);
    static constexpr uint16_t  GT911_GESTURE_SWITCH1         = (0X8075);
    static constexpr uint16_t  GT911_GESTURE_SWITCH2         = (0X8076);
    static constexpr uint16_t  GT911_GESTURE_REFRESH_RATE    = (0x8077);
    static constexpr uint16_t  GT911_GESTURE_TOUCH_LEVEL     = (0x8078);
    static constexpr uint16_t  GT911_NEWGREENWAKEUPLEVEL     = (0x8079);
    static constexpr uint16_t  GT911_FREQ_HOPPING_START      = (0x807A);
    static constexpr uint16_t  GT911_FREQ_HOPPING_END        = (0X807B);
    static constexpr uint16_t  GT911_NOISE_DETECT_TIMES      = (0x807C);
    static constexpr uint16_t  GT911_HOPPING_FLAG            = (0X807D);
    static constexpr uint16_t  GT911_HOPPING_THRESHOLD       = (0X807E);
    static constexpr uint16_t  GT911_NOISE_THRESHOLD         = (0X807F); // Reserve
    static constexpr uint16_t  GT911_NOISE_MIN_THRESHOLD     = (0X8080);
    static constexpr uint16_t  GT911_HOPPING_SENSOR_GROUP    = (0X8082);
    static constexpr uint16_t  GT911_HOPPING_SEG1_NORMALIZE  = (0X8083);
    static constexpr uint16_t  GT911_HOPPING_SEG1_FACTOR     = (0X8084);
    static constexpr uint16_t  GT911_MAIN_CLOCK_ADJUST       = (0X8085);
    static constexpr uint16_t  GT911_HOPPING_SEG2_NORMALIZE  = (0X8086);
    static constexpr uint16_t  GT911_HOPPING_SEG2_FACTOR     = (0X8087);
    static constexpr uint16_t  GT911_HOPPING_SEG3_NORMALIZE  = (0X8089);
    static constexpr uint16_t  GT911_HOPPING_SEG3_FACTOR     = (0X808A);
    static constexpr uint16_t  GT911_HOPPING_SEG4_NORMALIZE  = (0X808C);
    static constexpr uint16_t  GT911_HOPPING_SEG4_FACTOR     = (0X808D);
    static constexpr uint16_t  GT911_HOPPING_SEG5_NORMALIZE  = (0X808F);
    static constexpr uint16_t  GT911_HOPPING_SEG5_FACTOR     = (0X8090);
    static constexpr uint16_t  GT911_HOPPING_SEG6_NORMALIZE  = (0X8092);
    static constexpr uint16_t  GT911_KEY_1                   = (0X8093);
    static constexpr uint16_t  GT911_KEY_2                   = (0X8094);
    static constexpr uint16_t  GT911_KEY_3                   = (0X8095);
    static constexpr uint16_t  GT911_KEY_4                   = (0X8096);
    static constexpr uint16_t  GT911_KEY_AREA                = (0X8097);
    static constexpr uint16_t  GT911_KEY_TOUCH_LEVEL         = (0X8098);
    static constexpr uint16_t  GT911_KEY_LEAVE_LEVEL         = (0X8099);
    static constexpr uint16_t  GT911_KEY_SENS_1_2            = (0X809A);
    static constexpr uint16_t  GT911_KEY_SENS_3_4            = (0X809B);
    static constexpr uint16_t  GT911_KEY_RESTRAIN            = (0X809C);
    static constexpr uint16_t  GT911_KEY_RESTRAIN_TIME       = (0X809D);
    static constexpr uint16_t  GT911_GESTURE_LARGE_TOUCH     = (0X809E);
    static constexpr uint16_t  GT911_HOTKNOT_NOISE_MAP       = (0X80A1);
    static constexpr uint16_t  GT911_LINK_THRESHOLD          = (0X80A2);
    static constexpr uint16_t  GT911_PXY_THRESHOLD           = (0X80A3);
    static constexpr uint16_t  GT911_GHOT_DUMP_SHIFT         = (0X80A4);
    static constexpr uint16_t  GT911_GHOT_RX_GAIN            = (0X80A5);
    static constexpr uint16_t  GT911_FREQ_GAIN0              = (0X80A6);
    static constexpr uint16_t  GT911_FREQ_GAIN1              = (0X80A7);
    static constexpr uint16_t  GT911_FREQ_GAIN2              = (0X80A8);
    static constexpr uint16_t  GT911_FREQ_GAIN3              = (0X80A9);
    static constexpr uint16_t  GT911_COMBINE_DIS             = (0X80B3);
    static constexpr uint16_t  GT911_SPLIT_SET               = (0X80B4);
    static constexpr uint16_t  GT911_SENSOR_CH0              = (0X80B7);
    static constexpr uint16_t  GT911_DRIVER_CH0              = (0X80D5);
    static constexpr uint16_t  GT911_CONFIG_CHKSUM           = (0X80FF);
    static constexpr uint16_t  GT911_CONFIG_FRESH            = (0X8100);
    static constexpr uint16_t  GT911_CONFIG_SIZE             = (0xFF - 0x46);
    // Coordinate information
    static constexpr uint16_t  GT911_PRODUCT_ID              = (0x8140);
    static constexpr uint16_t  GT911_FIRMWARE_VERSION        = (0x8144);
    static constexpr uint16_t  GT911_X_RESOLUTION            = (0x8146);
    static constexpr uint16_t  GT911_Y_RESOLUTION            = (0x8148);

    static constexpr uint16_t  GT911_VENDOR_ID               = (0X8140);
    static constexpr uint16_t  GT911_INFORMATION             = (0X8140);
    static constexpr uint16_t  GT911_POINT_INFO              = (0X814E);
    static constexpr uint16_t  GT911_POINT_1                 = (0X814F);
    static constexpr uint16_t  GT911_POINT_2                 = (0X8157);
    static constexpr uint16_t  GT911_POINT_3                 = (0X815F);
    static constexpr uint16_t  GT911_POINT_4                 = (0X8167);
    static constexpr uint16_t  GT911_POINT_5                 = (0X816F);

    static constexpr uint16_t  COMMAND_SIZE                  = (2);

    static constexpr uint16_t  GT911_DEV_ID                   =  (911);
    static constexpr uint8_t   GT911_BASE_REF_RATE             =  (5);
    static constexpr uint8_t   GT911_REG_LENGTH                =  (186);
};

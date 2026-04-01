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
 * @file      TouchDrvGT911.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-05
 *
 */
#include "TouchDrvGT911.hpp"

void TouchDrvGT911::sleep()
{
    if (_pinsCfg.irqPin != -1) {
        hal->pinMode(_pinsCfg.irqPin, OUTPUT);
        hal->digitalWrite(_pinsCfg.irqPin, LOW);
    }
    writeCommand(0x05);

    /*
    * Depending on the chip and platform, setting it to input after removing sleep will affect power consumption.
    * The chip platform determines whether
    *
    * * */
    // if (_pinsCfg.irqPin != -1) {
    //     hal->digitalWrite(_pinsCfg.irqPin, INPUT);
    // }
}

void TouchDrvGT911::wakeup()
{
    if (_pinsCfg.irqPin != -1) {
        hal->pinMode(_pinsCfg.irqPin, OUTPUT);
        hal->digitalWrite(_pinsCfg.irqPin, HIGH);
        hal->delay(8);
        hal->pinMode(_pinsCfg.irqPin, INPUT);
    } else {
        reset();
    }
}

const TouchPoints &TouchDrvGT911::getTouchPoints()
{
    static constexpr uint8_t POINT_BUFFER_SIZE = (MAX_FINGER_NUM * BYTES_PER_POINT);
    uint8_t buffer[POINT_BUFFER_SIZE] = {0};
    int numPoints = 0;

    // Clear cached touch points
    _touchPoints.clear();

    numPoints = readGT911(GT911_POINT_INFO);
    // If there is a home button callback and the home button is pressed
    if (_HButtonCallback && (numPoints & 0x10)) {
        _HButtonCallback(_userData);
        clearBuffer();
        return _touchPoints;
    }

    // Clear the buffer
    clearBuffer();

    // Mask the number of points
    numPoints &= 0x0F;
    if (numPoints <= 0 || numPoints > MAX_FINGER_NUM) {
        return _touchPoints;
    }

    uint8_t expectedBytes = numPoints * BYTES_PER_POINT; // 8 bytes per touch point

    // GT911_POINT_1  0X814F
    addrToBeBuf(GT911_POINT_1, buffer);

    if (writeThenRead(buffer, COMMAND_SIZE, buffer, expectedBytes) == 0) {
        for (int i = 0; i < numPoints; i++) {
            const uint8_t id = buffer[i * BYTES_PER_POINT];
            const uint16_t x = buffer[1 + i * BYTES_PER_POINT] | (buffer[2 + i * BYTES_PER_POINT] << 8);
            const uint16_t y = buffer[3 + i * BYTES_PER_POINT] | (buffer[4 + i * BYTES_PER_POINT] << 8);
            const uint16_t pressure = buffer[5 + i * BYTES_PER_POINT] | (buffer[6 + i * BYTES_PER_POINT] << 8);
            _touchPoints.addPoint(x, y, pressure, id);
        }

        // Swap XY or mirroring coordinates,if set
        updateXY(_touchPoints);
    }

    return _touchPoints;
}

const char *TouchDrvGT911::getModelName()
{
    return "GT911";
}

bool TouchDrvGT911::setInterruptMode(InterruptMode mode)
{
    // GT911_MODULE_SWITCH_1 0x804D
    uint8_t oldTriggerLevel = _pinsCfg.irqTriggerLevel;
    uint8_t irqMode = readGT911(GT911_MODULE_SWITCH_1);
    irqMode &= (~IRQ_TRIGGER_MASK);
    switch (mode) {
    case FALLING_EDGE:
        irqMode |= 0x01;
        _pinsCfg.irqTriggerLevel = LOW;
        break;
    case RISING_EDGE:
        irqMode |= 0x00;
        _pinsCfg.irqTriggerLevel = HIGH;
        break;
    case LOW_LEVEL_QUERY:
        irqMode |= 0x02;
        _pinsCfg.irqTriggerLevel = LOW;
        break;
    case HIGH_LEVEL_QUERY:
        irqMode |= 0x03;
        _pinsCfg.irqTriggerLevel = HIGH;
        break;
    }
    writeGT911(GT911_MODULE_SWITCH_1, irqMode);
    if (!reloadConfig()) {
        _pinsCfg.irqTriggerLevel = oldTriggerLevel;
        return false;
    }
    return true;
}

TouchDrvGT911::InterruptMode TouchDrvGT911::getInterruptMode()
{
    uint8_t irqMode = readGT911(GT911_MODULE_SWITCH_1);
    irqMode &= IRQ_TRIGGER_MASK;
    switch (irqMode) {
    case 0:     ///< RISING_EDGE
        _pinsCfg.irqTriggerLevel = HIGH;
        break;
    case 1:    ///< FALLING_EDGE
        _pinsCfg.irqTriggerLevel = LOW;
        break;
    case 2:   ///< LOW_LEVEL_QUERY
        _pinsCfg.irqTriggerLevel = LOW;
        break;
    case 3:  ///< HIGH_LEVEL_QUERY
        _pinsCfg.irqTriggerLevel = HIGH;
        break;
    default:
        break;
    }
    return static_cast<InterruptMode>(irqMode);
}

uint32_t TouchDrvGT911::getChipID()
{
    char product_id[4] = {0};
    // GT911_PRODUCT_ID 0x8140
    for (int i = 0; i < 4; ++i) {
        product_id[i] = readGT911(GT911_PRODUCT_ID + i);
    }
    return atoi(product_id);
}

uint16_t TouchDrvGT911::getFwVersion()
{
    uint8_t fw_ver[2] = {0};
    // GT911_FIRMWARE_VERSION 0x8144
    for (int i = 0; i < 2; ++i) {
        fw_ver[i] = readGT911(GT911_FIRMWARE_VERSION + i);
    }
    return fw_ver[0] | (fw_ver[1] << 8);
}

uint8_t TouchDrvGT911::getConfigVersion()
{
    return readGT911(GT911_CONFIG_VERSION);
}

void TouchDrvGT911::updateRefreshRate(uint8_t rate_ms)
{
    if (rate_ms < 5) {
        rate_ms = 5;
    }
    if (rate_ms > 15) {
        rate_ms = 15;
    }
    rate_ms -= 5;
    writeGT911(GT911_REFRESH_RATE, rate_ms);
    reloadConfig();
}

uint8_t TouchDrvGT911::getRefreshRate()
{
    uint8_t rate_ms  = readGT911(GT911_REFRESH_RATE);
    return rate_ms + GT911_BASE_REF_RATE ;
}

int TouchDrvGT911::getVendorID()
{
    return readGT911(GT911_VENDOR_ID);
}

#ifdef ENABLE_GT911_CONFIG
bool TouchDrvGT911::writeConfig(const uint8_t *config_buffer, size_t buffer_size)
{
    uint8_t check_sum = 0;
    for (int i = 0; i < (GT911_REG_LENGTH - 2 ); i++) {
        check_sum += config_buffer[i];
    }
    check_sum =  (~check_sum) + 1;
    if (check_sum != config_buffer[GT911_REG_LENGTH - 2]) {
        log_e("Config checksum error !");
        return false;
    }
    log_d("Update touch config , write %lu Bytes check sum:0x%X", buffer_size, check_sum);
    uint8_t cmd[] = {lowByte(GT911_CONFIG_VERSION), highByte(GT911_CONFIG_VERSION)};
    int err =  writeBuff(GT911_CONFIG_VERSION, (uint8_t *)config_buffer, buffer_size);
    return err == 0;
}
#endif

// Load touch configuration ,need manual release memory
uint8_t *TouchDrvGT911::loadConfig(size_t *output_size, bool print_out)
{
    *output_size = 0;
    uint8_t *buffer = (uint8_t *)malloc(GT911_REG_LENGTH);
    if (!buffer)return NULL;
    addrToBeBuf(GT911_CONFIG_VERSION, buffer);
    if (writeThenRead(buffer, COMMAND_SIZE, buffer, GT911_REG_LENGTH) == -1) {
        return NULL;
    }
    if (print_out) {
        printf("const unsigned char config[186] = {");
        for (int i = 0; i < GT911_REG_LENGTH; ++i) {
            if ( (i % 8) == 0) {
                printf("\n");
            }
            printf(" 0x%02X", buffer[i]);
            if ((i + 1) < GT911_REG_LENGTH) {
                printf(",");
            }
        }
        printf("};\n");
    }
    *output_size = GT911_REG_LENGTH;
    return buffer;
}

bool TouchDrvGT911::reloadConfig()
{
    uint8_t buffer[GT911_REG_LENGTH] = {};
    buffer[0] = highByte(GT911_CONFIG_VERSION);
    buffer[1] = lowByte(GT911_CONFIG_VERSION);
    if (writeThenRead(buffer, 2, buffer, GT911_REG_LENGTH - 2) == -1) {
        return false;
    }

    uint8_t check_sum = 0;
    for (int i = 0; i < (GT911_REG_LENGTH - 2 ); i++) {
        check_sum += buffer[i];
    }
    check_sum =  (~check_sum) + 1;
    log_d("reloadConfig check_sum : 0x%X\n", check_sum);
    writeGT911(GT911_CONFIG_CHKSUM, check_sum);
    writeGT911(GT911_CONFIG_FRESH, 0x01);
    return true;
}

void TouchDrvGT911::setMaxTouchPoint(uint8_t num)
{
    if (num < 1)num = 1;
    if (num > 5) num = 5;
    writeGT911(GT911_TOUCH_NUMBER, num);
    reloadConfig();
}

uint8_t TouchDrvGT911::readGT911(uint16_t cmd)
{
    uint8_t value = 0x00;
    uint8_t write_buffer[2] = {highByte(cmd), lowByte(cmd)};
    writeThenRead(write_buffer, arraySize(write_buffer),
                  &value, 1);
    return value;
}

int TouchDrvGT911::writeGT911(uint16_t cmd, uint8_t value)
{
    uint8_t write_buffer[3] = {highByte(cmd), lowByte(cmd), value};
    return writeBuff(write_buffer, arraySize(write_buffer));
}

void TouchDrvGT911::writeCommand(uint8_t command)
{
    // GT911_COMMAND 0x8040
    uint8_t write_buffer[3] = {0x80, 0x40, command};
    writeBuff(write_buffer, arraySize(write_buffer));
}

void TouchDrvGT911::clearBuffer()
{
    writeGT911(GT911_POINT_INFO, 0x00);
}

bool TouchDrvGT911::probeAddress()
{
    const uint8_t device_address[2]  = {GT911_SLAVE_ADDRESS_L, GT911_SLAVE_ADDRESS_H};
    for (size_t i = 0; i < arraySize(device_address); ++i) {
        setAddress(device_address[i]);
        for (int retry = 0; retry < 3; ++retry) {
            _chipID = getChipID();
            if (_chipID == GT911_DEV_ID) {
                log_i("Touch device address found is : 0x%X", device_address[i]);
                return true;
            }
        }
    }
    log_e("GT911 not found, touch device 7-bit address should be 0x5D or 0x14");
    return false;
}

bool TouchDrvGT911::initImpl(uint8_t)
{
    if (_addr == GT911_SLAVE_ADDRESS_H  && _pinsCfg.rstPin != -1 && _pinsCfg.irqPin != -1) {

        log_i("Try using 0x14 as the device address");

        hal->pinMode(_pinsCfg.rstPin, OUTPUT);
        hal->pinMode(_pinsCfg.irqPin, OUTPUT);

        hal->digitalWrite(_pinsCfg.rstPin, LOW);
        hal->digitalWrite(_pinsCfg.irqPin, HIGH);
        hal->delayMicroseconds(120);
        hal->digitalWrite(_pinsCfg.rstPin, HIGH);
        // The stable value after testing was 18ms.
        hal->delay(18);

        hal->pinMode(_pinsCfg.irqPin, INPUT);

    } else if (_addr == GT911_SLAVE_ADDRESS_L && _pinsCfg.rstPin != -1 && _pinsCfg.irqPin != -1) {

        log_i("Try using 0x5D as the device address");

        hal->pinMode(_pinsCfg.rstPin, OUTPUT);
        hal->pinMode(_pinsCfg.irqPin, OUTPUT);

        hal->digitalWrite(_pinsCfg.rstPin, LOW);
        hal->digitalWrite(_pinsCfg.irqPin, LOW);
        hal->delayMicroseconds(120);
        hal->digitalWrite(_pinsCfg.rstPin, HIGH);
        // The stable value after testing was 18ms.
        hal->delay(18);
        hal->pinMode(_pinsCfg.irqPin, INPUT);

    } else {
        if (!autoProbe()) {
            return false;
        }
    }

    // For variants where the GPIO is controlled by I2C, a hal->delay is required here
    hal->delay(20);


    /*
    * For the ESP32 platform, the default buffer is 128.
    * Need to re-apply for a larger buffer to fully read the configuration table.
    *
    * TODO: NEED FIX
    if (!this->reallocBuffer(GT911_REG_LENGTH + 2)) {
        log_e("realloc i2c buffer failed !");
        return false;
    }
     */

    _chipID = getChipID();

    if (_chipID != GT911_DEV_ID) {
        log_i("Not found device GT911,Try to found the GT911");
        if (!autoProbe()) {
            return false;
        }
    }

    uint8_t x_resolution[2] = {0}, y_resolution[2] = {0};
    for (int i = 0; i < 2; ++i) {
        x_resolution[i] = readGT911(GT911_X_RESOLUTION + i);
    }
    for (int i = 0; i < 2; ++i) {
        y_resolution[i] = readGT911(GT911_Y_RESOLUTION + i);
    }

    _touchConfig.resolutionX = x_resolution[0] | (x_resolution[1] << 8);
    _touchConfig.resolutionY = y_resolution[0] | (y_resolution[1] << 8);
    log_d("Model:GT911");
    log_d("RST Pin:%d", _pinsCfg.rstPin);
    log_d("IRQ Pin:%d", _pinsCfg.irqPin);
    log_i("Product id:%ld", _chipID);
    log_d("Firmware version: 0x%x", getFwVersion());
    log_d("Resolution : X = %d Y = %d", _touchConfig.resolutionX, _touchConfig.resolutionY);
    log_d("Vendor id:%d", getVendorID());
    log_d("Refresh Rate:%d ms", getRefreshRate());
    _maxTouchPoints = readGT911(GT911_TOUCH_NUMBER) & 0x0F;
    log_d("MaxTouchPoint:%d", _maxTouchPoints);

    // Get the default interrupt trigger mode of the current screen
    uint8_t irqMode = getInterruptMode();
    switch (irqMode) {
    case 0:     ///< RISING
        log_d("Interrupt Mode:  RISING");
        break;
    case 1:    ///< FALLING
        log_d("Interrupt Mode:  FALLING");
        break;
    case 2:   ///< LOW_LEVEL_QUERY
        log_d("Interrupt Mode:  LOW_LEVEL_QUERY");
        break;
    case 3:  ///< HIGH_LEVEL_QUERY
        log_d("Interrupt Mode:  HIGH_LEVEL_QUERY");
        break;
    default:
        log_e("Interrupt Mode:  UNKNOWN");
        break;
    }

    return true;
}

bool TouchDrvGT911::autoProbe()
{
    if (_pinsCfg.rstPin != -1) {
        hal->pinMode(_pinsCfg.rstPin, OUTPUT);
        hal->digitalWrite(_pinsCfg.rstPin, HIGH);
        hal->delay(10);
    }

    // Automatically determine the current device
    // address when using the reset pin without connection
    if (!probeAddress()) {
        return false;
    }

    // Reset Config
    reset();

    if (_pinsCfg.irqPin != -1) {
        hal->pinMode(_pinsCfg.irqPin, INPUT);
    }

    return true;
}

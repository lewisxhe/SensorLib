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

void TouchDrvGT911::reset()
{
    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(10);
    }
    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }
    /*
    * If you perform a software reset on a board without a reset pin connected,
    * subsequent interrupt settings or re-writing of configurations will be invalid.
    * For example, when debugging a LilyGo T-Deck, resetting the interrupt mode will
    * be invalid after a software reset.
    * */
    // comm->writeRegister(GT911_COMMAND, 0x02);
    // writeCommand(0x02);
}

void TouchDrvGT911::sleep()
{
    if (_irq != -1) {
        hal->pinMode(_irq, OUTPUT);
        hal->digitalWrite(_irq, LOW);
    }
    // comm->writeRegister(GT911_COMMAND, 0x05);
    writeCommand(0x05);

    /*
    * Depending on the chip and platform, setting it to input after removing sleep will affect power consumption.
    * The chip platform determines whether
    *
    * * */
    // if (_irq != -1) {
    //     hal->digitalWrite(_irq, INPUT);
    // }
}

void TouchDrvGT911::wakeup()
{
    if (_irq != -1) {
        hal->pinMode(_irq, OUTPUT);
        hal->digitalWrite(_irq, HIGH);
        hal->delay(8);
        hal->pinMode(_irq, INPUT);
    } else {
        reset();
    }
}

const TouchPoints &TouchDrvGT911::getTouchPoints()
{
    static TouchPoints points;
    uint8_t buffer[39];
    uint8_t numPoints = 0;

    // Clear cached touch points
    points.clear();

    numPoints = readGT911(GT911_POINT_INFO);
    // If there is a home button callback and the home button is pressed
    if (_HButtonCallback && (numPoints & 0x10)) {
        _HButtonCallback(_userData);
        return points;
    }

    // Clear the buffer
    clearBuffer();

    // Mask the number of points
    numPoints &= 0x0F;
    if (numPoints == 0 || numPoints > MAX_FINGER_NUM) {
        return points;
    }

    // GT911_POINT_1  0X814F
    addrToBeBuf(GT911_POINT_1, buffer);

    if (comm->writeThenRead(buffer, 2, buffer, 39) == 0) {
        for (int i = 0; i < numPoints; i++) {
            points.addPoint(buffer[1 + i * BYTES_PER_POINT] | (buffer[2 + i * BYTES_PER_POINT] << 8),   ///< Touch point X
                            buffer[3 + i * BYTES_PER_POINT] | (buffer[4 + i * BYTES_PER_POINT] << 8),   ///< Touch point Y
                            buffer[5 + i * BYTES_PER_POINT] | (buffer[6 + i * BYTES_PER_POINT] << 8),   ///< Touch point size fill pressure
                            buffer[i * BYTES_PER_POINT]);                                 ///< Touch point ID
        }

        // Swap XY or mirroring coordinates,if set
        updateXY(points);
    }

    return points;
}

bool TouchDrvGT911::isPressed()
{
    if (_irq != -1) {
        if (_irq_mode == FALLING) {
            return hal->digitalRead(_irq) == LOW;
        } else if (_irq_mode == RISING ) {
            return hal->digitalRead(_irq) == HIGH;
        } else if (_irq_mode == LOW_LEVEL_QUERY) {
            return hal->digitalRead(_irq) == LOW;
        }  else if (_irq_mode == HIGH_LEVEL_QUERY) {
            return hal->digitalRead(_irq) == HIGH;
        }
    }
    return getPoint();
}

const char *TouchDrvGT911::getModelName()
{
    return "GT911";
}

bool TouchDrvGT911::setInterruptMode(uint8_t mode)
{
    // GT911_MODULE_SWITCH_1 0x804D
    uint8_t val = readGT911(GT911_MODULE_SWITCH_1);
    val &= 0XFC;
    if (mode == FALLING) {
        val |= 0x01;
    } else if (mode == RISING ) {
        val |= 0x00;
    } else if (mode == LOW_LEVEL_QUERY ) {
        val |= 0x02;
    } else if (mode == HIGH_LEVEL_QUERY ) {
        val |= 0x03;
    }
    _irq_mode = mode;
    writeGT911(GT911_MODULE_SWITCH_1, val);
    return reloadConfig();
}

uint8_t TouchDrvGT911::getInterruptMode()
{
    uint8_t val = readGT911(GT911_MODULE_SWITCH_1);
    // return val & 0x03;
    val &= 0x03;
    if (val == 0x00) {
        _irq_mode = RISING;
    } else if (val == 0x01) {
        _irq_mode = FALLING;
    } else if (val == 0x02) {
        _irq_mode = LOW_LEVEL_QUERY;
    } else if (val == 0x03) {
        _irq_mode = HIGH_LEVEL_QUERY;
    }
    return val;
}

uint8_t TouchDrvGT911::getPoint()
{
    uint8_t numPoints = readGT911(GT911_POINT_INFO);
    clearBuffer();
    return (numPoints & 0x0F);
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
    if ((rate_ms - 5) < 5) {
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

#if 0   //TODO:
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
    int err =  comm->writeRegister(GT911_CONFIG_VERSION, (uint8_t *)config_buffer, buffer_size);


#if 0
    while (digitalRead(_irq)) {
        log_i("Wait irq.."); hal->delay(500);
    }
    int err =   comm->writeBuffer((uint8_t *)config_buffer, buffer_size);
#endif
    return err == 0;
    return false;
}
#endif

uint8_t *TouchDrvGT911::loadConfig(size_t *output_size, bool print_out)
{
    *output_size = 0;
    uint8_t   *buffer = (uint8_t * )malloc(GT911_REG_LENGTH * sizeof(uint8_t));
    if (!buffer)return NULL;
    addrToBeBuf(GT911_CONFIG_VERSION, buffer);
    if (comm->writeThenRead(buffer, 2, buffer, GT911_REG_LENGTH) == -1) {
        free(buffer);
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
    uint8_t buffer[GT911_REG_LENGTH] = {highByte(GT911_CONFIG_VERSION), lowByte(GT911_CONFIG_VERSION)};
    if (comm->writeThenRead(buffer, 2, buffer, GT911_REG_LENGTH - 2) == -1) {
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

void TouchDrvGT911::setConfigData(uint8_t *data, uint16_t length)
{
    _config = data;
    _config_size = length;
}

uint8_t TouchDrvGT911::readGT911(uint16_t cmd)
{
    uint8_t value = 0x00;
    uint8_t write_buffer[2] = {highByte(cmd), lowByte(cmd)};
    comm->writeThenRead(write_buffer, arraySize(write_buffer),
                        &value, 1);
    return value;
}

int TouchDrvGT911::writeGT911(uint16_t cmd, uint8_t value)
{
    uint8_t write_buffer[3] = {highByte(cmd), lowByte(cmd), value};
    return comm->writeBuffer(write_buffer, arraySize(write_buffer));
}

void TouchDrvGT911::writeCommand(uint8_t command)
{
    // GT911_COMMAND 0x8040
    uint8_t write_buffer[3] = {0x80, 0x40, command};
    comm->writeBuffer(write_buffer, arraySize(write_buffer));
}

void TouchDrvGT911::clearBuffer()
{
    writeGT911(GT911_POINT_INFO, 0x00);
}

bool TouchDrvGT911::probeAddress()
{
    const uint8_t device_address[2]  = {GT911_SLAVE_ADDRESS_L, GT911_SLAVE_ADDRESS_H};
    for (size_t i = 0; i < arraySize(device_address); ++i) {
        I2CParam params(I2CParam::I2C_SET_ADDR, device_address[i]);
        comm->setParams(params);
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

bool TouchDrvGT911::initImpl(uint8_t addr)
{
    int16_t x = 0, y = 0;

    if (addr == GT911_SLAVE_ADDRESS_H  && _rst != -1 && _irq != -1) {

        log_i("Try using 0x14 as the device address");

        hal->pinMode(_rst, OUTPUT);
        hal->pinMode(_irq, OUTPUT);

        hal->digitalWrite(_rst, LOW);
        hal->digitalWrite(_irq, HIGH);
        hal->delayMicroseconds(120);
        hal->digitalWrite(_rst, HIGH);

#if   defined(ARDUINO)
        // In the Arduino ESP32 platform, the test delay is 8ms and the GT911
        // can be accessed correctly. If the time is too long, it will not be accessible.
        hal->delay(8);
#elif defined(ESP_PLATFORM)
        // For the variant of GPIO extended RST,
        // communication and delay are carried out simultaneously, and 18 ms is measured in T-RGB esp-idf new api
        hal->delay(18);
#endif

        hal->pinMode(_irq, INPUT);

    } else if (addr == GT911_SLAVE_ADDRESS_L && _rst != -1 && _irq != -1) {

        log_i("Try using 0x5D as the device address");

        hal->pinMode(_rst, OUTPUT);
        hal->pinMode(_irq, OUTPUT);

        hal->digitalWrite(_rst, LOW);
        hal->digitalWrite(_irq, LOW);
        hal->delayMicroseconds(120);
        hal->digitalWrite(_rst, HIGH);
#if   defined(ARDUINO)
        // In the Arduino ESP32 platform, the test hal->delay is 8ms and the GT911
        // can be accessed correctly. If the time is too long, it will not be accessible.
        hal->delay(8);
#elif defined(ESP_PLATFORM)
        // For the variant of GPIO extended RST,
        // communication and hal->delay are carried out simultaneously, and 18 ms is measured in T-RGB esp-idf new api
        hal->delay(18);
#endif
        hal->pinMode(_irq, INPUT);

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


#if 0
    /*If the configuration is not written, the touch screen may be damaged. */
    if (_config && _config_size != 0) {

        log_d("Current version char :%x", getConfigVersion());
        hal->delay(100);
        writeConfig(_config, _config_size);
        if (_irq != -1) {
            hal->pinMode(_irq, INPUT);
        }
        log_d("WriteConfig version char :%x", getConfigVersion());
        // hal->delay(1000);
        // size_t output_size;
        // loadConfig(&output_size, true);
        // log_d("loadConfig version char :%x", version_char);
    }
#endif

    uint8_t x_resolution[2] = {0}, y_resolution[2] = {0};
    for (int i = 0; i < 2; ++i) {
        x_resolution[i] = readGT911(GT911_X_RESOLUTION + i);
    }
    for (int i = 0; i < 2; ++i) {
        y_resolution[i] = readGT911(GT911_Y_RESOLUTION + i);
    }

    _resX = x_resolution[0] | (x_resolution[1] << 8);
    _resY = y_resolution[0] | (y_resolution[1] << 8);
    log_d("Model:CST3530");
    log_d("RST Pin:%d", _rst);
    log_d("IRQ Pin:%d", _irq);
    log_i("Product id:%ld", _chipID);
    log_d("Firmware version: 0x%x", getFwVersion());
    log_d("Resolution : X = %d Y = %d", _resX, _resY);
    log_d("Vendor id:%d", getVendorID());
    log_d("Refresh Rate:%d ms", getRefreshRate());
    _maxTouchPoints = readGT911(GT911_TOUCH_NUMBER) & 0x0F;
    log_d("MaxTouchPoint:%d", _maxTouchPoints);


    // Get the default interrupt trigger mode of the current screen
    getInterruptMode();

    if ( _irq_mode == RISING) {
        log_d("Interrupt Mode:  RISING");
    } else if (_irq_mode == FALLING) {
        log_d("Interrupt Mode:  FALLING");
    } else if (_irq_mode == LOW_LEVEL_QUERY) {
        log_d("Interrupt Mode:  LOW_LEVEL_QUERY");
    } else if (_irq_mode == HIGH_LEVEL_QUERY) {
        log_d("Interrupt Mode:  HIGH_LEVEL_QUERY");
    } else {
        log_e("UNKNOWN");
    }

    if (x == -1 || y == -1) {
        log_e("The screen configuration is lost, please update the configuration file again !");
        return false;
    }


    return true;
}

bool TouchDrvGT911::autoProbe()
{
    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(10);
    }

    // Automatically determine the current device
    // address when using the reset pin without connection
    if (!probeAddress()) {
        return false;
    }

    // Reset Config
    reset();

    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }

    return true;
}

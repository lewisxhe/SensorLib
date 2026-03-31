/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      TouchDrvCST92xx.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-07
 */
#include "TouchDrvCST92xx.h"

TouchDrvCST92xx::TouchDrvCST92xx(): _slave_addr(-1)
{
}

const TouchPoints &TouchDrvCST92xx::getTouchPoints()
{
    uint8_t numPoints = 0;
    uint8_t buffer[MAX_FINGER_NUM * 5 + 5] = {0};
    uint8_t write_buffer[4] = {0};

    // Clear cached touch points
    _touchPoints.clear();

    // Write read command
    write_buffer[0] = highByte(REG_READ);
    write_buffer[1] = lowByte(REG_READ);
    addrToBeBuf(REG_READ, buffer);

    if (writeThenRead(buffer, 2, buffer, sizeof(buffer)) == 0) {

        // Write read ACK
        write_buffer[0] = highByte(REG_READ);
        write_buffer[1] = lowByte(REG_READ);
        write_buffer[2] = CST92XX_ACK;

        if (writeBuff(write_buffer, 3) != 0) {
            return _touchPoints;
        }

        if (buffer[0] == CST92XX_ACK || buffer[6] != CST92XX_ACK) {
            return _touchPoints;
        }

        // Check cover screen gesture
        if (buffer[4] & 0xF0) {
            if ((buffer[4] >> 7) == 0x01) {
                if (_HButtonCallback) {
                    _HButtonCallback(_userData);
                }
                // Cover screen gesture detected, return zero points
                return _touchPoints;
            }
        }

        numPoints = (buffer[5] & 0x7F); // Get number of touch points
        if (numPoints > MAX_FINGER_NUM || numPoints == 0) {
            return _touchPoints;
        }

        for (uint8_t i = 0; i < numPoints; ++i) {
            uint8_t *pdat = buffer + (i * 5) + (i == 0 ? 0 : 2);
            const uint8_t id = (pdat[0] >> 4);
            const uint8_t event = (pdat[0] & 0x0F);
            const uint16_t x = ((pdat[1] << 4) | (pdat[3] >> 4));
            const uint16_t y = ((pdat[2] << 4) | (pdat[3] & 0x0F));
            if (event == 0x06 && id < MAX_FINGER_NUM) {
                _touchPoints.addPoint(x, y, 0, id, event);
            }
        }

        if (_touchPoints.getPoint(0).event == 0x00) {
            _touchPoints.clear();
            return _touchPoints;
        }
        // Swap XY or mirroring coordinates,if set
        updateXY(_touchPoints);
    }

    return _touchPoints;
}

const char *TouchDrvCST92xx::getModelName()
{
    switch (_chipID) {
    case CST9220_CHIP_ID:
        return "CST9220";
    case CST9217_CHIP_ID:
        return "CST9217";
    default:
        break;
    }
    return "UNKNOWN";
}

void TouchDrvCST92xx::sleep()
{

    uint8_t buffer[2] = {0};
    // Enter command mode
    setMode(MODE_DEBUG_INFO);
    //Send sleep command
    addrToBeBuf(REG_SLEEP_MODE, buffer);
    writeBuff(buffer, 2);
#ifdef ARDUINO_ARCH_ESP32
    if (_pinsCfg.irqPin != -1) {
        hal->pinMode(_pinsCfg.irqPin, OPEN_DRAIN);
    }
    if (_pinsCfg.rstPin != -1) {
        hal->pinMode(_pinsCfg.rstPin, OPEN_DRAIN);
    }
#endif
}

void TouchDrvCST92xx::setCoverScreenCallback(HomeButtonCallback cb, void *user_data)
{
    _HButtonCallback = cb;
    _userData = user_data;
}

/**
 * @note   Only when the device address is equal to 0X5A can it be accessed. If the device address is not equal to 0X5A, it can only be accessed after reset.
 */
uint32_t TouchDrvCST92xx::readWordFromMem(uint8_t type, uint16_t mem_addr)
{
    int res = 0;
    uint8_t write_buffer[4] = {0};
    uint8_t read_buffer[4] = {0};

    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        setAddress(CST92XX_BOOT_ADDRESS);
    }


    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x10;
    write_buffer[2] = type;

    res =  writeBuff(write_buffer, 3);
    if (res != 0) {
        log_e("Write 0A010 failed");
        goto ERROR;
    }

    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x0C;
    write_buffer[2] = mem_addr;
    write_buffer[3] = mem_addr >> 8;

    res = writeBuff(write_buffer, 4);
    if (res != 0) {
        log_e("Write 0A00C failed");
        goto ERROR;
    }

    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x04;
    write_buffer[2] = 0xE4;

    res = writeBuff(write_buffer, 3);
    if (res != 0) {
        log_e("Write 0A004E4 failed");
        goto ERROR;
    }

    for (uint8_t t = 0;; t++) {
        if (t >= 100) {
            goto ERROR;
        }
        write_buffer[0] = 0xA0;
        write_buffer[1] = 0x04;
        res = writeThenRead(write_buffer, 2, read_buffer, 1);
        if (res != 0) {
            log_e("Write 0A004 failed");
            goto ERROR;
        }

        if (read_buffer[0] == 0x00) {
            break;
        }
    }
    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x18;
    res = writeThenRead(write_buffer, 2, read_buffer, 4);
    if (res != 0) {
        goto ERROR;
    }

    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        setAddress(_slave_addr);
    }

    return ((uint32_t)(read_buffer[0])) |
           (((uint32_t)(read_buffer[1])) << 8) |
           (((uint32_t)(read_buffer[2])) << 16) |
           (((uint32_t)(read_buffer[3])) << 24);
ERROR:
    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        setAddress(_slave_addr);
    }
    return 0;
}

uint32_t TouchDrvCST92xx::getChipType()
{
    return chipType;
}

/**
 * @note   Only when the device address is equal to 0X5A can it be accessed. If the device address is not equal to 0X5A, it can only be accessed after reset.
 */
bool TouchDrvCST92xx::enterBootloader(void)
{
    int16_t res = 0;
    uint8_t check_cnt = 0;
    uint8_t write_buffer[4] = {0};
    uint8_t read_buffer[4] = {0};

    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        setAddress(CST92XX_BOOT_ADDRESS);
    }

    for (uint8_t i = 10;; i += 2) {

        if (i > 20) {
            log_e("Enter boot:try timeout");
            goto ERROR;
        }

        reset();

        hal->delay(i);

        for (check_cnt = 0; check_cnt < 5; check_cnt++) {
            write_buffer[0] = 0xA0;
            write_buffer[1] = 0x01;
            write_buffer[2] = 0xAA;
            res = writeBuff(write_buffer, 3);
            if (res != 0) {
                hal->delay(2);
                continue;
            }
            hal->delay(2);
            write_buffer[0] = 0xA0;
            write_buffer[1] = 0x02;
            res = writeThenRead(write_buffer, 2, read_buffer, 2);
            if (res != 0) {
                hal->delay(2);
                continue;
            }
            if ((read_buffer[0] == 0x55) && (read_buffer[1] == 0xB0)) {
                break;
            }
        }
        if ((read_buffer[0] == 0x55) && (read_buffer[1] == 0xB0)) {
            break;
        }
    }

    write_buffer[0] = 0xA0;
    write_buffer[1] = 0x01;
    write_buffer[2] = 0x00;
    res = writeBuff(write_buffer, 3);
    if (res != 0) {
        log_e("Enter boot exit error");
        goto ERROR;
    }
    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        setAddress(_slave_addr);
    }
    log_d("Enter boot mode success!");
    return true;

ERROR:
    if (_slave_addr != -1 && _slave_addr != CST92XX_BOOT_ADDRESS) {
        setAddress(_slave_addr);
    }
    return false;

}

bool TouchDrvCST92xx::getAttribute()
{
    reset();

    // Wait exit boot mode
    hal->delay(30);

    uint8_t buffer[8];
    // Enter Command mode
    writeReg(0xD1, 0x01);
    hal->delay(10);
    uint8_t write_buffer[2] = {0xD1, 0xFC};
    writeThenRead(write_buffer, 2, buffer, 4);
    uint32_t checkcode = 0;
    checkcode = buffer[3];
    checkcode <<= 8;
    checkcode |= buffer[2];
    checkcode <<= 8;
    checkcode |= buffer[1];
    checkcode <<= 8;
    checkcode |= buffer[0];

    log_i("Chip checkcode:0x%lx.", checkcode);

    write_buffer[0] = 0xD1;
    write_buffer[1] = 0xF8;
    writeThenRead(write_buffer, 2, buffer, 4);
    _touchConfig.resolutionX = ( buffer[1] << 8) | buffer[0];
    _touchConfig.resolutionY = ( buffer[3] << 8) | buffer[2];
    log_i("Chip resolution X:%u Y:%u", _touchConfig.resolutionX, _touchConfig.resolutionY);

    write_buffer[0] = 0xD2;
    write_buffer[1] = 0x04;
    writeThenRead(write_buffer, 2, buffer, 4);
    chipType = buffer[3];
    chipType <<= 8;
    chipType |= buffer[2];


    uint32_t ProjectID = buffer[1];
    ProjectID <<= 8;
    ProjectID |= buffer[0];
    log_i("Chip type :0x%x, ProjectID:0X%lx",
          chipType, ProjectID);

    write_buffer[0] = 0xD2;
    write_buffer[1] = 0x08;
    writeThenRead(write_buffer, 2, buffer, 8);

    uint32_t fwVersion = buffer[3];
    fwVersion <<= 8;
    fwVersion |= buffer[2];
    fwVersion <<= 8;
    fwVersion |= buffer[1];
    fwVersion <<= 8;
    fwVersion |= buffer[0];

    uint32_t checksum = buffer[7];
    checksum <<= 8;
    checksum |= buffer[6];
    checksum <<= 8;
    checksum |= buffer[5];
    checksum <<= 8;
    checksum |= buffer[4];

    log_i("Chip ic version:0x%lx, checksum:0x%lx",
          fwVersion, checksum);

    if (fwVersion == 0xA5A5A5A5) {
        log_e("Chip ic don't have firmware.");
        return false;
    }
    if ((checkcode & 0xffff0000) != 0xCACA0000) {
        log_e("Firmware info read error.");
        return false;
    }

    if ((chipType != CST9220_CHIP_ID) && (chipType != CST9217_CHIP_ID)) {
        log_e("Chip type error 0x%x", chipType);
        return false;
    }

    return true;
}

bool TouchDrvCST92xx::setMode(uint8_t mode)
{
    uint8_t read_buffer[4] = {0};
    uint8_t write_buffer[4] = {0};
    uint8_t i = 0;
    int16_t res = -1;
    uint8_t mode_cmd = 0;

    for (i = 0; i < 3; i++) {
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x1E;
        res = writeBuff(write_buffer, 2);
        if (res != 0) {
            hal->delay(200);
            continue;
        }
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x1E;
        res = writeBuff(write_buffer, 2);
        if (res != 0) {
            hal->delay(200);
            continue;
        }
        write_buffer[0] = 0x00;
        write_buffer[1] = 0x02;
        res = writeThenRead(write_buffer, 2, read_buffer, 4);
        if (res != 0) {
            hal->delay(200);
            continue;
        }
        if (read_buffer[1] == 0x1E)
            break;
    }

    switch (mode) {
    case MODE_NORMAL: {
        log_d("set_work_mode: ENUM_MODE_NORMAL");
        write_buffer[0] = highByte(REG_NORMAL_MODE);
        write_buffer[1] = lowByte(REG_NORMAL_MODE);
        break;
    }
    case MODE_DEBUG_DIFF: {
        log_d("set_work_mode: ENUM_MODE_DEBUG_DIFF");
        write_buffer[0] = highByte(REG_DIFF_MODE);
        write_buffer[1] = lowByte(REG_DIFF_MODE);
        break;
    }
    case MODE_DEBUG_RAWDATA: {
        log_d("set_work_mode: ENUM_MODE_DEBUG_RAWDATA");
        write_buffer[0] = highByte(REG_RAW_MODE);
        write_buffer[1] = lowByte(REG_RAW_MODE);
        break;
    }
    case MODE_DEBUG_INFO: {
        log_d("set_work_mode: ENUM_MODE_DEBUG_INFO");
        write_buffer[0] = highByte(REG_DEBUG_MODE);
        write_buffer[1] = lowByte(REG_DEBUG_MODE);
        break;
    }
    case MODE_FACTORY: {
        log_d("set_work_mode: ENUM_MODE_FACTORY");
        for (i = 0; i < 10; i++) {
            write_buffer[0] = highByte(REG_FACTORY_MODE);
            write_buffer[1] = lowByte(REG_FACTORY_MODE);
            res = writeBuff(write_buffer, 2);
            if (res != 0) {
                hal->delay(1);
                continue;
            }
            hal->delay(10);
            write_buffer[0] = 0x00;
            write_buffer[1] = 0x09;
            res = writeThenRead(write_buffer, 2, read_buffer, 1);
            if (res != 0) {
                hal->delay(1);
                continue;
            }
            if (read_buffer[0] == 0x14)
                break;
        }
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x19;
        res = writeBuff(write_buffer, 2);
        if (res != 0) {
            log_e("set_work_mode 0xD119 error");
            return false;
        }
        break;
    }
    case MODE_FACTORY_LOW_DRV: {
        log_d("set_work_mode: ENUM_MODE_FACTORY_LOWDRV");
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x11;
        break;
    }
    case MODE_FACTORY_HIGH_DRV: {
        log_d("set_work_mode: ENUM_MODE_FACTORY_HIGHDRV");
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x10;
        break;
    }
    case MODE_FACTORY_SHORT: {
        log_d("set_work_mode: ENUM_MODE_FACTORY_SHORT");
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x12;
        break;
    }
    case 0XFE: {
        log_d("set_work_mode: 0xFE");
        write_buffer[0] = 0xD1;
        write_buffer[1] = 0x1F;
        break;
    }
    default: {
        log_d("set_work_mode: NA return");
        return 0;
    }
    }
    mode_cmd = write_buffer[1];
    res = writeBuff(write_buffer, 2);
    if (res != 0) {
        log_e("set_work_mode 0x%x  0x%x error", write_buffer[0], write_buffer[1]);
        return false;
    }
    write_buffer[0] = 0x00;
    write_buffer[1] = 0x02;
    res = writeThenRead(write_buffer, 2, read_buffer, 2);
    if (res != 0) {
        log_e("set_work_mode read 0x0002 failed : 0x%X 0x%X", read_buffer[0], read_buffer[1]);
    }
    if (mode_cmd != read_buffer[1]) {
        log_e("set work mode read 0x0002=0x%x failed", read_buffer[1]);
        return false;
    }
    hal->delay(10);
    return true;
}

bool TouchDrvCST92xx::initImpl(uint8_t)
{
    if (!getAttribute()) {
        return false;
    }

    _chipID = chipType;

    log_d("Touch type:%s", getModelName());

    _maxTouchPoints = MAX_FINGER_NUM;

    return true;
}

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
 * @file      TouchDrvGT9895.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-21
 *
 */
#include "TouchDrvGT9895.hpp"

void TouchDrvGT9895::sleep()
{
    if (_pinsCfg.irqPin != -1) {
        hal->pinMode(_pinsCfg.irqPin, OUTPUT);
        hal->digitalWrite(_pinsCfg.irqPin, LOW);
    }

    uint8_t sleep_cmd[10] = {0};
    addrToBeBuf(REG_CMD, sleep_cmd);
    sleep_cmd[4] = 0x00;
    sleep_cmd[5] = 0x00;
    sleep_cmd[6] = 0x04;
    sleep_cmd[7] = 0x84;
    sleep_cmd[8] = 0x88;
    sleep_cmd[9] = 0x00;
    writeBuff(sleep_cmd, sizeof(sleep_cmd));
}

void TouchDrvGT9895::wakeup()
{
    if (_pinsCfg.irqPin != -1) {
        hal->pinMode(_pinsCfg.irqPin, OUTPUT);
        hal->digitalWrite(_pinsCfg.irqPin, HIGH);
        hal->delay(8);
    }
    reset();
}

const TouchPoints &TouchDrvGT9895::getTouchPoints()
{
    uint8_t type = 0;
    uint8_t write_buffer[4];
    uint8_t buffer[IRQ_EVENT_HEAD_LEN + MAX_FINGER_NUM * BYTES_PER_POINT + 2] = {0};

    _touchPoints.clear();

    // Read the event header (8 bytes) to get the number of fingers.
    addrToBeBuf(REG_POINT, write_buffer);
    if (writeThenRead(write_buffer, 4, buffer, IRQ_EVENT_HEAD_LEN) != 0) {
        return _touchPoints;
    }
    // Check if touch event is valid
    if (buffer[0] == 0x00) {
        return _touchPoints;
    }
    // Check if the number of touch points is valid
    uint8_t numPoints = buffer[2] & 0x0F;
    if (numPoints == 0 || numPoints > MAX_FINGER_NUM) {
        clearStatus();
        return _touchPoints;
    }
    // Read the remaining data from the offset address.
    uint16_t remaining_len = numPoints * BYTES_PER_POINT + COORDS_DATA_CHECKSUM_SIZE;
    addrToBeBuf(REG_POINT + IRQ_EVENT_HEAD_LEN, write_buffer);
    if (writeThenRead(write_buffer, 4, &buffer[IRQ_EVENT_HEAD_LEN], remaining_len) != 0) {
        return _touchPoints;
    }

    if (calculateChecksum(buffer, IRQ_EVENT_HEAD_LEN, CHECKSUM_MODE_U8_LE)) {
        // If touch data checksum fails
        clearStatus();
        return _touchPoints;
    }

    if (buffer[0] & 0x80) {
        type = buffer[IRQ_EVENT_HEAD_LEN] & 0x0F;
        uint16_t checksum_size = (type == POINT_TYPE_STYLUS || type == POINT_TYPE_STYLUS_HOVER) ?
                                 (BYTES_PER_POINT * 2 + 2) : (numPoints * BYTES_PER_POINT + 2);

        if (calculateChecksum(&buffer[IRQ_EVENT_HEAD_LEN], checksum_size, CHECKSUM_MODE_U8_LE)) {
            // If touch data checksum fails
            clearStatus();
            return _touchPoints;
        }
        // Add touch points
        for (int i = 0; i < numPoints; i++) {
            uint8_t offset = IRQ_EVENT_HEAD_LEN + i * BYTES_PER_POINT;
            uint16_t x = buffer[offset + 2] | (buffer[offset + 3] << 8);
            uint16_t y = buffer[offset + 4] | (buffer[offset + 5] << 8);
            uint16_t w = buffer[offset + 6] | (buffer[offset + 7] << 8);
            uint8_t id = (buffer[offset] >> 4) & 0x0F;
            _touchPoints.addPoint(x, y, w, id);
        }
        // Swap XY or mirroring coordinates,if set
        updateXY(_touchPoints);
    }

    clearStatus();
    return _touchPoints;
}

const char *TouchDrvGT9895::getModelName()
{
    return "GT9895";
}

int TouchDrvGT9895::calculateChecksum(const uint8_t *data, int size, CheckSumMode mode)
{
    uint32_t cal_checksum = 0;
    uint32_t r_checksum = 0;
    if (mode == CHECKSUM_MODE_U8_LE) {
        if (size < 2)
            return 1;
        for (int i = 0; i < size - 2; i++)
            cal_checksum += data[i];
        r_checksum = data[size - 2] + (data[size - 1] << 8);
        return (cal_checksum & 0xFFFF) == r_checksum ? 0 : 1;
    }

    if (size < 4)
        return 1;
    for (int i = 0; i < size - 4; i += 2)
        cal_checksum += data[i] + (data[i + 1] << 8);
    r_checksum = data[size - 4] + (data[size - 3] << 8) +
                 (data[size - 2] << 16) + (data[size - 1] << 24);
    return cal_checksum == r_checksum ? 0 : 1;
}

uint32_t TouchDrvGT9895::getChipPID()
{
    struct ChipInfo {
        uint8_t rom_pid[6];
        uint8_t rom_vid[3];
        uint8_t rom_vid_reserved;
        uint8_t patch_pid[8];
        uint8_t patch_vid[4];
        uint8_t patch_vid_reserved;
        uint8_t sensor_id;
        uint8_t reserved[2];
        uint16_t checksum;
    } info;

    int ret = 0;
    uint8_t temp_pid[8] = {0};
    uint8_t buffer[28] = {0};
    addrToBeBuf(REG_FW_VERSION, buffer);
    for (int i = 0; i < 2; i++) {
        if (writeThenRead(buffer, 4, buffer, sizeof(buffer)) == -1) {
            log_e("Failed to read firmware version");
            ret = -1;
            hal->delay(5);
            continue;
        }
        if (!calculateChecksum(buffer, sizeof(buffer), CHECKSUM_MODE_U8_LE)) {
            ret = 0;
            break;
        }
        log_e("Invalid fw version: checksum error!");
        log_e("Firmware version:%*ph", (int)sizeof(buffer), buffer);
        ret = -1;
        hal->delay(15);
    }
    if (ret == -1) {
        log_e("Failed get valid firmware version");
        return 0;
    }
    memcpy(&info, buffer, sizeof(info));
    memcpy(temp_pid, info.rom_pid, sizeof(info.rom_pid));
    log_d("ROM_PID:%s", (const char *)temp_pid);
    log_d("ROM_VID:%*p", (int)sizeof(info.rom_vid), info.rom_vid);
    log_d("PID:%s", (const char *)info.patch_pid);
    log_d("VID:%*p", (int)sizeof(info.patch_vid), info.patch_vid);
    log_d("Sensor ID:%d", info.sensor_id);
    if (info.patch_pid[0] == '9' && info.patch_pid[1] == '8'
            && info.patch_pid[2] == '9' && info.patch_pid[3] == '5') {
        return CHIP_PID;
    }
    return 0;
}

void TouchDrvGT9895::clearStatus()
{
    uint8_t buffer[5] =  { 0x00, 0x01, 0x03, 0x08, 0x00};
    writeBuff(buffer, 5);
}

bool TouchDrvGT9895::initImpl(uint8_t)
{
    if (getChipPID() != 0x9895) {
        return false;
    }
    _maxTouchPoints = MAX_FINGER_NUM;
    _chipID = CHIP_PID;
    return true;
}

/*
[   572][D][TouchDrvGT9895.cpp:236] getChipPID(): ROM_PID:BERLIN
[   578][D][TouchDrvGT9895.cpp:237] getChipPID(): ROM_VID:0x4ff3d66a
[   584][D][TouchDrvGT9895.cpp:238] getChipPID(): PID:9895
[   589][D][TouchDrvGT9895.cpp:239] getChipPID(): VID:0x4ff3d676
[   595][D][TouchDrvGT9895.cpp:240] getChipPID(): Sensor ID:255
*/

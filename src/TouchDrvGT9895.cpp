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

void TouchDrvGT9895::reset()
{
    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(10);
        hal->digitalWrite(_rst, LOW);
        hal->delay(30);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(100);
    }
    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }
}

void TouchDrvGT9895::sleep()
{
    if (_irq != -1) {
        hal->pinMode(_irq, OUTPUT);
        hal->digitalWrite(_irq, LOW);
    }

    uint8_t sleep_cmd[10] = {0};
    addrToBeBuf(REG_CMD, sleep_cmd);
    sleep_cmd[4] = 0x00;
    sleep_cmd[5] = 0x00;
    sleep_cmd[6] = 0x04;
    sleep_cmd[7] = 0x84;
    sleep_cmd[8] = 0x88;
    sleep_cmd[9] = 0x00;
    comm->writeBuffer(sleep_cmd, sizeof(sleep_cmd));
}

void TouchDrvGT9895::wakeup()
{
    if (_irq != -1) {
        hal->pinMode(_irq, OUTPUT);
        hal->digitalWrite(_irq, HIGH);
        hal->delay(8);
    }
    reset();
}

const TouchPoints &TouchDrvGT9895::getTouchPoints()
{
    static TouchPoints points;
    uint8_t type = 0;
    uint16_t length = IRQ_EVENT_HEAD_LEN + BYTES_PER_POINT * 2 + COORDS_DATA_CHECKSUM_SIZE;
    uint8_t write_buffer[4];
    uint8_t buffer[IRQ_EVENT_HEAD_LEN + BYTES_PER_POINT * MAX_FINGER_NUM + 2] = {0};

    // Clear cached touch points
    points.clear();

    addrToBeBuf(REG_POINT, write_buffer);

    if (comm->writeThenRead(write_buffer, 4, buffer, length) == 0) {

        // Check if touch event is valid
        if (buffer[0] == 0x00) {
            return points;
        }

        if (checksum(buffer, IRQ_EVENT_HEAD_LEN, CHECKSUM_MODE_U8_LE)) {
            // If checksum fails, clear points and return
            return points;
        }

        // Check if touch event is valid
        if (buffer[0] & 0x80) {

            uint8_t numPoints = buffer[2] & 0x0F;

            // Check if the number of touch points is valid
            if (numPoints > MAX_FINGER_NUM || numPoints == 0) {
                clearStatus();
                return points;
            }

            // Read additional touch points if necessary
            if (numPoints > 2) {
                addrToBeBuf(REG_POINT + length, write_buffer);
                if (comm->writeThenRead(write_buffer, 4, &buffer[length], (numPoints - 2) * BYTES_PER_POINT) == -1) {
                    log_e("Failed to get additional touch data");
                    clearStatus();
                    return points;
                }
            }

            // Check point type
            type = buffer[IRQ_EVENT_HEAD_LEN] & 0x0F;
            if (type == POINT_TYPE_STYLUS || type == POINT_TYPE_STYLUS_HOVER) {
                if (checksum(&buffer[IRQ_EVENT_HEAD_LEN], BYTES_PER_POINT * 2 + 2, CHECKSUM_MODE_U8_LE)) {
                    // If stylus touch data checksum fails
                    clearStatus();
                    return points;
                }
            } else {
                if (checksum(&buffer[IRQ_EVENT_HEAD_LEN], numPoints * BYTES_PER_POINT + 2, CHECKSUM_MODE_U8_LE)) {
                    // If touch data checksum fails
                    clearStatus();
                    return points;
                }
            }

            // Add touch points
            for (int i = 0; i < numPoints; i++) {
                int base = IRQ_EVENT_HEAD_LEN + i * BYTES_PER_POINT;
                uint16_t x = buffer[base + 2] | (buffer[base + 3] << 8);
                uint16_t y = buffer[base + 4] | (buffer[base + 5] << 8);
                uint16_t w = buffer[base + 6] | (buffer[base + 7] << 8);
                uint8_t id = (buffer[base] >> 4) & 0x0F;
                points.addPoint(x, y, w, id);
            }

            // Swap XY or mirroring coordinates,if set
            updateXY(points);

        }
        // Clear touch points if no valid touch event
        clearStatus();
    }

    return points;
}

bool TouchDrvGT9895::isPressed()
{
    if (_irq != -1) {
        return hal->digitalRead(_irq) == LOW;
    } else {
        return getTouchPoints().hasPoints();
    }
    return false;
}

const char *TouchDrvGT9895::getModelName()
{
    return "GT9895";
}

int TouchDrvGT9895::checksum(const uint8_t *data, int size, CheckSumMode mode)
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
        if (comm->writeThenRead(buffer, 4, buffer, sizeof(buffer)) == -1) {
            log_e("Failed to read firmware version");
            ret = -1;
            hal->delay(5);
            continue;
        }
        if (!checksum(buffer, sizeof(buffer), CHECKSUM_MODE_U8_LE)) {
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
    comm->writeBuffer(buffer, 5);
}

bool TouchDrvGT9895::initImpl(uint8_t addr)
{
    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }

    reset();

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

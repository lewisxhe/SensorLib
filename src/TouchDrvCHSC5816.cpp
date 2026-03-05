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
 * @file      TouchDrvCHSC5816.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-05
 *
 */
#include "TouchDrvCHSC5816.hpp"

void TouchDrvCHSC5816::reset()
{
    if (_rst != -1) {
        hal->digitalWrite(_rst, LOW);
        hal->delay(3);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(5);
    }
}

void TouchDrvCHSC5816::sleep()
{
    uint8_t CHSC5816_REG_SLEEP[] = {
        0x20, 0x00, 0x00, 0x00, // CHSC5816_REG_CMD_BUFF
        0xF8, 0x16, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xE9
    };
    comm->writeBuffer(CHSC5816_REG_SLEEP, arraySize(CHSC5816_REG_SLEEP));
}

void TouchDrvCHSC5816::wakeup()
{
    reset();
}

const TouchPoints &TouchDrvCHSC5816::getTouchPoints()
{
    static TouchPoints points;
    uint8_t buffer[8] = {0};

    points.clear(); // Clear cached touch points
    addrToBeBuf(REG_POINT, buffer);

    if (comm->writeThenRead(buffer, 4, buffer, 8) == 0) {

        // Get number of touch points
        uint8_t numPoints = buffer[1];

        if (numPoints > MAX_FINGER_NUM || numPoints == 0) {
            return points;
        }

        points.addPoint(buffer[2] | ((buffer[5] & 0x0F) << 8),
                        buffer[3] | (((buffer[5] >> 4) & 0x0F) << 8),
                        buffer[4],
                        buffer[6] & 0x0F);

        // Swap XY or mirroring coordinates,if set
        updateXY(points);
    }
    return points;
}

bool TouchDrvCHSC5816::isPressed()
{
    if (_irq != -1) {
        return hal->digitalRead(_irq) == LOW;
    }
    return getTouchPoints().hasPoints();
}

const char *TouchDrvCHSC5816::getModelName()
{
    return "CHSC5816";
}

bool TouchDrvCHSC5816::checkOnline()
{
    typedef struct {
        uint16_t fw_ver;
        uint16_t checksum;
        uint32_t sig;
        uint32_t vid_pid;
        uint16_t raw_offset;
        uint16_t dif_offset;
    } Header_t;

    Header_t first;
    Header_t second;

    memset(&second, 0, sizeof(Header_t));
    memset(&first, 0, sizeof(Header_t));

    // CHSC5816_REG_BOOT_STATE 0x20000018
    uint8_t CHSC5816_REG_BOOT_STATE[] = {0x20, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00};
    if (comm->writeBuffer(CHSC5816_REG_BOOT_STATE, arraySize(CHSC5816_REG_BOOT_STATE)) < 0) {
        log_e("comm->writeBuffer clean boot state failed!\n");
        return false;
    }

    reset();

    for (int i = 0; i < 10; ++i) {
        hal->delay(10);
        // CHSC5816_REG_IMG_HEAD 0x20000014
        uint8_t CHSC5816_REG_IMG_HEAD[] = {0x20, 0x00, 0x00, 0x14};
        if (comm->writeThenRead(CHSC5816_REG_IMG_HEAD,
                                arraySize(CHSC5816_REG_IMG_HEAD),
                                (uint8_t *)&first,
                                sizeof(Header_t)) < 0) {
            return false;
        }

        if (comm->writeThenRead(CHSC5816_REG_IMG_HEAD,
                                arraySize(CHSC5816_REG_IMG_HEAD),
                                (uint8_t *)&second,
                                sizeof(Header_t)) < 0) {
            return false;
        }

        if (memcmp(&second, &first, sizeof(Header_t)) != 0 ) {
            continue;
        }
        if (first.sig == CHSC5816_SIG_VALUE) {
            return true;
        }
    }
    return false;
}

bool TouchDrvCHSC5816::initImpl(uint8_t addr)
{

    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }

    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
    }

    reset();

    if (checkOnline()) {
        reset();
        _maxTouchPoints = 1;
        return true;
    }

    return false;
}

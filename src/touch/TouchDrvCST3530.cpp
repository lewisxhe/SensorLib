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
 * @file      TouchDrvCST3530.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-21
 */
#include "TouchDrvCST3530.h"


void TouchDrvCST3530::reset()
{
    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
        hal->digitalWrite(_rst, LOW);
        hal->delay(30);
        hal->digitalWrite(_rst, HIGH);
    }
}

uint16_t TouchDrvCST3530::check_sum_16(int val, uint8_t* buf, uint16_t len)
{
    uint16_t sum = val;
    while (len--) sum += *buf++;
    return sum;
}

uint8_t TouchDrvCST3530::getPoint(int16_t *x_array, int16_t *y_array, uint8_t get_point)
{
    uint8_t numPoints = 0;

    if (!get_point) {
        return 0;
    }

    TouchPoint_t points[MAX_FINGER_NUM];
    uint8_t read_buffer[MAX_READ_BYTES] = {0};
    uint8_t finger_num = 0, key_num = 0;
    uint8_t report_typ = 0;

    if (!writeCommand(READ_COMMAND, read_buffer, sizeof(read_buffer))) {
        writeCommand(CLEAR_COMMAND);
        return 0;
    }

    report_typ = read_buffer[2];
    if (report_typ != 0xFF) {
        writeCommand(CLEAR_COMMAND);
        return 0;
    }

    finger_num = read_buffer[3] & 0x0F;
    key_num = (read_buffer[3] & 0xF0) >> 4;

    if (finger_num == 0 || finger_num > MAX_FINGER_NUM) {
        writeCommand(CLEAR_COMMAND);
        return 0;
    }

    if (x_array == nullptr || y_array == nullptr) {
        writeCommand(CLEAR_COMMAND);
        return finger_num;
    }


    if (get_point > finger_num) {
        get_point = finger_num;
    }

    for (uint8_t i = 0; i < get_point; i++) {
        uint16_t idx = (key_num + i) * 5;
        if (idx + 8 >= MAX_READ_BYTES) {
            break;
        }
        points[i].pos_id   = read_buffer[idx + 8] & 0x0F;
        points[i].event    = read_buffer[idx + 8] >> 4;
        points[i].pos_x    = read_buffer[idx + 4] + ((uint16_t)(read_buffer[idx + 7] & 0x0F) << 8);
        points[i].pos_y    = read_buffer[idx + 5] + ((uint16_t)(read_buffer[idx + 7] & 0xF0) << 4);
        points[i].pres_z   = read_buffer[idx + 6];

        if (check_sum_16(0x55, &read_buffer[4], (key_num + finger_num) * 5) != (read_buffer[0] | read_buffer[1] << 8)) {
            writeCommand(CLEAR_COMMAND);
            return 0;
        }
        if (points[i].event != 0) {
            x_array[i] = points[i].pos_x;
            y_array[i] = points[i].pos_y;
            numPoints++;
            log_d("ID=%d , X=%d, Y=%d, press=%d", points[i].pos_id, points[i].pos_x, points[i].pos_y, points[i].pres_z);
        }
    }

    writeCommand(CLEAR_COMMAND);

    updateXY(numPoints, x_array, y_array);

    return numPoints;
}


bool TouchDrvCST3530::isPressed()
{
    if (_irq != -1) {
        return hal->digitalRead(_irq) == LOW;
    }
    return getPoint(nullptr, nullptr, 1) > 0;
}

const char *TouchDrvCST3530::getModelName()
{
    return "CST3530";
}

void TouchDrvCST3530::sleep()
{
    writeCommand(0xD0000400);
    hal->delay(1);
    writeCommand(0xD0000400);
    writeCommand(SLEEP_COMMAND);
}

void TouchDrvCST3530::wakeup()
{
    reset();
    writeCommand(0xD0000400);
    hal->delay(1);
    writeCommand(0xD0000400);
    writeCommand(0xD0000000);
    writeCommand(0xD0000C00);
    writeCommand(0xD0000100);
}

bool TouchDrvCST3530::initImpl(uint8_t addr)
{
    if (_irq != -1) {
        hal->pinMode(_irq, INPUT);
    }

    reset();

    int retry = 5;
    uint8_t buffer[50];
    while (retry--) {
        if (writeCommand(INFO_COMMAND, buffer, sizeof(buffer))) {
            if (buffer[2] == 0xCA && buffer[3] == 0xCA) {
                _maxTouchPoints = MAX_FINGER_NUM;
                _resY = (buffer[31] << 8) | buffer[30];
                _resX = (buffer[29] << 8) | buffer[28];
                _chipID = buffer[3] << 24 | buffer[2] << 16 | buffer[1] << 8 | buffer[0];
                log_d("Model:CST3530");
                log_d("RST Pin:%d", _rst);
                log_d("IRQ Pin:%d", _irq);
                log_d("Tx Channel:%d", buffer[48]);
                log_d("Rx Channel:%d", buffer[49]);
                log_d("Key Number:%d", buffer[27]);
                log_d("Resolution: %d x %d", _resX, _resY);
                log_d("Project ID:%04X", buffer[39] << 24 | buffer[38] << 16 | buffer[37] << 8 | buffer[36]);
                log_d("Chip Type:%04" PRIX32, _chipID);
                log_d("Firmware Version:%04X", buffer[35] << 24 | buffer[34] << 16 | buffer[33] << 8 | buffer[32]);
                return true;
            }
        }
        hal->delay(10);
    }
    return false;
}


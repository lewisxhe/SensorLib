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
 * @file      TouchDrvHI8561.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-06
 *
 */
#include "TouchDrvHI8561.hpp"

void TouchDrvHI8561::sleep()
{
    // Unable to achieve, relies on screen sleep
}

const TouchPoints &TouchDrvHI8561::getTouchPoints()
{
    static constexpr uint8_t POINT_BUFFER_SIZE = (MAX_FINGER_NUM * BYTES_PER_POINT);
    
    uint8_t buffer[POINT_BUFFER_SIZE] = {0};

    _touchPoints.clear(); // Clear cached touch points

    uint8_t numPoints = 0;
    if (!makePacketThenRead(REG_INFO, buffer, 1)) {
        return _touchPoints;
    }

    numPoints = buffer[0];
    if (numPoints == 0 || numPoints > MAX_FINGER_NUM) {
        return _touchPoints;
    }

    uint8_t expectedBytes = numPoints * BYTES_PER_POINT; // 6 bytes per touch point

    if (!makePacketThenRead(REG_POINT, buffer, expectedBytes)) {
        return _touchPoints;
    }

    for (uint8_t i = 0; i < numPoints; i++) {
        const uint8_t offset =  i * BYTES_PER_POINT ;
        uint16_t x = buffer[offset + 1] | (buffer[offset + 0] << 8);
        uint16_t y = buffer[offset + 3] | (buffer[offset + 2] << 8);
        uint8_t  pressure = buffer[offset + 4];
        // Once edge detection is triggered, subsequent touch points are untrusted and do not need to continue looping.
        if (x == UINT16_MAX || y == UINT16_MAX) {
            log_e("Edge trigger detected, id %d set to invalid coordinates", i);
            break;
        }
        _touchPoints.addPoint(x, y, pressure, i);
    }
    // Swap XY or mirroring coordinates,if set
    updateXY(_touchPoints);

    return _touchPoints;
}

const char *TouchDrvHI8561::getModelName()
{
    return "HI8561";
}

bool TouchDrvHI8561::initImpl(uint8_t)
{
    // Disable I2C stop bit
    setAck(false);

    // Check if the starting address matches
    uint8_t buffer[6];
    if (!makePacketThenRead(REG_SECTION_INFO, buffer, sizeof(buffer))) {
        return false;
    }
    uint32_t reg = buffer[0] + (buffer[1] << 8) + (buffer[2] << 16) + (buffer[3] << 24);
    if (reg != REG_INFO) {
        log_e("Invalid touch info start address: 0x%08" PRIu32, reg);
        return false;
    }

    // Use fixed values ​​to identify chip models.
    _chipID = 0x8561;

    log_d("HI8561 touch start address: 0x%08" PRIu32, reg);

    return true;
}

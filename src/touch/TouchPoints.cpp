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
 * @file      TouchPoints.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-05
 */
#include "TouchPoints.hpp"

TouchPoint TouchPoints::emptyPoint = {};  /**< Empty touch point. */

TouchPoints::TouchPoints() : pointCount(0), gestureValid(false), gesture(Gesture::NONE) {}

void TouchPoints::clear()
{
    pointCount = 0;
    gestureValid = false;
    gesture = Gesture::NONE;
}

bool TouchPoints::addPoint(uint16_t x, uint16_t y, uint16_t pressure, uint8_t id, uint8_t event)
{
    if (pointCount >= MAX_POINTS) return false;
    points[pointCount].x = x;
    points[pointCount].y = y;
    points[pointCount].pressure = pressure;
    points[pointCount].id = id;
    points[pointCount].event = event;
    pointCount++;
    return true;
}

void TouchPoints::setGesture(Gesture g)
{
    gesture = g;
    gestureValid = true;
}

uint8_t TouchPoints::getPointCount() const
{
    return pointCount;
}

TouchPoint &TouchPoints::getPoint(uint8_t index)
{
    if (index < pointCount) return points[index];
    log_e("Invalid touch point index: %u", index);
    return emptyPoint;
}

const TouchPoint &TouchPoints::getPoint(uint8_t index) const
{
    if (index < pointCount) return points[index];
    log_e("Invalid touch point index: %u", index);
    return emptyPoint;
}

bool TouchPoints::hasPoints() const
{
    return pointCount > 0;
}


bool TouchPoints::hasGesture() const
{
    return gestureValid;
}

Gesture TouchPoints::getGesture() const
{
    return gesture;
}

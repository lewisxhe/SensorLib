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
 * @file      TouchPoints.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-05
 */
#pragma once

#include "SensorPlatform.hpp"

/**
 * @brief Represents a single touch point with its properties.
 */
struct TouchPoint {
    uint16_t x;        /**< X coordinate of the touch point. */
    uint16_t y;        /**< Y coordinate of the touch point. */
    uint16_t pressure; /**< Pressure value (0 if not supported by the chip). */
    uint8_t id;        /**< Touch point ID for multi‑touch tracking. */
    uint8_t event;     /**< Event type based on actual hardware feedback, most touch devices do not support. */
};

/**
 * @brief Gesture types recognized by the touch controller.
 */
enum class Gesture {
    NONE,       /**< No gesture detected. */
    TAP,        /**< Single tap gesture. */
    SWIPE_UP,   /**< Swipe upward gesture. */
    SWIPE_DOWN, /**< Swipe downward gesture. */
    SWIPE_LEFT, /**< Swipe leftward gesture. */
    SWIPE_RIGHT,/**< Swipe rightward gesture. */
    HOLD        /**< Hold (long press) gesture. */
};

/**
 * @brief Container for touch point data and optional gesture information.
 *
 * This class holds up to MAX_POINTS touch points and an optional gesture.
 * It is designed to be filled by a touch driver and consumed by the application.
 */
class TouchPoints
{
public:
    /** Maximum number of touch points supported. */
    static constexpr uint8_t MAX_POINTS = 5;

    /**
     * @brief Constructs an empty TouchPoints object.
     */
    TouchPoints();

    ~TouchPoints() = default;

    /**
     * @brief Clears all stored touch points and gesture.
     *
     * Resets the object to its initial empty state.
     */
    void clear();

    /**
     * @brief Adds a touch point to the container.
     *
     * @param x        X coordinate of the touch point.
     * @param y        Y coordinate of the touch point.
     * @param pressure Pressure value (default 0).
     * @param id       Touch point ID (default 0).
     * @param event    Event type (default 0).
     * @return true if the point was added successfully, false if the container is full.
     */
    bool addPoint(uint16_t x, uint16_t y, uint16_t pressure = 0, uint8_t id = 0, uint8_t event = 0);

    /**
     * @brief Sets the gesture for this touch event.
     *
     * @param g The gesture to set.
     */
    void setGesture(Gesture g);

    /**
     * @brief Returns the number of stored touch points.
     *
     * @return Number of valid touch points.
     */
    uint8_t getPointCount() const;

    /**
     * @brief  Gets a reference to a touch point at the specified index.
     * @note   The index must be valid (0 to getPointCount()-1).
     * @param  index: The index of the touch point to retrieve.
     * @retval A reference to the touch point at the specified index.
     */
    TouchPoint &getPoint(uint8_t index);

    /**
     * @brief  Gets a constant reference to a touch point at the specified index.
     * @note   The index must be valid (0 to getPointCount()-1).
     * @param  index: The index of the touch point to retrieve.
     * @retval A constant reference to the touch point at the specified index.
     */
    const TouchPoint &getPoint(uint8_t index) const;

    /**
     * @brief Checks whether any touch points are stored.
     *
     * @return true if at least one touch point is present, false otherwise.
     */
    bool hasPoints() const;

    /**
     * @brief Checks whether a gesture is stored.
     *
     * @return true if a valid gesture is present, false otherwise.
     */
    bool hasGesture() const;

    /**
     * @brief Returns the stored gesture.
     *
     * @return The gesture value. Valid only if hasGesture() returns true.
     */
    Gesture getGesture() const;

    TouchPoint& operator[](uint8_t index) { return getPoint(index); }
    const TouchPoint& operator[](uint8_t index) const { return getPoint(index); }

    TouchPoint* begin() { return points; }
    TouchPoint* end() { return points + pointCount; }
    const TouchPoint* begin() const { return points; }
    const TouchPoint* end() const { return points + pointCount; }

private:
    TouchPoint points[MAX_POINTS]; /**< Array of touch points. */
    uint8_t pointCount;            /**< Number of valid touch points. */
    bool gestureValid;             /**< Flag indicating whether gesture is valid. */
    Gesture gesture;               /**< Stored gesture. */
    static TouchPoint emptyPoint;  /**< Empty touch point. */
};
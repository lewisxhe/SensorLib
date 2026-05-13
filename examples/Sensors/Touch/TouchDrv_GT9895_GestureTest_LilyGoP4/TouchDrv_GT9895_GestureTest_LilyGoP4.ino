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
 * @file      TouchDrv_GT9895_GestureTest_LilyGoP4.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-05-13
 *
 */
#include <Wire.h>   // Adafruit NRF52 CI failure was avoided.
#include <touch/TouchDrvGT9895.hpp>
#include <expander/IoExpanderXL9555.hpp>

// Pin definitions for the LilyGo-P4
#define P4_TOUCH_SDA  7
#define P4_TOUCH_SCL  8
#define IO_EXPANDER_TOUCH_RST   (3 | 0x80)
#define IO_EXPANDER_TOUCH_IRQ   (4 | 0x80)

#ifdef ARDUINO_ESP32P4_DEV

TouchDrvGT9895 touch;
IoExpanderXL9555 expander;

enum GestureType {
    GESTURE_NONE,
    GESTURE_TAP,
    GESTURE_LEFT,
    GESTURE_RIGHT,
    GESTURE_UP,
    GESTURE_DOWN
};

struct GestureState {
    bool active = false;
    uint8_t id = 0xFF;
    int16_t startX = 0;
    int16_t startY = 0;
    int16_t lastX = 0;
    int16_t lastY = 0;
    uint32_t startTime = 0;
    uint32_t lastTime = 0;
    uint16_t samples = 0;
};

GestureState gesture;

static const int SWIPE_THRESHOLD = 70;  // Minimum movement for swipe gesture
static const int TAP_MOVE_THRESHOLD = 18;   // Maximum movement for tap gesture
static const uint32_t TAP_TIME_THRESHOLD = 250; // Maximum time for tap gesture
static const uint32_t MAX_GESTURE_TIME = 1200; // Maximum gesture duration
static const float DOMINANCE_RATIO = 1.2f;  // Dominance ratio for swipe gestures
static const bool enableTap = false;        // Disable print tap event

// only swipe gesture is limited by cooldown
static const uint32_t SWIPE_COOLDOWN_MS = 600;

uint32_t lastSwipeEventTime = 0;
bool swipeEventLocked = false;


void TouchDrvDigitalWrite(uint8_t gpio, uint8_t level)
{
    if (gpio & 0x80) {
        expander.digitalWrite(gpio & 0x7F, level);
    } else {
        digitalWrite(gpio, level);
    }
}

uint8_t TouchDrvDigitalRead(uint8_t gpio)
{
    if (gpio & 0x80) {
        return expander.digitalRead(gpio & 0x7F);
    } else {
        return digitalRead(gpio);
    }
}

void TouchDrvPinMode(uint8_t gpio, uint8_t mode)
{
    if (gpio & 0x80) {
        expander.pinMode(gpio & 0x7F, mode);
    } else {
        pinMode(gpio, mode);
    }
}

const char *gestureToString(GestureType g)
{
    switch (g) {
    case GESTURE_TAP:   return "TAP";
    case GESTURE_LEFT:  return "LEFT";
    case GESTURE_RIGHT: return "RIGHT";
    case GESTURE_UP:    return "UP";
    case GESTURE_DOWN:  return "DOWN";
    default:            return "NONE";
    }
}

bool isSwipeGesture(GestureType g)
{
    return g == GESTURE_LEFT ||
           g == GESTURE_RIGHT ||
           g == GESTURE_UP ||
           g == GESTURE_DOWN;
}

void resetGesture()
{
    gesture.active = false;
    gesture.id = 0xFF;
    gesture.startX = 0;
    gesture.startY = 0;
    gesture.lastX = 0;
    gesture.lastY = 0;
    gesture.startTime = 0;
    gesture.lastTime = 0;
    gesture.samples = 0;
}

GestureType classifyGesture(int dx, int dy, uint32_t dt)
{
    int adx = abs(dx);
    int ady = abs(dy);

    if (dt <= TAP_TIME_THRESHOLD && adx <= TAP_MOVE_THRESHOLD && ady <= TAP_MOVE_THRESHOLD) {
        return GESTURE_TAP;
    }

    if (dt > MAX_GESTURE_TIME) {
        return GESTURE_NONE;
    }

    if (adx >= SWIPE_THRESHOLD && (float)adx > (float)ady * DOMINANCE_RATIO) {
        return dx > 0 ? GESTURE_RIGHT : GESTURE_LEFT;
    }

    if (ady >= SWIPE_THRESHOLD && (float)ady > (float)adx * DOMINANCE_RATIO) {
        return dy > 0 ? GESTURE_DOWN : GESTURE_UP;
    }

    return GESTURE_NONE;
}

void unlockSwipeIfNeeded(uint32_t now)
{
    if (swipeEventLocked && (now - lastSwipeEventTime) >= SWIPE_COOLDOWN_MS) {
        swipeEventLocked = false;
    }
}

bool canEmitSwipe(uint32_t now)
{
    if (!swipeEventLocked) {
        return true;
    }
    return (now - lastSwipeEventTime) >= SWIPE_COOLDOWN_MS;
}

void lockSwipeEvent(uint32_t now)
{
    lastSwipeEventTime = now;
    swipeEventLocked = true;
}

void printGestureResult(GestureType type, int dx, int dy, uint32_t dt)
{
    Serial.println("===== Gesture Result =====");
    Serial.printf("id      : %u\n", gesture.id);
    Serial.printf("start   : (%d, %d)\n", gesture.startX, gesture.startY);
    Serial.printf("end     : (%d, %d)\n", gesture.lastX, gesture.lastY);
    Serial.printf("delta   : dx=%d dy=%d\n", dx, dy);
    Serial.printf("time    : %lu ms\n", (unsigned long)dt);
    Serial.printf("samples : %u\n", gesture.samples);
    Serial.printf("gesture : %s\n", gestureToString(type));
    if (isSwipeGesture(type)) {
        Serial.printf("swipe cooldown: %lu ms\n", (unsigned long)SWIPE_COOLDOWN_MS);
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    Serial.println("GT9895 Gesture Test - swipe cooldown only, tap excluded");

    Wire.begin(P4_TOUCH_SDA, P4_TOUCH_SCL);

    if (!expander.begin(Wire, XL9555_SLAVE_ADDRESS0)) {
        while (1) {
            Serial.println("Failed to find io expander - check your wiring!");
            delay(1000);
        }
    }

    touch.setPins(IO_EXPANDER_TOUCH_RST, IO_EXPANDER_TOUCH_IRQ);
    touch.setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);

    if (!touch.begin(Wire, GT9895_SLAVE_ADDRESS_L)) {
        while (1) {
            Serial.println("Failed to find touch device - check your wiring!");
            delay(1000);
        }
    }

    Serial.println("GT9895 Touch Initialized");
    Serial.print("Chip ID : 0x");
    Serial.println(touch.getChipID(), HEX);

    touch.setResolution(1060, 2400);
    touch.setTargetResolution(568, 1232);

    // If direction is wrong, try:
    // touch.setSwapXY(true);
    // touch.setMirrorXY(true, false);
    // touch.setMirrorXY(false, true);
    // touch.setMirrorXY(true, true);

    Serial.printf("Ready. Swipe cooldown = %lu ms, TAP is not limited.\n",
                  (unsigned long)SWIPE_COOLDOWN_MS);
    Serial.println();
}

void loop()
{
    uint32_t now = millis();
    unlockSwipeIfNeeded(now);

    TouchPoints touch_points = touch.getTouchPoints();

    if (touch_points.hasPoints()) {
        const TouchPoint &point = touch_points.getPoint(0);

        if (!gesture.active) {
            gesture.active = true;
            gesture.id = point.id;
            gesture.startX = point.x;
            gesture.startY = point.y;
            gesture.lastX = point.x;
            gesture.lastY = point.y;
            gesture.startTime = now;
            gesture.lastTime = now;
            gesture.samples = 1;
        } else {
            gesture.lastX = point.x;
            gesture.lastY = point.y;
            gesture.lastTime = now;
            gesture.samples++;
        }
    } else {
        if (gesture.active) {
            int dx = gesture.lastX - gesture.startX;
            int dy = gesture.lastY - gesture.startY;
            uint32_t dt = gesture.lastTime - gesture.startTime;

            GestureType type = classifyGesture(dx, dy, dt);

            if (type == GESTURE_TAP) {
                if (enableTap) {
                    printGestureResult(type, dx, dy, dt);
                }
            } else if (isSwipeGesture(type)) {
                if (canEmitSwipe(now)) {
                    printGestureResult(type, dx, dy, dt);
                    lockSwipeEvent(now);
                }
            }
            resetGesture();
        }
    }

    delay(10);
}

#else
void setup()
{
    Serial.begin(115200);
}
void loop()
{
    Serial.println("Sketch only works with LilyGo-T-Display-P4 AMOLED(GT9895) Version");
    delay(1000);
}
#endif
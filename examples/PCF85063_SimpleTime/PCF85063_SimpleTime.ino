/**
 *
 * @license MIT License
 *
 * Copyright (c) 2023 lewis he
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
 * @file      PCF85063_SimpleTime.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-07
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorPCF85063.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  7
#endif

SensorPCF85063 rtc;
uint32_t intervalue;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    pinMode(SENSOR_IRQ, INPUT_PULLUP);

    if (!rtc.begin(Wire, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find PCF85063 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    uint16_t year = 2023;
    uint8_t month = 9;
    uint8_t day = 7;
    uint8_t hour = 11;
    uint8_t minute = 24;
    uint8_t second = 30;

    rtc.setDateTime(year, month, day, hour, minute, second);

}


void loop()
{
    if (millis() - intervalue > 1000) {
        intervalue = millis();
        RTC_DateTime datetime = rtc.getDateTime();
        Serial.print(" Year :"); Serial.print(datetime.year);
        Serial.print(" Month:"); Serial.print(datetime.month);
        Serial.print(" Day :"); Serial.print(datetime.day);
        Serial.print(" Hour:"); Serial.print(datetime.hour);
        Serial.print(" Minute:"); Serial.print(datetime.minute);
        Serial.print(" Sec :"); Serial.println(datetime.second);

    }
}




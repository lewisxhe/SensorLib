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
 * @file      BHI360_NoMotion.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-07
 * @note      Changed from Boschsensortec API https://github.com/boschsensortec/BHY2_SensorAPI
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <SensorBHI360.hpp>
#include <bosch/BoschSensorDataHelper.hpp>

// #define USE_I2C_INTERFACE        true
// #define USE_SPI_INTERFACE        true

#if !defined(USE_I2C_INTERFACE) && !defined(USE_SPI_INTERFACE)
#define USE_I2C_INTERFACE
#warning "No interface type is selected, use I2C interface"
#endif

#if defined(USE_SPI_INTERFACE)
#ifndef SPI_MOSI
#define SPI_MOSI    33
#endif

#ifndef SPI_MISO
#define SPI_MISO    34
#endif

#ifndef SPI_SCK
#define SPI_SCK     35
#endif

// If BHI360_IRQ is set to -1, sensor interrupts are not used and the sensor polling method is used instead.
#ifndef BHI360_IRQ
#define BHI360_IRQ  37
#endif

#ifndef BHI360_CS
#define BHI360_CS   36
#endif

#else   //* I2C */

#ifndef BHI360_SDA
#define BHI360_SDA  2
#endif

#ifndef BHI360_SCL
#define BHI360_SCL  3
#endif

// If BHI360_IRQ is set to -1, sensor interrupts are not used and the sensor polling method is used instead.
#ifndef BHI360_IRQ
#define BHI360_IRQ  8
#endif
#endif  /*USE_SPI_INTERFACE*/

#ifndef BHI360_RST
#define BHI360_RST -1
#endif

SensorBHI360 bhy;
SensorNoMotion noMotion(bhy);

// #define BOSCH_DATA_INJECT_BHI360
// #define BOSCH_DATA_INJECT_BHI360_BMM150
// #define BOSCH_DATA_INJECT_BHI360_BMM150_HEAD_ORIENTATION
// #define BOSCH_DATA_INJECT_BHI360_BMM350
// #define BOSCH_DATA_INJECT_BHI360_BMM350_HEAD_ORIENTATION
// #define BOSCH_DATA_INJECT_BHI360_ENV
// #define BOSCH_DATA_INJECT_BHI360_HEAD_ORIENTATION
// #define BOSCH_DATA_INJECT_BHI360_MOTION_AI
#define BOSCH_SHUTTLE3_BHI360
// #define BOSCH_SHUTTLE3_BHI360_AUX_BMM150
// #define BOSCH_SHUTTLE3_BHI360_BMM150
// #define BOSCH_SHUTTLE3_BHI360_BMM150_BMP580_BME688
// #define BOSCH_SHUTTLE3_BHI360_BMM150_HEAD_ORIENTATION
// #define BOSCH_SHUTTLE3_BHI360_BMM350C
// #define BOSCH_SHUTTLE3_BHI360_BMM350C_BME688_IAQ
// #define BOSCH_SHUTTLE3_BHI360_BMM350C_BMP580
// #define BOSCH_SHUTTLE3_BHI360_BMM350C_BMP580_BME688
// #define BOSCH_SHUTTLE3_BHI360_BMM350C_HEAD_ORIENTATION
// #define BOSCH_SHUTTLE3_BHI360_BMM350C_POLL
// #define BOSCH_SHUTTLE3_BHI360_BMM350C_TURBO
// #define BOSCH_SHUTTLE3_BHI360_BMP580_TEST_EXAMPLE
// #define BOSCH_SHUTTLE3_BHI360_HW_ACTIVITY
// #define BOSCH_SHUTTLE3_BHI360_HW_ACTIVITY_TURBO
// #define BOSCH_SHUTTLE3_BHI360_IMU_HEAD_ORIENTATION
// #define BOSCH_SHUTTLE3_BHI360_POLLING_STEP_COUNTER
// #define BOSCH_SHUTTLE3_BHI360_TEMP
// #define BOSCH_SHUTTLE3_BHI360_TEST_DATA_SOURCE
// #define BOSCH_SHUTTLE3_BHI360_TURBO

// The selected firmware for running the BHI360 is included by including `#include <BoschFirmware.h>`.
#include <BoschFirmware.h>


#if BHI360_IRQ > 0
#define USING_SENSOR_IRQ_METHOD
#endif

#ifdef USING_SENSOR_IRQ_METHOD
volatile bool isInterruptTriggered = false;

void dataReadyISR()
{
    isInterruptTriggered = true;
}
#endif /*USING_SENSOR_IRQ_METHOD*/


// Firmware update progress callback
void progress_callback(uint32_t total, uint32_t transferred, void *user_data)
{
    float progress = (float)transferred / total * 100;
    Serial.print("Upload progress: ");
    Serial.print(progress);
    Serial.println("%");
}


void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // Set the reset pin
    bhy.setPins(BHI360_RST);

    Serial.println("Initializing Sensors...");

    // Set the firmware array address and firmware size
    bhy.setFirmware(bosch_firmware_image, bosch_firmware_size);

    // Set the firmware update processing progress callback function
    // bhy.setUpdateProcessCallback(progress_callback, NULL);

    // Set the maximum transfer bytes of I2C/SPI,The default size is I2C 32 bytes, SPI 256 bytes.
    // bhy.setMaxiTransferSize(256);

    // Set the processing fifo data buffer size,The default size is 512 bytes.
    // bhy.setProcessBufferSize(1024);

#ifdef USE_I2C_INTERFACE
    // Using I2C interface
    // BHI360_SLAVE_ADDRESS_L = 0x28
    // BHI360_SLAVE_ADDRESS_H = 0x29
    if (!bhy.begin(Wire, BHI360_SLAVE_ADDRESS_L, BHI360_SDA, BHI360_SCL)) {
        Serial.print("Failed to initialize sensor - error code:");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#endif

#ifdef USE_SPI_INTERFACE
    // Using SPI interface
    if (!bhy.begin(SPI, BHI360_CS, SPI_MOSI, SPI_MISO, SPI_SCK)) {
        Serial.print("Failed to initialize sensor - error code:");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#endif

    Serial.println("Initializing the sensor successfully!");

#ifdef USING_SENSOR_IRQ_METHOD
    // Set the specified pin (BHI360_IRQ) ​​to an input pin.
    // This makes the pin ready to receive external signals.
    // If the interrupt is already connected, if BHI360_IRQ is equal to -1 then the polling method will be used
    pinMode(BHI360_IRQ, INPUT);

    // Attach an interrupt service routine (ISR) to the specified pin (BHI360_IRQ).
    // The ISR 'dataReadyISR' will be called whenever a rising edge is detected on the pin.
    attachInterrupt(BHI360_IRQ, dataReadyISR, RISING);
#endif

    // Enable no motion detection
    // A single trigger can be performed by passing MotionMode::ONCE,
    // noMotion.enable(MotionMode::ONCE);

    // Continuous triggering can be achieved by passing MotionMode::CONTINUOUS.
    noMotion.enable(MotionMode::CONTINUOUS);

    // If no action is detected, the callback function will be triggered.
    noMotion.setOnMotionCallback([]() {
        Serial.print("[");
        Serial.print(millis());
        Serial.print("] ");
        Serial.print("No motion detected.");
        Serial.print(" state :");
        Serial.println(noMotion.isEnabled() ? "continuous mode" : "disabled");
    });

    Serial.println("Sensor is ready, please keep the sensor stationary.");
}


void loop()
{
#ifdef USING_SENSOR_IRQ_METHOD
    if (isInterruptTriggered) {
        isInterruptTriggered = false;
#endif /*USING_SENSOR_IRQ_METHOD*/

        /* If the interrupt is connected to the sensor and BHI360_IRQ is not equal to -1,
         * the interrupt function will be enabled, otherwise the method of polling the sensor is used
         */
        bhy.update();

#ifdef USING_SENSOR_IRQ_METHOD
    }
#endif /*USING_SENSOR_IRQ_METHOD*/


    delay(50);
}

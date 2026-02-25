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
 * @file      BHI360_Multi_Tap.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-24
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

/*
* Define the USING_DATA_HELPER use of data assistants.
* No callback function will be used. Data can be obtained directly through
* the data assistant. Note that this method is not a thread-safe function.
* Please pay attention to protecting data access security.
* */
#define USING_DATA_HELPER

#ifdef USING_DATA_HELPER
SensorMultiTap multiTap(bhy);
#endif

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

#include <BoschFirmware.h>

// Force update of current firmware, whether it exists or not.
// Only works when external SPI Flash is connected to BHI360.
// After uploading firmware once, you can change this to false to speed up boot time.
bool force_update_flash_firmware = true;

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

#ifndef USING_DATA_HELPER
void parse_multitap(uint8_t  sensor_id, const uint8_t *data_ptr, uint32_t len, uint64_t *timestamp, void *user_data)
{
    MultiTapDataType multitap_data = BHI360_NO_TAP;
    if (data_ptr == NULL) {
        Serial.println("Data pointer is null");
        return;
    }
    multitap_data = (MultiTapDataType)(*data_ptr);
    Serial.print("SID: ");
    Serial.print(sensor_id);
    Serial.print("; T: ");
    Serial.print((uint32_t)(*timestamp / 1000000000ULL));
    Serial.print(".");
    Serial.print((uint32_t)(*timestamp % 1000000000ULL));
    Serial.print("; ");
    Serial.print(bhi360_event_data_multi_tap_string_out[multitap_data]);
    Serial.println("; ");
}
#endif

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

    Serial.println("Initializing Sensors...");

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

    // Output all sensors info to Serial
    BoschSensorInfo info = bhy.getSensorInfo();

#ifdef ARDUINO
    ArduinoStreamPrinter printer(Serial);
    info.printInfo(printer);
#else
    info.printInfo([](const char *format, ...) -> int {
        va_list args;
        va_start(args, format);
        int result = vprintf(format, args);
        va_end(args);
        return result;
    });
#endif

    // The tap detect sensor will only report when it changes, so the value is 0 ~ 1
    float sample_rate = 1.0;
    uint32_t report_latency_ms = 0; /* Report immediately */

#ifdef USING_DATA_HELPER
    multiTap.enable(sample_rate, report_latency_ms);
#else
    // Enable Multi Tap detector
    bhy.configure(BoschSensorID::MULTI_TAP, sample_rate, report_latency_ms);
    // Set the callback function for multi tap events
    bhy.onResultEvent(BoschSensorID::MULTI_TAP, parse_multitap);
#endif

#ifdef USING_SENSOR_IRQ_METHOD
    // Set the specified pin (BHI360_IRQ) ​​to an input pin.
    // This makes the pin ready to receive external signals.
    // If the interrupt is already connected, if BHI360_IRQ is equal to -1 then the polling method will be used
    pinMode(BHI360_IRQ, INPUT);

    // Attach an interrupt service routine (ISR) to the specified pin (BHI360_IRQ).
    // The ISR 'dataReadyISR' will be called whenever a rising edge is detected on the pin.
    attachInterrupt(BHI360_IRQ, dataReadyISR, RISING);
#endif

    MultiTapDataType buffer[8] {};
    MultiTapDataType multitap_setting = BHI360_TRIPLE_DOUBLE_SINGLE_TAP;
    MultiTapDetectorConfig multitap_cnfg {};

    Serial.println("--- Multi tap log start ---");

    if (bhy.getMultiTapParamConfig(buffer)) {
        Serial.print("Multi Tap Info : ");
        Serial.println(bhi360_event_data_multi_tap_string_out[buffer[0]]);
    } else {
        Serial.println("Failed to get multi tap config");
    }

    if (bhy.setMultiTapParamConfig(multitap_setting)) {
        Serial.println("Set multi tap config successfully");
    } else {
        Serial.println("Failed to set multi tap config");
    }

    if (bhy.getMultiTapDetectorConfig(multitap_cnfg)) {
        Serial.print("Single Tap CNFG : ");
        Serial.println(((multitap_setting & (uint8_t)BHI360_SINGLE_TAP) == (uint8_t)BHI360_SINGLE_TAP) ? "Enabled" : "Disabled");
        Serial.print("    \t\t -<axis_sel> : ");
        Serial.println(multitap_cnfg.stap_setting.as_s.axis_sel);
        Serial.print("    \t\t -<wait_for_timeout> : ");
        Serial.println(multitap_cnfg.stap_setting.as_s.wait_for_timeout);
        Serial.print("    \t\t -<max_pks_for_tap> : ");
        Serial.println(multitap_cnfg.stap_setting.as_s.max_peaks_for_tap);
        Serial.print("    \t\t -<mode> : ");
        Serial.println(multitap_cnfg.stap_setting.as_s.mode);
        Serial.print("Double Tap CNFG : ");
        Serial.println(((multitap_setting & (uint8_t)BHI360_DOUBLE_TAP) == (uint8_t)BHI360_DOUBLE_TAP) ? "Enabled" : "Disabled");
        Serial.print("    \t\t -<tap_peak_thrs> : ");
        Serial.println(multitap_cnfg.dtap_setting.as_s.tap_peak_thres);
        Serial.print("    \t\t -<max_ges_dur> : ");
        Serial.println(multitap_cnfg.dtap_setting.as_s.max_gesture_dur);
        Serial.print("Triple Tap CNFG : ");
        Serial.println(((multitap_setting & (uint8_t)BHI360_TRIPLE_TAP) == (uint8_t)BHI360_TRIPLE_TAP) ? "Enabled" : "Disabled");
        Serial.print("    \t\t -<max_dur_bw_pks> : ");
        Serial.println(multitap_cnfg.ttap_setting.as_s.max_dur_between_peaks);
        Serial.print("    \t\t -<tap_shock_settl_dur> : ");
        Serial.println(multitap_cnfg.ttap_setting.as_s.tap_shock_settling_dur);
        Serial.print("    \t\t -<min_quite_dur_bw_taps> : ");
        Serial.println(multitap_cnfg.ttap_setting.as_s.min_quite_dur_between_taps);
        Serial.print("    \t\t -<quite_time_after_ges> : ");
        Serial.println(multitap_cnfg.ttap_setting.as_s.quite_time_after_gesture);

        multitap_cnfg.stap_setting.as_s.mode = 0; /* Sensitive mode*/

        if (!bhy.setMultiTapDetectorConfig(multitap_cnfg)) {
            Serial.println("Failed to set multi tap detector config");
            return;
        }

        if (bhy.getMultiTapDetectorConfig(multitap_cnfg)) {
            Serial.print("Multi tap mode after setting : ");
            Serial.println(multitap_cnfg.stap_setting.as_s.mode);
        } else {
            Serial.println("Failed to get multi tap detector config");
        }
    } else {
        Serial.println("Failed to get multi tap detector config");
    }
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

#ifdef USING_DATA_HELPER
    uint32_t s;
    uint32_t ns;
    if (multiTap.hasUpdated()) {
        multiTap.getLastTime(s, ns);
        Serial.print("[T: ");
        Serial.print(s);
        Serial.print(".");
        Serial.print(ns);
        Serial.print("]: Multi Tap:");
        MultiTapDataType tap = multiTap.getValue();
        switch (tap) {
        case BHI360_NO_TAP:
            Serial.println("No Tap");
            break;
        case BHI360_SINGLE_TAP:
            Serial.println("Single Tap");
            break;
        case BHI360_DOUBLE_TAP:
            Serial.println("Double Tap");
            break;
        case BHI360_DOUBLE_SINGLE_TAP:
            Serial.println("Double Single Tap");
            break;
        case BHI360_TRIPLE_TAP:
            Serial.println("Triple Tap");
            break;
        case BHI360_TRIPLE_SINGLE_TAP:
            Serial.println("Triple Single Tap");
            break;
        case BHI360_TRIPLE_DOUBLE_TAP:
            Serial.println("Triple Double Tap");
            break;
        case BHI360_TRIPLE_DOUBLE_SINGLE_TAP:
            Serial.println("Triple Double Single Tap");
            break;
        default:
            Serial.println("Unknown Tap");
            break;
        }
    }
#endif
    delay(50);
}



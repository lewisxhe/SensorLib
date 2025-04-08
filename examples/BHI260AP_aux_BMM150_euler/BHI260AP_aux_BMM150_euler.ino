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
 * @file      BHI260AP_aux_BMM150_euler.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-23
 * @note      Changed from Boschsensortec API https://github.com/boschsensortec/BHY2_SensorAPI
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <SensorBHI260AP.hpp>
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

// If BHI260_IRQ is set to -1, sensor interrupts are not used and the sensor polling method is used instead.
#ifndef BHI260_IRQ
#define BHI260_IRQ  37
#endif

#ifndef BHI260_CS
#define BHI260_CS   36
#endif

#else   //* I2C */

#ifndef BHI260_SDA
#define BHI260_SDA  2
#endif

#ifndef BHI260_SCL
#define BHI260_SCL  3
#endif

// If BHI260_IRQ is set to -1, sensor interrupts are not used and the sensor polling method is used instead.
#ifndef BHI260_IRQ
#define BHI260_IRQ  8
#endif
#endif  /*USE_SPI_INTERFACE*/

#ifndef BHI260_RST
#define BHI260_RST -1
#endif

SensorBHI260AP bhy;

/*
* Define the USING_DATA_HELPER use of data assistants.
* No callback function will be used. Data can be obtained directly through
* the data assistant. Note that this method is not a thread-safe function.
* Please pay attention to protecting data access security.
* */
#define USING_DATA_HELPER

#ifdef USING_DATA_HELPER
SensorEuler euler(bhy);
#endif

// The firmware runs in RAM and will be lost if the power is off. The firmware will be loaded from RAM each time it is run.
// #define BOSCH_APP30_SHUTTLE_BHI260_FW
#define BOSCH_APP30_SHUTTLE_BHI260_AUX_BMM150FW
// #define BOSCH_APP30_SHUTTLE_BHI260_BME68X
// #define BOSCH_APP30_SHUTTLE_BHI260_BMP390
// #define BOSCH_APP30_SHUTTLE_BHI260_TURBO
// #define BOSCH_BHI260_AUX_BEM280
// #define BOSCH_BHI260_AUX_BMM150_BEM280
// #define BOSCH_BHI260_AUX_BMM150_BEM280_GPIO
// #define BOSCH_BHI260_AUX_BMM150_GPIO
// #define BOSCH_BHI260_GPIO

// Firmware is stored in flash and booted from flash,Depends on BHI260 hardware connected to SPI Flash
// #define BOSCH_APP30_SHUTTLE_BHI260_AUX_BMM150_FLASH
// #define BOSCH_APP30_SHUTTLE_BHI260_BME68X_FLASH
// #define BOSCH_APP30_SHUTTLE_BHI260_BMP390_FLASH
// #define BOSCH_APP30_SHUTTLE_BHI260_FLASH
// #define BOSCH_APP30_SHUTTLE_BHI260_TURBO_FLASH
// #define BOSCH_BHI260_AUX_BEM280_FLASH
// #define BOSCH_BHI260_AUX_BMM150_BEM280_FLASH
// #define BOSCH_BHI260_AUX_BMM150_BEM280_GPIO_FLASH
// #define BOSCH_BHI260_AUX_BMM150_GPIO_FLASH
// #define BOSCH_BHI260_GPIO_FLASH

#include <BoschFirmware.h>

// Force update of current firmware, whether it exists or not.
// Only works when external SPI Flash is connected to BHI260.
// After uploading firmware once, you can change this to false to speed up boot time.
bool force_update_flash_firmware = true;

#if BHI260_IRQ > 0
#define USING_SENSOR_IRQ_METHOD
#endif

#ifdef USING_SENSOR_IRQ_METHOD
bool isReadyFlag = false;

void dataReadyISR()
{
    isReadyFlag = true;
}
#endif /*USING_SENSOR_IRQ_METHOD*/

#ifndef USING_DATA_HELPER
void parse_euler(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len, uint64_t *timestamp, void *user_data)
{
    struct bhy2_data_orientation data;
    uint32_t s, ns;
    uint64_t tns;
    // Function to parse FIFO frame data into orientation
    bhy2_parse_orientation(data_ptr, &data);
    uint64_t _timestamp =  *timestamp;
    time_to_s_ns(_timestamp, &s, &ns, &tns);
    uint8_t accuracy =  bhy.getAccuracy();
    if (accuracy) {
        Serial.print("SID:"); Serial.print(sensor_id);
        Serial.print(" T:"); Serial.print(s);
        Serial.print("."); Serial.print(ns);
        Serial.print(" x:"); Serial.print(data.heading * 360.0f / 32768.0f);
        Serial.print(" y:"); Serial.print(data.pitch * 360.0f / 32768.0f);
        Serial.print(" x:"); Serial.print(data.roll * 360.0f / 32768.0f);
        Serial.print(" acc:"); Serial.println(accuracy);
    } else {
        Serial.print("SID:"); Serial.print(sensor_id);
        Serial.print(" T:"); Serial.print(s);
        Serial.print("."); Serial.print(ns);
        Serial.print(" x:"); Serial.print(data.heading * 360.0f / 32768.0f);
        Serial.print(" y:"); Serial.print(data.pitch * 360.0f / 32768.0f);
        Serial.print(" x:"); Serial.println(data.roll * 360.0f / 32768.0f);
    }
}
#endif

// Firmware update progress callback
void progress_callback(void *user_data, uint32_t total, uint32_t transferred)
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
    bhy.setPins(BHI260_RST);

    Serial.println("Initializing Sensors...");

    // Set the firmware array address and firmware size
    bhy.setFirmware(bosch_firmware_image, bosch_firmware_size, bosch_firmware_type, force_update_flash_firmware);

    // Set the firmware update processing progress callback function
    // bhy.setUpdateProcessCallback(progress_callback, NULL);

    // Set the maximum transfer bytes of I2C/SPI,The default size is I2C 32 bytes, SPI 256 bytes.
    // bhy.setMaxiTransferSize(256);

    // Set the processing fifo data buffer size,The default size is 512 bytes.
    // bhy.setProcessBufferSize(1024);

    // Set to load firmware from flash
    bhy.setBootFromFlash(bosch_firmware_type);

#ifdef USE_I2C_INTERFACE
    // Using I2C interface
    // BHI260AP_SLAVE_ADDRESS_L = 0x28
    // BHI260AP_SLAVE_ADDRESS_H = 0x29
    if (!bhy.begin(Wire, BHI260AP_SLAVE_ADDRESS_L, BHI260_SDA, BHI260_SCL)) {
        Serial.print("Failed to initialize sensor - error code:");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#endif

#ifdef USE_SPI_INTERFACE
    // Using SPI interface
    if (!bhy.begin(SPI, BHI260_CS, SPI_MOSI, SPI_MISO, SPI_SCK)) {
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
#ifdef PLATFORM_HAS_PRINTF
    info.printInfo(Serial);
#else
    info.printInfo();
#endif

    float sample_rate = 100.0;      /* Read out data measured at 100Hz */
    uint32_t report_latency_ms = 0; /* Report immediately */

    /*
    * Enable Euler function
    * The Euler function depends on BMM150.
    * If the hardware is not connected to BMM150, the Euler function cannot be used.
    * * */
#ifdef USING_DATA_HELPER
    euler.enable(sample_rate, report_latency_ms);
#else
    bhy.configure(SensorBHI260AP::ORIENTATION_WAKE_UP, sample_rate, report_latency_ms);
    // Register event callback function
    bhy.onResultEvent(SensorBHI260AP::ORIENTATION_WAKE_UP, parse_euler);
#endif

#ifdef USING_SENSOR_IRQ_METHOD
    // Set the specified pin (BHI260_IRQ) ​​to an input pin.
    // This makes the pin ready to receive external signals.
    // If the interrupt is already connected, if BHI260_IRQ is equal to -1 then the polling method will be used
    pinMode(BHI260_IRQ, INPUT);

    // Attach an interrupt service routine (ISR) to the specified pin (BHI260_IRQ).
    // The ISR 'dataReadyISR' will be called whenever a rising edge is detected on the pin.
    attachInterrupt(BHI260_IRQ, dataReadyISR, RISING);
#endif
}


void loop()
{
#ifdef USING_SENSOR_IRQ_METHOD
    if (isReadyFlag) {
        isReadyFlag = false;
#endif /*USING_SENSOR_IRQ_METHOD*/

        /* If the interrupt is connected to the sensor and BHI260_IRQ is not equal to -1,
         * the interrupt function will be enabled, otherwise the method of polling the sensor is used
         */
        bhy.update();

#ifdef USING_SENSOR_IRQ_METHOD
    }
#endif /*USING_SENSOR_IRQ_METHOD*/

#ifdef USING_DATA_HELPER
    if (euler.hasUpdated()) {
        // Print the roll angle to the serial monitor.
        Serial.print(euler.getRoll());
        // Print a comma as a separator between the roll and pitch angles.
        Serial.print(",");
        // Print the pitch angle to the serial monitor.
        Serial.print(euler.getPitch());
        // Print a comma as a separator between the pitch and yaw angles.
        Serial.print(",");
        // Print the yaw angle to the serial monitor and start a new line.
        Serial.println(euler.getHeading());
    }
#endif
    delay(50);
}




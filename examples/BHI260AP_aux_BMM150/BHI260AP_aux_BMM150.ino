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
 * @file      BHI260AP_aux_BMM150.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-07-22
 * @note      Changed from Boschsensortec API https://github.com/boschsensortec/BHY2_SensorAPI
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorBHI260AP.hpp"


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

#ifndef BHI260_IRQ
#define BHI260_IRQ  8
#endif
#endif  /*USE_SPI_INTERFACE*/

#ifndef BHI260_RST
#define BHI260_RST -1
#endif

/*
Write the firmware containing the BMM150 magnetometer function into the flash.
This function requires the BHI260AP external SPI Flash.
If there is no Flash, it can only be written and run in RAM.
Example firmware source: https://github.com/boschsensortec/BHY2_SensorAPI/tree/master/firmware
You can also compile custom firmware to write
How to build custom firmware see : https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bhi260ab-an000.pdf
*/
#define WRITE_TO_FLASH          1           //Set 1 write fw to flash ,set 0 write fw to ram

#if   WRITE_TO_FLASH
#include "BHI260AP_aux_BMM150-flash.fw.h"
const uint8_t *firmware = bhi26ap_aux_bmm150_flash_fw;
const size_t fw_size = sizeof(bhi26ap_aux_bmm150_flash_fw);

#else
#include "BHI260AP_aux_BMM150.fw.h"
const uint8_t *firmware = bhi26ap_aux_bmm150_fw;
const size_t fw_size = sizeof(bhi26ap_aux_bmm150_fw);
#endif


SensorBHI260AP bhy;

bool isReadyFlag = false;

void dataReadyISR()
{
    isReadyFlag = true;
}

void bhy_process_callback(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len, uint64_t *timestamp)
{
    struct bhy2_data_xyz data;
    float scaling_factor = get_sensor_default_scaling(sensor_id);
    bhy2_parse_xyz(data_ptr, &data);
    Serial.print(bhy.getSensorName(sensor_id));
    Serial.print(":");
    Serial.printf("x: %f, y: %f, z: %f;\r\n",
                  data.x * scaling_factor,
                  data.y * scaling_factor,
                  data.z * scaling_factor
                 );
}


void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // Set the reset pin
    bhy.setPins(BHI260_RST);

    // Force update of the current firmware, regardless of whether it exists.
    // After uploading the firmware once, you can change it to false to speed up the startup time.
    bool force_update = true;
    // Set the firmware array address and firmware size
    bhy.setFirmware(firmware, fw_size, WRITE_TO_FLASH, force_update);

#if WRITE_TO_FLASH
    // Set to load firmware from flash
    bhy.setBootFromFlash(true);
#endif

    Serial.println("Initializing Sensors...");

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
    info.printInfo(Serial);

    float sample_rate = 100.0;      /* Read out hintr_ctrl measured at 100Hz */
    uint32_t report_latency_ms = 0; /* 0 = report immediately */

    // Enable acceleration
    report_latency_ms = 1000;   // Report once per second
    bhy.configure(SENSOR_ID_ACC_PASS, sample_rate, report_latency_ms);

    // Enable gyroscope
    report_latency_ms = 1000;   //Report once per second
    bhy.configure(SENSOR_ID_GYRO_PASS, sample_rate, report_latency_ms);

    // Enable magnetometer
    report_latency_ms = 500;    //Report every 500 milliseconds
    bhy.configure(SENSOR_ID_MAG_PASS, sample_rate, report_latency_ms);

    // Set the acceleration sensor result callback function
    bhy.onResultEvent(SENSOR_ID_ACC_PASS, bhy_process_callback);

    // Set the gyroscope sensor result callback function
    bhy.onResultEvent(SENSOR_ID_GYRO_PASS, bhy_process_callback);

    // Set the magnetometer sensor result callback function
    bhy.onResultEvent(SENSOR_ID_MAG_PASS, bhy_process_callback);

    // Register interrupt function
    pinMode(BHI260_IRQ, INPUT);
    attachInterrupt(BHI260_IRQ, dataReadyISR, RISING);
}


void loop()
{
    // Update sensor fifo
    if (isReadyFlag) {
        isReadyFlag = false;
        bhy.update();
    }
    delay(50);
}




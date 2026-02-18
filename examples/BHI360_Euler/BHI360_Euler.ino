/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
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
 * @file      BHI360_Euler.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-14
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
SensorQuaternion quaternion(bhy);
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

#if BHI360_IRQ > 0
#define USING_SENSOR_IRQ_METHOD
#endif

#ifdef USING_SENSOR_IRQ_METHOD
bool isReadyFlag = false;

void dataReadyISR()
{
    isReadyFlag = true;
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

/**
 * @brief Parse the quaternion data from the sensor and convert it to Euler angles.
 *
 * This function serves as a callback to handle the sensor data related to the game rotation vector.
 * It takes the raw quaternion data received from the sensor, converts it into Euler angles (roll, pitch, and yaw),
 *
 * @param sensor_id The ID of the sensor that generated the data. It helps in identifying which specific sensor
 *                  the data is coming from, especially in systems with multiple sensors.
 * @param data_ptr A pointer to the buffer containing the raw quaternion data. The data in this buffer
 *                 represents the orientation of the sensor in quaternion format.
 * @param len The length of the data buffer, indicating the size of the quaternion data in bytes.
 * @param timestamp A pointer to a 64 - bit unsigned integer representing the timestamp when the sensor data was captured.
 *                  This can be used to correlate the data with a specific point in time.
 * @param user_data A generic pointer to user - defined data.
 */
#ifndef USING_DATA_HELPER
void parse_quaternion(uint8_t sensor_id, const uint8_t *data_ptr, uint32_t len, uint64_t *timestamp, void *user_data)
{
    // Declare variables to store the Euler angles (roll, pitch, and yaw).
    float roll, pitch, yaw;
    // Call the bhy2_quaternion_to_euler function to convert the raw quaternion data
    // pointed to by data_ptr into Euler angles (roll, pitch, and yaw).
    bhy2_quaternion_to_euler(data_ptr, &roll,  &pitch, &yaw);
    // Print the roll angle to the serial monitor.
    Serial.print(roll);
    // Print a comma as a separator between the roll and pitch angles.
    Serial.print(",");
    // Print the pitch angle to the serial monitor.
    Serial.print(pitch);
    // Print a comma as a separator between the pitch and yaw angles.
    Serial.print(",");
    // Print the yaw angle to the serial monitor and start a new line.
    Serial.println(yaw);
}
#endif

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // Set the reset pin
    bhy.setPins(BHI360_RST);

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

    /**
    * @brief Set the axis remapping for the sensor based on the specified orientation.
    *
    * This function allows you to configure the sensor's axis remapping according to a specific
    * physical orientation of the chip. By passing one of the values from the SensorRemap enum,
    * you can ensure that the sensor data is correctly interpreted based on how the chip is placed.
    * [bst-bhi260ab-ds000.pdf](https://www.mouser.com/datasheet/2/783/bst-bhi260ab-ds000-1816249.pdf)
    * 20.3 Sensing axes and axes remapping
    * @param remap An enumeration value from SensorRemap that specifies the desired axis remapping.
    * @return Returns true if the axis remapping is successfully set; false otherwise.
    */

    // Set the sensor's axis remapping based on the chip's orientation.
    // These commented-out lines show different ways to configure the sensor according to the chip's corners.
    // When the chip is viewed from the top, set the orientation to the top-left corner.
    // bhy.setRemapAxes(SensorRemap::TOP_LAYER_LEFT_CORNER);
    // When the chip is viewed from the top, set the orientation to the top-right corner.
    // bhy.setRemapAxes(SensorRemap::TOP_LAYER_RIGHT_CORNER);
    // When the chip is viewed from the top, set the orientation to the bottom-right corner of the top layer.
    // bhy.setRemapAxes(SensorRemap::TOP_LAYER_BOTTOM_RIGHT_CORNER);
    // When the chip is viewed from the top, set the orientation to the bottom-left corner of the top layer.
    // bhy.setRemapAxes(SensorRemap::TOP_LAYER_BOTTOM_LEFT_CORNER);
    // When the chip is viewed from the bottom, set the orientation to the top-left corner of the bottom layer.
    // bhy.setRemapAxes(SensorRemap::BOTTOM_LAYER_TOP_LEFT_CORNER);
    // When the chip is viewed from the bottom, set the orientation to the top-right corner of the bottom layer.
    // bhy.setRemapAxes(SensorRemap::BOTTOM_LAYER_TOP_RIGHT_CORNER);
    // When the chip is viewed from the bottom, set the orientation to the bottom-right corner of the bottom layer.
    // bhy.setRemapAxes(SensorRemap::BOTTOM_LAYER_BOTTOM_RIGHT_CORNER);
    // When the chip is viewed from the bottom, set the orientation to the bottom-left corner of the bottom layer.
    // bhy.setRemapAxes(SensorRemap::BOTTOM_LAYER_BOTTOM_LEFT_CORNER);

    // Define the sample rate for data reading.
    // The sensor will read out data measured at a frequency of 100Hz.
    float sample_rate = 100.0;
    // Define the report latency in milliseconds.
    // A value of 0 means the sensor will report the measured data immediately.
    uint32_t report_latency_ms = 0;

#ifdef USING_DATA_HELPER
    quaternion.enable(sample_rate, report_latency_ms);
#else
    // GAME_ROTATION_VECTOR virtual sensor does not rely on external magnetometers, such as BMM150, BMM350
    // Configure the sensor to measure the game rotation vector.
    // Set the sample rate and report latency for this measurement.
    bhy.configure(BoschSensorID::GAME_ROTATION_VECTOR, sample_rate, report_latency_ms);
    // Register a callback function 'parse_quaternion' to handle the result events of the game rotation vector measurement.
    // When the sensor has new data for the game rotation vector, the 'parse_quaternion' function will be called.
    bhy.onResultEvent(BoschSensorID::GAME_ROTATION_VECTOR, parse_quaternion);
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
}


void loop()
{
#ifdef USING_SENSOR_IRQ_METHOD
    if (isReadyFlag) {
        isReadyFlag = false;
#endif /*USING_SENSOR_IRQ_METHOD*/

        /* If the interrupt is connected to the sensor and BHI360_IRQ is not equal to -1,
         * the interrupt function will be enabled, otherwise the method of polling the sensor is used
         */
        bhy.update();

#ifdef USING_SENSOR_IRQ_METHOD
    }
#endif /*USING_SENSOR_IRQ_METHOD*/

#ifdef USING_DATA_HELPER
    if (quaternion.hasUpdated()) {
        // Convert rotation vector to Euler angles
        quaternion.toEuler();
        // Print the roll angle to the serial monitor.
        Serial.print(quaternion.getRoll());
        // Print a comma as a separator between the roll and pitch angles.
        Serial.print(",");
        // Print the pitch angle to the serial monitor.
        Serial.print(quaternion.getPitch());
        // Print a comma as a separator between the pitch and yaw angles.
        Serial.print(",");
        // Print the yaw angle to the serial monitor and start a new line.
        Serial.println(quaternion.getHeading());
    }
#endif

    delay(50);
}




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
 * @file      BHI360_aux_BMM350_BME688_IAQ.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-11
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
// #define USING_DATA_HELPER

#ifdef USING_DATA_HELPER
SensorIAQ air_quality(bhy);
#endif


#define BOSCH_SHUTTLE3_BHI360_BMM350C_BME688_IAQ

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

#ifndef USING_DATA_HELPER
void parse_sensor_data(uint8_t sensor_id, const uint8_t *data_ptr, uint32_t len, uint64_t *timestamp, void *user_data)
{
    float humidity = 0;
    float temperature = 0;
    float pressure = 0;
    switch (sensor_id) {
    case BoschSensorID::HUMIDITY:
        bhy2_parse_humidity(data_ptr, &humidity);
        Serial.print("humidity:"); Serial.print(humidity); Serial.println("%");
        break;
    case BoschSensorID::TEMPERATURE:
        bhy2_parse_temperature_celsius(data_ptr, &temperature);
        Serial.print("temperature:"); Serial.print(temperature); Serial.println("*C");
        break;
    case BoschSensorID::BAROMETER:
        bhy2_parse_pressure(data_ptr, &pressure);
        Serial.print("pressure:"); Serial.print(pressure); Serial.println("Pa");
        break;
    case BoschSensorID::IAQ: {
        /*SCALE FACTOR from BHI3 data sheet, IAQ data format*/
        const float  SCALE_IAQ_VOC  = 100.0;
        const float  SCALE_IAQ_TEMP = 256.0;
        const float  SCALE_IAQ_HUMI = 500.0;
        bhi360_event_data_iaq_output_t data{};
        bhi360_event_data_parse_air_quality(data_ptr, &data);
        Serial.print("ACCU:"); Serial.print(data.iaq_accuracy); Serial.print(" ");
        Serial.print("IAQ:"); Serial.print(data.iaq); Serial.print(" ");
        Serial.print("SIAQ:"); Serial.print(data.siaq); Serial.print(" ");
        Serial.print("VOC:"); Serial.print(data.voc / SCALE_IAQ_VOC); Serial.print("ppm ");
        Serial.print("CO2:"); Serial.print(data.co2); Serial.print("ppm ");
        Serial.print("Temperature:"); Serial.print(data.comp_temperature / SCALE_IAQ_TEMP); Serial.print("*C ");
        Serial.print("Humidity:"); Serial.print(data.comp_humidity / SCALE_IAQ_HUMI); Serial.print("% ");
        Serial.print("GAS:"); Serial.print(data.raw_gas); Serial.println(" ohms ");
    }
    break;
    default:
        Serial.println("Unkown.");
        break;
    }
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

void printResult(uint8_t sensor_id, float sample_rate, bool rslt)
{
    const char  *sensorName = bhy.getSensorName(sensor_id);
    Serial.print("Configure ");
    Serial.print(sensorName);
    Serial.print(" sensor ");
    Serial.print(sample_rate, 2);
    Serial.print(" HZ ");
    if (rslt) {
        Serial.print("successfully");
    } else {
        Serial.print("failed");
    }
    Serial.println();
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
    bhy.setUpdateProcessCallback(progress_callback, NULL);

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

    /*
    * Enable BME688 function
    * Function depends on BME688.
    * If the hardware is not connected to BME688, the function cannot be used.
    * * */
#ifdef USING_DATA_HELPER
    air_quality.enable();
#else

    float sample_rate = 1.0;    // Invalid param,iaq sensor only be updated when there is a change.
    uint32_t report_latency_ms = 0; // Invalid param,iaq sensor only be updated when there is a change.
    bool rslt = false;
    rslt = bhy.configure(BoschSensorID::IAQ, sample_rate, report_latency_ms);
    printResult(BoschSensorID::IAQ, sample_rate, rslt);
    
    // Register BME688 data parse callback function
    bhy.onResultEvent(BoschSensorID::IAQ, parse_sensor_data);
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
    if (air_quality.hasUpdated()) {
        air_quality.getLastTime(s, ns);
        Serial.print("[T: ");
        Serial.print(s);
        Serial.print(".");
        Serial.print(ns);
        Serial.print("]:");
        Serial.print("TEMP:"); Serial.print(air_quality.getCelsius()); Serial.print("°C");
        Serial.print("ACCU:"); Serial.print(air_quality.getIAQAccuracy()); Serial.print(" ");
        Serial.print("IAQ:"); Serial.print(air_quality.getIAQ()); Serial.print(" ");
        Serial.print("SIAQ:"); Serial.print(air_quality.getSIAQ()); Serial.print(" ");
        Serial.print("VOC:"); Serial.print(air_quality.getVOC()); Serial.print("ppm ");
        Serial.print("CO2:"); Serial.print(air_quality.getCO2()); Serial.print("ppm ");
        Serial.print("HUMI:"); Serial.println(air_quality.getHumidity());
        Serial.print("GAS:"); Serial.print(air_quality.getRawGas()); Serial.print(" ohms ");
        Serial.println();
    }
#endif
    delay(50);
}




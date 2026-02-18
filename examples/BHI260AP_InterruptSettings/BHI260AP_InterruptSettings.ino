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
 * @file      BHI260AP_InterruptSettings.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-04
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
SensorAcceleration accel(bhy);
SensorGyroscope gyro(bhy);

// The firmware runs in RAM and will be lost if the power is off. The firmware will be loaded from RAM each time it is run.
#define BOSCH_APP30_SHUTTLE_BHI260_FW
// #define BOSCH_APP30_SHUTTLE_BHI260_AUX_BMM150FW
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


    // Interrupt configuration
    InterruptConfig config;
    config.wakeUpFIFOEnabled = false;
    config.nonWakeUpFIFOEnabled = true; // It must be enabled, otherwise FIFO cannot be updated.
    config.faultEnabled = false;
    config.debuggingEnabled = false;
    config.polarity = InterruptConfig::Polarity::ACTIVE_HIGH;
    config.triggerMode = InterruptConfig::TriggerMode::EDGE;
    bhy.configureInterrupt(config);

    uint8_t regValue = bhy.getInterruptRegisterValue();
    String desc;
    desc += "Register Value: 0x" + String(regValue, HEX) + "\n";
    desc += "Bit 0 - Wake-up FIFO:   " + String((regValue & (1 << 0)) ? "MASKED" : "ACTIVE") + "\n";
    desc += "Bit 1 - Non-Wake FIFO:  " + String((regValue & (1 << 1)) ? "MASKED" : "ACTIVE") + "\n";
    desc += "Bit 2 - Status FIFO:    " + String((regValue & (1 << 2)) ? "MASKED" : "ACTIVE") + "\n";
    desc += "Bit 3 - Debug:          " + String((regValue & (1 << 3)) ? "MASKED" : "ACTIVE") + "\n";
    desc += "Bit 4 - Fault:          " + String((regValue & (1 << 4)) ? "MASKED" : "ACTIVE") + "\n";
    desc += "Bit 5 - Polarity:       " + String((regValue & (1 << 5)) ? "ACTIVE_LOW" : "ACTIVE_HIGH") + "\n";
    desc += "Bit 6 - Trigger:        " + String((regValue & (1 << 6)) ? "EDGE" : "LEVEL") + "\n";
    desc += "Bit 7 - Pin Mode:       " + String((regValue & (1 << 7)) ? "OPEN_DRAIN" : "PUSH_PULL");

    /*
    Register Value: 0x59
    Bit 0 - Wake-up FIFO:   MASKED
    Bit 1 - Non-Wake FIFO:  ACTIVE
    Bit 2 - Status FIFO:    ACTIVE
    Bit 3 - Debug:          MASKED
    Bit 4 - Fault:          MASKED
    Bit 5 - Polarity:       ACTIVE_HIGH
    Bit 6 - Trigger:        EDGE
    Bit 7 - Pin Mode:       PUSH_PULL
    */
    Serial.println(desc);


    float sample_rate = 1.0;      /* Read out data measured at 1Hz */
    uint32_t report_latency_ms = 0; /* Report immediately */

    // Enable acceleration
    accel.enable(sample_rate, report_latency_ms);
    // Enable gyroscope
    gyro.enable(sample_rate, report_latency_ms);

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
    if (isInterruptTriggered) {
        isInterruptTriggered = false;
#endif /*USING_SENSOR_IRQ_METHOD*/

        /* If the interrupt is connected to the sensor and BHI260_IRQ is not equal to -1,
         * the interrupt function will be enabled, otherwise the method of polling the sensor is used
         */
        bhy.update();

#ifdef USING_SENSOR_IRQ_METHOD
    }
#endif /*USING_SENSOR_IRQ_METHOD*/


    uint32_t s;
    uint32_t ns;
    AccelerometerData accel_data;
    if (accel.readData(accel_data)) {
        accel.getLastTime(s, ns);
#ifdef PLATFORM_HAS_PRINTF
        Serial.printf("[T: %" PRIu32 ".%09" PRIu32 "] [AX:%+7.2f AY:%+7.2f AZ:%+7.2f] \n",
                      s, ns, accel_data.mps2.x, accel_data.mps2.y,  accel_data.mps2.z);
#else
        Serial.print("[T: ");
        Serial.print(s);
        Serial.print(".");
        Serial.print(ns);
        Serial.print("] [AX:");
        Serial.print(accel_data.mps2.x, 2);
        Serial.print(" AY:");
        Serial.print(accel_data.mps2.y, 2);
        Serial.print(" AZ:");
        Serial.print(accel_data.mps2.z, 2);
        Serial.println("]");
#endif
    }
    GyroscopeData gyro_data;
    if (gyro.readData(gyro_data)) {
        gyro.getLastTime(s, ns);
#ifdef PLATFORM_HAS_PRINTF
        Serial.printf("[T: %" PRIu32 ".%09" PRIu32 "] [GX:%+7.2f GY:%+7.2f GZ:%+7.2f] \n",
                      s, ns, gyro_data.dps.x, gyro_data.dps.y, gyro_data.dps.z);
#else
        Serial.print("[T: ");
        Serial.print(s);
        Serial.print(".");
        Serial.print(ns);
        Serial.print("] [GX:");
        Serial.print(gyro_data.dps.x, 2);
        Serial.print(" GY:");
        Serial.print(gyro_data.dps.y, 2);
        Serial.print(" GZ:");
        Serial.print(gyro_data.dps.z, 2);
        Serial.println("]");
#endif
    }
    delay(50);
}




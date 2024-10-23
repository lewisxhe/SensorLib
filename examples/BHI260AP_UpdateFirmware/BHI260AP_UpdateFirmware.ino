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
 * @file      BHI260AP_UpdateFirmware.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-10-23
 * @note      Demonstrates loading firmware from a file into BHI260, only testing the NRF52840 platform
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#if defined(ARDUINO_ARCH_NRF52)

#include <SdFat.h>              //Deplib https://github.com/adafruit/SdFat.git
#include <SensorBHI260AP.hpp>

#ifdef BHY2_USE_I2C
#define BHI260AP_SDA          21
#define BHI260AP_SCL          22
#define BHI260AP_IRQ          39
#define BHI260AP_RST          -1
#else
#define BHI260AP_MOSI         27
#define BHI260AP_MISO         46
#define BHI260AP_SCK          3
#define BHI260AP_CS           28
#define BHI260AP_IRQ          30
#define BHI260AP_RST          -1
#endif

SensorBHI260AP bhy;


#define CS_PIN                5

/***************************************
 *  SD CARD
 ***************************************/
SdFat32 sd;

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(8)
//------------------------------------------------------------------------------
// Store error strings in flash to save RAM.
#define error(s) sd.errorHalt(&Serial, F(s))
#define SD_CONFIG SdSpiConfig(CS_PIN, DEDICATED_SPI, SPI_CLOCK)


void parse_bme280_sensor_data(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len, uint64_t *timestamp)
{
    float humidity = 0;
    float temperature = 0;
    float pressure = 0;
    switch (sensor_id) {
    case SENSOR_ID_HUM:
    case SENSOR_ID_HUM_WU:
        bhy2_parse_humidity(data_ptr, &humidity);
        Serial.print("humidity:"); Serial.print(humidity); Serial.println("%");
        break;
    case SENSOR_ID_TEMP:
        bhy2_parse_temperature_celsius(data_ptr, &temperature);
        Serial.print("temperature:"); Serial.print(temperature); Serial.println("*C");
        break;
    case SENSOR_ID_BARO:
        bhy2_parse_pressure(data_ptr, &pressure);
        Serial.print("pressure:"); Serial.print(pressure); Serial.println("hPa");
        break;
    default:
        Serial.println("Unkown.");
        break;
    }
}


void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // In this example, BHI260 and SD Card are on the same SPI bus
    SPI.setPins(BHI260AP_MISO, BHI260AP_SCK, BHI260AP_MOSI);

    SPI.begin();

    /***************************************
      *  SD CARD
      ***************************************/
    //  If multiple SPI peripherals are mounted on a single bus, first set the CS of other peripherals to HIGH
    const uint8_t other_spi_dev_cs_pin[] = {5, 28, 40};
    for (size_t i = 0; i < sizeof(other_spi_dev_cs_pin); ++i) {
        pinMode(other_spi_dev_cs_pin[i], OUTPUT);
        digitalWrite(other_spi_dev_cs_pin[i], HIGH);
    }

    // Initialize the SD card.
    Serial.println("Initializing SD Card ...");
    if (!sd.begin(SD_CONFIG)) {
        sd.initErrorHalt(&Serial);
        while (1);
    } else {
        Serial.println(" success");
    }

    File firmware_file = sd.open("/BHI260AP_aux_BMM150_BME280_GPIO-flash.fw", FILE_READ);
    if (!firmware_file) {
        Serial.println("Open firmware file failed!");
        while (1);
    }

    size_t fw_size = firmware_file.size();
    Serial.printf("Read firmware file successfully .");

    uint8_t *firmware = (uint8_t *)malloc(fw_size);
    if (!firmware) {
        Serial.println("malloc memory failed!");
        while (1);
    }
    
    firmware_file.readBytes(firmware, fw_size);

    firmware_file.close();


    /***************************************
      *  BHI260 Initializing
      ***************************************/
    Serial.println("Initializing Sensors...");
    // Set the reset pin and interrupt pin, if any
    bhy.setPins(BHI260AP_RST, BHI260AP_IRQ);
    // Force update of the current firmware, regardless of whether it exists.
    // After uploading the firmware once, you can change it to false to speed up the startup time.
    bool force_update = true;
    // true : Write firmware to flash , false : Write to ram
    bool write_to_flash = true;
    // Set the firmware array address and firmware size
    bhy.setFirmware(firmware, fw_size, write_to_flash, force_update);
    // Set to load firmware from flash or ram
    bhy.setBootFormFlash(write_to_flash);

#ifdef BHY2_USE_I2C
    // Using I2C interface
    // BHI260AP_SLAVE_ADDRESS_L = 0x28
    // BHI260AP_SLAVE_ADDRESS_H = 0x29
    if (!bhy.init(Wire, BHI260AP_SDA, BHI260AP_SCL, BHI260AP_SLAVE_ADDRESS_L)) {
        Serial.print("Failed to initialize sensor - error code:");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#else
    // Using SPI interface
    if (!bhy.init(SPI, BHI260AP_CS, BHI260AP_MOSI, BHI260AP_MISO, BHI260AP_SCK)) {
        Serial.print("Failed to initialize sensor - error code:");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#endif

    // Release the requested memory space
    free(firmware);

    Serial.println("Initializing the sensor successfully!");

    // Output all current sensor information
    bhy.printInfo(Serial);

    // Output interrupt configuration information to Serial
    bhy.printInterruptCtrl(Serial);

    /*
    * Enable monitoring.
    * sample_rate ​​can only control the rate of the pressure sensor.
    * Temperature and humidity will only be updated when there is a change.
    * * */
    float sample_rate = 1.0;      /* Set to 1Hz update frequency */
    uint32_t report_latency_ms = 0; /* Report immediately */
    bool rlst = false;

    rlst = bhy.configure(SENSOR_ID_TEMP, sample_rate, report_latency_ms);
    Serial.printf("Configure temperature sensor %.2f HZ %s\n", sample_rate, rlst ? "successfully" : "failed");
    rlst = bhy.configure(SENSOR_ID_BARO, sample_rate, report_latency_ms);
    Serial.printf("Configure pressure sensor %.2f HZ %s\n", sample_rate,  rlst ? "successfully" : "failed");
    rlst = bhy.configure(SENSOR_ID_HUM, sample_rate, report_latency_ms);
    Serial.printf("Configure humidity sensor %.2f HZ %s\n", sample_rate,  rlst ? "successfully" : "failed");

    // Register BME280 data parse callback function
    Serial.println("Register sensor result callback function");
    bhy.onResultEvent(SENSOR_ID_TEMP, parse_bme280_sensor_data);
    bhy.onResultEvent(SENSOR_ID_HUM, parse_bme280_sensor_data);
    bhy.onResultEvent(SENSOR_ID_BARO, parse_bme280_sensor_data);
}

void loop()
{
    // Update sensor fifo
    bhy.update();
}
#else
void setup() {}
void loop() {}
#endif

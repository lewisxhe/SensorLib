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
 * @file      BHI260AP_6DoF.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-06
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorBHI260AP.hpp"

#ifdef BHY2_USE_I2C
#define BHI260AP_SDA          21
#define BHI260AP_SCL          22
#define BHI260AP_IRQ          39
#define BHI260AP_RST          -1
#else
#define BHI260AP_MOSI         33
#define BHI260AP_MISO         34
#define BHI260AP_SCK          35
#define BHI260AP_CS           36
#define BHI260AP_IRQ          37
#define BHI260AP_RST          -1
#endif


SensorBHI260AP bhy;

void setup()
{
    Serial.begin(115200);
    while (!Serial);


    //TODO:
    bhy.onEvent( BHY2_EVENT_INITIALIZED, [](uint8_t event, uint8_t *data, uint32_t size) {
        Serial.println("===========BHY2_EVENT_INITIALIZED==========");
    });

    bhy.onEvent( BHY2_EVENT_SAMPLE_RATE_CHANGED, [](uint8_t event, uint8_t *data, uint32_t size) {
        Serial.println("===========BHY2_EVENT_SAMPLE_RATE_CHANGED==========");
    });

    bhy.onEvent( BHY2_EVENT_POWER_MODE_CHANGED, [](uint8_t event, uint8_t *data, uint32_t size) {
        Serial.println("===========BHY2_EVENT_POWER_MODE_CHANGED==========");
    });



    bhy.setPins(BHI260AP_RST, BHI260AP_IRQ);

#ifdef BHY2_USE_I2C
    if (!bhy.init(Wire, BHI260AP_SLAVE_ADDRESS, BHI260AP_SDA, BHI260AP_SCL)) {
        Serial.println("Failed to find BHI260AP - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
#else
    if (!bhy.init(SPI, BHI260AP_CS, BHI260AP_MOSI, BHI260AP_MISO, BHI260AP_SCK)) {
        Serial.print("Failed to find BHI260AP - ");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#endif

    Serial.println("Init BHI260AP Sensor success!");


    bhy.printSensors(Serial);

    float sample_rate = 100.0;      /* Read out hintr_ctrl measured at 100Hz */
    uint32_t report_latency_ms = 0; /* Report immediately */

    bhy.configure(SENSOR_ID_ACC_PASS, sample_rate, report_latency_ms);
    bhy.configure(SENSOR_ID_GYRO_PASS, sample_rate, report_latency_ms);


    bhy.onResultEvent(SENSOR_ID_ACC_PASS, [](float scaling_factor, uint8_t *data_ptr, uint32_t len) {
        struct bhy2_data_xyz data;
        bhy2_parse_xyz(data_ptr, &data);
        Serial.printf("ACCEL-> x: %f, y: %f, z: %f;\r\n",
                      data.x * scaling_factor,
                      data.y * scaling_factor,
                      data.z * scaling_factor
                     );

    });

    bhy.onResultEvent(SENSOR_ID_GYRO_PASS, [](float scaling_factor, uint8_t *data_ptr, uint32_t len) {
        struct bhy2_data_xyz data;
        bhy2_parse_xyz(data_ptr, &data);
        Serial.printf("GYRO-> x: %f, y: %f, z: %f;\r\n",
                      data.x * scaling_factor,
                      data.y * scaling_factor,
                      data.z * scaling_factor
                     );
    });

}


void loop()
{
    bhy.update();
    delay(50);
}




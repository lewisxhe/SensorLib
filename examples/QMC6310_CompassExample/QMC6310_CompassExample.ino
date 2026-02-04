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
 * @file      QMC6310_GetDataExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-26
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include "SensorQMC6310.hpp"
#include "SH1106Wire.h"         //Oled display from https://github.com/ThingPulse/esp8266-oled-ssd1306
#ifdef ARDUINO_T_BEAM_S3_SUPREME
#include <XPowersAXP2101.tpp>   //PMU Library https://github.com/lewisxhe/XPowersLib.git
#endif

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#ifndef OLED_SDA
#define OLED_SDA    22      // Display Wire SDA Pin
#endif

#ifndef OLED_SCL
#define OLED_SCL    21      // Display Wire SCL Pin
#endif

SH1106Wire display(0x3c, OLED_SDA, OLED_SCL);
SensorQMC6310 magnetometer;

//Compass application from  https://github.com/G6EJD/ESP8266_micro_compass_HMC5883_OLED
void arrow(int x2, int y2, int x1, int y1, int alength, int awidth, OLEDDISPLAY_COLOR  color)
{
    display.setColor(color);
    float distance;
    int dx, dy, x2o, y2o, x3, y3, x4, y4, k;
    distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    dx = x2 + (x1 - x2) * alength / distance;
    dy = y2 + (y1 - y2) * alength / distance;
    k = awidth / alength;
    x2o = x2 - dx;
    y2o = dy - y2;
    x3 = y2o * k + dx;
    y3 = x2o * k + dy;
    x4 = dx - y2o * k;
    y4 = dy - x2o * k;
    display.drawLine(x1, y1, x2, y2);
    display.drawLine(x1, y1, dx, dy);
    display.drawLine(x3, y3, x4, y4);
    display.drawLine(x3, y3, x2, y2);
    display.drawLine(x2, y2, x4, y4);
}

void beginPower()
{
#if defined(ARDUINO_T_BEAM_S3_SUPREME)
    XPowersAXP2101 power;
    power.begin(Wire1, AXP2101_SLAVE_ADDRESS, 42, 41);
    power.disableALDO1();
    power.disableALDO2();
    delay(250);
    power.setALDO1Voltage(3300);
    power.enableALDO1();
    power.setALDO2Voltage(3300);
    power.enableALDO2();
#endif
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // LilyGo T-Beam-Supreme sensor requires a power source to function.
    beginPower();

    /**
     * Supports QMC6310U and QMC6310N; simply pass the corresponding device address
     * during initialization.
     * - QMC6310U_SLAVE_ADDRESS
     * - QMC6310N_SLAVE_ADDRESS
     */
    uint8_t address = QMC6310U_SLAVE_ADDRESS;
    //  uint8_t address = QMC6310N_SLAVE_ADDRESS;

    if (!magnetometer.begin(Wire, address, SENSOR_SDA, SENSOR_SCL)) {
        while (1) {
            Serial.println("Failed to find QMC6310 - check your wiring!");
            delay(1000);
        }
    }

    // The desired output data rate in Hz.  Allowed values are 10.0, 50.0, 100.0 and 200.0HZ.
    float data_rate_hz = 200.0f;
    // op_mode: Allowed values are SUSPEND, NORMAL, SINGLE_MEASUREMENT, CONTINUOUS_MEASUREMENT
    OperationMode op_mode = OperationMode::CONTINUOUS_MEASUREMENT;
    // full_scale: Allowed values are FS_2G, FS_8G, FS_12G ,FS_30G
    MagFullScaleRange full_scale = MagFullScaleRange::FS_8G;
    // over_sample_ratio: Allowed values are OSR_1, OSR_2, OSR_4, OSR_8
    MagOverSampleRatio over_sample_ratio = MagOverSampleRatio::OSR_1;
    // down_sample_ratio: Allowed values are DSR_1, DSR_2, DSR_4, DSR_8
    MagDownSampleRatio down_sample_ratio = MagDownSampleRatio::DSR_1;

    /* Config Magnetometer */
    if (magnetometer.configMagnetometer(
                op_mode,
                full_scale,
                data_rate_hz,
                over_sample_ratio,
                down_sample_ratio)) {
        Serial.println("Magnetometer configured successfully.");
    } else {
        Serial.println("Magnetometer configuration failed.");
        while (1);
    }

    SensorInfo info = magnetometer.getSensorInfo();
    Serial.print("Manufacturer: "); Serial.println(info.manufacturer);
    Serial.print("Model: "); Serial.println(info.model);
    Serial.print("I2C Address: 0x"); Serial.println(info.i2c_address, HEX);
    Serial.print("Version: "); Serial.println(info.version);
    Serial.print("UID: 0x"); Serial.println(info.uid);
    Serial.print("Type: "); Serial.println(SensorUtils::typeToString(info.type));

    SensorConfig cfg = magnetometer.getConfig();
    Serial.print("DataRate: "); Serial.println(cfg.sample_rate);
    Serial.print("FullScaleRange: "); Serial.println(cfg.range);
    Serial.print("Mode: "); Serial.println((uint8_t)cfg.mode);
    Serial.println();


    display.init();

    //Find the magnetic declination : https://www.magnetic-declination.com/
    float declination_deg = magnetometer.dmsToDecimalDegrees(-3, 20);   // -3.3333

    magnetometer.setDeclination(declination_deg);

    Serial.print(" Magnetic Declination: ");
    Serial.print(declination_deg, 2);
    Serial.println("°");

    Serial.print(" Sensitivity: ");
    Serial.print(magnetometer.getSensitivity(), 6);
    Serial.println(" Gauss/LSB");

}

void loop()
{
    MagnetometerData data;

    static int last_angle = -1;

    if (magnetometer.readData(data)) {
        int angle = static_cast<int>(data.heading_degrees + 0.5f);

        if (angle != last_angle) {
            display.clear();

            display.setFont(ArialMT_Plain_10);
            display.setTextAlignment(TEXT_ALIGN_CENTER);
            display.drawString(32, 0, "N");    // North
            display.drawString(0, 28, "W");    // West
            display.drawString(64, 28, "E");   // East
            display.drawString(32, 53, "S");   // South

            display.drawCircle(32, 32, 20);

            // Calculate arrow direction
            // Note: data.heading_degrees is already 0-360°, 0° = North, 90° = East
            // Display coordinate system: 0° points to the top of the screen (North), and the angle increases clockwise.
            float arrow_angle_rad = data.heading_degrees * M_PI / 180.0f;
            // Note: sin corresponds to the x-axis
            int arrow_x = 32 + static_cast<int>(25 * cosf(arrow_angle_rad));
            // Note: cos corresponds to the y-axis, the negative sign is because the screen's y-axis points downwards.
            int arrow_y = 32 - static_cast<int>(25 * sinf(arrow_angle_rad));

            display.drawLine(32, 32, arrow_x, arrow_y);
            display.fillCircle(arrow_x, arrow_y, 2);

            display.setTextAlignment(TEXT_ALIGN_LEFT);
            display.drawString(75, 5, "Angle:" +  String(angle) + "°");
            display.drawString(75, 25, "Decl:" + String(magnetometer.getDeclinationDeg(), 1) + "°");
            float strength = magnetometer.calculateMagneticStrength(data);
            display.drawString(75, 45, "Str:" + String(strength, 1) + "uT");

            display.display();

            Serial.print("Heading: ");
            Serial.print(angle);
            Serial.print("°, Raw: ");
            Serial.print(data.heading_degrees, 2);
            Serial.print("°, Declination: ");
            Serial.print(magnetometer.getDeclinationDeg(), 2);
            Serial.println("°");

            last_angle = angle;
        }
    }

    delay(100);
}
#else
void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println("The graphics library may not be supported on the esp32 platform"); delay(1000);
}
#endif

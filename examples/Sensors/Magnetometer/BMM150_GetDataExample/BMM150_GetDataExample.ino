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
 * @file      BMM150_GetDataExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-22
 *
 */
#include <MagnetometerDrv.hpp>

#ifndef SENSOR_SDA
#define SENSOR_SDA          33
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL          35
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ          -1
#endif

#ifndef SENSOR_RST
#define SENSOR_RST          -1
#endif


SensorBMM150 magnetometer;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // Using I2C interface
    if (!magnetometer.begin(Wire, BMM150_I2C_ADDRESS_CSB_HIGH_SDO_LOW, SENSOR_SDA, SENSOR_SCL)) {
        Serial.print("Failed to init BMM150 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    Serial.println("Init BMM150 Sensor success!");

    // The desired output data rate in Hz.  Allowed values are 2.0, 6.0, 8.0, 15.0, 20.0, 25.0 and 30.0HZ.
    float data_rate_hz = 200.0f;
    // op_mode: Allowed values are SUSPEND, NORMAL, SINGLE_MEASUREMENT, CONTINUOUS_MEASUREMENT
    OperationMode op_mode = OperationMode::CONTINUOUS_MEASUREMENT;
    // full_scale: Allowed values are FS_16G
    MagFullScaleRange full_scale = MagFullScaleRange::FS_16G;
    // over_sample_ratio: Allowed values are OSR_1, OSR_2, OSR_4, OSR_8
    MagOverSampleRatio over_sample_ratio = MagOverSampleRatio::OSR_1;

    /* Config Magnetometer */
    if (magnetometer.configMagnetometer(
                op_mode,
                full_scale,
                data_rate_hz,
                over_sample_ratio)) {
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

    //Find the magnetic declination : https://www.magnetic-declination.com/
    float declination_deg = MagnetometerUtils::dmsToDecimalDegrees(-3, 20);   // -3.3333

    magnetometer.setDeclination(declination_deg);

    Serial.print(" Magnetic Declination: ");
    Serial.print(declination_deg, 2);
    Serial.println("°");

    Serial.print(" Sensitivity: ");
    Serial.print(magnetometer.getSensitivity(), 6);
    Serial.println(" Gauss/LSB");

    delay(3000);

    Serial.println("Read data now...");
}


void loop()
{
    MagnetometerData data;

    if (magnetometer.readData(data)) {

        // Gauss to μT
        float x = MagnetometerUtils::gaussToMicroTesla(data.magnetic_field.x);
        float y = MagnetometerUtils::gaussToMicroTesla(data.magnetic_field.y);
        float z = MagnetometerUtils::gaussToMicroTesla(data.magnetic_field.z);

        Serial.print("Mag:");
        Serial.print(" X:"); Serial.print(x);
        Serial.print(" Y:"); Serial.print(y);
        Serial.print(" Z:"); Serial.print(z);
        Serial.print(" μT");

        Serial.print(" Sensitivity: ");
        Serial.print(magnetometer.getSensitivity(), 6);
        Serial.print(" Gauss/LSB");

        Serial.print(" Metadata:");
        Serial.print(" X:");
        Serial.print(data.raw.x);
        Serial.print(" Y:");
        Serial.print(data.raw.y);
        Serial.print(" Z:");
        Serial.print(data.raw.z);

        Serial.print(" Heading (rad): ");
        Serial.print(data.heading, 6);
        Serial.print(" rad");

        Serial.print(" Heading (deg): ");
        Serial.print(data.heading_degrees, 2);
        Serial.print("°");

        float strength = MagnetometerUtils::calculateMagneticStrength(data);
        strength = MagnetometerUtils::gaussToMicroTesla(strength);
        Serial.print(" Magnetic Strength: ");
        Serial.print(strength, 2);
        Serial.println(" μT");

        if (data.overflow) {
            Serial.println("\tWarning: Data Overflow occurred!");
        }
        if (data.skip_data) {
            Serial.println("\tWarning: Data Skip occurred!");
        }
    }
    delay(50);
}

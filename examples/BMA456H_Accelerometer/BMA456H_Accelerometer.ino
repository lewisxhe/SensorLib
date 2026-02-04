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
 * @file      BMA456H_Accelerometer.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-30
 * @note      Example of initializing the Bosch BMA456H characteristic profile using the BMA456 triaxial accelerometer.
 */
#include <SensorBMA456H.hpp>

#ifndef SENSOR_SDA
#define SENSOR_SDA  21
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  22
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  39
#endif

// Configuring using BMA456H features
SensorBMA456H accelSensor;

volatile bool isInterruptTriggered = false;

void setup()
{
    bool rslt;

    Serial.begin(115200);
    while (!Serial);

    pinMode(SENSOR_IRQ, INPUT);
    attachInterrupt(SENSOR_IRQ, []() {
        isInterruptTriggered = true;
    }, RISING);

    // Sensor initialization
    rslt = accelSensor.begin(Wire, BMA4XX_I2C_ADDR_SDO_HIGH, SENSOR_SDA, SENSOR_SCL);
    if (!rslt) {
        while (1) {
            Serial.println("BMA4xx sensor initialization failed");
            delay(1000);
        }
    }

    Serial.print(accelSensor.getModelName());
    Serial.println(" Sensor initialized successfully");

    // Set sensor orientation, based on the hardware placement setting
    rslt = accelSensor.setRemapAxes(SensorRemap::TOP_LAYER_RIGHT_CORNER);
    if (!rslt) {
        Serial.println("Failed to set remap axes");
    }

    // Accelerometer configuration
    // The desired operation mode. Allowed values are SUSPEND, NORMAL.
    AccelOperationMode operationMode = AccelOperationMode::NORMAL;
    // The desired full-scale range. Allowed values are FS_2G, FS_4G, FS_8G, FS_16G.
    AccelFullScaleRange fullScaleRange = AccelFullScaleRange::FS_2G;
    // The desired bandwidth. Allowed values are OSR4_AVG1, OSR2_AVG2, NORMAL_AVG4, etc.
    AccelBandwidth bandwidth = AccelBandwidth::OSR2_AVG2;
    // The desired data rate in Hz. Allowed values are 0.78, 1.56, 3.12, 6.25, 12.5, 25, 50, 100, 200, 400, 800, 1600.
    float data_rate_hz = 100.0f;
    // The desired performance mode. Allowed values are CIC_AVG_MODE, CONTINUOUS_MODE
    AccelPerfMode perfMode = AccelPerfMode::CIC_AVG_MODE;

    if (!accelSensor.configAccelerometer(operationMode, fullScaleRange, data_rate_hz, bandwidth, perfMode)) {
        Serial.println("Failed to configure accelerometer");
        while (1);
    }

    // Enable data ready feature
    rslt = accelSensor.enableDataReady(true);
    if (!rslt) {
        Serial.println("Failed to enable data ready");
        while (1);
    }

    delay(3000);

    Serial.println("Now read accelerometer data form sensor");
}

void loop()
{
    AccelerometerData  accelData;
    if (accelSensor.isDataReady()) {
        // Read the accelerometer data
        if (accelSensor.readData(accelData)) {
            Serial.print("Temperature: ");
            Serial.print(accelData.temperature);
            Serial.print(" °C, Acc_Raw: (");
            Serial.print(accelData.raw.x);
            Serial.print(", ");
            Serial.print(accelData.raw.y);
            Serial.print(", ");
            Serial.print(accelData.raw.z);
            Serial.print("), Acc_ms2: (");
            Serial.print(accelData.mps2.x, 2);
            Serial.print(", ");
            Serial.print(accelData.mps2.y, 2);
            Serial.print(", ");
            Serial.print(accelData.mps2.z, 2);
            Serial.print(") ");
            uint32_t timeSampleMs = accelSensor.getTimeSampleMs();
            Serial.print(timeSampleMs);
            Serial.print("(ms)/");
            Serial.print(millis());
            Serial.println(" ");
        } else {
            Serial.println("Failed to read accelerometer data");
        }
    }
}

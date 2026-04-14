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
 * @file      BMA456H_StepActivity.ino
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

void onActivity(ActivityType activity)
{
    Serial.print("Activity detected: ");
    switch (activity) {
    case ActivityType::STATIONARY:
        Serial.println("Stationary");
        break;
    case ActivityType::WALKING:
        Serial.println("Walking");
        break;
    case ActivityType::RUNNING:
        Serial.println("Running");
        break;
    default:
        Serial.println("Unknown");
        break;
    }
}

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
    OperationMode operationMode = OperationMode::NORMAL;
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

    bool feature_enable = true; // Enable feature
    bool interrupt_enable = true;   //True: hardware interrupt enabled
    InterruptPinMap pin_map = InterruptPinMap::PIN1;

    rslt = accelSensor.setInterruptPinConfig(
               pin_map, // Which pin should the hardware interrupt be routed to?
               false,   // level trigger
               false,   // active high
               true,    // output enable
               false);  // input disable
    if (!rslt) {
        Serial.println("Failed to set interrupt pin configuration");
        while (1);
    }

    // Enable activity recognition feature
    rslt = accelSensor.enableActivityRecognition(feature_enable, interrupt_enable, pin_map);
    if (!rslt) {
        Serial.println("Failed to enable activity recognition");
        while (1);
    }

    // Set activity detection callback
    accelSensor.setOnActivityCallback(onActivity);

    delay(3000);

    Serial.println("Now you can move the sensor.");
}

void loop()
{
    if (isInterruptTriggered) {
        isInterruptTriggered = false;
        accelSensor.update();
    }
    delay(30);
}

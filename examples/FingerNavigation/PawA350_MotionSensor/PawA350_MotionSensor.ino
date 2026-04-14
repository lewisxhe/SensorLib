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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @file      PawA350_MotionSensor.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-14
 *
 * @brief     PAW-A350 Optical Finger Navigation Sensor Example
 *
 * @section   Description
 *
 * This example demonstrates the full functionality of the PAW-A350 sensor including:
 * - Sensor initialization and soft reset
 * - CPI resolution configuration (set/get all values)
 * - LED control (current and always-on mode set/get)
 * - Power management (downshift times and frame periods set/get)
 * - Motion detection and data reading
 * - Interrupt threshold configuration
 * - Finger presence detection
 *
 */

#include <FingerNavigationDrv.hpp>

#ifndef SENSOR_SDA
#define SENSOR_SDA  1
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  2
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  43
#endif

#ifndef SENSOR_RST
#define SENSOR_RST  44
#endif

#ifndef SENSOR_SHUTDOWN
#define SENSOR_SHUTDOWN 48
#endif

#ifndef SENSOR_FPD_INT
#define SENSOR_FPD_INT 18
#endif

SensorPawA350 sensor;

volatile bool motionIrqTriggered = false;
volatile bool fingerStateChanged = false;

void setup()
{
    bool rslt;

    Serial.begin(115200);
    while (!Serial);


#if SENSOR_SHUTDOWN != -1
    // Sensor Power On Pin,HIGH is active
    pinMode(SENSOR_SHUTDOWN, OUTPUT);
    digitalWrite(SENSOR_SHUTDOWN, HIGH);
#endif
#if SENSOR_RST != -1
    // Sensor Reset Pin, LOW is reset sensor
    pinMode(SENSOR_RST, OUTPUT);
    digitalWrite(SENSOR_RST, HIGH);
#endif
#if SENSOR_FPD_INT != -1
    // Sensor Finger Presence Detection Pin, HIGH is active
    pinMode(SENSOR_FPD_INT, INPUT);
#endif
    // Sensor Motion Interrupt Pin, LOW is active
    pinMode(SENSOR_IRQ, INPUT);
    attachInterrupt(SENSOR_IRQ, []() {
        motionIrqTriggered = true;
    }, FALLING);

    rslt = sensor.begin(Wire, PAW_A350_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL);
    if (!rslt) {
        Serial.println("PAW-A350 sensor initialization failed!");
        while (1) {
            Serial.println("Ensure sensor is connected and I2C wiring is correct");
            delay(2000);
        }
    }

    Serial.print(sensor.getChipName());
    Serial.println(" initialized successfully");

    rslt = sensor.softReset();
    if (!rslt) {
        Serial.println("Software reset failed!");
        while (1);
    }
    Serial.println("Software reset successful");

    Serial.println("\n========== ID Test ==========");
    int pid = sensor.getProductID();
    int rev = sensor.getRevisionID();
    Serial.print("Product ID: 0x");
    Serial.print(pid, HEX);
    Serial.print(" (expected 0x88)");
    Serial.println(pid == 0x88 ? " [OK]" : " [FAIL]");
    Serial.print("Revision ID: 0x");
    Serial.println(rev, HEX);

    Serial.println("\n========== CPI Test ==========");
    rslt = sensor.setCpi(SensorPawA350::CpiResolution::CPI_125);
    Serial.print("Set CPI 125: ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get CPI: ");
    Serial.print((int)sensor.getCpi());
    Serial.println(sensor.getCpi() == SensorPawA350::CpiResolution::CPI_125 ? " [OK]" : " [FAIL]");

    rslt = sensor.setCpi(SensorPawA350::CpiResolution::CPI_250);
    Serial.print("Set CPI 250: ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get CPI: ");
    Serial.print((int)sensor.getCpi());
    Serial.println(sensor.getCpi() == SensorPawA350::CpiResolution::CPI_250 ? " [OK]" : " [FAIL]");

    rslt = sensor.setCpi(SensorPawA350::CpiResolution::CPI_500);
    Serial.print("Set CPI 500: ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get CPI: ");
    Serial.print((int)sensor.getCpi());
    Serial.println(sensor.getCpi() == SensorPawA350::CpiResolution::CPI_500 ? " [OK]" : " [FAIL]");

    rslt = sensor.setCpi(SensorPawA350::CpiResolution::CPI_750);
    Serial.print("Set CPI 750: ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get CPI: ");
    Serial.print((int)sensor.getCpi());
    Serial.println(sensor.getCpi() == SensorPawA350::CpiResolution::CPI_750 ? " [OK]" : " [FAIL]");

    rslt = sensor.setCpi(SensorPawA350::CpiResolution::CPI_1000);
    Serial.print("Set CPI 1000: ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get CPI: ");
    Serial.print((int)sensor.getCpi());
    Serial.println(sensor.getCpi() == SensorPawA350::CpiResolution::CPI_1000 ? " [OK]" : " [FAIL]");

    rslt = sensor.setCpi(SensorPawA350::CpiResolution::CPI_1250);
    Serial.print("Set CPI 1250: ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get CPI: ");
    Serial.print((int)sensor.getCpi());
    Serial.println(sensor.getCpi() == SensorPawA350::CpiResolution::CPI_1250 ? " [OK]" : " [FAIL]");

    Serial.println("\n========== Power Management Test ==========");
    Serial.println("-- Run Downshift --");
    rslt = sensor.setRunDownshiftTime(8);
    Serial.print("Set RunDownshift(8): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get RunDownshift: ");
    Serial.print(sensor.getRunDownshiftTime());
    Serial.println(sensor.getRunDownshiftTime() == 8 ? " [OK]" : " [FAIL]");
    Serial.print("Get RunDownshiftMs: ");
    Serial.print(sensor.getRunDownshiftTimeMs());
    Serial.println(sensor.getRunDownshiftTimeMs() == 512 ? " [OK]" : " [FAIL]");

    rslt = sensor.setRunDownshiftTime(4);
    Serial.print("Set RunDownshift(4): ");
    Serial.println(rslt ? "OK" : "FAILED");

    Serial.println("-- Rest1 Period --");
    rslt = sensor.setRest1Period(9);
    Serial.print("Set Rest1Period(9): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get Rest1Period: ");
    Serial.print(sensor.getRest1Period());
    Serial.println(sensor.getRest1Period() == 9 ? " [OK]" : " [FAIL]");
    Serial.print("Get Rest1PeriodMs: ");
    Serial.print(sensor.getRest1PeriodMs());
    Serial.println(sensor.getRest1PeriodMs() == 100 ? " [OK]" : " [FAIL]");

    rslt = sensor.setRest1Period(1);
    Serial.print("Set Rest1Period(1): ");
    Serial.println(rslt ? "OK" : "FAILED");

    Serial.println("-- Rest1 Downshift --");
    rslt = sensor.setRest1DownshiftTime(50);
    Serial.print("Set Rest1Downshift(50): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get Rest1Downshift: ");
    Serial.print(sensor.getRest1DownshiftTime());
    Serial.println(sensor.getRest1DownshiftTime() == 50 ? " [OK]" : " [FAIL]");

    rslt = sensor.setRest1DownshiftTime(31);
    Serial.print("Set Rest1Downshift(31): ");
    Serial.println(rslt ? "OK" : "FAILED");

    Serial.println("-- Rest2 Period --");
    rslt = sensor.setRest2Period(19);
    Serial.print("Set Rest2Period(19): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get Rest2Period: ");
    Serial.print(sensor.getRest2Period());
    Serial.println(sensor.getRest2Period() == 19 ? " [OK]" : " [FAIL]");
    Serial.print("Get Rest2PeriodMs: ");
    Serial.print(sensor.getRest2PeriodMs());
    Serial.println(sensor.getRest2PeriodMs() == 200 ? " [OK]" : " [FAIL]");

    rslt = sensor.setRest2Period(9);
    Serial.print("Set Rest2Period(9): ");
    Serial.println(rslt ? "OK" : "FAILED");

    Serial.println("-- Rest2 Downshift --");
    rslt = sensor.setRest2DownshiftTime(100);
    Serial.print("Set Rest2Downshift(100): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get Rest2Downshift: ");
    Serial.print(sensor.getRest2DownshiftTime());
    Serial.println(sensor.getRest2DownshiftTime() == 100 ? " [OK]" : " [FAIL]");

    rslt = sensor.setRest2DownshiftTime(47);
    Serial.print("Set Rest2Downshift(47): ");
    Serial.println(rslt ? "OK" : "FAILED");

    Serial.println("-- Rest3 Period --");
    rslt = sensor.setRest3Period(99);
    Serial.print("Set Rest3Period(99): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get Rest3Period: ");
    Serial.print(sensor.getRest3Period());
    Serial.println(sensor.getRest3Period() == 99 ? " [OK]" : " [FAIL]");
    Serial.print("Get Rest3PeriodMs: ");
    Serial.print(sensor.getRest3PeriodMs());
    Serial.println(sensor.getRest3PeriodMs() == 1000 ? " [OK]" : " [FAIL]");

    rslt = sensor.setRest3Period(49);
    Serial.print("Set Rest3Period(49): ");
    Serial.println(rslt ? "OK" : "FAILED");

    Serial.println("\n========== LED Control Test ==========");
    rslt = sensor.setLedOn(true);
    Serial.print("Set LedOn(true): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get LedOn: ");
    Serial.print(sensor.getLedOn() ? "true" : "false");
    Serial.println(sensor.getLedOn() ? " [OK]" : " [FAIL]");

    rslt = sensor.setLedOn(false);
    Serial.print("Set LedOn(false): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get LedOn: ");
    Serial.print(sensor.getLedOn() ? "true" : "false");
    Serial.println(!sensor.getLedOn() ? " [OK]" : " [FAIL]");

    Serial.println("-- LED Drive Current --");
    rslt = sensor.setLedDriveCurrent(0);
    Serial.print("Set LedDriveCurrent(0=13mA): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get LedDriveCurrent: ");
    Serial.print(sensor.getLedDriveCurrent());
    Serial.println(sensor.getLedDriveCurrent() == 0 ? " [OK]" : " [FAIL]");

    rslt = sensor.setLedDriveCurrent(2);
    Serial.print("Set LedDriveCurrent(2=9.6mA): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get LedDriveCurrent: ");
    Serial.print(sensor.getLedDriveCurrent());
    Serial.println(sensor.getLedDriveCurrent() == 2 ? " [OK]" : " [FAIL]");

    rslt = sensor.setLedDriveCurrent(7);
    Serial.print("Set LedDriveCurrent(7=27mA): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get LedDriveCurrent: ");
    Serial.print(sensor.getLedDriveCurrent());
    Serial.println(sensor.getLedDriveCurrent() == 7 ? " [OK]" : " [FAIL]");

    Serial.println("\n========== Shutter Test ==========");
    rslt = sensor.setShutterMax(2000);
    Serial.print("Set ShutterMax(2000): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get ShutterMax: ");
    Serial.print(sensor.getShutterMax());
    Serial.println(sensor.getShutterMax() == 2000 ? " [OK]" : " [FAIL]");

    rslt = sensor.setShutterMax(2929);
    Serial.print("Set ShutterMax(2929): ");
    Serial.println(rslt ? "OK" : "FAILED");
    Serial.print("Get ShutterMax: ");
    Serial.print(sensor.getShutterMax());
    Serial.println(sensor.getShutterMax() == 2929 ? " [OK]" : " [FAIL]");

    Serial.println("\n========== Motion Interrupt Test ==========");
    for (int i = 0; i <= 7; i++) {
        rslt = sensor.setMotionInterruptThreshold(i);
        Serial.print("Set threshold(");
        Serial.print(i);
        Serial.print("): ");
        Serial.print(rslt ? "OK" : "FAILED");
        Serial.print(" Get: ");
        Serial.print(sensor.getMotionInterruptThreshold());
        Serial.println(sensor.getMotionInterruptThreshold() == i ? " [OK]" : " [FAIL]");
    }

    Serial.println("\n========== Power Timing Summary ==========");
    Serial.print("Run downshift: ");
    Serial.print(sensor.getRunDownshiftTimeMs());
    Serial.println(" ms");
    Serial.print("Rest1 period: ");
    Serial.print(sensor.getRest1PeriodMs());
    Serial.println(" ms");
    Serial.print("Rest1 downshift: ");
    Serial.print(sensor.getRest1DownshiftTimeMs() / 1000);
    Serial.println(" s");
    Serial.print("Rest2 period: ");
    Serial.print(sensor.getRest2PeriodMs());
    Serial.println(" ms");
    Serial.print("Rest2 downshift: ");
    Serial.print(sensor.getRest2DownshiftTimeMs() / 60000);
    Serial.println(" min");
    Serial.print("Rest3 period: ");
    Serial.print(sensor.getRest3PeriodMs());
    Serial.println(" ms");

    sensor.setCpi(SensorPawA350::CpiResolution::CPI_1000);
    sensor.setMotionInterruptThreshold(0);

    Serial.println("\n========== Motion Detection Started ==========");
    Serial.println("Move finger across sensor to detect motion");
    Serial.println("------------------------------------------");
}

void loop()
{
    uint32_t currentTime = millis();
    static uint32_t lastPrintTime = 0;
    static int32_t cumulativeX = 0;
    static int32_t cumulativeY = 0;

#if SENSOR_FPD_INT != -1
    static uint32_t proximityTriggerPrintInterval = 0;
    // Simply bring your finger close to the sensor to trigger it.
    if (digitalRead(SENSOR_FPD_INT) == HIGH) {
        if (millis() - proximityTriggerPrintInterval >= 1000) {
            Serial.println("Finger proximity sensor triggered.");
            proximityTriggerPrintInterval = millis();
        }
    }
#endif

    if (motionIrqTriggered) {
        motionIrqTriggered = false;

        SensorPawA350::MotionStatus status;
        if (sensor.checkMotion(status)) {
            if (status == SensorPawA350::MotionStatus::MOTION_DETECTED) {
                SensorPawA350::MotionData data;
                if (sensor.getMotionData(data)) {
                    cumulativeX += data.delta_x;
                    cumulativeY += data.delta_y;

                    if (currentTime - lastPrintTime >= 100) {
                        float inchesX = (float)cumulativeX / 1000.0f;
                        float inchesY = (float)cumulativeY / 1000.0f;

                        Serial.print("Motion: DX=");
                        Serial.print(data.delta_x);
                        Serial.print(", DY=");
                        Serial.print(data.delta_y);
                        Serial.print(", CumX=");
                        Serial.print(cumulativeX);
                        Serial.print(", CumY=");
                        Serial.print(cumulativeY);
                        Serial.print(", InchX=");
                        Serial.print(inchesX, 3);
                        Serial.print(", InchY=");
                        Serial.print(inchesY, 3);
                        Serial.print(", Time=");
                        Serial.println(currentTime);

                        lastPrintTime = currentTime;
                    }
                }
            }
        }
    }
}

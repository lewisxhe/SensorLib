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
 * @file      BQ27220_GaugeExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-16
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "GaugeBQ27220.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  2
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  3
#endif

GaugeBQ27220 gauge;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    if (!gauge.begin(Wire, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to BQ27220 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("Init BQ27220 Sensor success!");
}


void loop()
{
    int rate = gauge.getAtRate();                                       // mA
    uint16_t atRateTimeToEmpty = gauge.getAtRateTimeToEmpty();          // minutes
    float ntcTemperature = gauge.getTemperature();                      // Celsius
    uint16_t batteryVoltage = gauge.getBatteryVoltage();                // mV
    uint16_t batteryStatusRaw = gauge.getBatteryStatusRaw();            // N.A
    int instantaneousCurrent = gauge.getInstantaneousCurrent();         // mAh
    uint16_t remainingCapacity = gauge.getRemainingCapacity();          // mAh
    uint16_t fullChargeCapacity = gauge.getFullChargeCapacity();        // mAh
    uint16_t time2Empty = gauge.getTimeToEmpty();                       // minutes
    uint16_t time2Full = gauge.getTimeToFull();                         // minutes
    uint16_t standbyCurrent = gauge.getStandbyCurrent();                // mA


    uint16_t standbyTimeToEmpty = gauge.getStandbyTimeToEmpty();        // minutes
    int maxLoadCurrent = gauge.getMaxLoadCurrent();                     // mA
    uint16_t maxLoadTimeToEmpty = gauge.getMaxLoadTimeToEmpty();        // minutes
    uint16_t coulombCountRaw = gauge.getRawCoulombCount();              // mAh
    uint16_t averagePower = gauge.getAveragePower();                    // mW
    float internalTemperature = gauge.getInternalTemperature();         // Celsius
    uint16_t cycleCount = gauge.getCycleCount();                        // Number

    uint16_t stateOfCharge = gauge.getStateOfCharge();                  // %
    uint16_t stateOfHealth = gauge.getStateOfHealth();                  // %
    uint16_t chargingVoltage = gauge.getChargingVoltage();              // mV
    uint16_t chargingCurrent = gauge.getChargingCurrent();              // mA
    uint16_t BTPDischargeSet = gauge.getBTPDischargeSet();              // mAh
    uint16_t BTPChargeSet = gauge.getBTPChargeSet();                    // mAh
    uint16_t operationStatusRaw = gauge.getOperationStatusRaw();        // N.A
    uint16_t designCapacity = gauge.getDesignCapacity();                // mAh


    Serial.printf("getAtRate:%u mA\n", rate);
    Serial.printf("getAtRateTimeToEmpty:%u minutes\n", atRateTimeToEmpty);
    Serial.printf("getTemperature:%.2f ℃\n", ntcTemperature);
    Serial.printf("getBatteryVoltage:%u mV\n", batteryVoltage);
    Serial.printf("getBatteryStatusRaw:%u \n", batteryStatusRaw);
    Serial.printf("getInstantaneousCurrent:%u mAh\n", instantaneousCurrent);
    Serial.printf("getRemainingCapacity:%u mAh\n", remainingCapacity);
    Serial.printf("getFullChargeCapacity:%u mAh\n", fullChargeCapacity);
    Serial.printf("getTimeToEmpty:%u minutes\n", time2Empty);
    Serial.printf("getTimeToFull:%u minutes\n", time2Full);
    Serial.printf("getStandbyCurrent:%u mA\n", standbyCurrent);


    Serial.printf("getStandbyTimeToEmpty:%u minutes\n", standbyTimeToEmpty);
    Serial.printf("getMaxLoadCurrent:%d mA\n", maxLoadCurrent);
    Serial.printf("getMaxLoadTimeToEmpty:%u minute\n", maxLoadTimeToEmpty);
    Serial.printf("getRawCoulombCount:%u mAh\n", coulombCountRaw);
    Serial.printf("getAveragePower:%u mW\n", averagePower);
    Serial.printf("getInternalTemperature:%.2f ℃\n", internalTemperature);
    Serial.printf("getCycleCount:%u \n", cycleCount);

    Serial.printf("getStateOfCharge:%u %%\n", stateOfCharge);
    Serial.printf("getStateOfHealth:%u %%\n", stateOfHealth);
    Serial.printf("getChargingVoltage:%u mV\n", chargingVoltage);
    Serial.printf("getChargingCurrent:%u mA\n", chargingCurrent);
    Serial.printf("getBTPDischargeSet:%u mAh\n", BTPDischargeSet);
    Serial.printf("getBTPChargeSet:%u mAh\n", BTPChargeSet);
    Serial.printf("getOperationStatusRaw:%u \n", operationStatusRaw);
    Serial.printf("getDesignCapacity:%u mAh\n", designCapacity);

    BatteryStatus_t s = gauge.getBatteryStatus();

    if (s.FD) {
        Serial.println("1.Full discharge detected.");
    }
    if (s.OCVCOMP) {
        Serial.println("2.OCV measurement update is complete.");
    }
    if (s.OCVFAIL) {
        Serial.println("3.Status bit indicating that an OCV read failed due to current.");
        Serial.println("\tThis bit can only be set if a battery is present after receiving an OCV_CMD().");
    }
    if (s.SLEEP) {
        Serial.println("4.The device operates in SLEEP mode");
    }
    if (s.OTC) {
        Serial.println("5.Over-temperature is detected during charging.");
    }
    if (s.OTD) {
        Serial.println("6.Over-temperature detected during discharge condition.");
    }
    if (s.FC) {
        Serial.println("7.Full charge detected.");
    }
    if (s.CHGINH) {
        Serial.println("8.Charge Inhibit: If set, indicates that charging should not begin because the Temperature() is outside the range");
        Serial.println("\t[Charge Inhibit Temp Low, Charge Inhibit Temp High]. ");
    }
    if (s.TCA) {
        Serial.println("9.Termination of charging alarm. This flag is set and cleared based on the selected SOC Flag Config A option.");
    }
    if (s.OCVGD) {
        Serial.println("10.A good OCV measurement was made.");
    }
    if (s.AUTH_GD) {
        Serial.println("11.Detects inserted battery.");
    }
    if (s.BATTPRES) {
        Serial.println("12.Battery presence detected.");
    }
    if (s.TDA) {
        Serial.println("13.Termination discharge alarm. This flag is set and cleared according to the selected SOC Flag Config A option.");
    }
    if (s.SYSDWN) {
        Serial.println("14.System shutdown bit indicating that the system should be shut down. True when set. If set, the SOC_INT pin toggles once.");
    }
    if (s.DSG) {
        Serial.println("15.When set, the device is in DISCHARGE mode; when cleared, the device is in CHARGING or RELAXATION mode.");
    }

    Serial.println("===============================================");
    delay(3000);
}




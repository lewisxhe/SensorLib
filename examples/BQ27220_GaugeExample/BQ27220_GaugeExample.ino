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
#include <GaugeBQ27220.hpp>

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



    /**
    * @brief Set the new design capacity and full charge capacity of the battery.
    *
    * This function is responsible for updating the design capacity and full charge capacity of the battery.
    * It first checks the device's access settings, enters the configuration update mode, writes the new capacity values
    * and checksums, and finally exits the configuration update mode. If the device was previously in a sealed state,
    * it will be restored to the sealed mode after the operation is completed.
    * For new devices, use the default key for access. If it is an encrypted device, use setAccessKey(uint32_t key) to set the key.
    * @param newDesignCapacity The new design capacity to be set, of type uint16_t.
    * @param newFullChargeCapacity The new full charge capacity to be set, of type uint16_t.
    * @return bool Returns true if the setting is successful, false otherwise.
    */

    // uint32_t key = 0x12345678;
    // gauge.setAccessKey(key)
        
    uint16_t newDesignCapacity = 4000;
    uint16_t newFullChargeCapacity = 4000;
    gauge.setNewCapacity(newDesignCapacity, newFullChargeCapacity);
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


    Serial.print("getAtRate:"); Serial.print(rate); Serial.println(" mA");
    Serial.print("getAtRateTimeToEmpty:"); Serial.print(atRateTimeToEmpty); Serial.println(" minutes");
    Serial.print("getTemperature:"); Serial.print(ntcTemperature, 2); Serial.println(" ℃");
    Serial.print("getBatteryVoltage:"); Serial.print(batteryVoltage); Serial.println(" mV");
    Serial.print("getBatteryStatusRaw:"); Serial.println(batteryStatusRaw);
    Serial.print("getInstantaneousCurrent:"); Serial.print(instantaneousCurrent); Serial.println(" mAh");
    Serial.print("getRemainingCapacity:"); Serial.print(remainingCapacity); Serial.println(" mAh");
    Serial.print("getFullChargeCapacity:"); Serial.print(fullChargeCapacity); Serial.println(" mAh");
    Serial.print("getTimeToEmpty:"); Serial.print(time2Empty); Serial.println(" minutes");
    Serial.print("getTimeToFull:"); Serial.print(time2Full); Serial.println(" minutes");
    Serial.print("getStandbyCurrent:"); Serial.print(standbyCurrent); Serial.println(" mA");
    Serial.print("getStandbyTimeToEmpty:"); Serial.print(standbyTimeToEmpty); Serial.println(" minutes");
    Serial.print("getMaxLoadCurrent:"); Serial.print(maxLoadCurrent); Serial.println(" mA");
    Serial.print("getMaxLoadTimeToEmpty:"); Serial.print(maxLoadTimeToEmpty); Serial.println(" minute");
    Serial.print("getRawCoulombCount:"); Serial.print(coulombCountRaw); Serial.println(" mAh");
    Serial.print("getAveragePower:"); Serial.print(averagePower); Serial.println(" mW");
    Serial.print("getInternalTemperature:"); Serial.print(internalTemperature, 2); Serial.println(" ℃");
    Serial.print("getCycleCount:"); Serial.println(cycleCount);
    Serial.print("getStateOfCharge:"); Serial.print(stateOfCharge); Serial.println(" %");
    Serial.print("getStateOfHealth:"); Serial.print(stateOfHealth); Serial.println(" %");
    Serial.print("getChargingVoltage:"); Serial.print(chargingVoltage); Serial.println(" mV");
    Serial.print("getChargingCurrent:"); Serial.print(chargingCurrent); Serial.println(" mA");
    Serial.print("getBTPDischargeSet:"); Serial.print(BTPDischargeSet); Serial.println(" mAh");
    Serial.print("getBTPChargeSet:"); Serial.print(BTPChargeSet); Serial.println(" mAh");
    Serial.print("getOperationStatusRaw:"); Serial.println(operationStatusRaw);
    Serial.print("getDesignCapacity:"); Serial.print(designCapacity); Serial.println(" mAh");

    BatteryStatus s = gauge.getBatteryStatus();
    Serial.println("Battery Status:");
    if (s.isFullDischargeDetected()) {
        Serial.println("1.Full discharge detected.");
    }
    if (s.isOcvMeasurementUpdateComplete()) {
        Serial.println("2.OCV measurement update is complete.");
    }
    if (s.isOcvReadFailedDueToCurrent()) {
        Serial.println("3.Status bit indicating that an OCV read failed due to current.");
        Serial.println("\tThis bit can only be set if a battery is present after receiving an OCV_CMD().");
    }
    if (s.isInSleepMode()) {
        Serial.println("4.The device operates in SLEEP mode");
    }
    if (s.isOverTemperatureDuringCharging()) {
        Serial.println("5.Over-temperature is detected during charging.");
    }
    if (s.isOverTemperatureDuringDischarge()) {
        Serial.println("6.Over-temperature detected during discharge condition.");
    }
    if (s.isFullChargeDetected()) {
        Serial.println("7.Full charge detected.");
    }
    if (s.isChargeInhibited()) {
        Serial.println("8.Charge Inhibit: If set, indicates that charging should not begin because the Temperature() is outside the range");
        Serial.println("\t[Charge Inhibit Temp Low, Charge Inhibit Temp High]. ");
    }
    if (s.isChargingTerminationAlarm()) {
        Serial.println("9.Termination of charging alarm. This flag is set and cleared based on the selected SOC Flag Config A option.");
    }
    if (s.isGoodOcvMeasurement()) {
        Serial.println("10.A good OCV measurement was made.");
    }
    if (s.isBatteryInserted()) {
        Serial.println("11.Detects inserted battery.");
    }
    if (s.isBatteryPresent()) {
        Serial.println("12.Battery presence detected.");
    }
    if (s.isDischargeTerminationAlarm()) {
        Serial.println("13.Termination discharge alarm. This flag is set and cleared according to the selected SOC Flag Config A option.");
    }
    if (s.isSystemShutdownRequired()) {
        Serial.println("14.System shutdown bit indicating that the system should be shut down. True when set. If set, the SOC_INT pin toggles once.");
    }
    if (s.isInDischargeMode()) {
        Serial.println("15.When set, the device is in DISCHARGE mode; when cleared, the device is in CHARGING or RELAXATION mode.");
    }
    Serial.println("===============================================");
    delay(3000);
}




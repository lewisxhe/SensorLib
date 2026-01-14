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
 * @file      AXP2602_GaugeExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-01-14
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <GaugeAXP2602.hpp>

#ifndef SENSOR_SDA
#define SENSOR_SDA  2
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  3
#endif

GaugeAXP2602 gauge;

void setup()
{
    Serial.begin(115200);
    // while (!Serial);

    Wire.begin(SENSOR_SDA, SENSOR_SCL);

    if (!gauge.begin(Wire, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to AXP2602 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
    Serial.println("Init AXP2602 Sensor success!");

    /*
    *  Gauge Work mode:
    *      // It has relatively high power consumption, but can be used for both charging and discharging.
    *      - OPERATING_MODE_HIGH_PRECISION
    *
    *      // In default mode, it can be used for both charging and discharging.
    *      - OPERATING_MODE_NORMAL
    *
    *      // With relatively low power consumption, it can only be used in low-power scenarios during discharge.
    *      // In other scenarios, software switching to normal mode or high-precision mode is required.
    *      - OPERATING_MODE_LOW_POWER
    */
    // gauge.setOperatingMode(GaugeAXP2602::OPERATING_MODE_HIGH_PRECISION);

    /*
    *  The sampling resistor size is 10 milliohms by default.
    *   SENSE_RESISTOR_10_MOHM
    *   SENSE_RESISTOR_5_MOHM
    *   SENSE_RESISTOR_20_MOHM
    *   SENSE_RESISTOR_40_MOHM
    */
    // gauge.setCurrentSenseResistor(GaugeAXP2602::SENSE_RESISTOR_10_MOHM);

    // gauge.setBatteryDetection(true);         // Enabled by default

    // gauge.setCurrentMeasurement(true);       // Enabled by default

    // gauge.setThermalDieMeasurement(true);    // Enabled by default

    // gauge.setLowBatterySOCThreshold(60);  // Set low battery SOC threshold to 60%

    //! =========================== Battery calibration array, 128 bytes ===========================
    //! The calibration battery only needs to be written once.

    /*

    Replace with the battery curve you want to use.

    static uint8_t BATTER_PARAMS[] = {
        0x01, 0xf5, 0x40, 0x00, 0x1b, 0x1e, 0x28, 0x0f, 0x0c, 0x1e, 0x32, 0x02, 0x14, 0x05, 0x0a, 0x04,
        0x74, 0xfc, 0xf4, 0x0d, 0x43, 0x10, 0x52, 0xfb, 0xa6, 0x01, 0xea, 0x04, 0x64, 0x06, 0x52, 0x06,
        0x18, 0x0a, 0xe7, 0x0f, 0x9f, 0x0f, 0x51, 0x09, 0xf7, 0x0e, 0x89, 0x0e, 0x71, 0x04, 0x58, 0x04,
        0x43, 0x09, 0x32, 0x0e, 0x1c, 0x0e, 0x14, 0x09, 0x04, 0x0d, 0xe9, 0x0d, 0xde, 0x03, 0xc8, 0x03,
        0xb3, 0x08, 0x9d, 0x0d, 0x79, 0x0d, 0x3a, 0x07, 0xf5, 0x9e, 0x56, 0x47, 0x36, 0x20, 0x24, 0x17,
        0xc5, 0x98, 0x7e, 0x66, 0x4e, 0x44, 0x38, 0x1a, 0x12, 0x0a, 0xf6, 0x00, 0x00, 0xf6, 0x00, 0xf6,
        0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xf6, 0x00, 0x00, 0xf6, 0x00, 0xf6,
        0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xfb, 0x00, 0x00, 0xf6, 0x00, 0x00, 0xf6, 0x00, 0xf6
    };


    if (gauge.writeGaugeData(BATTER_PARAMS, sizeof(BATTER_PARAMS))) {
        Serial.println("Battery calibration data write success");
    } else {
        Serial.println("Battery calibration data write failed");
    }

    */
}


void loop()
{
    uint32_t startMeasTime = millis();

    if (gauge.refresh()) {

        uint32_t endMesTime = millis();

        Serial.print("Polling time: "); Serial.print(endMesTime - startMeasTime); Serial.println(" ms");
        Serial.print("\t- Sleep: "); Serial.print(gauge.isSleepModeEnabled() ? "Enabled" : "Disabled"); Serial.println();
        Serial.print("\t- Temperature:"); Serial.print(gauge.getTemperature()); Serial.println(" ℃");
        Serial.print("\t- BatteryVoltage:"); Serial.print(gauge.getVoltage()); Serial.println(" mV");
        Serial.print("\t- InstantaneousCurrent:"); Serial.print(gauge.getCurrent()); Serial.println(" mAh");
        Serial.print("\t- InstantaneousCurrentRaw:"); Serial.print(gauge.getCurrentRaw()); Serial.println(" RAW");
        Serial.print("\t- ThermalDieTemperature:"); Serial.print(gauge.getThermalDieTemperature()); Serial.println(" ℃");
        Serial.print("\t- ThermalDieTemperatureRaw:"); Serial.print(gauge.getThermalDieTemperatureRaw()); Serial.println(" RAW");
        Serial.print("\t- ChargingPower:"); Serial.print(gauge.getChargingPower()); Serial.println(" W");
        Serial.print("\t- DischargingPower:"); Serial.print(gauge.getDischargingPower()); Serial.println(" W");
        Serial.print("\t- AbsolutePower:"); Serial.print(gauge.getAbsolutePower()); Serial.println(" W");
        Serial.print("\t- isCharging:"); Serial.print(gauge.isCharging() ? "Yes" : "No"); Serial.println();
        Serial.print("\t- isDischarging:"); Serial.print(gauge.isDischarging() ? "Yes" : "No"); Serial.println();
        Serial.print("\t- TimeToEmpty:"); Serial.print(gauge.getTimeToEmpty()); Serial.println(" minutes");
        Serial.print("\t- TimeToFull:"); Serial.print(gauge.getTimeToFull()); Serial.println(" minutes");
        Serial.print("\t- StateOfCharge:"); Serial.print(gauge.getStateOfCharge()); Serial.println(" %");
        Serial.print("\t- Battery Status:"); Serial.print(gauge.getBatteryStatus()); Serial.println(" RAW");

        GaugeAXP2602::IRQStatus irqStatus = gauge.getIRQStatus();

        switch (irqStatus) {
        case GaugeAXP2602::IRQ_STATUS_LOW_BATTERY:
            Serial.println("Low battery detected!");
            Serial.println("Low battery detected!");
            break;
        case GaugeAXP2602::IRQ_STATUS_WDT_TIMEOUT:
            Serial.println("Watchdog timer timeout!");
            Serial.println("Watchdog timer timeout!");
            break;
        case GaugeAXP2602::IRQ_STATUS_OVER_TEMPERATURE:
            Serial.println("Over temperature detected!");
            Serial.println("Over temperature detected!");
            break;
        case GaugeAXP2602::IRQ_STATUS_SOC_UPDATE:
            Serial.println("State of Charge updated!");
            Serial.println("State of Charge updated!");
            break;
        default:
            break;
        }

        if (irqStatus != GaugeAXP2602::IRQ_STATUS_NONE) {
            gauge.clearIRQStatus();
        }

        Serial.println("===============================================");

    }
    delay(3000);
}




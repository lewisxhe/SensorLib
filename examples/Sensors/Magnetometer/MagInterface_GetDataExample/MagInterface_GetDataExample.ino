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
 * @file      MagInterface_GetDataExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-08
 *
 */
#include <MagnetometerDrv.hpp>
#include <SensorWireHelper.h>

#ifdef ARDUINO_T_BEAM_S3_SUPREME
#include <XPowersAXP2101.tpp>   //PMU Library https://github.com/lewisxhe/XPowersLib.git
#endif

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

MagnetometerBase *magnetometer;
void calibrate();

#if defined(ARDUINO_T_BEAM_S3_SUPREME)
XPowersAXP2101 power;
#endif

void beginPower()
{
#if defined(ARDUINO_T_BEAM_S3_SUPREME)
    Wire.begin(SENSOR_SDA, SENSOR_SCL);
    SensorWireHelper::dumpDevices(Wire);
    power.begin(Wire1, AXP2101_SLAVE_ADDRESS, 42, 41);
    power.disableALDO1();
    power.disableALDO2();
    delay(250);
    power.setALDO1Voltage(3300);
    power.enableALDO1();
    power.setALDO2Voltage(3300);
    power.enableALDO2();
    power.setChargingLedMode(XPOWERS_CHG_LED_OFF);
#endif
}

void setup()
{

    // The desired output data rate in Hz.  Allowed values are 1.0, 10.0, 50.0, 100.0 and 200.0HZ.
    float data_rate_hz = 200.0f;
    // op_mode: Allowed values are SUSPEND, NORMAL, SINGLE_MEASUREMENT, CONTINUOUS_MEASUREMENT
    OperationMode op_mode = OperationMode::CONTINUOUS_MEASUREMENT;
    // full_scale: Allowed values are  FS_8G, FS_16G ,FS_32G
    MagFullScaleRange full_scale = MagFullScaleRange::FS_8G;
    // over_sample_ratio: Allowed values are OSR_1, OSR_2, OSR_4, OSR_8
    MagOverSampleRatio over_sample_ratio = MagOverSampleRatio::OSR_1;
    // down_sample_ratio: QMC6309 does not support downsampling rate settings; this parameter is ignored.
    MagDownSampleRatio down_sample_ratio = MagDownSampleRatio::DSR_1;


    Serial.begin(115200);
    while (!Serial);

    // LilyGo T-Beam-Supreme sensor requires a power source to function.
    beginPower();

    if (magnetometer == nullptr) {
        magnetometer = new SensorQMC6310();
        if (!static_cast<SensorQMC6310*>(magnetometer)->begin(Wire, QMC6310U_SLAVE_ADDRESS)) {
            Serial.println("Failed to find QMC6310U - check your wiring!");
            delete magnetometer;
            magnetometer = nullptr;
        } else {
            Serial.println("QMC6310U found!");
            // The desired output data rate in Hz.  Allowed values are 10.0, 50.0, 100.0 and 200.0HZ.
            data_rate_hz = 200.0f;
            // op_mode: Allowed values are SUSPEND, NORMAL, SINGLE_MEASUREMENT, CONTINUOUS_MEASUREMENT
            op_mode = OperationMode::CONTINUOUS_MEASUREMENT;
            // full_scale: Allowed values are FS_2G, FS_8G, FS_12G ,FS_30G
            full_scale = MagFullScaleRange::FS_8G;
            // over_sample_ratio: Allowed values are OSR_1, OSR_2, OSR_4, OSR_8
            over_sample_ratio = MagOverSampleRatio::OSR_1;
            // down_sample_ratio: Allowed values are DSR_1, DSR_2, DSR_4, DSR_8
            down_sample_ratio = MagDownSampleRatio::DSR_1;
        }
    }

    if (magnetometer == nullptr) {
        magnetometer = new SensorQMC6310();
        if (!static_cast<SensorQMC6310*>(magnetometer)->begin(Wire, QMC6310N_SLAVE_ADDRESS)) {
            Serial.println("Failed to find QMC6310 - check your wiring!");
            delete magnetometer;
            magnetometer = nullptr;
        } else {
            Serial.println("QMC6310N found!");
            // The desired output data rate in Hz.  Allowed values are 10.0, 50.0, 100.0 and 200.0HZ.
            data_rate_hz = 200.0f;
            // op_mode: Allowed values are SUSPEND, NORMAL, SINGLE_MEASUREMENT, CONTINUOUS_MEASUREMENT
            op_mode = OperationMode::CONTINUOUS_MEASUREMENT;
            // full_scale: Allowed values are FS_2G, FS_8G, FS_12G ,FS_30G
            full_scale = MagFullScaleRange::FS_8G;
            // over_sample_ratio: Allowed values are OSR_1, OSR_2, OSR_4, OSR_8
            over_sample_ratio = MagOverSampleRatio::OSR_1;
            // down_sample_ratio: Allowed values are DSR_1, DSR_2, DSR_4, DSR_8
            down_sample_ratio = MagDownSampleRatio::DSR_1;
        }
    }

    if (magnetometer == nullptr) {
        magnetometer = new SensorQMC6309();
        if (!static_cast<SensorQMC6309*>(magnetometer)->begin(Wire, QMC6309_SLAVE_ADDRESS)) {
            Serial.println("Failed to find QMC6309 - check your wiring!");
            delete magnetometer;
            magnetometer = nullptr;
        } else {
            Serial.println("QMC6309 found!");
            // The desired output data rate in Hz.  Allowed values are 1.0, 10.0, 50.0, 100.0 and 200.0HZ.
            data_rate_hz = 200.0f;
            // op_mode: Allowed values are SUSPEND, NORMAL, SINGLE_MEASUREMENT, CONTINUOUS_MEASUREMENT
            op_mode = OperationMode::CONTINUOUS_MEASUREMENT;
            // full_scale: Allowed values are  FS_8G, FS_16G ,FS_32G
            full_scale = MagFullScaleRange::FS_8G;
            // over_sample_ratio: Allowed values are OSR_1, OSR_2, OSR_4, OSR_8
            over_sample_ratio = MagOverSampleRatio::OSR_1;
            // down_sample_ratio: QMC6309 does not support downsampling rate settings; this parameter is ignored.
            down_sample_ratio = MagDownSampleRatio::DSR_1;
        }
    }

    while (magnetometer == nullptr) {
        Serial.println("No magnetometer found!");
        delay(1000);
    }

    /* Config Magnetometer */
    if (magnetometer->configMagnetometer(
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

    // Calibration algorithm reference from
    // https://github.com/CoreElectronics/CE-PiicoDev-QMC6310-MicroPython-Module
    calibrate();

    Serial.println("Calibration done.");
    delay(5000);
    // magnetometer->setOffset(1295,-1478,398);


    SensorInfo info = magnetometer->getSensorInfo();
    Serial.print("Manufacturer: "); Serial.println(info.manufacturer);
    Serial.print("Model: "); Serial.println(info.model);
    Serial.print("I2C Address: 0x"); Serial.println(info.i2c_address, HEX);
    Serial.print("Version: "); Serial.println(info.version);
    Serial.print("UID: 0x"); Serial.println(info.uid);
    Serial.print("Type: "); Serial.println(SensorUtils::typeToString(info.type));

    SensorConfig cfg = magnetometer->getConfig();
    Serial.print("DataRate: "); Serial.println(cfg.sample_rate);
    Serial.print("FullScaleRange: "); Serial.println(cfg.range);
    Serial.print("Mode: "); Serial.println((uint8_t)cfg.mode);
    Serial.println();

    //Find the magnetic declination : https://www.magnetic-declination.com/
    float declination_deg = MagnetometerUtils::dmsToDecimalDegrees(-3, 20);   // -3.3333

    magnetometer->setDeclination(declination_deg);

    Serial.print(" Magnetic Declination: ");
    Serial.print(declination_deg, 2);
    Serial.println("°");

    Serial.print(" Sensitivity: ");
    Serial.print(magnetometer->getSensitivity(), 6);
    Serial.println(" Gauss/LSB");

    delay(3000);

    Serial.println("Read data now...");
}

void loop()
{

    MagnetometerData data;

    if (magnetometer->readData(data)) {

        // Gauss to μT
        float x = MagnetometerUtils::gaussToMicroTesla(data.magnetic_field.x);
        float y = MagnetometerUtils::gaussToMicroTesla(data.magnetic_field.y);
        float z = MagnetometerUtils::gaussToMicroTesla(data.magnetic_field.z);

        Serial.print("Mag:");
        Serial.print(" X:"); Serial.print(x);
        Serial.print(" Y:"); Serial.print(y);
        Serial.print(" Z:"); Serial.print(z);
        Serial.print(" μT");

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
    }
    delay(10);
}


void calibrate()
{
    if (!magnetometer)return;

    if (!magnetometer->setOutputDataRate(200.0f)) {
        Serial.println("Failed to set output data rate");
        return ;
    }

    Serial.println("========================================");
    Serial.println("Calibration Instructions:");
    Serial.println("1. Rotate sensor in FIGURE-8 pattern");
    Serial.println("2. Cover all axes (X, Y, Z directions)");
    Serial.println("3. Rotate slowly and completely");
    Serial.println("4. Wait for progress bar to complete");
    Serial.println("5. Expected: Magnetic Strength ~25-65 uT");
    Serial.println("========================================");
    Serial.println();

    Serial.println("Place the sensor on the plane and slowly rotate the sensor...");
    Serial.println("Rotate in FIGURE-8 pattern to cover all directions!");
    Serial.println("If the heading angle does not change with the direction, please rerun the calibration until it is normal.");
    Serial.println();
    delay(3000);


#if defined(ARDUINO_T_BEAM_S3_SUPREME)
    power.setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
#endif

    int32_t x_min = 65535;
    int32_t x_max = -65535;
    int32_t y_min = 65535;
    int32_t y_max = -65535;
    int32_t z_min = 65535;
    int32_t z_max = -65535;

    int32_t range = 1000;
    int32_t i = 0;
    int32_t x = 0, y = 0, z = 0;
    int16_t x_offset = 0;
    int16_t y_offset = 0;
    int16_t z_offset = 0;

    MagnetometerData data;
    while (i < range) {
        i += 1;

        if (magnetometer->isDataReady()) {

            magnetometer->readData(data);

            x = (data.raw.x + x) / 2;
            y = (data.raw.y + y) / 2;
            z = (data.raw.z + z) / 2;
            if (x < x_min) {
                x_min = x;
                i = 0;
            }
            if (x > x_max) {
                x_max = x;
                i = 0;
            }
            if (y < y_min) {
                y_min = y;
                i = 0;
            }
            if (y > y_max) {
                y_max = y;
                i = 0;
            }
            if (z < z_min) {
                z_min = z;
                i = 0;
            }
            if (z > z_max) {
                z_max = z;
                i = 0;
            }
            int j = round(10 * i / range);

            Serial.print("[");
            for (int k = 0; k < j; ++k) {
                Serial.print("*");
            }
            Serial.println("]");
        }
        delay(5);
    }

    x_offset = (x_max + x_min) / 2;
    y_offset = (y_max + y_min) / 2;
    z_offset = (z_max + z_min) / 2;

    Serial.print("x_min:");
    Serial.println(x_min);

    Serial.print("x_max:");
    Serial.println(x_max);

    Serial.print("y_min:");
    Serial.println(y_min);

    Serial.print("y_max:");
    Serial.println(y_max);

    Serial.print("z_min:");
    Serial.println(z_min);

    Serial.print("z_max:");
    Serial.println(z_max);

    Serial.print("x_offset:");
    Serial.println(x_offset);

    Serial.print("y_offset:");
    Serial.println(y_offset);

    Serial.print("z_offset:");
    Serial.println(z_offset);


    // Set the calibration value and the user calculates the deviation
    magnetometer->setOffset(x_offset, y_offset, z_offset);

    Serial.println();
    Serial.println("Calibration complete!");
    Serial.println("Check if Magnetic Strength is ~25-65 uT");
    Serial.println("If too low, repeat calibration with better rotation");

#if defined(ARDUINO_T_BEAM_S3_SUPREME)
    power.setChargingLedMode(XPOWERS_CHG_LED_ON);
#endif
}

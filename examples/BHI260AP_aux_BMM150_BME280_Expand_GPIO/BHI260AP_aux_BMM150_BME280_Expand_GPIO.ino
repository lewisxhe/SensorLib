/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      BHI260AP_aux_BMM150_BME280_Expand_GPIO.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-10-10
 * @note      Changed from Boschsensortec API https://github.com/boschsensortec/BHY2_SensorAPI
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorBHI260AP.hpp"

#include <Commander.h>  //Deplib https://github.com/CreativeRobotics/Commander
Commander cmd;
void initialiseCommander();

/*
Write the firmware containing the BMM150 magnetometer function into the flash.
This function requires the BHI260AP external SPI Flash.
If there is no Flash, it can only be written and run in RAM.
Example firmware source: https://github.com/boschsensortec/BHY2_SensorAPI/tree/master/firmware
You can also compile custom firmware to write
How to build custom firmware see : https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bhi260ab-an000.pdf
*/
#define WRITE_TO_FLASH          true           //Set 1 write fw to flash ,set 0 write fw to ram

#if   WRITE_TO_FLASH
// Custom firmware with GPIO input and output functions
#include "BHI260AP_aux_BMM150_BME280_GPIO_flash.fw.h"
const uint8_t *firmware = BHI260AP_aux_BMM150_BME280_GPIO_flash;
const size_t fw_size = sizeof(BHI260AP_aux_BMM150_BME280_GPIO_flash);
#else
#include "BHI260AP_aux_BMM150_BME280_GPIO.fw.h"
// Custom firmware with GPIO input and output functions
const uint8_t *firmware = BHI260AP_aux_BMM150_BME280_GPIO;
const size_t fw_size = sizeof(BHI260AP_aux_BMM150_BME280_GPIO);
#endif

/*
* GPIO Comparison Table
* M1SCX = N.A
* M1SDX = N.A
* M1SDI = N.A
* M2SCX = 14    !OK
* M2SDX = 15    !OK
* M2SDI = 16    !OK
* MCSB1 = 1     !OK
* MCSB2 = 4     /aux BMM150
* M3SCL = 17    /aux BMM150
* M3SDA = 18    /aux BMM150
* MCSB3 = 5     ! OK
* MCSB4 = 6     ! OK
* JTAG_CLK = 19 ! OK
* JTAG_DIO = 20 ! OK
* RESV1 = 2     ! no test
* RESV2 = 3     ! no test
* RESV3 = N.A
* */

#ifdef BHY2_USE_I2C
#define BHI260AP_SDA          21
#define BHI260AP_SCL          22
#define BHI260AP_IRQ          39
#define BHI260AP_RST          -1
#else
#define BHI260AP_MOSI         27
#define BHI260AP_MISO         46
#define BHI260AP_SCK          3
#define BHI260AP_CS           28
#define BHI260AP_IRQ          30
#define BHI260AP_RST          -1
#endif

SensorBHI260AP bhy;

void parse_bme280_sensor_data(uint8_t sensor_id, uint8_t *data_ptr, uint32_t len, uint64_t *timestamp)
{
    float humidity = 0;
    float temperature = 0;
    float pressure = 0;
    switch (sensor_id) {
    case SENSOR_ID_HUM:
    case SENSOR_ID_HUM_WU:
        bhy2_parse_humidity(data_ptr, &humidity);
        Serial.print("humidity:"); Serial.print(humidity); Serial.println("%");
        break;
    case SENSOR_ID_TEMP:
        bhy2_parse_temperature_celsius(data_ptr, &temperature);
        Serial.print("temperature:"); Serial.print(temperature); Serial.println("*C");
        break;
    case SENSOR_ID_BARO:
        bhy2_parse_pressure(data_ptr, &pressure);
        Serial.print("pressure:"); Serial.print(pressure); Serial.println("hPa");
        break;
    default:
        Serial.println("Unkown.");
        break;
    }
}

void sensor_event_callback(uint8_t event, uint8_t sensor_id, uint8_t data)
{
    Serial.print("Sensor Event:");
    switch (event) {
    case BHY2_META_EVENT_SAMPLE_RATE_CHANGED:
        Serial.printf("Sample rate changed for %s sensor\n", bhy.getSensorName(sensor_id));
        break;
    case BHY2_META_EVENT_POWER_MODE_CHANGED:
        Serial.printf("Power mode changed for %s sensor\n", bhy.getSensorName(sensor_id));
        break;
    default:
        Serial.printf("Other event : %u\n", event);
        break;
    }
}


void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // Set the reset pin and interrupt pin, if any
    bhy.setPins(BHI260AP_RST, BHI260AP_IRQ);
    // Force update of the current firmware, regardless of whether it exists.
    // After uploading the firmware once, you can change it to false to speed up the startup time.
    bool force_update = true;
    // Set the firmware array address and firmware size
    bhy.setFirmware(firmware, fw_size, WRITE_TO_FLASH, force_update);

    // Set to load firmware from flash
    bhy.setBootFormFlash(WRITE_TO_FLASH);

    Serial.println("Initializing Sensors...");
#ifdef BHY2_USE_I2C
    // Using I2C interface
    // BHI260AP_SLAVE_ADDRESS_L = 0x28
    // BHI260AP_SLAVE_ADDRESS_H = 0x29
    if (!bhy.init(Wire, BHI260AP_SDA, BHI260AP_SCL, BHI260AP_SLAVE_ADDRESS_L)) {
        Serial.print("Failed to initialize sensor - error code:");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#else
    // Using SPI interface
    if (!bhy.init(SPI, BHI260AP_CS, BHI260AP_MOSI, BHI260AP_MISO, BHI260AP_SCK)) {
        Serial.print("Failed to initialize sensor - error code:");
        Serial.println(bhy.getError());
        while (1) {
            delay(1000);
        }
    }
#endif

    Serial.println("Initializing the sensor successfully!");

    // Register sensor change event callback
    bhy.onEvent(sensor_event_callback);

    // Register BME280 data parse callback function
    bhy.onResultEvent(SENSOR_ID_TEMP, parse_bme280_sensor_data);
    bhy.onResultEvent(SENSOR_ID_HUM, parse_bme280_sensor_data);
    bhy.onResultEvent(SENSOR_ID_BARO, parse_bme280_sensor_data);

    // Output all current sensor information
    bhy.printInfo(Serial);

    // Output interrupt configuration information to Serial
    bhy.printInterruptCtrl(Serial);

    initialiseCommander();

    Serial.println("Hello: Type 'help' to get help");

    cmd.printCommandPrompt();
}

uint32_t check_millis = 0;

void loop()
{
    //Call the update functions using the activeCommander pointer
    cmd.update();
    // Update sensor fifo
    bhy.update();
}


//All commands for 'master'
//COMMAND ARRAY ------------------------------------------------------------------------------
const commandList_t masterCommands[] = {
    {"help",       helpHandler,     "help"},
    {"set gpio",   setGpioLevel,    "set gpio level"},
    {"get gpio",   getGpioLevel,    "get gpio level"},
    {"dis gpio",   disGpioMode,     "disable gpio"},
    {"temperature", setTemperature, "Temperature"},
    {"humidity", setHumidity, "Humidity"},
    {"pressure", setPressure, "Pressure"},
};

void initialiseCommander()
{
    cmd.begin(&Serial, masterCommands, sizeof(masterCommands));
    cmd.commandPrompt(ON); //enable the command prompt
    cmd.echo(true);     //Echo incoming characters to theoutput port
    cmd.errorMessages(ON); //error messages are enabled - it will tell us if we issue any unrecognised commands
    //Error messaged do NOT work for quick set and get commands
}

bool helpHandler(Commander &Cmdr)
{
    Serial.println("Help:");
    Serial.println("\tCustom firmware valid gpio : 1, 2, 3, 5, 6, 14, 15, 16, 19, 20");
    Serial.println("\tset gpio [gpio num] [level]");
    Serial.println("\tget gpio [gpio num] [pullup]");
    Serial.println("\tdis gpio [gpio num]");
    Serial.println("\ttemperature [on/off] : range 0 ~ 1");
    Serial.println("\thumidity [on/off] : range 0 ~ 1");
    Serial.println("\tpressure [sample_rate/HZ] : range 0~1000");
    return 0;
}

bool setTemperature(Commander &Cmdr)
{
    float sample_rate;
    int items = Cmdr.countItems();
    if (items < 1) {
        return 0;
    }
    Cmdr.getFloat(sample_rate);
    bhy.configure(SENSOR_ID_TEMP, sample_rate, 0);
    return 0;
}

bool setHumidity(Commander &Cmdr)
{
    float sample_rate;
    int items = Cmdr.countItems();
    if (items < 1) {
        return 0;
    }
    Cmdr.getFloat(sample_rate);
    bhy.configure(SENSOR_ID_HUM, sample_rate, 0);
    return 0;
}

bool setPressure(Commander &Cmdr)
{
    float sample_rate;
    int items = Cmdr.countItems();
    if (items < 1) {
        return 0;
    }
    Cmdr.getFloat(sample_rate);
    bhy.configure(SENSOR_ID_BARO, sample_rate, 0);
    return 0;
}



bool setGpioLevel(Commander &Cmdr)
{
    int values[2] = {0, 0};
    int items = Cmdr.countItems();
    if (items < 2) {
        return false;
    }
    for (int n = 0; n < 2; n++) {
        Cmdr.getInt(values[n]);
    }
    uint8_t pin = values[0];
    uint8_t level = values[1];
    // Serial.printf("Set GPIO : %u to %u\n", pin, level);
    bhy.digitalWrite(pin, level);
    return 0;
}

bool getGpioLevel(Commander &Cmdr)
{
    int values[2] = {0, 0};
    int items = Cmdr.countItems();
    if (items < 1) {
        return 0;
    }
    if (items > 2 )items = 2;
    for (int n = 0; n < items; n++) {
        Cmdr.getInt(values[n]);
    }
    bool pullup = false;
    uint8_t pin = values[0];
    if (items == 2 ) {
        pullup = values[1];
    }
    uint8_t level = bhy.digitalRead(pin, pullup);
    Serial.printf("Get GPIO : %u level is %u\n", pin, level);
    return 0;
}

bool disGpioMode(Commander &Cmdr)
{
    int values[1] = {0};
    int items = Cmdr.countItems();
    if (items < 1) {
        return 0;
    }
    Cmdr.getInt(values[0]);
    uint8_t pin = values[0];
    // Serial.printf("Disable GPIO : %u\n", pin);
    bhy.disableGpio(pin);
    return 0;
}

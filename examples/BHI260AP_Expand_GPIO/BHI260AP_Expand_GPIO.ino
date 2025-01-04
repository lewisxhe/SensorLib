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
 * @file      BHI260AP_Expand_GPIO.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-04
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
Example firmware source: https://github.com/boschsensortec/BHY2_SensorAPI/tree/master/firmware
You can also compile custom firmware to write
How to build custom firmware see : https://www.bosch-sensortec.com/media/boschsensortec/downloads/application_notes_1/bst-bhi260ab-an000.pdf
*/

#include "BHI260AP_GPIO.fw.h"
// Custom firmware with GPIO input and output functions
const uint8_t *firmware = bhy2_gpio_firmware_image;
const size_t fw_size = sizeof(bhy2_gpio_firmware_image);

/*
* GPIO Comparison Table
* M1SCX = N.A   ! INVALID PIN
* M1SDX = N.A   ! INVALID PIN
* M1SDI = N.A   ! INVALID PIN
* M2SCX = 14    ! OK
* M2SDX = 15    ! OK
* M2SDI = 16    ! OK
* MCSB1 = 1     ! OK
* MCSB2 = 4     ! OK
* M3SCL = 17    ! OK
* M3SDA = 18    ! OK
* MCSB3 = 5     ! OK
* MCSB4 = 6     ! OK
* JTAG_CLK = 19 ! OK
* JTAG_DIO = 20 ! OK
* RESV1 = 2     ! INVALID PIN
* RESV2 = 3     ! INVALID PIN
* RESV3 = N.A   ! INVALID PIN
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

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    // Set the reset pin and interrupt pin, if any
    bhy.setPins(BHI260AP_RST, BHI260AP_IRQ);
    // Set the firmware array address and firmware size
    bhy.setFirmware(firmware, fw_size, false);
    // Set to load firmware from RAM
    bhy.setBootFromFlash(false);

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
    Serial.println("\tCustom firmware valid gpio : 1, 4, 5, 6, 14, 15, 16, 17, 18, 19, 20");
    Serial.println("\tset gpio [gpio num] [level]");
    Serial.println("\tget gpio [gpio num] [pullup]");
    Serial.println("\tdis gpio [gpio num]");
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

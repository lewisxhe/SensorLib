/**
 * @file      aw9523_led.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @date      2026-09-23
 */
#include <IoExpanderDrv.hpp>

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

// Pin 8 is P1_0, one of the six channels optimized for low dropout.
// Connect the LED anode to its positive supply and cathode to P1_0.
static constexpr uint8_t LED_PIN = 8;

IoExpanderAW9523 expander;

void setup()
{
    Serial.begin(115200);

    if (!expander.begin(Wire, AW9523_DEFAULT_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        while (1) {
            Serial.println("Failed to find AW9523 - check your wiring and address pins!");
            delay(1000);
        }
    }

    expander.setLedCurrentRange(IoExpanderAW9523::LedCurrentRange::QUARTER);
    expander.analogWrite(LED_PIN, 0);
    expander.setLedMode(LED_PIN);
}

void loop()
{
    for (int brightness = 0; brightness <= 255; ++brightness) {
        expander.analogWrite(LED_PIN, brightness);
        delay(5);
    }
    for (int brightness = 255; brightness >= 0; --brightness) {
        expander.analogWrite(LED_PIN, brightness);
        delay(5);
    }
}

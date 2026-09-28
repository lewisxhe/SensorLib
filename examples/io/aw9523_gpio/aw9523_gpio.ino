/**
 * @file      aw9523_gpio.ino
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

static constexpr uint8_t OUTPUT_PIN = 0; // P0_0
static constexpr uint8_t INPUT_PIN = 1;  // P0_1, requires an external pull-up/down

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

    // P0 defaults to open-drain. Select push-pull for a directly driven output.
    expander.setPort0PushPull(true);
    expander.pinMode(OUTPUT_PIN, OUTPUT);
    expander.pinMode(INPUT_PIN, INPUT);

    // INTN is active-low and open-drain. Reading P0 clears P0 interrupts.
    expander.enableInterrupt(INPUT_PIN);
}

void loop()
{
    expander.digitalToggle(OUTPUT_PIN);

    Serial.print("P0_1: ");
    Serial.println(expander.digitalRead(INPUT_PIN) ? "HIGH" : "LOW");
    delay(500);
}

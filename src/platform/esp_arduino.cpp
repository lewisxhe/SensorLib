/**
 * @file      esp_arduino.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2024  Shenzhen Xinyuan Electronic Technology Co., Ltd
 * @date      2024-01-08
 *
 */
#if !defined(ARDUINO)  && defined(ESP_PLATFORM)
#include "SensorLib.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

void pinMode(uint32_t gpio, uint8_t mode)
{
    gpio_config_t config;
    memset(&config, 0, sizeof(config));
    config.pin_bit_mask = 1ULL << gpio;
    switch (mode) {
    case INPUT:
        config.mode = GPIO_MODE_INPUT;
        break;
    case OUTPUT:
        config.mode = GPIO_MODE_OUTPUT;
        break;
    }
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&config));
}

void digitalWrite(uint32_t gpio, uint8_t level)
{
    gpio_set_level((gpio_num_t )gpio, level);
}

int digitalRead(uint32_t gpio)
{
    return gpio_get_level((gpio_num_t)gpio);
}

void delay(uint32_t ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

uint32_t millis()
{
    return esp_timer_get_time();
}

uint32_t micros()
{
    return (esp_timer_get_time() / 1000ULL);
}

void delayMicroseconds(uint32_t us)
{
    uint64_t m = (uint64_t)esp_timer_get_time();
    if (us) {
        uint64_t e = (m + us);
        if (m > e) {
            while ((uint64_t)esp_timer_get_time() > e) {
                asm volatile ("nop");
            }
        }
        while ((uint64_t)esp_timer_get_time() < e) {
            asm volatile ("nop");
        }
    }
}
#endif
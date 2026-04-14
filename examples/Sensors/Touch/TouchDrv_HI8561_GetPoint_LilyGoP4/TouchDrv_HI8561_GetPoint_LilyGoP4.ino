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
 * @file      TouchDrv_HI8561_GetPoint_LilyGoP4.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-05
 * @note      The LilyGo-P4 TFT version integrates an HI8561 MIPI DSI display and a touch controller connected via I2C.
 *            The display reset and touch interrupt pins are connected to an I/O extender (XL9555), which is also accessed via I2C.
 *            This example demonstrates how to initialize the display and read touch points from the HI8561 touch controller.
 *            Note: The screen must be initialized before touch can be initialized; initializing touch alone is not possible.
 *            This is consistent with the characteristics of most touch display integrated chips.
 *            HI8561 driver is available at https://github.com/Xinyuan-LilyGO/T-Display-P4
 *
 */
#include <Arduino.h>
#include <TouchDrv.hpp>

#ifdef ARDUINO_ESP32P4_DEV

#if ESP_ARDUINO_VERSION_VAL(3,3,7) > ESP_ARDUINO_VERSION
#error "This example requires Arduino-ESP32 version 3.3.7 or higher.Please update your Arduino-ESP32 core to the latest version to run this example."
#endif

#include <IoExpanderXL9555.hpp>
#include <SensorWireHelper.h>
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "hi8561_driver.h"
#include "esp_ldo_regulator.h"
#include <algorithm>

#if (ESP_ARDUINO_VERSION < ESP_ARDUINO_VERSION_VAL(3,2,0))
#error "This example requires Arduino-ESP32 version 3.2.0 or higher. Please update your Arduino-ESP32 core to the latest version to run this example."
#endif

// Pin definitions for the LilyGo-P4
#define P4_TOUCH_SDA  7
#define P4_TOUCH_SCL  8
#define P4_DISPLAY_BACKLIGHT 51
#define P4_IO_EXPANDER_IRQ 5
#define IO_EXPANDER_TOUCH_RST   (3 | 0x80)
#define IO_EXPANDER_TOUCH_IRQ   (4 | 0x80)
#define IO_EXPANDER_3V3_POWER_EN 0
#define IO_EXPANDER_5V0_POWER_EN 6
#define IO_EXPANDER_P4_VCCA_POWER_EN 8
#define IO_EXPANDER_DISP_RST 2

// HI8561 display parameters
#define HI8561_SCREEN_WIDTH 540
#define HI8561_SCREEN_HEIGHT 1168
#define HI8561_SCREEN_MIPI_DSI_DPI_CLK_MHZ 60
#define HI8561_SCREEN_MIPI_DSI_HSYNC 28
#define HI8561_SCREEN_MIPI_DSI_HBP 26
#define HI8561_SCREEN_MIPI_DSI_HFP 20
#define HI8561_SCREEN_MIPI_DSI_VSYNC 2
#define HI8561_SCREEN_MIPI_DSI_VBP 22
#define HI8561_SCREEN_MIPI_DSI_VFP 200
#define HI8561_SCREEN_DATA_LANE_NUM 2
#define HI8561_SCREEN_LANE_BIT_RATE_MBPS 1000
#define DISPLAY_WIDTH 540
#define DISPLAY_HEIGHT 1168

static uint16_t *drawBuffer = nullptr;
static const size_t pixel_count = DISPLAY_WIDTH * DISPLAY_HEIGHT;
static const size_t buffer_size = pixel_count * sizeof(uint16_t);

TouchDrvHI8561 touch;
IoExpanderXL9555 expander;
esp_lcd_panel_handle_t panelDrv = NULL;

static bool setupDisplay(esp_lcd_panel_handle_t *disp_panel);

void TouchDrvDigitalWrite(uint8_t gpio, uint8_t level)
{
    if (gpio & 0x80) {
        Serial.print("Expander DigitalWrite: "); Serial.print(gpio & 0x7F); Serial.print(" Level: "); Serial.println(level);
        expander.digitalWrite(gpio & 0x7F, level);
    } else {
        Serial.print("GPIO DigitalWrite: "); Serial.print(gpio); Serial.print(" Level: "); Serial.println(level);
        digitalWrite(gpio, level);
    }
}

uint8_t TouchDrvDigitalRead(uint8_t gpio)
{
    if (gpio & 0x80) {
        Serial.print("Expander DigitalRead: "); Serial.print(gpio & 0x7F); Serial.println();
        return expander.digitalRead(gpio & 0x7F);
    } else {
        Serial.print("GPIO DigitalRead: "); Serial.print(gpio); Serial.println();
        return digitalRead(gpio);
    }
}

void TouchDrvPinMode(uint8_t gpio, uint8_t mode)
{
    if (gpio & 0x80) {
        Serial.print("Expander PinMode: "); Serial.print(gpio & 0x7F); Serial.print(" Mode: "); Serial.println(mode);
        expander.pinMode(gpio & 0x7F, mode);
    } else {
        Serial.print("GPIO PinMode: "); Serial.print(gpio); Serial.print(" Mode: "); Serial.println(mode);
        pinMode(gpio, mode);
    }
}

void setup()
{
    Serial.begin(115200);

    while (!Serial);

    Wire.begin(P4_TOUCH_SDA, P4_TOUCH_SCL);

    // Scan I2C bus for devices
    SensorWireHelper::dumpDevices(Wire);

    // The LilyGo-P4 relies on an external I/O extender to control the power supply of peripheral devices.
    // First, initialize the external extender.
    if (!expander.begin(Wire, XL9555_SLAVE_ADDRESS0)) {
        while (1) {
            Serial.println("Failed to find XL9555 - check your wiring!");
            delay(1000);
        }
    }
    Serial.println("Sensor Expander Initialized");

    // Enable power supply for T-Display-P4 peripherals
    expander.pinMode(IO_EXPANDER_P4_VCCA_POWER_EN, OUTPUT);
    expander.digitalWrite(IO_EXPANDER_P4_VCCA_POWER_EN, LOW);

    expander.pinMode(IO_EXPANDER_5V0_POWER_EN, OUTPUT);
    expander.digitalWrite(IO_EXPANDER_5V0_POWER_EN, HIGH);

    expander.pinMode(IO_EXPANDER_3V3_POWER_EN, OUTPUT);
    expander.digitalWrite(IO_EXPANDER_3V3_POWER_EN, LOW);

    // Configure LDO for 1.8V output
    esp_ldo_channel_handle_t ldo_channel_handle = NULL;
    esp_ldo_channel_config_t ldo_channel_config = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    if (esp_ldo_acquire_channel(&ldo_channel_config, &ldo_channel_handle) != ESP_OK) {
        Serial.println("Set LDO channel failed");
        return;
    }

    // Reset the display
    expander.pinMode(IO_EXPANDER_DISP_RST, OUTPUT);
    expander.digitalWrite(IO_EXPANDER_DISP_RST, HIGH);
    delay(20);
    expander.digitalWrite(IO_EXPANDER_DISP_RST, LOW);
    delay(60);
    expander.digitalWrite(IO_EXPANDER_DISP_RST, HIGH);
    delay(20);


    // The screen must be initialized and made functional before touch initialization can proceed.
    if (!setupDisplay(&panelDrv)) {
        while (1) {
            Serial.println("Failed to setup display - check your wiring!");
            delay(1000);
        }
    }

    // LilyGo-Display-P4 touch interrupt pin and touch reset pin aux to expander , use gpio custom callback
    touch.setGpioCallback(TouchDrvPinMode, TouchDrvDigitalWrite, TouchDrvDigitalRead);

    // Set touch interrupt and reset Pin
    touch.setPins(IO_EXPANDER_TOUCH_RST, IO_EXPANDER_TOUCH_IRQ);

    // Screen must be initialized first before calling this class for initialization.
    if (!touch.begin(Wire, HI8561_SLAVE_ADDRESS)) {
        while (1) {
            Serial.println("Failed to find HI8561 - check your wiring!");
            delay(1000);
        }
    }
    Serial.println("Touch Driver Initialized");

    Serial.print("Chip ID : 0x"); Serial.println(touch.getChipID(), HEX);

    // Set the maximum coordinates ，used for mirroring touch coordinates
    // touch.setMaxCoordinates(568, 1232);

    // Set swap xy
    // touch.setSwapXY(true);

    // Set mirror xy
    // touch.setMirrorXY(true, true);
}

void loop()
{

    // * TouchPoints is configured with a 5-point touch point buffer by default,
    // * so it can return touch data from a maximum of 5 points. Although the HI8561
    // * supports 10-point touch
    TouchPoints touch_points = touch.getTouchPoints();
    if (touch_points.hasPoints()) {
        for (int i = 0; i < touch_points.getPointCount(); ++i) {
            const TouchPoint &point = touch_points.getPoint(i);
            Serial.print("ID: ");
            Serial.print(point.id);
            Serial.print(" ");
            Serial.print("X: ");
            Serial.print(point.x);
            Serial.print(" ");
            Serial.print("Y: ");
            Serial.print(point.y);
            Serial.print(" ");
            Serial.print("Pressure: ");
            Serial.print(point.pressure);
            Serial.println();
        }

        Serial.println();

        // When touch points are detected, fill the draw buffer with random colors
        std::fill(drawBuffer, drawBuffer + pixel_count, random(0x0000, 0xFFFF));
        esp_lcd_panel_draw_bitmap(panelDrv, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, drawBuffer);

    }
    delay(50);
}


static bool setupDisplay(esp_lcd_panel_handle_t *disp_panel)
{
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_panel_io_handle_t mipi_dbi_io;
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = HI8561_SCREEN_DATA_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_PLLREF_CLK_SRC_DEFAULT_LEGACY,
        .lane_bit_rate_mbps = HI8561_SCREEN_LANE_BIT_RATE_MBPS,
    };

    esp_err_t ret = esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus);
    if (ret != ESP_OK) {
        Serial.printf("esp_lcd_new_dsi_bus fail (error code: %#X)\n", ret);
        return false;
    }
    esp_lcd_dbi_io_config_t dbi_io_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ret = esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_io_config, &mipi_dbi_io);
    if (ret != ESP_OK) {
        Serial.printf("esp_lcd_new_panel_io_dbi fail (error code: %#X)\n", ret);
        return false;
    }
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = HI8561_SCREEN_MIPI_DSI_DPI_CLK_MHZ,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .num_fbs = 0,
        .video_timing = {
            .h_size = HI8561_SCREEN_WIDTH,
            .v_size = HI8561_SCREEN_HEIGHT,
            .hsync_pulse_width = HI8561_SCREEN_MIPI_DSI_HSYNC,
            .hsync_back_porch = HI8561_SCREEN_MIPI_DSI_HBP,
            .hsync_front_porch = HI8561_SCREEN_MIPI_DSI_HFP,
            .vsync_pulse_width = HI8561_SCREEN_MIPI_DSI_VSYNC,
            .vsync_back_porch = HI8561_SCREEN_MIPI_DSI_VBP,
            .vsync_front_porch = HI8561_SCREEN_MIPI_DSI_VFP,
        },
        .flags = {
            .use_dma2d = true,
        }
    };
    hi8561_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
    };
    esp_lcd_panel_dev_config_t dev_config = {
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };
    ret = esp_lcd_new_panel_hi8561(mipi_dbi_io, &dev_config, disp_panel);
    if (ret != ESP_OK) {
        Serial.printf("esp_lcd_new_panel_hi8561 fail (error code: %#X)\n", ret);
        return false;
    }

    // Initialize the display panel
    ret = esp_lcd_panel_init(*disp_panel);
    if (ret != ESP_OK) {
        Serial.printf("esp_lcd_panel_init fail (error code: %#X)\n", ret);
    }

    // Allocate memory for the draw buffer
    drawBuffer = (uint16_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!drawBuffer) {
        Serial.println("Failed to allocate memory for draw buffer");
        while (1);
    }

    // Fill the draw buffer with random colors
    std::fill(drawBuffer, drawBuffer + pixel_count, random(0x0000, 0xFFFF));

    esp_lcd_panel_draw_bitmap(panelDrv, 0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT, drawBuffer);

    // Enable backlight
    ledcAttach(P4_DISPLAY_BACKLIGHT, 200, 8);
    ledcWrite(P4_DISPLAY_BACKLIGHT, 255);

    return true;
}

#else
void setup()
{
    Serial.begin(115200);
}
void loop()
{
    Serial.println("Sketch only works with LilyGo-T-Display-P4 TFT(HI8561) Version");
    delay(1000);
}
#endif

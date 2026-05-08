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
 * @file      SensorBHI260AP.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-05-27
 *
 */
#include "SensorBHI260AP.hpp"

void SensorBHI260AP::setBootFromFlash(bool boot_from_flash)
{
    _boot_from_flash = boot_from_flash;
}

uint8_t SensorBHI260AP::digitalRead(uint8_t pin, bool pullup)
{
    if (pin > JTAG_DIO)return 0;
    uint32_t pin_mask = pin   | BHY2_GPIO_SET;
    if (pullup) {
        pin_mask |= (BHY2_INPUT_PULLUP << 8);
    } else {
        pin_mask |= (BHY2_INPUT << 8);
    }
    bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GPIO_EXP, (float)pin_mask, 0, dev.get());
    pin_mask = pin /*GetCmd*/;
    bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GPIO_EXP, (float)pin_mask, 0, dev.get());
    bhy2_virt_sensor_conf conf;
    bhy2_get_virt_sensor_cfg(BHY2_SENSOR_ID_GPIO_EXP, &conf, dev.get());
    uint8_t level = conf.sample_rate;
    return level;
}

void SensorBHI260AP::digitalWrite(uint8_t pin, uint8_t level)
{
    if (pin > JTAG_DIO)return;
    uint32_t pin_mask = pin  | (BHY2_OUTPUT << 8) | (level << 6) | BHY2_GPIO_SET ;
    bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GPIO_EXP, (float)pin_mask, 0, dev.get());
}

void SensorBHI260AP::disableGpio(uint8_t pin)
{
    if (pin > JTAG_DIO)return;
    uint32_t pin_mask = pin  | (BHY2_OPEN_DRAIN << 8) | BHY2_GPIO_SET;
    bhy2_set_virt_sensor_cfg(BHY2_SENSOR_ID_GPIO_EXP, (float)pin_mask, 0, dev.get());
}

uint16_t SensorBHI260AP::getConfirmationIDImpl()
{
    return BHI260_CHIP_ID;
}

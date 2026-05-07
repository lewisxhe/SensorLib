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
 * @file      AXP202Core.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-29
 *
 * @brief AXP202 Core I2C Communication Layer
 *
 * Low-level I2C register access for the AXP202 PMIC. Inherits from
 * I2CDeviceWithHal for bus abstraction and platform HAL compatibility
 * (Arduino / ESP-IDF).
 *
 * Functional modules (charging, ADC, timer, IRQ, etc.) are controlled
 * through their dedicated subsystem interfaces:
 *   - Charger:  pmic.charger().enableCharging()
 *   - ADC:      pmic.adc().enableChannels()
 *   - Timer:    pmic.timer().setTimer()
 *   - IRQ:      pmic.irq().enable()
 *
 * @see AXP202Regs.hpp
 */
#pragma once
#include "../../../platform/comm/I2CDeviceWithHal.hpp"
#include "../axp1xx/AXP1xxCore.hpp"

class AXP202Core : public I2CDeviceWithHal
{
public:
    AXP202Core() = default;
    ~AXP202Core() = default;

    /**
     * @brief Implicit conversion to SensorCommWrapper reference
     * @return Reference to this object as SensorCommWrapper
     */
    operator SensorCommWrapper&() { return *this; }

private:
    /**
     * @brief I2C device initialization implementation
     * @param param Opaque parameter from begin() call
     * @return true on success
     */
    bool initImpl(uint8_t param) override;
};

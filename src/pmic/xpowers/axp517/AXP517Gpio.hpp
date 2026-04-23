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
 * @file      AXP517Gpio.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */

#pragma once

#include "../../PmicGpioBase.hpp"
#include "AXP517Core.hpp"

/**
 * @brief AXP517 GPIO module (REG 0x11).
 *
 * From manual:
 *  - bit7: OD function configure: 0 floating, 1 open drain output
 *  - bit6: OD output configure: 0 hiz, 1 low
 *  - bit5: reserved (RO)
 *  - bit4: GPIO function: 0 input, 1 output
 *  - bit3:2: GPIO output source select:
 *        0: by reg11H[1:0]
 *        1: PD_IRQ
 *  - bit1: GPIO input status (RO): 0 low, 1 high
 *  - bit1:0: GPIO output configure:
 *        00: hiz
 *        01: low
 *        10: high
 *        11: reserved
 *
 * Notes:
 *  - When output source is PD_IRQ, manual output config bits are ignored by HW.
 *  - When OD mode enabled (bit7=1), OD output uses bit6 (hiz/low). "high" is not driven.
 */
class AXP517Gpio : public PmicGpioBase
{
public:

    enum class OutputSource : uint8_t {
        ByReg11 = 0,  ///< by reg11[1:0]
        PdIrq   = 1,  ///< PD_IRQ
    };

    explicit AXP517Gpio(AXP517Core &core);

    ~AXP517Gpio() = default;

    bool setDirection(uint8_t pin, Direction dir) override;

    Direction getDirection(uint8_t pin) override;

    bool setDrive(uint8_t pin, Drive drive) override;

    Drive getDrive(uint8_t pin) override;

    bool read(uint8_t pin, bool &high) override;

    bool write(uint8_t pin, Level level) override;

    bool setOutputSource(uint8_t pin, OutputSource src);

    OutputSource getOutputSource(uint8_t pin);

    /**
     * @brief Read raw REG11 for debug.
     */
    int readRaw();

private:
    AXP517Core &_core;
};

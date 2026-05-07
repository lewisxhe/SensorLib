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
 * @file      AXP2101Led.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Charging LED Control
 *
 */
#pragma once
#include "../../PmicLedBase.hpp"
#include "AXP2101Core.hpp"

/**
 * @brief Charging indicator LED control for the AXP2101 PMIC.
 *
 * Controls the CHGLED pin (register 0x69) which can be configured as:
 * - Automatic mode: LED follows charge state (on during charging,
 *   off when charged/standby)
 * - Manual mode: LED forced on or off or blinking
 * - Output type: push-pull or open-drain
 */
class AXP2101Led : public PmicLedBase
{
public:
    explicit AXP2101Led(AXP2101Core &core);
    ~AXP2101Led() = default;

    /**
     * @brief Set the LED output driver type.
     * @param type PushPull or OpenDrain configuration.
     * @return true on success.
     */
    bool setOutputType(OutputType type) override;

    /**
     * @brief Set the LED operating mode.
     * @param mode Automatic or Manual control mode.
     * @return true on success.
     */
    bool setMode(Mode mode) override;

    /**
     * @brief Get the current LED operating mode.
     * @return Current Mode setting.
     */
    Mode getMode() override;

    /**
     * @brief For manual mode, set the LED output state.
     * @param state On or Off.
     * @return true on success.
     */
    bool setManualState(ManualState state) override;

    /**
     * @brief Get the current manual LED state.
     * @return Current ManualState setting.
     */
    ManualState getManualState() override;

private:
    AXP2101Core &_core;
};

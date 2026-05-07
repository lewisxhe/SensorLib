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
 * @file      AXP2101Pwron.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Power Button Control
 *
 */
#pragma once
#include "AXP2101Core.hpp"
#include "../../PmicButtonBase.hpp"

/**
 * @brief Power-on / power-button timing control for the AXP2101 PMIC.
 *
 * Configures the PEKEY (power key) press durations for:
 * - Power-on trigger: how long the key must be held to turn the system on
 * - Power-off trigger: how long the key must be held to turn the system off
 * - IRQ trigger: how long the key must be held to generate an interrupt
 *   (short press detection)
 *
 */
class AXP2101Pwron : public PmicButtonBase
{
public:
    explicit AXP2101Pwron(AXP2101Core &core);
    ~AXP2101Pwron() = default;

    /**
     * @brief Set the power-on delay (held duration to turn on).
     * @param ms Duration in milliseconds.
     * Allowed value is 128ms,512ms,1000ms,2000ms
     * @return true on success.
     */
    bool setOnDurationMs(uint16_t ms) override;

    /**
     * @brief Get the power-on delay setting.
     * @param[out] ms Duration in milliseconds.
     * @return true on success.
     */
    bool getOnDurationMs(uint16_t &ms) const override;

    /**
     * @brief Set the power-off delay (held duration to turn off).
     * @param ms Duration in milliseconds.
     * Allowed value is 4000ms,6000ms,8000ms,10000ms
     * @return true on success.
     */
    bool setOffDurationMs(uint16_t ms) override;

    /**
     * @brief Get the power-off delay setting.
     * @param[out] ms Duration in milliseconds.
     * @return true on success.
     */
    bool getOffDurationMs(uint16_t &ms) const override;

    /**
     * @brief Set the short-press / IRQ trigger delay.
     * @param ms Duration in milliseconds.
     * Allowed value is 1000ms,1500ms,2000ms,2500ms
     * @return true on success.
     */
    bool setIrqDurationMs(uint16_t ms) override;

    /**
     * @brief Get the short-press / IRQ trigger delay.
     * @param[out] ms Duration in milliseconds.
     * @return true on success.
     */
    bool getIrqDurationMs(uint16_t &ms) const override;

private:
    /**
     * @brief Update specific bits in the power-on configuration register.
     * @param mask Bitmask of the field to update.
     * @param value New value for the field.
     * @return true on success.
     */
    bool updateBits(uint8_t mask, uint8_t value);

    AXP2101Core &_core;
};

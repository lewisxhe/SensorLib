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
 * @file      AXP517Pwron.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 * @brief     PWRON button timing control for AXP517.
 */
#pragma once

#include "AXP517Core.hpp"
#include "../../PmicButtonBase.hpp"

/**
 * @brief AXP517 PWRON button configuration driver.
 *
 * Provides debounce and interrupt timing controls for the PMIC PWRON input.
 */
class AXP517Pwron : public PmicButtonBase
{
public:
    /**
     * @brief Construct an AXP517 PWRON button driver.
     * @param core Shared AXP517 register transport.
     */
    explicit AXP517Pwron(AXP517Core &core);

    /**
     * @brief  Set the power-on debounce duration.
     * @note   Allowed values: 128ms, 512ms, 1000ms, 2000ms.
     *         Input is rounded up to the nearest allowed value.
     * @param  ms Desired on-level duration in milliseconds.
     * @retval true on success, false on register access failure.
     */
    bool setOnDurationMs(uint16_t ms) override;

    /**
     * @brief  Get the current power-on debounce duration.
     * @param  ms Output parameter receiving the on-level duration in milliseconds.
     * @retval true on success, false on register access failure.
     */
    bool getOnDurationMs(uint16_t &ms) const override;

    /**
     * @brief  Set the power-off debounce duration.
     * @note   Allowed values: 4000ms, 6000ms, 8000ms, 10000ms.
     *         Input is rounded up to the nearest allowed value.
     * @param  ms Desired off-level duration in milliseconds.
     * @retval true on success, false on register access failure.
     */
    bool setOffDurationMs(uint16_t ms) override;

    /**
     * @brief  Get the current power-off debounce duration.
     * @param  ms Output parameter receiving the off-level duration in milliseconds.
     * @retval true on success, false on register access failure.
     */
    bool getOffDurationMs(uint16_t &ms) const override;

    /**
     * @brief  Set the button IRQ trigger duration.
     * @note   Allowed values: 1000ms, 1500ms, 2000ms, 2500ms.
     *         Input is rounded up to the nearest allowed value.
     * @param  ms Desired IRQ-level duration in milliseconds.
     * @retval true on success, false on register access failure.
     */
    bool setIrqDurationMs(uint16_t ms) override;

    /**
     * @brief  Get the current button IRQ trigger duration.
     * @param  ms Output parameter receiving the IRQ-level duration in milliseconds.
     * @retval true on success, false on register access failure.
     */
    bool getIrqDurationMs(uint16_t &ms) const override;

private:
    /**
     * @brief  Read-modify-write a subset of bits in REG 0x1C.
     * @param  mask Bitmask of bits to modify.
     * @param  value Pre-shifted value to write; only bits in mask are used.
     * @retval true on success, false on register access failure.
     */
    bool updateBits(uint8_t mask, uint8_t value);

private:
    AXP517Core &_core; ///< Shared AXP517 register transport.
};

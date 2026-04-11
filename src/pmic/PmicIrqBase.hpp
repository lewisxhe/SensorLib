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
 * @file      PmicIrqBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 * @brief PMIC interrupt interface.
 *
 * This interface standardizes the most common interrupt controls across PMICs.
 *
 * Notes:
 *  - Some PMICs do not support interrupts; implementations
 *    should return false.
 */
#pragma once

#include <stdint.h>

class PmicIrqBase
{
public:
    virtual ~PmicIrqBase() = default;

    /**
     * @brief  Enable interrupts.
     * @note   The mask value is different for each chip and should be defined in the subclass.
     * @param  mask: Bitmask of interrupts to enable.
     * @retval True on success, false on failure.
     */
    virtual bool enable(uint32_t mask) = 0;

    /**
     * @brief  Disable interrupts.
     * @note   The mask value is different for each chip and should be defined in the subclass.
     * @param  mask: Bitmask of interrupts to disable.
     * @retval True on success, false on failure.
     */
    virtual bool disable(uint32_t mask) = 0;

    /**
     * @brief  Read interrupt status.
     * @param  clear: True to clear the status after reading, false to keep it.
     * @retval Bitmask of active interrupts.
     */
    virtual uint32_t readStatus(bool clear = true) = 0;

    /**
     * @brief  Clear interrupt status.
     * @retval True on success, false on failure.
     */
    virtual bool clear() = 0;
};

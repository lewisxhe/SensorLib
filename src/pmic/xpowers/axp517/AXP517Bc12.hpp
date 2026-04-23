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
 * @file      AXP517Bc12.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#pragma once

#include "../../PmicBc12Base.hpp"
#include "AXP517Core.hpp"

/**
 * @brief AXP517 BC1.2 detection module.
 *
 * Depends on:
 *  - AXP517Core for register IO
 *  - axp517::regs::bmu / axp517::regs::bc12 from AXP517Regs.hpp
 */
class AXP517Bc12 : public PmicBc12Base
{
public:
    explicit AXP517Bc12(AXP517Core &core);
    /**
     * @brief  Enable or disable automatic BC1.2 detection.
     * @note   This function controls the automatic detection feature of the BC1.2 module.
     * @param  enable: true to enable, false to disable.
     * @retval true if the operation is successful, false otherwise.
     */
    bool enableAutoDetect(bool enable) override;
    /**
     * @brief  Trigger a BC1.2 detection cycle.
     * @note   This function forces the PMIC to perform a BC1.2 detection cycle immediately.
     * @retval true if the operation is successful, false otherwise.
     */
    bool triggerDetect() override;
    /**
     * @brief  Check if BC1.2 detection is currently in progress.
     * @note   This function checks the PMIC status to determine if a BC1.2 detection cycle is currently active.
     * @retval true if detection is in progress, false otherwise.
     */
    bool isDetecting() override;
    /**
     * @brief  Read the current BC1.2 detection result.
     * @note   This function retrieves the result of the most recent BC1.2 detection cycle, including the detected port type and raw code.
     * @retval A Result struct containing the detection status, port type, and raw code.
     */
    Result readResult() override;

private:
    // Helper function to map raw PMIC code to common PortType enum
    static PortType mapResult(uint8_t code);
    AXP517Core &_core;
};

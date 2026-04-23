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
 * @file      AXP517Led.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#include "../../PmicLedBase.hpp"
#include "AXP517Core.hpp"

class AXP517Led : public PmicLedBase
{
public:
    explicit AXP517Led(AXP517Core &core);

    ~AXP517Led() = default;

    /**
     * @brief  Set the LED output type.
     * @note   This function sets the output type for the LED.
     * @param  type: The output type to set.
     * @retval True if successful, false otherwise.
     */
    bool setOutputType(OutputType type) override;

    /**
     * @brief  Set the LED mode.
     * @note   This function sets the mode for the LED.
     * @param  mode: The mode to set.
     * @retval True if successful, false otherwise.
     */
    bool setMode(Mode mode) override;

    /**
    * @brief  Get LED operating mode.
    * @note   This retrieves the current LED behavior (e.g. auto/manual).
    * @retval Current operating mode.
    */
    Mode getMode() override;

    /**
     * @brief  Set the LED manual state.
     * @note   This function sets the manual state for the LED.
     * @param  state: The manual state to set.
     * @retval True if successful, false otherwise.
     */
    bool setManualState(ManualState state) override;

    /**
    * @brief  Get manual LED state (only meaningful when Mode::Manual is active).
    * @note   This retrieves the current LED behavior (e.g. on/off/Hi-Z).
    * @retval Current manual state.
    */
    ManualState getManualState() override;

private:
    AXP517Core &_core;
};

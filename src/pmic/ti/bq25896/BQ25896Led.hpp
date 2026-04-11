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
 * @file      BQ25896Led.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-11
 *
 * @brief BQ25896 LED (STAT Pin) Interface
 *
 * This class provides control over the BQ25896 STAT (status) pin, which is
 * an open-drain output that indicates charging status.
 *
 * @section STAT Pin Behavior
 * The STAT pin indicates charging status through different patterns:
 * - High impedance (no pull-up): No charging
 * - Low (pulled to GND): Charging in progress
 * - Blinking (1Hz): Charge done
 * - Blinking (4Hz): Fault condition
 *
 * @section Register
 * - REG07 Bit 6: STAT_DIS - Disable/enable STAT pin function
 *
 * @see BQ25896Charger for charging control
 * @see PmicLedBase for base interface
 */
#pragma once

#include "../../PmicLedBase.hpp"
#include "BQ25896Core.hpp"

/**
 * @brief BQ25896 LED/STAT Pin Control
 */
class BQ25896Led : public PmicLedBase
{
public:
    /**
     * @brief Construct LED interface
     * @param core Reference to BQ25896 core communication
     */
    explicit BQ25896Led(BQ25896Core &core);

    ~BQ25896Led() = default;

    /**
     * @brief Set LED output type
     *
     * Sets the STAT pin output configuration. BQ25896 only supports OpenDrain mode.
     *
     * @param type Output type (OpenDrain or PushPull)
     * @return true on success, false on I2C error
     *
     * @note BQ25896 STAT pin is open-drain; PushPull is not supported
     *       This function always returns false.
     */
    bool setOutputType(OutputType type) override;

    /**
     * @brief Set LED operating mode
     *
     * Controls the STAT pin behavior. BQ25896 supports:
     * - AUTO: Hardware-controlled (charging indicator)
     * - DISABLE: STAT pin disabled
     *
     * @param mode Operating mode
     * @return true on success, false on I2C error
     */
    bool setMode(Mode mode) override;

    /**
     * @brief Get LED operating mode
     * @return Current LED mode (AUTO, DISABLE, or VENDOR on error)
     */
    Mode getMode() override;

    /**
     * @brief Set manual LED state
     *
     * @note BQ25896 does not support manual LED control
     * @return false always
     */
    bool setManualState(ManualState state) override;

    /**
     * @brief Get manual LED state
     * @return UNDEFINED as manual control is not supported
     */
    ManualState getManualState() override;

private:
    BQ25896Core &_core;
};

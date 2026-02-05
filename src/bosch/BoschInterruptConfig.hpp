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
 * @file      BoschInterruptConfig.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-05
 *
 */
#pragma once

#include <stdint.h>

/**
* @brief Interrupt control configuration for BHI260 or BHI360 sensor
*
* Provides a high-level interface for configuring interrupt settings
* with full support for reading/writing raw register values.
*/
struct InterruptConfig {
    // Interrupt source enables (masks)
    bool wakeUpFIFOEnabled = true;          ///< Wake-up FIFO interrupt enabled (bit 0)
    bool nonWakeUpFIFOEnabled = true;       ///< Non-wake-up FIFO interrupt enabled (bit 1)
    bool statusFIFOEnabled = true;          ///< Status FIFO interrupt enabled (bit 2)
    bool debuggingEnabled = false;          ///< Debugging interrupt enabled (bit 3)
    bool faultEnabled = true;               ///< Fault interrupt enabled (bit 4)

    // Interrupt signal characteristics
    enum class Polarity { ACTIVE_HIGH, ACTIVE_LOW };
    enum class TriggerMode { LEVEL, EDGE };
    enum class PinMode { MODE_PUSH_PULL, MODE_OPEN_DRAIN };

    Polarity polarity = Polarity::ACTIVE_HIGH;  ///< Interrupt polarity (bit 5)
    TriggerMode triggerMode = TriggerMode::LEVEL; ///< Trigger mode (bit 6)
    PinMode pinMode = PinMode::MODE_PUSH_PULL;       ///< Interrupt pin mode (bit 7)

    /**
     * @brief Get the current register configuration as a bitmask
     * @return uint8_t register value reflecting current configuration
     *
     * Bits:
     *   0: Wake-up FIFO Interrupt Mask (0=active, 1=masked)
     *   1: Non-Wake-up FIFO Interrupt Mask (0=active, 1=masked)
     *   2: Status FIFO Interrupt Mask (0=active, 1=masked)
     *   3: Debug Interrupt Mask (0=active, 1=masked)
     *   4: Fault Interrupt Mask (0=active, 1=masked)
     *   5: Active Low (0=active high, 1=active low)
     *   6: Edge Trigger (0=level, 1=edge)
     *   7: Open Drain (0=push-pull, 1=open drain)
     */
    uint8_t getRegisterValue() const
    {
        uint8_t reg = 0;

        // Bit 0: Wake-up FIFO (inverted: 0=active, 1=masked)
        if (!wakeUpFIFOEnabled) reg |= (1 << 0);

        // Bit 1: Non-Wake-up FIFO (inverted: 0=active, 1=masked)
        if (!nonWakeUpFIFOEnabled) reg |= (1 << 1);

        // Bit 2: Status FIFO (inverted: 0=active, 1=masked)
        if (!statusFIFOEnabled) reg |= (1 << 2);

        // Bit 3: Debug (inverted: 0=active, 1=masked)
        if (!debuggingEnabled) reg |= (1 << 3);

        // Bit 4: Fault (inverted: 0=active, 1=masked)
        if (!faultEnabled) reg |= (1 << 4);

        // Bit 5: Active Low
        if (polarity == Polarity::ACTIVE_LOW) reg |= (1 << 5);

        // Bit 6: Edge Trigger
        if (triggerMode == TriggerMode::EDGE) reg |= (1 << 6);

        // Bit 7: Open Drain
        if (pinMode == PinMode::MODE_OPEN_DRAIN) reg |= (1 << 7);

        return reg;
    }

    /**
     * @brief Update configuration from register value
     * @param regValue Raw register value to parse
     * @return Reference to self for chaining
     */
    InterruptConfig &setFromRegisterValue(uint8_t regValue)
    {
        // Bit 0: Wake-up FIFO (0=active, 1=masked)
        wakeUpFIFOEnabled = !(regValue & (1 << 0));

        // Bit 1: Non-Wake-up FIFO (0=active, 1=masked)
        nonWakeUpFIFOEnabled = !(regValue & (1 << 1));

        // Bit 2: Status FIFO (0=active, 1=masked)
        statusFIFOEnabled = !(regValue & (1 << 2));

        // Bit 3: Debug (0=active, 1=masked)
        debuggingEnabled = !(regValue & (1 << 3));

        // Bit 4: Fault (0=active, 1=masked)
        faultEnabled = !(regValue & (1 << 4));

        // Bit 5: Active Low
        polarity = (regValue & (1 << 5)) ? Polarity::ACTIVE_LOW : Polarity::ACTIVE_HIGH;

        // Bit 6: Edge Trigger
        triggerMode = (regValue & (1 << 6)) ? TriggerMode::EDGE : TriggerMode::LEVEL;

        // Bit 7: Open Drain
        pinMode = (regValue & (1 << 7)) ? PinMode::MODE_OPEN_DRAIN : PinMode::MODE_PUSH_PULL;

        return *this;
    }

    /**
     * @brief Create InterruptConfig from register value
     * @param regValue Raw register value
     * @return InterruptConfig object
     */
    static InterruptConfig fromRegisterValue(uint8_t regValue)
    {
        InterruptConfig config;
        config.setFromRegisterValue(regValue);
        return config;
    }


    /**
     * @brief Check if configuration would produce given register value
     * @param expectedRegValue Expected register value
     * @return true if configuration matches register value
     */
    bool matchesRegisterValue(uint8_t expectedRegValue) const
    {
        return getRegisterValue() == expectedRegValue;
    }

    /**
     * @brief Get mask of enabled interrupt sources
     * @return Bitmask of enabled sources (bits 0-4, 1=enabled)
     */
    uint8_t getEnabledSourcesMask() const
    {
        uint8_t mask = 0;
        if (wakeUpFIFOEnabled)    mask |= (1 << 0);
        if (nonWakeUpFIFOEnabled) mask |= (1 << 1);
        if (statusFIFOEnabled)    mask |= (1 << 2);
        if (debuggingEnabled)     mask |= (1 << 3);
        if (faultEnabled)         mask |= (1 << 4);
        return mask;
    }

    /**
     * @brief Get mask of interrupt signal configuration
     * @return Bitmask of signal config (bits 5-7)
     */
    uint8_t getSignalConfigMask() const
    {
        uint8_t mask = 0;
        if (polarity == Polarity::ACTIVE_LOW) mask |= (1 << 5);
        if (triggerMode == TriggerMode::EDGE) mask |= (1 << 6);
        if (pinMode == PinMode::MODE_OPEN_DRAIN)   mask |= (1 << 7);
        return mask;
    }

};

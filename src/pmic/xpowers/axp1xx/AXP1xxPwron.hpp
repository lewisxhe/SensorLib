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
 * @file      AXP1xxPwron.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-28
 *
 * @brief AXP192/AXP202 Power-on Key (POK) Control Interface
 *
 * This template class manages the power key press timing configuration
 * via the POK_SET register. The register encodes three independent
 * timing thresholds:
 * - On-level:  press duration to power on (128ms / 512ms / 1s / 2s)
 * - Off-level: press duration to power off (4s / 6s / 8s / 10s)
 * - IRQ-level: press duration to trigger an interrupt (1s / 1.5s / 2s / 2.5s)
 *
 * @tparam Regs Register map struct type (AXP192Regs or AXP202Regs)
 */
#pragma once
#include "../../PmicButtonBase.hpp"
#include "../../../platform/SensorCommWrapper.hpp"

template<typename Regs>
class AXP1xxPwron : public PmicButtonBase
{
public:
    /**
     * @brief Construct power-on key control interface.
     * @param core Reference to the I2C communication wrapper.
     */
    explicit AXP1xxPwron(SensorCommWrapper &core) : _core(core) {}

    /**
     * @brief Set the power-on press duration threshold.
     *
     * Configures POK_SET bits 0-1. The closest valid value is selected.
     *
     * @param ms Desired on-level duration in milliseconds.
     *           Valid values: 128, 512, 1000, 2000.
     * @retval true  On-level was set successfully.
     * @retval false I2C communication failed.
     */
    bool setOnDurationMs(uint16_t ms) override
    {
        uint8_t idx = (ms >= 2000) ? 3 :
                      (ms >= 1000) ? 2 :
                      (ms >= 512)  ? 1 : 0;
        return _core.updateBits(Regs::pmu::POK_SET, 0x03, idx) >= 0;
    }

    /**
     * @brief Get the power-on press duration threshold.
     * @param[out] ms On-level duration in milliseconds.
     * @retval true  Value was read successfully.
     * @retval false I2C communication failed.
     */
    bool getOnDurationMs(uint16_t &ms) const override
    {
        const uint16_t kOnLevelMs[] = {128, 512, 1000, 2000};
        int v = _core.readReg(Regs::pmu::POK_SET);
        if (v < 0) return false;
        ms = kOnLevelMs[static_cast<uint8_t>(v) & 0x03];
        return true;
    }

    /**
     * @brief Set the power-off press duration threshold.
     *
     * Configures POK_SET bits 2-3. The closest valid value is selected.
     *
     * @param ms Desired off-level duration in milliseconds.
     *           Valid values: 4000, 6000, 8000, 10000.
     * @retval true  Off-level was set successfully.
     * @retval false I2C communication failed.
     */
    bool setOffDurationMs(uint16_t ms) override
    {
        uint8_t idx = (ms >= 10000) ? 3 :
                      (ms >= 8000)  ? 2 :
                      (ms >= 6000)  ? 1 : 0;
        return _core.updateBits(Regs::pmu::POK_SET, 0x0C, idx << 2) >= 0;
    }

    /**
     * @brief Get the power-off press duration threshold.
     * @param[out] ms Off-level duration in milliseconds.
     * @retval true  Value was read successfully.
     * @retval false I2C communication failed.
     */
    bool getOffDurationMs(uint16_t &ms) const override
    {
        const uint16_t kOffLevelMs[] = {4000, 6000, 8000, 10000};
        int v = _core.readReg(Regs::pmu::POK_SET);
        if (v < 0) return false;
        ms = kOffLevelMs[(static_cast<uint8_t>(v) >> 2) & 0x03];
        return true;
    }

    /**
     * @brief Set the interrupt-trigger press duration threshold.
     *
     * Configures POK_SET bits 4-5. The closest valid value is selected.
     *
     * @param ms Desired IRQ-level duration in milliseconds.
     *           Valid values: 1000, 1500, 2000, 2500.
     * @retval true  IRQ-level was set successfully.
     * @retval false I2C communication failed.
     */
    bool setIrqDurationMs(uint16_t ms) override
    {
        uint8_t idx = (ms >= 2500) ? 3 :
                      (ms >= 2000) ? 2 :
                      (ms >= 1500) ? 1 : 0;
        return _core.updateBits(Regs::pmu::POK_SET, 0x30, idx << 4) >= 0;
    }

    /**
     * @brief Get the interrupt-trigger press duration threshold.
     * @param[out] ms IRQ-level duration in milliseconds.
     * @retval true  Value was read successfully.
     * @retval false I2C communication failed.
     */
    bool getIrqDurationMs(uint16_t &ms) const override
    {
        const uint16_t kIrqLevelMs[] = {1000, 1500, 2000, 2500};
        int v = _core.readReg(Regs::pmu::POK_SET);
        if (v < 0) return false;
        ms = kIrqLevelMs[(static_cast<uint8_t>(v) >> 4) & 0x03];
        return true;
    }

private:
    SensorCommWrapper &_core;
};

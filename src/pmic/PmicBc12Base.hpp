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
 * @file      PmicBc12Base.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 * @brief BC1.2 / charger type detection capability.
 *
 * A portable interface to start BC1.2 detection and read detected source type.
 * Implementations may support auto-detect or manual trigger.
 * The prerequisite is that the hardware DP/DM is already connected; 
 * otherwise, the status detection is invalid.
 */
#pragma once
#include <stdint.h>

class PmicBc12Base
{
public:
    virtual ~PmicBc12Base() = default;

    /**
     * @brief Common detected port/source type.
     */
    enum class PortType : uint8_t {
        Unknown = 0,
        None,       ///< no valid detection / not connected
        SDP,        ///< Standard Downstream Port (USB 500mA/900mA)
        CDP,        ///< Charging Downstream Port
        DCP,        ///< Dedicated Charging Port
        Apple_1A,   ///< optional
        Apple_2A,   ///< optional
        Apple_2_4A, ///< optional
    };

    struct Result {
        bool detecting = false;  ///< detection in progress
        PortType type = PortType::Unknown;
        uint8_t raw = 0;         ///< raw code from PMIC (chip-specific)
    };

    /**
     * @brief Enable/disable auto BC1.2 detection (if supported).
     */
    virtual bool enableAutoDetect(bool enable) = 0;

    /**
     * @brief Force/trigger a BC1.2 detection cycle (if supported).
     */
    virtual bool triggerDetect() = 0;

    /**
     * @brief Query whether PMIC is currently detecting.
     */
    virtual bool isDetecting() = 0;

    /**
     * @brief Read current BC1.2 result.
     */
    virtual Result readResult() = 0;
};

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
 * @file      AXP517Irq.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#pragma once

#include "../../PmicIrqBase.hpp"
#include "AXP517Core.hpp"

class AXP517Irq : public PmicIrqBase
{
public:
    // ==================== AXP517 IRQ Status 32-bit Masks ====================
    // Bit mapping: [31:24] = Status0, [23:16] = Status1, [15:8] = Status2, [7:0] = Status3

    // --- IRQ Status 0 (REG 48H) ---
    static constexpr uint64_t IRQ_SOC_WARNING_LEVEL      = (1ULL << 31);   ///< SOC drop to Warning Level
    static constexpr uint64_t IRQ_SOC_SHUTDOWN_LEVEL     = (1ULL << 30);   ///< SOC drop to Shutdown Level
    // Bit 29 reserved
    static constexpr uint64_t IRQ_GAUGE_NEW_SOC          = (1ULL << 28);   ///< Gauge New SOC
    static constexpr uint64_t IRQ_CHARGE_TO_NORMAL       = (1ULL << 27);   ///< Charge to normal
    static constexpr uint64_t IRQ_BOOST_OVER_VOLTAGE     = (1ULL << 26);   ///< BOOST Over Voltage
    static constexpr uint64_t IRQ_VBUS_OVER_VOLTAGE      = (1ULL << 25);   ///< VBUS Over Voltage
    static constexpr uint64_t IRQ_VBUS_FAULT             = (1ULL << 24);   ///< VBUS Fault

    // --- IRQ Status 1 (REG 49H) ---
    static constexpr uint64_t IRQ_VBUS_INSERT            = (1ULL << 23);   ///< VBUS Insert
    static constexpr uint64_t IRQ_VBUS_REMOVE            = (1ULL << 22);   ///< VBUS Remove
    static constexpr uint64_t IRQ_BATTERY_INSERT         = (1ULL << 21);   ///< Battery Insert
    static constexpr uint64_t IRQ_BATTERY_REMOVE         = (1ULL << 20);   ///< Battery Remove
    static constexpr uint64_t IRQ_PWR_ON_SHORT_PRESS     = (1ULL << 19);   ///< PWR_ON pin Short PRESS
    static constexpr uint64_t IRQ_PWR_ON_LONG_PRESS      = (1ULL << 18);   ///< PWR_ON pin Long PRESS
    static constexpr uint64_t IRQ_PWR_ON_NEGATIVE_EDGE   = (1ULL << 17);   ///< PWR_ON pin Negative Edge
    static constexpr uint64_t IRQ_PWR_ON_POSITIVE_EDGE   = (1ULL << 16);   ///< PWR_ON pin Positive Edge

    // --- IRQ Status 2 (REG 4AH) ---
    static constexpr uint64_t IRQ_WATCHDOG_EXPIRE        = (1ULL << 15);   ///< Watchdog Expire
    // Bit 14 reserved
    static constexpr uint64_t IRQ_BAT_FET_OCP            = (1ULL << 13);   ///< BAT FET Over Current Protection
    static constexpr uint64_t IRQ_CHARGE_DONE            = (1ULL << 12);   ///< Battery charge done
    static constexpr uint64_t IRQ_CHARGE_START           = (1ULL << 11);   ///< Battery charge start
    static constexpr uint64_t IRQ_DIE_OVER_TEMP_LEVEL1   = (1ULL << 10);   ///< DIE Over Temperature level1
    static constexpr uint64_t IRQ_CHG_SAFETY_TIMER       = (1ULL << 9);    ///< Charger Safety Timer1/2 expire
    static constexpr uint64_t IRQ_BAT_OVER_VOLTAGE       = (1ULL << 8);    ///< Battery Over Voltage Protection

    // --- IRQ Status 3 (REG 4BH) ---
    static constexpr uint64_t IRQ_BC12_DETECT_FINISHED   = (1ULL << 7);    ///< BC1.2 detect finished
    static constexpr uint64_t IRQ_BC12_DETECT_CHANGE     = (1ULL << 6);    ///< BC1.2 detect result change
    // Bit 5 reserved
    static constexpr uint64_t IRQ_BAT_OVER_TEMP_QUIT_CHG = (1ULL << 4);    ///< Battery Over Temperature Quit in Charge mode
    static constexpr uint64_t IRQ_BAT_OVER_TEMP_CHG      = (1ULL << 3);    ///< Battery Over Temperature in Charge mode
    static constexpr uint64_t IRQ_BAT_UNDER_TEMP_CHG     = (1ULL << 2);    ///< Battery Under Temperature in Charge mode
    static constexpr uint64_t IRQ_BAT_OVER_TEMP_WORK     = (1ULL << 1);    ///< Battery Over Temperature in Work mode
    static constexpr uint64_t IRQ_BAT_UNDER_TEMP_WORK    = (1ULL << 0);    ///< Battery Under Temperature in Work mode

    static constexpr uint64_t IRQ_ALL_MASK               = 0xffffffffULL;   ///< Mask for all valid IRQ bits
    static constexpr uint64_t IRQ_RESERVED_MASK = (1ULL << 29) | (1ULL << 14) | (1ULL << 5); ///< Reserved IRQ bits

    
    // *INDENT-OFF*
    ///< IRQ helper functions

    // Check if a specific IRQ is set
    static bool inline isIrqSet(uint64_t irqMask, uint64_t status)  {  return (status & irqMask) != 0; }

    // --- IRQ Status 0 (REG 48H) ---
    static bool inline isSocWarningLevel(uint64_t mask)  { return isIrqSet(IRQ_SOC_WARNING_LEVEL, mask); }
    static bool inline isSocShutdownLevel(uint64_t mask)  { return isIrqSet(IRQ_SOC_SHUTDOWN_LEVEL, mask); }
    static bool inline isGaugeNewSoc(uint64_t mask)  { return isIrqSet(IRQ_GAUGE_NEW_SOC, mask); }
    static bool inline isChargeToNormal(uint64_t mask)  { return isIrqSet(IRQ_CHARGE_TO_NORMAL, mask); }
    static bool inline isBoostOverVoltage(uint64_t mask)  { return isIrqSet(IRQ_BOOST_OVER_VOLTAGE, mask); }
    static bool inline isVbusOverVoltage(uint64_t mask)  { return isIrqSet(IRQ_VBUS_OVER_VOLTAGE, mask); }
    static bool inline isVbusFault(uint64_t mask)  { return isIrqSet(IRQ_VBUS_FAULT, mask); }

    // --- IRQ Status 1 (REG 49H) ---
    static bool inline isVbusInsert(uint64_t mask)  { return isIrqSet(IRQ_VBUS_INSERT, mask); }
    static bool inline isVbusRemove(uint64_t mask)  { return isIrqSet(IRQ_VBUS_REMOVE, mask); }
    static bool inline isBatteryInsert(uint64_t mask)  { return isIrqSet(IRQ_BATTERY_INSERT, mask); }
    static bool inline isBatteryRemove(uint64_t mask)  { return isIrqSet(IRQ_BATTERY_REMOVE, mask); }
    static bool inline isPwrOnShortPress(uint64_t mask)  { return isIrqSet(IRQ_PWR_ON_SHORT_PRESS, mask); }
    static bool inline isPwrOnLongPress(uint64_t mask)  { return isIrqSet(IRQ_PWR_ON_LONG_PRESS, mask); }
    static bool inline isPwrOnNegativeEdge(uint64_t mask)  { return isIrqSet(IRQ_PWR_ON_NEGATIVE_EDGE, mask); }
    static bool inline isPwrOnPositiveEdge(uint64_t mask)  { return isIrqSet(IRQ_PWR_ON_POSITIVE_EDGE, mask); }

    // --- IRQ Status 2 (REG 4AH) ---
    static bool inline isWatchdogExpire(uint64_t mask)  { return isIrqSet(IRQ_WATCHDOG_EXPIRE, mask); }
    static bool inline isBatFetOcp(uint64_t mask)  { return isIrqSet(IRQ_BAT_FET_OCP, mask); }
    static bool inline isChargeDone(uint64_t mask)  { return isIrqSet(IRQ_CHARGE_DONE, mask); }
    static bool inline isChargeStart(uint64_t mask)  { return isIrqSet(IRQ_CHARGE_START, mask); }
    static bool inline isDieOverTempLevel1(uint64_t mask)  { return isIrqSet(IRQ_DIE_OVER_TEMP_LEVEL1, mask); }
    static bool inline isChgSafetyTimer(uint64_t mask)  { return isIrqSet(IRQ_CHG_SAFETY_TIMER, mask); }
    static bool inline isBatOverVoltage(uint64_t mask)  { return isIrqSet(IRQ_BAT_OVER_VOLTAGE, mask); }

    // --- IRQ Status 3 (REG 4BH) ---
    static bool inline isBc12DetectFinished(uint64_t mask)  { return isIrqSet(IRQ_BC12_DETECT_FINISHED, mask); }
    static bool inline isBc12DetectChange(uint64_t mask)  { return isIrqSet(IRQ_BC12_DETECT_CHANGE, mask); }
    static bool inline isBatOverTempQuitChg(uint64_t mask)  { return isIrqSet(IRQ_BAT_OVER_TEMP_QUIT_CHG, mask); }
    static bool inline isBatOverTempChg(uint64_t mask)  { return isIrqSet(IRQ_BAT_OVER_TEMP_CHG, mask); }
    static bool inline isBatUnderTempChg(uint64_t mask)  { return isIrqSet(IRQ_BAT_UNDER_TEMP_CHG, mask); }
    static bool inline isBatOverTempWork(uint64_t mask)  { return isIrqSet(IRQ_BAT_OVER_TEMP_WORK, mask); }
    static bool inline isBatUnderTempWork(uint64_t mask)  { return isIrqSet(IRQ_BAT_UNDER_TEMP_WORK, mask); }
    // *INDENT-ON*

    explicit AXP517Irq(AXP517Core &core);

    ~AXP517Irq() = default;

    /**
     * @brief  Check if a specific IRQ is set.
     * @note   This function checks the status of a specific IRQ bit.
     * @param  mask: The IRQ mask to check.
     * @retval true if the IRQ is set, false otherwise.
     */
    bool enable(uint64_t mask) override;

    /**
     * @brief  Disable a specific IRQ.
     * @note   This function disables a specific IRQ bit.
     *         This setting disables IRQ pin triggering but does not disable interrupt status register updates.
     * @param  mask: The IRQ mask to disable.
     * @retval true on success, false on failure.
     */
    bool disable(uint64_t mask) override;

    /**
     * @brief  Read the current IRQ status.
     * @note   This function reads the current IRQ status register.
     * @param  clear: If true, the status register will be cleared after reading.
     * @retval The current IRQ status.
     */
    uint64_t readStatus(bool clear = true) override;

    /**
     * @brief  Clear the current IRQ status.
     * @note   This function clears the current IRQ status register.
     * @retval true on success, false on failure.
     */
    bool clearStatus() override;

private:
    AXP517Core &_core;
};

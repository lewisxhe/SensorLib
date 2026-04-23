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
    static constexpr uint32_t IRQ_SOC_WARNING_LEVEL      = (1UL << 31);   ///< SOC drop to Warning Level
    static constexpr uint32_t IRQ_SOC_SHUTDOWN_LEVEL     = (1UL << 30);   ///< SOC drop to Shutdown Level
    // Bit 29 reserved
    static constexpr uint32_t IRQ_GAUGE_NEW_SOC          = (1UL << 28);   ///< Gauge New SOC
    static constexpr uint32_t IRQ_CHARGE_TO_NORMAL       = (1UL << 27);   ///< Charge to normal
    static constexpr uint32_t IRQ_BOOST_OVER_VOLTAGE     = (1UL << 26);   ///< BOOST Over Voltage
    static constexpr uint32_t IRQ_VBUS_OVER_VOLTAGE      = (1UL << 25);   ///< VBUS Over Voltage
    static constexpr uint32_t IRQ_VBUS_FAULT             = (1UL << 24);   ///< VBUS Fault

    // --- IRQ Status 1 (REG 49H) ---
    static constexpr uint32_t IRQ_VBUS_INSERT            = (1UL << 23);   ///< VBUS Insert
    static constexpr uint32_t IRQ_VBUS_REMOVE            = (1UL << 22);   ///< VBUS Remove
    static constexpr uint32_t IRQ_BATTERY_INSERT         = (1UL << 21);   ///< Battery Insert
    static constexpr uint32_t IRQ_BATTERY_REMOVE         = (1UL << 20);   ///< Battery Remove
    static constexpr uint32_t IRQ_PWR_ON_SHORT_PRESS     = (1UL << 19);   ///< PWR_ON pin Short PRESS
    static constexpr uint32_t IRQ_PWR_ON_LONG_PRESS      = (1UL << 18);   ///< PWR_ON pin Long PRESS
    static constexpr uint32_t IRQ_PWR_ON_NEGATIVE_EDGE   = (1UL << 17);   ///< PWR_ON pin Negative Edge
    static constexpr uint32_t IRQ_PWR_ON_POSITIVE_EDGE   = (1UL << 16);   ///< PWR_ON pin Positive Edge

    // --- IRQ Status 2 (REG 4AH) ---
    static constexpr uint32_t IRQ_WATCHDOG_EXPIRE        = (1UL << 15);   ///< Watchdog Expire
    // Bit 14 reserved
    static constexpr uint32_t IRQ_BAT_FET_OCP            = (1UL << 13);   ///< BAT FET Over Current Protection
    static constexpr uint32_t IRQ_CHARGE_DONE            = (1UL << 12);   ///< Battery charge done
    static constexpr uint32_t IRQ_CHARGE_START           = (1UL << 11);   ///< Battery charge start
    static constexpr uint32_t IRQ_DIE_OVER_TEMP_LEVEL1   = (1UL << 10);   ///< DIE Over Temperature level1
    static constexpr uint32_t IRQ_CHG_SAFETY_TIMER       = (1UL << 9);    ///< Charger Safety Timer1/2 expire
    static constexpr uint32_t IRQ_BAT_OVER_VOLTAGE       = (1UL << 8);    ///< Battery Over Voltage Protection

    // --- IRQ Status 3 (REG 4BH) ---
    static constexpr uint32_t IRQ_BC12_DETECT_FINISHED   = (1UL << 7);    ///< BC1.2 detect finished
    static constexpr uint32_t IRQ_BC12_DETECT_CHANGE     = (1UL << 6);    ///< BC1.2 detect result change
    // Bit 5 reserved
    static constexpr uint32_t IRQ_BAT_OVER_TEMP_QUIT_CHG = (1UL << 4);    ///< Battery Over Temperature Quit in Charge mode
    static constexpr uint32_t IRQ_BAT_OVER_TEMP_CHG      = (1UL << 3);    ///< Battery Over Temperature in Charge mode
    static constexpr uint32_t IRQ_BAT_UNDER_TEMP_CHG     = (1UL << 2);    ///< Battery Under Temperature in Charge mode
    static constexpr uint32_t IRQ_BAT_OVER_TEMP_WORK     = (1UL << 1);    ///< Battery Over Temperature in Work mode
    static constexpr uint32_t IRQ_BAT_UNDER_TEMP_WORK    = (1UL << 0);    ///< Battery Under Temperature in Work mode

    static constexpr uint32_t IRQ_ALL_MASK               = 0xffffffffU;   ///< Mask for all valid IRQ bits
    static constexpr uint32_t IRQ_RESERVED_MASK = (1UL << 29) | (1UL << 14) | (1UL << 5); ///< Reserved IRQ bits

    
    // *INDENT-OFF*
    ///< IRQ helper functions

    // Check if a specific IRQ is set
    bool inline isIrqSet(uint32_t irqMask, uint32_t status) const {  return (status & irqMask) != 0; }

    // --- IRQ Status 0 (REG 48H) ---
    bool inline isSocWarningLevel(uint32_t mask) const { return isIrqSet(IRQ_SOC_WARNING_LEVEL, mask); }
    bool inline isSocShutdownLevel(uint32_t mask) const { return isIrqSet(IRQ_SOC_SHUTDOWN_LEVEL, mask); }
    bool inline isGaugeNewSoc(uint32_t mask) const { return isIrqSet(IRQ_GAUGE_NEW_SOC, mask); }
    bool inline isChargeToNormal(uint32_t mask) const { return isIrqSet(IRQ_CHARGE_TO_NORMAL, mask); }
    bool inline isBoostOverVoltage(uint32_t mask) const { return isIrqSet(IRQ_BOOST_OVER_VOLTAGE, mask); }
    bool inline isVbusOverVoltage(uint32_t mask) const { return isIrqSet(IRQ_VBUS_OVER_VOLTAGE, mask); }
    bool inline isVbusFault(uint32_t mask) const { return isIrqSet(IRQ_VBUS_FAULT, mask); }

    // --- IRQ Status 1 (REG 49H) ---
    bool inline isVbusInsert(uint32_t mask) const { return isIrqSet(IRQ_VBUS_INSERT, mask); }
    bool inline isVbusRemove(uint32_t mask) const { return isIrqSet(IRQ_VBUS_REMOVE, mask); }
    bool inline isBatteryInsert(uint32_t mask) const { return isIrqSet(IRQ_BATTERY_INSERT, mask); }
    bool inline isBatteryRemove(uint32_t mask) const { return isIrqSet(IRQ_BATTERY_REMOVE, mask); }
    bool inline isPwrOnShortPress(uint32_t mask) const { return isIrqSet(IRQ_PWR_ON_SHORT_PRESS, mask); }
    bool inline isPwrOnLongPress(uint32_t mask) const { return isIrqSet(IRQ_PWR_ON_LONG_PRESS, mask); }
    bool inline isPwrOnNegativeEdge(uint32_t mask) const { return isIrqSet(IRQ_PWR_ON_NEGATIVE_EDGE, mask); }
    bool inline isPwrOnPositiveEdge(uint32_t mask) const { return isIrqSet(IRQ_PWR_ON_POSITIVE_EDGE, mask); }

    // --- IRQ Status 2 (REG 4AH) ---
    bool inline isWatchdogExpire(uint32_t mask) const { return isIrqSet(IRQ_WATCHDOG_EXPIRE, mask); }
    bool inline isBatFetOcp(uint32_t mask) const { return isIrqSet(IRQ_BAT_FET_OCP, mask); }
    bool inline isChargeDone(uint32_t mask) const { return isIrqSet(IRQ_CHARGE_DONE, mask); }
    bool inline isChargeStart(uint32_t mask) const { return isIrqSet(IRQ_CHARGE_START, mask); }
    bool inline isDieOverTempLevel1(uint32_t mask) const { return isIrqSet(IRQ_DIE_OVER_TEMP_LEVEL1, mask); }
    bool inline isChgSafetyTimer(uint32_t mask) const { return isIrqSet(IRQ_CHG_SAFETY_TIMER, mask); }
    bool inline isBatOverVoltage(uint32_t mask) const { return isIrqSet(IRQ_BAT_OVER_VOLTAGE, mask); }

    // --- IRQ Status 3 (REG 4BH) ---
    bool inline isBc12DetectFinished(uint32_t mask) const { return isIrqSet(IRQ_BC12_DETECT_FINISHED, mask); }
    bool inline isBc12DetectChange(uint32_t mask) const { return isIrqSet(IRQ_BC12_DETECT_CHANGE, mask); }
    bool inline isBatOverTempQuitChg(uint32_t mask) const { return isIrqSet(IRQ_BAT_OVER_TEMP_QUIT_CHG, mask); }
    bool inline isBatOverTempChg(uint32_t mask) const { return isIrqSet(IRQ_BAT_OVER_TEMP_CHG, mask); }
    bool inline isBatUnderTempChg(uint32_t mask) const { return isIrqSet(IRQ_BAT_UNDER_TEMP_CHG, mask); }
    bool inline isBatOverTempWork(uint32_t mask) const { return isIrqSet(IRQ_BAT_OVER_TEMP_WORK, mask); }
    bool inline isBatUnderTempWork(uint32_t mask) const { return isIrqSet(IRQ_BAT_UNDER_TEMP_WORK, mask); }
    // *INDENT-ON*

    explicit AXP517Irq(AXP517Core &core);

    ~AXP517Irq() = default;

    /**
     * @brief  Check if a specific IRQ is set.
     * @note   This function checks the status of a specific IRQ bit.
     * @param  mask: The IRQ mask to check.
     * @retval true if the IRQ is set, false otherwise.
     */
    bool enable(uint32_t mask) override;

    /**
     * @brief  Disable a specific IRQ.
     * @note   This function disables a specific IRQ bit.
     *         This setting disables IRQ pin triggering but does not disable interrupt status register updates.
     * @param  mask: The IRQ mask to disable.
     * @retval true on success, false on failure.
     */
    bool disable(uint32_t mask) override;

    /**
     * @brief  Read the current IRQ status.
     * @note   This function reads the current IRQ status register.
     * @param  clear: If true, the status register will be cleared after reading.
     * @retval The current IRQ status.
     */
    uint32_t readStatus(bool clear = true) override;

    /**
     * @brief  Clear the current IRQ status.
     * @note   This function clears the current IRQ status register.
     * @retval true on success, false on failure.
     */
    bool clear() override;

private:
    AXP517Core &_core;
};

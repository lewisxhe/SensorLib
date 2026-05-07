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
 * @file      AXP2101Irq.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-27
 * @brief     AXP2101 Interrupt Controller
 *
 */
#pragma once
#include "../../PmicIrqBase.hpp"
#include "AXP2101Core.hpp"

/**
 * @brief Interrupt controller interface for the AXP2101 PMIC.
 *
 * Manages the three interrupt status registers (REG 0x48-0x4A) and
 * three enable registers (REG 0x40-0x42). Supports 32 interrupt
 * sources covering:
 * - Battery temperature events (JEITA)
 * - VBUS insertion/removal
 * - Battery insertion/removal
 * - Power key press events (short/long/edge)
 * - Battery charge state transitions
 * - Watchdog expiry
 * - Over-current and over-voltage faults
 * - Die over-temperature
 *
 * Interrupt status is returned as a 64-bit mask where each bit
 * corresponds to a specific event. Provides inline query helpers
 * for checking individual interrupt flags.
 */
class AXP2101Irq : public PmicIrqBase
{
public:
    // IRQ Status 0 (REG 48H) -- Battery temperature and gauge events
    /** @brief Battery temperature dropped below the working threshold. */
    static constexpr uint64_t IRQ_BAT_UNDER_TEMP_WORK     = (1ULL << 0);
    /** @brief Battery temperature exceeded the working threshold. */
    static constexpr uint64_t IRQ_BAT_OVER_TEMP_WORK      = (1ULL << 1);
    /** @brief Battery temperature dropped below the charging threshold. */
    static constexpr uint64_t IRQ_BAT_UNDER_TEMP_CHG      = (1ULL << 2);
    /** @brief Battery temperature exceeded the charging threshold. */
    static constexpr uint64_t IRQ_BAT_OVER_TEMP_CHG       = (1ULL << 3);
    /** @brief Coulomb gauge reported a new state-of-charge value. */
    static constexpr uint64_t IRQ_GAUGE_NEW_SOC           = (1ULL << 4);
    /** @brief Coulomb gauge watchdog timer expired. */
    static constexpr uint64_t IRQ_GAUGE_WDT_TIMEOUT       = (1ULL << 5);
    /** @brief Battery voltage crossed warning level 1 threshold. */
    static constexpr uint64_t IRQ_WARNING_LEVEL1          = (1ULL << 6);
    /** @brief Battery voltage crossed warning level 2 threshold. */
    static constexpr uint64_t IRQ_WARNING_LEVEL2          = (1ULL << 7);

    // IRQ Status 1 (REG 49H) -- Power path and key events
    /** @brief Power key rising edge detected. */
    static constexpr uint64_t IRQ_PKEY_POSITIVE_EDGE      = (1ULL << 8);
    /** @brief Power key falling edge detected. */
    static constexpr uint64_t IRQ_PKEY_NEGATIVE_EDGE      = (1ULL << 9);
    /** @brief Power key held for a long press duration. */
    static constexpr uint64_t IRQ_PKEY_LONG_PRESS         = (1ULL << 10);
    /** @brief Power key pressed briefly (short press). */
    static constexpr uint64_t IRQ_PKEY_SHORT_PRESS        = (1ULL << 11);
    /** @brief Battery removed from the system. */
    static constexpr uint64_t IRQ_BAT_REMOVE              = (1ULL << 12);
    /** @brief Battery inserted into the system. */
    static constexpr uint64_t IRQ_BAT_INSERT              = (1ULL << 13);
    /** @brief VBUS power source removed. */
    static constexpr uint64_t IRQ_VBUS_REMOVE             = (1ULL << 14);
    /** @brief VBUS power source inserted. */
    static constexpr uint64_t IRQ_VBUS_INSERT             = (1ULL << 15);

    // IRQ Status 2 (REG 4AH) -- Fault and charge status events
    /** @brief Battery voltage exceeded the over-voltage threshold. */
    static constexpr uint64_t IRQ_BAT_OVER_VOLT           = (1ULL << 16);
    /** @brief Battery charge cycle timed out. */
    static constexpr uint64_t IRQ_CHG_TIMEOUT             = (1ULL << 17);
    /** @brief Die temperature exceeded the thermal shutdown threshold. */
    static constexpr uint64_t IRQ_DIE_OVER_TEMP           = (1ULL << 18);
    /** @brief Battery charging has started. */
    static constexpr uint64_t IRQ_BAT_CHG_START           = (1ULL << 19);
    /** @brief Battery charging has completed. */
    static constexpr uint64_t IRQ_BAT_CHG_DONE            = (1ULL << 20);
    /** @brief BATFET over-current condition detected. */
    static constexpr uint64_t IRQ_BATFET_OVER_CURR        = (1ULL << 21);
    /** @brief LDO output over-current condition detected. */
    static constexpr uint64_t IRQ_LDO_OVER_CURR           = (1ULL << 22);
    /** @brief Watchdog timer expired. */
    static constexpr uint64_t IRQ_WDT_EXPIRE              = (1ULL << 23);

    /** Mask covering all defined interrupt bits. */
    static constexpr uint64_t IRQ_ALL_MASK                = 0xFFFFFFULL;
    /** Mask for reserved / unused interrupt bits. */
    static constexpr uint64_t IRQ_RESERVED_MASK           = ~0xFFFFFFULL;

    // *INDENT-OFF*

    /**
     * @brief Check whether a specific interrupt bit is set in a status word.
     * @param irqMask Bitmask of the interrupt(s) to test.
     * @param status The full status word to check.
     * @return true if any of the specified bits are set.
     */
    static bool inline isIrqSet(uint64_t irqMask, uint64_t status) { return (status & irqMask) != 0; }

    /** @brief Check if battery over-temperature (working range) interrupt is set. */
    static bool inline isBatOverTempWork(uint64_t mask)  { return isIrqSet(IRQ_BAT_OVER_TEMP_WORK, mask); }
    /** @brief Check if battery under-temperature (working range) interrupt is set. */
    static bool inline isBatUnderTempWork(uint64_t mask)  { return isIrqSet(IRQ_BAT_UNDER_TEMP_WORK, mask); }
    /** @brief Check if battery over-temperature (charging range) interrupt is set. */
    static bool inline isBatOverTempChg(uint64_t mask)  { return isIrqSet(IRQ_BAT_OVER_TEMP_CHG, mask); }
    /** @brief Check if battery under-temperature (charging range) interrupt is set. */
    static bool inline isBatUnderTempChg(uint64_t mask)  { return isIrqSet(IRQ_BAT_UNDER_TEMP_CHG, mask); }
    /** @brief Check if gauge new state-of-charge interrupt is set. */
    static bool inline isGaugeNewSoc(uint64_t mask)  { return isIrqSet(IRQ_GAUGE_NEW_SOC, mask); }
    /** @brief Check if gauge watchdog timeout interrupt is set. */
    static bool inline isGaugeWdtTimeout(uint64_t mask)  { return isIrqSet(IRQ_GAUGE_WDT_TIMEOUT, mask); }
    /** @brief Check if battery warning level 1 interrupt is set. */
    static bool inline isWarningLevel1(uint64_t mask)  { return isIrqSet(IRQ_WARNING_LEVEL1, mask); }
    /** @brief Check if battery warning level 2 interrupt is set. */
    static bool inline isWarningLevel2(uint64_t mask)  { return isIrqSet(IRQ_WARNING_LEVEL2, mask); }

    /** @brief Check if VBUS insertion interrupt is set. */
    static bool inline isVbusInsert(uint64_t mask)  { return isIrqSet(IRQ_VBUS_INSERT, mask); }
    /** @brief Check if VBUS removal interrupt is set. */
    static bool inline isVbusRemove(uint64_t mask)  { return isIrqSet(IRQ_VBUS_REMOVE, mask); }
    /** @brief Check if battery insertion interrupt is set. */
    static bool inline isBatInsert(uint64_t mask)  { return isIrqSet(IRQ_BAT_INSERT, mask); }
    /** @brief Check if battery removal interrupt is set. */
    static bool inline isBatRemove(uint64_t mask)  { return isIrqSet(IRQ_BAT_REMOVE, mask); }
    /** @brief Check if power key short press interrupt is set. */
    static bool inline isPkeyShortPress(uint64_t mask)  { return isIrqSet(IRQ_PKEY_SHORT_PRESS, mask); }
    /** @brief Check if power key long press interrupt is set. */
    static bool inline isPkeyLongPress(uint64_t mask)  { return isIrqSet(IRQ_PKEY_LONG_PRESS, mask); }
    /** @brief Check if power key negative (falling) edge interrupt is set. */
    static bool inline isPkeyNegativeEdge(uint64_t mask)  { return isIrqSet(IRQ_PKEY_NEGATIVE_EDGE, mask); }
    /** @brief Check if power key positive (rising) edge interrupt is set. */
    static bool inline isPkeyPositiveEdge(uint64_t mask)  { return isIrqSet(IRQ_PKEY_POSITIVE_EDGE, mask); }

    /** @brief Check if watchdog timer expired interrupt is set. */
    static bool inline isWdtExpire(uint64_t mask)  { return isIrqSet(IRQ_WDT_EXPIRE, mask); }
    /** @brief Check if LDO over-current interrupt is set. */
    static bool inline isLdoOverCurr(uint64_t mask)  { return isIrqSet(IRQ_LDO_OVER_CURR, mask); }
    /** @brief Check if BATFET over-current interrupt is set. */
    static bool inline isBatfetOverCurr(uint64_t mask)  { return isIrqSet(IRQ_BATFET_OVER_CURR, mask); }
    /** @brief Check if battery charge done interrupt is set. */
    static bool inline isBatChgDone(uint64_t mask)  { return isIrqSet(IRQ_BAT_CHG_DONE, mask); }
    /** @brief Check if battery charge start interrupt is set. */
    static bool inline isBatChgStart(uint64_t mask)  { return isIrqSet(IRQ_BAT_CHG_START, mask); }
    /** @brief Check if die over-temperature interrupt is set. */
    static bool inline isDieOverTemp(uint64_t mask)  { return isIrqSet(IRQ_DIE_OVER_TEMP, mask); }
    /** @brief Check if charge timeout interrupt is set. */
    static bool inline isChgTimeout(uint64_t mask)  { return isIrqSet(IRQ_CHG_TIMEOUT, mask); }
    /** @brief Check if battery over-voltage interrupt is set. */
    static bool inline isBatOverVolt(uint64_t mask)  { return isIrqSet(IRQ_BAT_OVER_VOLT, mask); }

    // *INDENT-ON*

    explicit AXP2101Irq(AXP2101Core &core);
    ~AXP2101Irq() = default;

    /**
     * @brief Enable one or more interrupt sources.
     * @param mask Bitmask of interrupts to enable.
     * @return true on success.
     */
    bool enable(uint64_t mask) override;

    /**
     * @brief Disable one or more interrupt sources.
     * @param mask Bitmask of interrupts to disable.
     * @return true on success.
     */
    bool disable(uint64_t mask) override;

    /**
     * @brief Read the current interrupt status.
     *
     * Optionally clears all pending interrupts after reading.
     *
     * @param clear If true, clear status registers after read (default: true).
     * @return 64-bit mask of pending interrupts.
     */
    uint64_t readStatus(bool clear = true) override;

    /**
     * @brief Clear all pending interrupt status registers.
     * @return true on success.
     */
    bool clearStatus() override;

private:
    AXP2101Core &_core;
};

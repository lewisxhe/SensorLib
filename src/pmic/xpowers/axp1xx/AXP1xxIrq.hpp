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
 * @file      AXP1xxIrq.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-28
 *
 * @brief AXP192/AXP202 Interrupt Controller Interface
 *
 * This template class manages the five interrupt status and enable
 * registers (INTSTS1-5 / ENABLE1-5). It provides:
 * - Bitmask constants for all supported interrupt sources
 * - Static helper methods to decode which interrupts are active
 * - Runtime enable/disable and read/clear operations
 *
 * Interrupt sources are grouped by register:
 * - INTSTS1 (bits 0-7):  ACIN, VBUS events
 * - INTSTS2 (bits 8-15): Battery events
 * - INTSTS3 (bits 16-23): Temperature, voltage, PEK key
 * - INTSTS4 (bits 24-31): N-oe, VBUS session, low voltage
 * - INTSTS5 (bits 32-39): Watchdog, GPIO edge triggers
 *
 * Chip support annotation convention:
 *   [AXP192+AXP202]  = shared by both chips
 *   [AXP202 only]    = AXP202-specific, reserved/unused on AXP192
 *
 * AXP192 naming aliases are provided for compatibility with the
 * legacy XPowersLib API.
 *
 * @tparam Regs Register map struct type (AXP192Regs or AXP202Regs)
 */
#pragma once
#include "../../PmicIrqBase.hpp"
#include "../../../platform/SensorCommWrapper.hpp"

template<typename Regs>
class AXP1xxIrq : public PmicIrqBase
{
public:
    // ========================================================================
    // INTSTS1 -> bits 0-7  (STATUS1: AXP192=REG44H, AXP202=REG48H)
    // All shared [AXP192+AXP202]
    // ========================================================================
    /** @brief ACIN overvoltage interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_ACIN_OV           = (1ULL << 7);
    /** @brief ACIN insertion interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_ACIN_INSERT        = (1ULL << 6);
    /** @brief ACIN removal interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_ACIN_REMOVE        = (1ULL << 5);
    /** @brief VBUS overvoltage interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_VBUS_OV            = (1ULL << 4);
    /** @brief VBUS insertion interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_VBUS_INSERT        = (1ULL << 3);
    /** @brief VBUS removal interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_VBUS_REMOVE        = (1ULL << 2);
    /** @brief VBUS low VHOLD interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_VBUS_LOW_VHOLD     = (1ULL << 1);

    // ========================================================================
    // INTSTS2 -> bits 8-15  (STATUS2: AXP192=REG45H, AXP202=REG49H)
    // All shared [AXP192+AXP202]
    // ========================================================================
    /** @brief Battery insertion interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_BAT_INSERT         = (1ULL << 15);
    /** @brief Battery removal interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_BAT_REMOVE         = (1ULL << 14);
    /** @brief Battery entered active mode interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_BAT_ENTER_ACTIVATE = (1ULL << 13);
    /** @brief Battery exited active mode interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_BAT_EXIT_ACTIVATE  = (1ULL << 12);
    /** @brief Charging started interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_BAT_CHG_START      = (1ULL << 11);
    /** @brief Charging completed interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_BAT_CHG_DONE       = (1ULL << 10);
    /** @brief Battery temperature high interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_BAT_TEMP_HIGH      = (1ULL << 9);
    /** @brief Battery temperature low interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_BAT_TEMP_LOW       = (1ULL << 8);

    // ========================================================================
    // INTSTS3 -> bits 16-23  (STATUS3: AXP192=REG46H, AXP202=REG4AH)
    // ========================================================================
    /** @brief Chip overtemperature interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_CHIP_OV_TEMP       = (1ULL << 23);
    /** @brief Charge current less than target interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_CHG_CURR_LESS      = (1ULL << 22);
    /**
     * @brief DC-DC1 voltage less than target interrupt mask. [AXP192+AXP202]
     *
     * @note AXP202 datasheet inconsistency: REG4AH IRQ table lists IRQ18
     * (DCDC1 voltage too low) at bit 5, but REG42H (IRQ enable3) describes
     * bit 5 as "Reserved and unchangeable". This code follows the IRQ table
     * definition. Verify on actual hardware that this interrupt can be
     * enabled and triggered on AXP202.
     */
    static constexpr uint64_t IRQ_DC1_VOLT_LESS      = (1ULL << 21);
    /** @brief DC-DC2 voltage less than target interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_DC2_VOLT_LESS      = (1ULL << 20);
    /** @brief DC-DC3 voltage less than target interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_DC3_VOLT_LESS      = (1ULL << 19);
    // NOTE: STATUS3 bit 2 is Reserved on both AXP192 (REG46H[2]) and
    // AXP202 (REG4AH[2]).
    /** @brief PEK short press interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_PEKEY_SHORT_PRESS  = (1ULL << 17);
    /** @brief PEK long press interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_PEKEY_LONG_PRESS   = (1ULL << 16);

    // ========================================================================
    // INTSTS4 -> bits 24-31  (STATUS4: AXP192=REG47H, AXP202=REG4BH)
    // ========================================================================
    /** @brief N-oe power-on interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_NOE_POWERON        = (1ULL << 31);
    /** @brief N-oe power-down interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_NOE_POWERDOWN      = (1ULL << 30);
    /** @brief VBUS effective interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_VBUS_EFFECTIVE     = (1ULL << 29);
    /** @brief VBUS invalid interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_VBUS_INVALID       = (1ULL << 28);
    /** @brief VBUS session request interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_VBUS_SESSION       = (1ULL << 27);
    /** @brief VBUS session end interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_VBUS_SESSION_END   = (1ULL << 26);
    /** @brief Low voltage level 1 interrupt mask. [AXP202 only] */
    static constexpr uint64_t IRQ_LOW_VOLT_LEVEL1    = (1ULL << 25);
    /** @brief Low voltage level 2 interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_LOW_VOLT_LEVEL2    = (1ULL << 24);

    // ========================================================================
    // INTSTS5 -> bits 32-39  (STATUS5: AXP192=REG4DH, AXP202=REG4CH)
    // ========================================================================
    /** @brief GPIO0 edge trigger interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_GPIO0_EDGE         = (1ULL << 32);
    /** @brief GPIO1 edge trigger interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_GPIO1_EDGE         = (1ULL << 33);
    /** @brief GPIO2 edge trigger interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_GPIO2_EDGE         = (1ULL << 34);
    /** @brief GPIO3 edge trigger interrupt mask. [AXP202 only] */
    static constexpr uint64_t IRQ_GPIO3_EDGE         = (1ULL << 35);
    /** @brief PEK press falling edge interrupt mask. [AXP202 only] */
    static constexpr uint64_t IRQ_PKEY_NEGATIVE      = (1ULL << 37);
    /** @brief PEK press rising edge interrupt mask. [AXP202 only] */
    static constexpr uint64_t IRQ_PKEY_POSITIVE      = (1ULL << 38);
    /** @brief Timer timeout interrupt mask. [AXP192+AXP202] */
    static constexpr uint64_t IRQ_TIMER_TIMEOUT      = (1ULL << 39);

    /** @brief Mask covering all defined interrupt sources (bits 1-39). */
    static constexpr uint64_t IRQ_ALL_MASK           = 0xFFFFFFFFFFULL;

    // *INDENT-OFF*
    /**
     * @brief Check if a specific interrupt is set in a status mask.
     * @param irqMask Bitmask of the interrupt of interest.
     * @param status  Full status mask from readStatus().
     * @return true if the interrupt is active.
     */
    static bool isIrqSet(uint64_t irqMask, uint64_t status) { return (status & irqMask) != 0; }

    // ========================================================================
    // INTSTS1 interrupt check functions [AXP192+AXP202]
    // ========================================================================
    /** @brief Check ACIN overvoltage interrupt. */
    static bool isAcinOv(uint64_t m) { return isIrqSet(IRQ_ACIN_OV, m); }
    /** @brief Check ACIN insertion interrupt. */
    static bool isAcinInsert(uint64_t m) { return isIrqSet(IRQ_ACIN_INSERT, m); }
    /** @brief Check ACIN removal interrupt. */
    static bool isAcinRemove(uint64_t m) { return isIrqSet(IRQ_ACIN_REMOVE, m); }
    /** @brief Check VBUS overvoltage interrupt. */
    static bool isVbusOv(uint64_t m) { return isIrqSet(IRQ_VBUS_OV, m); }
    /** @brief Check VBUS insertion interrupt. */
    static bool isVbusInsert(uint64_t m) { return isIrqSet(IRQ_VBUS_INSERT, m); }
    /** @brief Check VBUS removal interrupt. */
    static bool isVbusRemove(uint64_t m) { return isIrqSet(IRQ_VBUS_REMOVE, m); }
    /** @brief Check VBUS low VHOLD interrupt. */
    static bool isVbusLowVhold(uint64_t m) { return isIrqSet(IRQ_VBUS_LOW_VHOLD, m); }

    // ========================================================================
    // INTSTS2 interrupt check functions [AXP192+AXP202]
    // ========================================================================
    /** @brief Check battery insertion interrupt. */
    static bool isBatInsert(uint64_t m) { return isIrqSet(IRQ_BAT_INSERT, m); }
    /** @brief Check battery removal interrupt. */
    static bool isBatRemove(uint64_t m) { return isIrqSet(IRQ_BAT_REMOVE, m); }
    /** @brief Check battery entered active mode interrupt. */
    static bool isBatEnterActivate(uint64_t m) { return isIrqSet(IRQ_BAT_ENTER_ACTIVATE, m); }
    /** @brief Check battery exited active mode interrupt. */
    static bool isBatExitActivate(uint64_t m) { return isIrqSet(IRQ_BAT_EXIT_ACTIVATE, m); }
    /** @brief Check charging started interrupt. */
    static bool isBatChgStart(uint64_t m) { return isIrqSet(IRQ_BAT_CHG_START, m); }
    /** @brief Check charging completed interrupt. */
    static bool isBatChgDone(uint64_t m) { return isIrqSet(IRQ_BAT_CHG_DONE, m); }
    /** @brief Check battery temperature high interrupt. */
    static bool isBatTempHigh(uint64_t m) { return isIrqSet(IRQ_BAT_TEMP_HIGH, m); }
    /** @brief Check battery temperature low interrupt. */
    static bool isBatTempLow(uint64_t m) { return isIrqSet(IRQ_BAT_TEMP_LOW, m); }

    // ========================================================================
    // INTSTS3 interrupt check functions [AXP192+AXP202]
    // ========================================================================
    /** @brief Check chip overtemperature interrupt. */
    static bool isChipOvTemp(uint64_t m) { return isIrqSet(IRQ_CHIP_OV_TEMP, m); }
    /** @brief Check charge current less than target interrupt. */
    static bool isChgCurrLess(uint64_t m) { return isIrqSet(IRQ_CHG_CURR_LESS, m); }
    /** @brief Check DC-DC1 voltage low interrupt. */
    static bool isDc1VoltLess(uint64_t m) { return isIrqSet(IRQ_DC1_VOLT_LESS, m); }
    /** @brief Check DC-DC2 voltage low interrupt. */
    static bool isDc2VoltLess(uint64_t m) { return isIrqSet(IRQ_DC2_VOLT_LESS, m); }
    /** @brief Check DC-DC3 voltage low interrupt. */
    static bool isDc3VoltLess(uint64_t m) { return isIrqSet(IRQ_DC3_VOLT_LESS, m); }
    /** @brief Check PEK short press interrupt. */
    static bool isPekeyShortPress(uint64_t m) { return isIrqSet(IRQ_PEKEY_SHORT_PRESS, m); }
    /** @brief Check PEK long press interrupt. */
    static bool isPekeyLongPress(uint64_t m) { return isIrqSet(IRQ_PEKEY_LONG_PRESS, m); }

    // ========================================================================
    // INTSTS4 interrupt check functions
    // ========================================================================
    /** @brief Check N-oe power-on interrupt. [AXP192+AXP202] */
    static bool isNoePoweron(uint64_t m) { return isIrqSet(IRQ_NOE_POWERON, m); }
    /** @brief Check N-oe power-down interrupt. [AXP192+AXP202] */
    static bool isNoePowerdown(uint64_t m) { return isIrqSet(IRQ_NOE_POWERDOWN, m); }
    /** @brief Check VBUS effective interrupt. [AXP192+AXP202] */
    static bool isVbusEffective(uint64_t m) { return isIrqSet(IRQ_VBUS_EFFECTIVE, m); }
    /** @brief Check VBUS invalid interrupt. [AXP192+AXP202] */
    static bool isVbusInvalid(uint64_t m) { return isIrqSet(IRQ_VBUS_INVALID, m); }
    /** @brief Check VBUS session request interrupt. [AXP192+AXP202] */
    static bool isVbusSession(uint64_t m) { return isIrqSet(IRQ_VBUS_SESSION, m); }
    /** @brief Check VBUS session end interrupt. [AXP192+AXP202] */
    static bool isVbusSessionEnd(uint64_t m) { return isIrqSet(IRQ_VBUS_SESSION_END, m); }
    /** @brief Check low voltage level 2 interrupt. [AXP192+AXP202] */
    static bool isLowVoltLevel2(uint64_t m) { return isIrqSet(IRQ_LOW_VOLT_LEVEL2, m); }
    /** @brief Check low voltage level 1 interrupt. [AXP202 only] */
    static bool isLowVoltLevel1(uint64_t m) { return isIrqSet(IRQ_LOW_VOLT_LEVEL1, m); }

    // ========================================================================
    // INTSTS5 interrupt check functions
    // ========================================================================
    /** @brief Check GPIO0 edge trigger interrupt. [AXP192+AXP202] */
    static bool isGpio0EdgeTrigger(uint64_t m) { return isIrqSet(IRQ_GPIO0_EDGE, m); }
    /** @brief Check GPIO1 edge trigger interrupt. [AXP192+AXP202] */
    static bool isGpio1EdgeTrigger(uint64_t m) { return isIrqSet(IRQ_GPIO1_EDGE, m); }
    /** @brief Check GPIO2 edge trigger interrupt. [AXP192+AXP202] */
    static bool isGpio2EdgeTrigger(uint64_t m) { return isIrqSet(IRQ_GPIO2_EDGE, m); }
    /** @brief Check GPIO3 edge trigger interrupt. [AXP202 only] */
    static bool isGpio3EdgeTrigger(uint64_t m) { return isIrqSet(IRQ_GPIO3_EDGE, m); }
    /** @brief Check PEK press falling edge interrupt. [AXP202 only] */
    static bool isPkeyNegative(uint64_t m) { return isIrqSet(IRQ_PKEY_NEGATIVE, m); }
    /** @brief Check PEK press rising edge interrupt. [AXP202 only] */
    static bool isPkeyPositive(uint64_t m) { return isIrqSet(IRQ_PKEY_POSITIVE, m); }
    /** @brief Check timer timeout interrupt. [AXP192+AXP202] */
    static bool isTimerTimeout(uint64_t m) { return isIrqSet(IRQ_TIMER_TIMEOUT, m); }

    // ========================================================================
    // AXP192 naming aliases (legacy XPowersLib compatibility)
    // ========================================================================
    /** @brief AXP192 alias for isAcinOv(). */
    static bool isAcinOverVol(uint64_t m) { return isAcinOv(m); }
    /** @brief AXP192 alias for isAcinInsert(). */
    static bool isAcinConnect(uint64_t m) { return isAcinInsert(m); }
    /** @brief AXP192 alias for isVbusOv(). */
    static bool isVbusOverVol(uint64_t m) { return isVbusOv(m); }
    /** @brief AXP192 alias for isVbusLowVhold(). */
    static bool isVbusVholdLow(uint64_t m) { return isVbusLowVhold(m); }
    /** @brief AXP192 alias for isBatEnterActivate(). */
    static bool isBattActivate(uint64_t m) { return isBatEnterActivate(m); }
    /** @brief AXP192 alias for isBatExitActivate(). */
    static bool isBattExitActivate(uint64_t m) { return isBatExitActivate(m); }
    /** @brief AXP192 alias for isBatTempHigh(). */
    static bool isBattOverTemp(uint64_t m) { return isBatTempHigh(m); }
    /** @brief AXP192 alias for isBatTempLow(). */
    static bool isBattLowTemp(uint64_t m) { return isBatTempLow(m); }
    /** @brief AXP192 alias for isChipOvTemp(). */
    static bool isChipTempHigh(uint64_t m) { return isChipOvTemp(m); }
    /** @brief AXP192 alias for isChgCurrLess(). */
    static bool isChargeLowCur(uint64_t m) { return isChgCurrLess(m); }
    /** @brief AXP192 alias for isDc1VoltLess(). */
    static bool isDc1LowVol(uint64_t m) { return isDc1VoltLess(m); }
    /** @brief AXP192 alias for isDc2VoltLess(). */
    static bool isDc2LowVol(uint64_t m) { return isDc2VoltLess(m); }
    /** @brief AXP192 alias for isDc3VoltLess(). */
    static bool isDc3LowVol(uint64_t m) { return isDc3VoltLess(m); }
    /** @brief AXP192 alias for isPekeyShortPress(). */
    static bool isPkeyShortPress(uint64_t m) { return isPekeyShortPress(m); }
    /** @brief AXP192 alias for isPekeyLongPress(). */
    static bool isPkeyLongPress(uint64_t m) { return isPekeyLongPress(m); }
    /** @brief AXP192 alias for isNoePoweron(). */
    static bool isNoeOn(uint64_t m) { return isNoePoweron(m); }
    /** @brief AXP192 alias for isNoePowerdown(). */
    static bool isNoeOff(uint64_t m) { return isNoePowerdown(m); }
    /** @brief AXP192 alias for isVbusEffective(). */
    static bool isVbusValid(uint64_t m) { return isVbusEffective(m); }
    /** @brief AXP192 alias for isVbusSession(). */
    static bool isVbusSessionAB(uint64_t m) { return isVbusSession(m); }
    /** @brief AXP192 alias for isLowVoltLevel2(). */
    static bool isApsLowVolLevel(uint64_t m) { return isLowVoltLevel2(m); }
    /** @brief AXP192 alias for isGpio0EdgeTrigger(). */
    static bool isGpio0Irq(uint64_t m) { return isGpio0EdgeTrigger(m); }
    /** @brief AXP192 alias for isGpio1EdgeTrigger(). */
    static bool isGpio1Irq(uint64_t m) { return isGpio1EdgeTrigger(m); }
    /** @brief AXP192 alias for isGpio2EdgeTrigger(). */
    static bool isGpio2Irq(uint64_t m) { return isGpio2EdgeTrigger(m); }
    /** @brief AXP192 alias for isTimerTimeout(). */
    static bool isTimerIrq(uint64_t m) { return isTimerTimeout(m); }

    // ========================================================================
    // AXP202 naming aliases (legacy XPowersLib compatibility)
    // ========================================================================
    /** @brief AXP202 alias for isLowVoltLevel1(). */
    static bool isApsLowVolLevel1(uint64_t m) { return isLowVoltLevel1(m); }
    /** @brief AXP202 alias for isGpio3EdgeTrigger(). */
    static bool isGpio3Irq(uint64_t m) { return isGpio3EdgeTrigger(m); }

    // *INDENT-ON*

    /**
     * @brief Construct interrupt controller interface.
     * @param core Reference to the I2C communication wrapper.
     */
    explicit AXP1xxIrq(SensorCommWrapper &core) : _core(core) {}

    /**
     * @brief Enable one or more interrupt sources.
     *
     * Reads the current ENABLE1-5 registers, ORs the mask, and writes back.
     *
     * @param mask Bitmask of interrupts to enable (use IRQ_* constants).
     * @retval true  Interrupts were enabled successfully.
     * @retval false I2C communication failed.
     */
    bool enable(uint64_t mask) override;

    /**
     * @brief Disable one or more interrupt sources.
     *
     * Reads the current ENABLE1-5 registers, clears the mask, and writes back.
     *
     * @param mask Bitmask of interrupts to disable (use IRQ_* constants).
     * @retval true  Interrupts were disabled successfully.
     * @retval false I2C communication failed.
     */
    bool disable(uint64_t mask) override;

    /**
     * @brief Read and optionally clear interrupt status.
     *
     * Reads INTSTS1-5 registers and packs them into a 64-bit mask.
     * If clear is true and status is non-zero, only the status bits that
     * were read are cleared (W1C safe), avoiding loss of newly arrived
     * interrupts.
     *
     * @param clear If true, clear only the read interrupt bits after reading.
     * @return 64-bit status mask (0 on error or if no interrupts pending).
     */
    uint64_t readStatus(bool clear = true) override;

    /**
     * @brief Clear all pending interrupt status registers.
     *
     * Writes 0xFF to INTSTS1-4 and 0xFF to INTSTS5.
     *
     * @retval true  Status was cleared successfully.
     * @retval false I2C communication failed.
     */
    bool clearStatus() override;

private:
    /**
     * @brief Read ENABLE1-5 register bytes into a 5-byte buffer.
     * @param buf Output buffer for ENABLE1..ENABLE5.
     * @return true on success, false on I2C error.
     */
    bool readEnableRegs(uint8_t buf[5]);

    /**
     * @brief Write ENABLE1-5 registers per-byte only when changed.
     * @note Must not use continuous writes for these registers.
     * @param before Previous ENABLE register bytes.
     * @param after New ENABLE register bytes.
     * @return true on success, false on I2C error.
     */
    bool writeEnableRegsIfChanged(const uint8_t before[5], const uint8_t after[5]);

    /**
     * @brief Clear STATUS1-5 using W1C semantics with per-byte writes.
     * @note Must not use continuous writes for these registers.
     * @param maskBytes Bits to clear for STATUS1..STATUS5.
     * @return true on success, false on I2C error.
     */
    bool clearStatusByMask(const uint8_t maskBytes[5]);

    /**
     * @brief Clear all STATUS1-5 bits (write 0xFF per byte).
     * @return true on success, false on I2C error.
     */
    bool clearAllStatusRegs();

    /**
     * @brief Pack a 5-byte register buffer into a 64-bit value.
     * @param buf Pointer to 5-byte buffer.
     * @return 64-bit packed value.
     */
    static uint64_t pack5(const uint8_t buf[5]);

    /**
     * @brief Unpack a 64-bit value into a 5-byte register buffer.
     * @param val 64-bit value to unpack.
     * @param buf Pointer to 5-byte output buffer.
     */
    static void unpack5(uint64_t val, uint8_t buf[5]);

    SensorCommWrapper &_core;
};

template<typename Regs>
bool AXP1xxIrq<Regs>::readEnableRegs(uint8_t buf[5])
{
    if (_core.readRegBuff(Regs::irq::ENABLE1, buf, 4) < 0) {
        return false;
    }

    int en5 = _core.readReg(Regs::irq::ENABLE5);
    if (en5 < 0) {
        return false;
    }
    buf[4] = static_cast<uint8_t>(en5);
    return true;
}

template<typename Regs>
bool AXP1xxIrq<Regs>::writeEnableRegsIfChanged(const uint8_t before[5], const uint8_t after[5])
{
    // Warning: It cannot be optimized to continuous writing, otherwise an error will occur.
    for (int i = 0; i < 4; ++i) {
        if (before[i] != after[i]) {
            if (_core.writeReg(Regs::irq::ENABLE1 + i, after[i]) < 0) {
                return false;
            }
        }
    }

    if (before[4] != after[4]) {
        if (_core.writeReg(Regs::irq::ENABLE5, after[4]) < 0) {
            return false;
        }
    }
    return true;
}

template<typename Regs>
bool AXP1xxIrq<Regs>::clearStatusByMask(const uint8_t maskBytes[5])
{
    // Warning: It cannot be optimized to continuous writing, otherwise an error will occur.
    for (int i = 0; i < 4; ++i) {
        if (maskBytes[i] != 0) {
            if (_core.writeReg(Regs::irq::STATUS1 + i, maskBytes[i]) < 0) {
                return false;
            }
        }
    }

    if (maskBytes[4] != 0) {
        if (_core.writeReg(Regs::irq::STATUS5, maskBytes[4]) < 0) {
            return false;
        }
    }
    return true;
}

template<typename Regs>
bool AXP1xxIrq<Regs>::clearAllStatusRegs()
{
    const uint8_t all[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    return clearStatusByMask(all);
}

template<typename Regs>
inline uint64_t AXP1xxIrq<Regs>::pack5(const uint8_t buf[5])
{
    return (static_cast<uint64_t>(buf[0])) |
           (static_cast<uint64_t>(buf[1]) << 8) |
           (static_cast<uint64_t>(buf[2]) << 16) |
           (static_cast<uint64_t>(buf[3]) << 24) |
           (static_cast<uint64_t>(buf[4]) << 32);
}

template<typename Regs>
inline void AXP1xxIrq<Regs>::unpack5(uint64_t val, uint8_t buf[5])
{
    buf[0] = static_cast<uint8_t>(val & 0xFF);
    buf[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>((val >> 16) & 0xFF);
    buf[3] = static_cast<uint8_t>((val >> 24) & 0xFF);
    buf[4] = static_cast<uint8_t>((val >> 32) & 0xFF);
}

template<typename Regs>
bool AXP1xxIrq<Regs>::enable(uint64_t mask)
{
    uint8_t before[5], after[5];
    if (!readEnableRegs(before)) {
        return false;
    }

    uint64_t current = pack5(before);
    current |= mask;
    unpack5(current, after);

    return writeEnableRegsIfChanged(before, after);
}

template<typename Regs>
bool AXP1xxIrq<Regs>::disable(uint64_t mask)
{
    uint8_t before[5], after[5];
    if (!readEnableRegs(before)) {
        return false;
    }

    uint64_t current = pack5(before);
    current &= ~mask;
    unpack5(current, after);

    return writeEnableRegsIfChanged(before, after);
}

template<typename Regs>
uint64_t AXP1xxIrq<Regs>::readStatus(bool clear)
{
    uint8_t buf[5];
    if (_core.readRegBuff(Regs::irq::STATUS1, buf, 4) < 0) {
        return 0;
    }
    int st5 = _core.readReg(Regs::irq::STATUS5);
    if (st5 < 0) return 0;
    buf[4] = static_cast<uint8_t>(st5);

    uint64_t mask = pack5(buf);
    if (clear && mask != 0) {
        // W1C safe: only write back the bits that were actually read,
        // so newly arrived interrupts between read and clear are preserved.
        if (!clearStatusByMask(buf)) {
            return 0;
        }
    }
    return mask;
}

template<typename Regs>
bool AXP1xxIrq<Regs>::clearStatus()
{
    return clearAllStatusRegs();
}

/**
 * @file      AXP202Coulomb.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-05-02
 * @brief     AXP202 Coulomb counter for battery charge/discharge measurement
 *
 * Provides 32-bit charge and discharge coulomb counters for accurate
 * battery energy tracking. The formula is:
 *   C = 65536 * current_LSB * (charge - discharge) / 3600 / sample_rate
 * where current_LSB = 0.5mA, result in mAh.
 *
 */
#pragma once
#include "AXP202Core.hpp"
#include "AXP202Regs.hpp"

class AXP202Coulomb
{
public:
    explicit AXP202Coulomb(AXP202Core &core) : _core(core) {}

    /**
     * @brief Enable coulomb counter (REG B8H bit 7)
     * @param enable true to enable counting
     * @return true on success
     */
    bool enable(bool enable)
    {
        return enable ? _core.setRegBit(axp202_regs::coulomb::COULOMB_CTL, 7)
                      : _core.clrRegBit(axp202_regs::coulomb::COULOMB_CTL, 7);
    }

    bool isEnabled()
    {
        return _core.getRegBit(axp202_regs::coulomb::COULOMB_CTL, 7);
    }

    /**
     * @brief Pause coulomb counter (REG B8H bit 6)
     * @return true on success
     */
    bool pause()
    {
        return _core.setRegBit(axp202_regs::coulomb::COULOMB_CTL, 6);
    }

    /**
     * @brief Resume coulomb counter (REG B8H bit 6 clear)
     * @return true on success
     */
    bool resume()
    {
        return _core.clrRegBit(axp202_regs::coulomb::COULOMB_CTL, 6);
    }

    /**
     * @brief Clear/reset coulomb counter (REG B8H bit 5)
     * @return true on success
     */
    bool clear()
    {
        return _core.setRegBit(axp202_regs::coulomb::COULOMB_CTL, 5);
    }

    /**
     * @brief Read charge coulomb counter (REG B0H-B3H, 32-bit big-endian)
     * @return 32-bit counter value, 0 on error
     */
    uint32_t getChargeCoulomb()
    {
        return readCoulomb32(axp202_regs::coulomb::BAT_CHGCOULOMB3);
    }

    /**
     * @brief Read discharge coulomb counter (REG B4H-B7H, 32-bit big-endian)
     * @return 32-bit counter value, 0 on error
     */
    uint32_t getDischargeCoulomb()
    {
        return readCoulomb32(axp202_regs::coulomb::BAT_DISCHGCOULOMB3);
    }

private:
    uint32_t readCoulomb32(uint8_t regH)
    {
        uint8_t buf[4];
        if (_core.readRegBuff(regH, buf, 4) < 0) return 0;
        return (static_cast<uint32_t>(buf[0]) << 24) |
               (static_cast<uint32_t>(buf[1]) << 16) |
               (static_cast<uint32_t>(buf[2]) << 8) |
               static_cast<uint32_t>(buf[3]);
    }

    AXP202Core &_core;
};

#include "AXP202Gpio.hpp"
#include "AXP202Regs.hpp"


AXP202Gpio::AXP202Gpio(AXP202Core &core) : _core(core)
{
}

uint8_t AXP202Gpio::getPinCount() const
{
    return 4;
}

uint8_t AXP202Gpio::getCtrlReg(uint8_t pin) const
{
    switch (pin) {
    case 0: return axp202_regs::gpio::GPIO0_CTL;
    case 1: return axp202_regs::gpio::GPIO1_CTL;
    case 2: return axp202_regs::gpio::GPIO2_CTL;
    case 3: return axp202_regs::gpio::GPIO3_CTL;
    default: return 0;
    }
}

PmicGpioBase::Status AXP202Gpio::setDirection(uint8_t pin, Direction dir)
{
    uint8_t reg = getCtrlReg(pin);
    if (reg == 0) return Status::InvalidPin;

    if (pin <= 2) {
        uint8_t val;
        switch (dir) {
        case Direction::Input:
            val = 0x02;
            break;
        case Direction::Output:
            val = 0x00;
            break;
        default:
            return Status::Failed;
        }
        return _core.updateBits(reg, 0x07, val) >= 0 ? Status::Ok : Status::Failed;
    }
    // GPIO3
    if (dir == Direction::Output) {
        int v = _core.readReg(reg);
        if (v < 0) return Status::Failed;
        v = (v & 0xF8) | 0x00;
        return _core.writeReg(reg, static_cast<uint8_t>(v)) >= 0 ? Status::Ok : Status::Failed;
    }
    int v = _core.readReg(reg);
    if (v < 0) return Status::Failed;
    v = (v & 0xF8) | 0x04;
    return _core.writeReg(reg, static_cast<uint8_t>(v)) >= 0 ? Status::Ok : Status::Failed;
}

PmicGpioBase::Direction AXP202Gpio::getDirection(uint8_t pin)
{
    uint8_t reg = getCtrlReg(pin);
    if (reg == 0) return Direction::Input;
    int val = _core.readReg(reg);
    if (val < 0) return Direction::Input;
    uint8_t func = val & 0x07;
    if (pin <= 2) {
        return (func == 0x02) ? Direction::Input : Direction::Output;
    }
    return ((func & 0x04) == 0) ? Direction::Output : Direction::Input;
}

PmicGpioBase::Status AXP202Gpio::setDrive(uint8_t pin, DriveType drive)
{
    if (pin >= getPinCount()) return Status::InvalidPin;
    (void)drive;
    // AXP202 GPIO0-2 are push-pull (no open-drain mode in function select).
    // GPIO3 is NMOS open-drain (fixed, not configurable).
    // Drive type cannot be independently configured on any pin.
    return Status::Failed;
}

PmicGpioBase::DriveType AXP202Gpio::getDrive(uint8_t pin)
{
    // GPIO0-2: push-pull (function 0x00=Low, 0x01=High)
    // GPIO3: NMOS open-drain
    return (pin == 3) ? DriveType::OpenDrain : DriveType::PushPull;
}

PmicGpioBase::Status AXP202Gpio::read(uint8_t pin, bool &high)
{
    uint8_t reg = getCtrlReg(pin);
    if (reg == 0) return Status::InvalidPin;

    if (pin <= 2) {
        int val = _core.readReg(axp202_regs::gpio::GPIO012_SIGNAL);
        if (val < 0) return Status::Failed;
        high = (val >> (pin + 4)) & 0x01;
        return Status::Ok;
    }
    // GPIO3: REG 95H bit0 is input status, active-low per datasheet:
    // bit0 = 0 means pin is electrically HIGH, bit0 = 1 means LOW
    int val = _core.readReg(reg);
    if (val < 0) return Status::Failed;
    high = !(val & 0x01);
    return Status::Ok;
}

PmicGpioBase::Status AXP202Gpio::write(uint8_t pin, Level level)
{
    uint8_t reg = getCtrlReg(pin);
    if (reg == 0) return Status::InvalidPin;

    int val = _core.readReg(reg);
    if (val < 0) return Status::Failed;

    if (pin <= 2) {
        // GPIO0-2 function select: 0x00=Low, 0x01=High, 0x07=Floating(HiZ)
        switch (level) {
        case Level::Low:  val = (val & 0xF8) | 0x00; break;
        case Level::High: val = (val & 0xF8) | 0x01; break;
        case Level::HiZ:  val = (val & 0xF8) | 0x07; break;
        default: return Status::Failed;
        }
    } else {
        // GPIO3: REG 95H bit1 controls output (NMOS open-drain)
        // bit1=0: low (NMOS on), bit1=1: floating (NMOS off)
        // Level::High is not supported — GPIO3 cannot actively drive high
        if (level == Level::High) return Status::Failed;
        val = (val & 0xFD);
        switch (level) {
        case Level::Low: val |= (0x00 << 1); break;
        case Level::HiZ: val |= (0x01 << 1); break;
        default: return Status::Failed;
        }
    }
    return _core.writeReg(reg, static_cast<uint8_t>(val)) >= 0 ? Status::Ok : Status::Failed;
}

PmicGpioBase::Status AXP202Gpio::setFunction(uint8_t pin, uint8_t func)
{
    uint8_t reg = getCtrlReg(pin);
    if (reg == 0) return Status::InvalidPin;
    // GPIO3 has no function multiplexing (only open-drain or input)
    if (pin == 3) return Status::Failed;

    // GPIO0-2: REG 90H/92H/93H [2:0] function select
    return _core.updateBits(reg, 0x07, func & 0x07) >= 0 ? Status::Ok : Status::Failed;
}

uint8_t AXP202Gpio::getFunction(uint8_t pin)
{
    uint8_t reg = getCtrlReg(pin);
    if (reg == 0) return 0xFF;
    int v = _core.readReg(reg);
    if (v < 0) return 0xFF;
    if (pin <= 2) {
        return v & 0x07;
    }
    // GPIO3: return bit2 (0=NMOS open-drain, 1=input) as function value
    return (v & 0x04) ? 0x01 : 0x00;
}

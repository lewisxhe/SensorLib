#include "AXP517Gpio.hpp"
#include "AXP517Regs.hpp"


AXP517Gpio::AXP517Gpio(AXP517Core &core)
    : _core(core)
{
}

uint8_t AXP517Gpio::getPinCount() const
{
    return 1;
}

int AXP517Gpio::readRaw()
{
    return _core.readReg(axp517_regs::ctrl::GPIO_CFG);
}

PmicGpioBase::Status AXP517Gpio::setDirection(uint8_t pin, Direction dir)
{
    if (pin != 0) return Status::InvalidPin;
    // bit4: 0 input, 1 output
    return _core.updateBits(axp517_regs::ctrl::GPIO_CFG, 0x10, (dir == Direction::Output) ? 0x10 : 0x00) ? Status::Ok : Status::Failed;
}

PmicGpioBase::Direction AXP517Gpio::getDirection(uint8_t pin)
{
    if (pin != 0) return Direction::Input;
    int v = readRaw();
    if (v < 0) return Direction::Input;
    return (v & 0x10) ? Direction::Output : Direction::Input;
}

PmicGpioBase::Status AXP517Gpio::setDrive(uint8_t pin, DriveType drive)
{
    if (pin != 0) return Status::InvalidPin;
    // bit7: 0 floating(push-pull), 1 open drain output
    return _core.updateBits(axp517_regs::ctrl::GPIO_CFG, 0x80, (drive == DriveType::OpenDrain) ? 0x80 : 0x00) ? Status::Ok : Status::Failed;
}

PmicGpioBase::DriveType AXP517Gpio::getDrive(uint8_t pin)
{
    if (pin != 0) return DriveType::PushPull;
    int v = readRaw();
    if (v < 0) return DriveType::PushPull;
    return (v & 0x80) ? DriveType::OpenDrain : DriveType::PushPull;
}

bool AXP517Gpio::setOutputSource(uint8_t pin, OutputSource src)
{
    if (pin != 0) return false;
    // bit[3:2]: 0=by reg11[1:0], 1=PD_IRQ
    uint8_t field = (src == OutputSource::PdIrq) ? 0x01 : 0x00;
    return _core.updateBits(axp517_regs::ctrl::GPIO_CFG, 0x0C, (uint8_t)(field << 2));
}

AXP517Gpio::OutputSource AXP517Gpio::getOutputSource(uint8_t pin)
{
    if (pin != 0) return OutputSource::ByReg11;
    int v = readRaw();
    if (v < 0) return OutputSource::ByReg11;

    uint8_t field = (uint8_t)((v >> 2) & 0x03);
    return (field == 0x01) ? OutputSource::PdIrq : OutputSource::ByReg11;
}

PmicGpioBase::Status AXP517Gpio::read(uint8_t pin, bool &high)
{
    if (pin != 0) return Status::InvalidPin;
    int v = readRaw();
    if (v < 0) return Status::Failed;

    // bit1: GPIO input status
    high = (v & 0x02) != 0;
    return Status::Ok;
}

PmicGpioBase::Status AXP517Gpio::write(uint8_t pin, Level level)
{
    if (pin != 0) return Status::InvalidPin;

    if (getOutputSource(pin) == OutputSource::PdIrq) {
        return Status::Failed;
    }

    if (getDirection(pin) != Direction::Output) {
        return Status::Failed;
    }

    DriveType drive = getDrive(pin);

    if (drive == DriveType::OpenDrain) {
        // bit6: OD output configure: 0 hiz, 1 low
        // Open-drain does not support forced drive high: High -> HiZ (external pull-up)
        uint8_t od = (level == Level::Low) ? 0x40 : 0x00;
        return _core.updateBits(axp517_regs::ctrl::GPIO_CFG, 0x40, od) ? Status::Ok : Status::Failed;
    }

    uint8_t code = 0;
    switch (level) {
    case Level::HiZ:  code = 0b00; break;
    case Level::Low:  code = 0b01; break;
    case Level::High: code = 0b10; break;
    default:          code = 0b00; break;
    }

    // bits[1:0]
    return _core.updateBits(axp517_regs::ctrl::GPIO_CFG, 0x03, code) ? Status::Ok : Status::Failed;
}

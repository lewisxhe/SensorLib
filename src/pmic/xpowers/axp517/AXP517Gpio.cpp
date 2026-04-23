#include "AXP517Gpio.hpp"
#include "AXP517Regs.hpp"

using namespace axp517::regs;

AXP517Gpio::AXP517Gpio(AXP517Core &core)
    : _core(core)
{
}

int AXP517Gpio::readRaw()
{
    return _core.readReg(ctrl::GPIO_CFG);
}

bool AXP517Gpio::setDirection(uint8_t pin, Direction dir)
{
    // bit4: 0 input, 1 output
    return _core.updateBits(ctrl::GPIO_CFG, 0x10, (dir == Direction::Output) ? 0x10 : 0x00);
}

PmicGpioBase::Direction AXP517Gpio::getDirection(uint8_t pin)
{
    int v = readRaw();
    if (v < 0) return Direction::Input;
    return (v & 0x10) ? Direction::Output : Direction::Input;
}

bool AXP517Gpio::setDrive(uint8_t pin, Drive drive)
{
    // bit7: 0 floating, 1 open drain output
    return _core.updateBits(ctrl::GPIO_CFG, 0x80, (drive == Drive::OpenDrain) ? 0x80 : 0x00);
}

PmicGpioBase::Drive AXP517Gpio::getDrive(uint8_t pin)
{
    int v = readRaw();
    if (v < 0) return Drive::Floating;
    return (v & 0x80) ? Drive::OpenDrain : Drive::Floating;
}

bool AXP517Gpio::setOutputSource(uint8_t pin, OutputSource src)
{
    // bit[3:2]: 0=by reg11[1:0], 1=PD_IRQ
    uint8_t field = (src == OutputSource::PdIrq) ? 0x01 : 0x00;
    return _core.updateBits(ctrl::GPIO_CFG, 0x0C, (uint8_t)(field << 2));
}

AXP517Gpio::OutputSource AXP517Gpio::getOutputSource(uint8_t pin)
{
    int v = readRaw();
    if (v < 0) return OutputSource::ByReg11;

    uint8_t field = (uint8_t)((v >> 2) & 0x03);
    return (field == 0x01) ? OutputSource::PdIrq : OutputSource::ByReg11;
}

bool AXP517Gpio::read(uint8_t pin, bool &high)
{
    int v = readRaw();
    if (v < 0) return false;

    // bit1: GPIO input status
    high = (v & 0x02) != 0;
    return true;
}

bool AXP517Gpio::write(uint8_t pin, Level level)
{
    if (getOutputSource(pin) == OutputSource::PdIrq) {
        return false;
    }

    if (getDirection(pin) != Direction::Output) {
        return false;
    }

    Drive drive = getDrive(pin);

    if (drive == Drive::OpenDrain) {
        // bit6: OD output configure: 0 hiz, 1 low
        // Open-drain does not support forced drive high: High -> HiZ (external pull-up)
        uint8_t od = (level == Level::Low) ? 0x40 : 0x00;
        return _core.updateBits(ctrl::GPIO_CFG, 0x40, od);
    }

    uint8_t code = 0;
    switch (level) {
    case Level::HiZ:  code = 0b00; break;
    case Level::Low:  code = 0b01; break;
    case Level::High: code = 0b10; break;
    default:          code = 0b00; break;
    }

    // bits[1:0]
    return _core.updateBits(ctrl::GPIO_CFG, 0x03, code);
}

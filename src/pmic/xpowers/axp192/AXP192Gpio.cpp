#include "AXP192Gpio.hpp"
#include "AXP192Regs.hpp"


AXP192Gpio::AXP192Gpio(AXP192Core &core) : _core(core)
{
}

uint8_t AXP192Gpio::getPinCount() const
{
    return 6;
}

PmicGpioBase::Status AXP192Gpio::setDirection(uint8_t pin, Direction dir)
{
    switch (pin) {
    case 0:
        // GPIO0_CTL[2:0]: 000=open-drain out, 001=input
        return _core.updateBits(axp192_regs::gpio::GPIO0_CTL, 0x07,
                                dir == Direction::Input ? 0x01 : 0x00) >= 0 ? Status::Ok : Status::Failed;
    case 1:
        // GPIO1_CTL[2:0]: 000=open-drain out, 001=input
        return _core.updateBits(axp192_regs::gpio::GPIO1_CTL, 0x07,
                                dir == Direction::Input ? 0x01 : 0x00) >= 0 ? Status::Ok : Status::Failed;
    case 2:
        // GPIO2_CTL[2:0]: 000=open-drain out, 001=input
        return _core.updateBits(axp192_regs::gpio::GPIO2_CTL, 0x07,
                                dir == Direction::Input ? 0x01 : 0x00) >= 0 ? Status::Ok : Status::Failed;
    case 3: {
        // GPIO34_CTL[7]=1 enables GPIO mode for GPIO3/4.
        // GPIO3 mode: [1:0] 01=output, 10=input.
        int v = _core.readReg(axp192_regs::gpio::GPIO34_CTL);
        if (v < 0) return Status::Failed;
        v |= 0x80;
        v &= ~0x03;
        v |= (dir == Direction::Input) ? 0x02 : 0x01;
        return _core.writeReg(axp192_regs::gpio::GPIO34_CTL, static_cast<uint8_t>(v)) >= 0 ? Status::Ok : Status::Failed;
    }
    case 4: {
        // GPIO34_CTL[7]=1 enables GPIO mode for GPIO3/4.
        // GPIO4 mode: [3:2] 01=output, 10=input.
        int v = _core.readReg(axp192_regs::gpio::GPIO34_CTL);
        if (v < 0) return Status::Failed;
        v |= 0x80;
        v &= ~0x0C;
        v |= (dir == Direction::Input) ? 0x08 : 0x04;
        return _core.writeReg(axp192_regs::gpio::GPIO34_CTL, static_cast<uint8_t>(v)) >= 0 ? Status::Ok : Status::Failed;
    }
    case 5: {
        // GPIO5_CTL[7]=1 GPIO5 mode, GPIO5_CTL[6]: 0=output,1=input
        int v = _core.readReg(axp192_regs::gpio::GPIO5_CTL);
        if (v < 0) return Status::Failed;
        v |= 0x80;
        if (dir == Direction::Input) {
            v |= 0x40;
        } else {
            v &= ~0x40;
        }
        return _core.writeReg(axp192_regs::gpio::GPIO5_CTL, static_cast<uint8_t>(v)) >= 0 ? Status::Ok : Status::Failed;
    }
    default:
        return Status::InvalidPin;
    }
}

PmicGpioBase::Direction AXP192Gpio::getDirection(uint8_t pin)
{
    int v;
    switch (pin) {
    case 0:
        v = _core.readReg(axp192_regs::gpio::GPIO0_CTL);
        if (v < 0) return Direction::Input;
        return ((v & 0x07) == 0x01) ? Direction::Input : Direction::Output;
    case 1:
        v = _core.readReg(axp192_regs::gpio::GPIO1_CTL);
        if (v < 0) return Direction::Input;
        return ((v & 0x07) == 0x01) ? Direction::Input : Direction::Output;
    case 2:
        v = _core.readReg(axp192_regs::gpio::GPIO2_CTL);
        if (v < 0) return Direction::Input;
        return ((v & 0x07) == 0x01) ? Direction::Input : Direction::Output;
    case 3:
        v = _core.readReg(axp192_regs::gpio::GPIO34_CTL);
        if (v < 0) return Direction::Input;
        return ((v & 0x03) == 0x02) ? Direction::Input : Direction::Output;
    case 4:
        v = _core.readReg(axp192_regs::gpio::GPIO34_CTL);
        if (v < 0) return Direction::Input;
        return (((v >> 2) & 0x03) == 0x02) ? Direction::Input : Direction::Output;
    case 5:
        v = _core.readReg(axp192_regs::gpio::GPIO5_CTL);
        if (v < 0) return Direction::Input;
        return (v & 0x40) ? Direction::Input : Direction::Output;
    default:
        return Direction::Input;
    }
}

PmicGpioBase::Status AXP192Gpio::setDrive(uint8_t pin, DriveType drive)
{
    if (pin >= getPinCount()) return Status::InvalidPin;
    (void)drive;
    // AXP192 GPIO drive type is implicit in function select (REG 90H/92H/93H).
    // 0x00 = NMOS open-drain output. Cannot be independently configured.
    return Status::Failed;
}

PmicGpioBase::DriveType AXP192Gpio::getDrive(uint8_t pin)
{
    (void)pin;
    // AXP192 GPIOs are NMOS open-drain when used as GPIO output (function 0x00)
    return DriveType::OpenDrain;
}

PmicGpioBase::Status AXP192Gpio::read(uint8_t pin, bool &high)
{
    uint8_t reg;
    uint8_t bit;
    switch (pin) {
    case 0: reg = axp192_regs::gpio::GPIO012_SIGNAL; bit = 4; break;
    case 1: reg = axp192_regs::gpio::GPIO012_SIGNAL; bit = 5; break;
    case 2: reg = axp192_regs::gpio::GPIO012_SIGNAL; bit = 6; break;
    case 3: reg = axp192_regs::gpio::GPIO34_SIGNAL;  bit = 4; break;
    case 4: reg = axp192_regs::gpio::GPIO34_SIGNAL;  bit = 5; break;
    case 5: reg = axp192_regs::gpio::GPIO5_CTL;      bit = 4; break;
    default: return Status::InvalidPin;
    }
    high = _core.getRegBit(reg, bit);
    return Status::Ok;
}

PmicGpioBase::Status AXP192Gpio::write(uint8_t pin, Level level)
{
    uint8_t reg;
    uint8_t bit;

    // AXP192 GPIO outputs are open-drain for most pins:
    // Low -> NMOS on, HiZ/High -> NMOS off.
    bool release = (level == Level::HiZ || level == Level::High);

    switch (pin) {
    case 0: reg = axp192_regs::gpio::GPIO012_SIGNAL; bit = 0; break;
    case 1: reg = axp192_regs::gpio::GPIO012_SIGNAL; bit = 1; break;
    case 2: reg = axp192_regs::gpio::GPIO012_SIGNAL; bit = 2; break;
    case 3: reg = axp192_regs::gpio::GPIO34_SIGNAL;  bit = 0; break;
    case 4: reg = axp192_regs::gpio::GPIO34_SIGNAL;  bit = 1; break;
    case 5: reg = axp192_regs::gpio::GPIO5_CTL;      bit = 5; break;
    default: return Status::InvalidPin;
    }

    return (release ? _core.setRegBit(reg, bit) : _core.clrRegBit(reg, bit)) ? Status::Ok : Status::Failed;
}

PmicGpioBase::Status AXP192Gpio::setFunction(uint8_t pin, uint8_t func)
{
    switch (pin) {
    case 0:
        // REG 90H [2:0]: GPIO0 function select
        return _core.updateBits(axp192_regs::gpio::GPIO0_CTL, 0x07, func & 0x07) >= 0 ? Status::Ok : Status::Failed;
    case 1:
        // REG 92H [2:0]: GPIO1 function select
        return _core.updateBits(axp192_regs::gpio::GPIO1_CTL, 0x07, func & 0x07) >= 0 ? Status::Ok : Status::Failed;
    case 2:
        // REG 93H [2:0]: GPIO2 function select
        return _core.updateBits(axp192_regs::gpio::GPIO2_CTL, 0x07, func & 0x07) >= 0 ? Status::Ok : Status::Failed;
    case 3: {
        // REG 95H: GPIO3/4 shared register. bit7=1 enables GPIO mode.
        // GPIO3 function: [1:0] 00=ext charge, 01=NMOS out, 10=input, 11=ADC
        int v = _core.readReg(axp192_regs::gpio::GPIO34_CTL);
        if (v < 0) return Status::Failed;
        v |= 0x80;   // enable GPIO mode
        v &= ~0x03;
        v |= (func & 0x03);
        return _core.writeReg(axp192_regs::gpio::GPIO34_CTL, static_cast<uint8_t>(v)) >= 0 ? Status::Ok : Status::Failed;
    }
    case 4: {
        // REG 95H: GPIO4 function: [3:2] 00=ext charge, 01=NMOS out, 10=input
        int v = _core.readReg(axp192_regs::gpio::GPIO34_CTL);
        if (v < 0) return Status::Failed;
        v |= 0x80;   // enable GPIO mode
        v &= ~0x0C;
        v |= ((func & 0x03) << 2);
        return _core.writeReg(axp192_regs::gpio::GPIO34_CTL, static_cast<uint8_t>(v)) >= 0 ? Status::Ok : Status::Failed;
    }
    case 5: {
        // REG 9EH: GPIO5/N_RSTO. bit7=1 for GPIO5 mode, bit6=direction.
        // No function multiplexing beyond input/output. Only 0=output, 1=input.
        int v = _core.readReg(axp192_regs::gpio::GPIO5_CTL);
        if (v < 0) return Status::Failed;
        v |= 0x80;  // enable GPIO5 mode
        if (func == 0x01) {
            v |= 0x40;   // input
        } else {
            v &= ~0x40;  // output
        }
        return _core.writeReg(axp192_regs::gpio::GPIO5_CTL, static_cast<uint8_t>(v)) >= 0 ? Status::Ok : Status::Failed;
    }
    default:
        return Status::InvalidPin;
    }
}

uint8_t AXP192Gpio::getFunction(uint8_t pin)
{
    int v;
    switch (pin) {
    case 0:
        v = _core.readReg(axp192_regs::gpio::GPIO0_CTL);
        if (v < 0) return 0xFF;
        return v & 0x07;
    case 1:
        v = _core.readReg(axp192_regs::gpio::GPIO1_CTL);
        if (v < 0) return 0xFF;
        return v & 0x07;
    case 2:
        v = _core.readReg(axp192_regs::gpio::GPIO2_CTL);
        if (v < 0) return 0xFF;
        return v & 0x07;
    case 3:
        v = _core.readReg(axp192_regs::gpio::GPIO34_CTL);
        if (v < 0) return 0xFF;
        return v & 0x03;
    case 4:
        v = _core.readReg(axp192_regs::gpio::GPIO34_CTL);
        if (v < 0) return 0xFF;
        return (v >> 2) & 0x03;
    case 5:
        v = _core.readReg(axp192_regs::gpio::GPIO5_CTL);
        if (v < 0) return 0xFF;
        return (v & 0x40) ? 0x01 : 0x00;
    default:
        return 0xFF;
    }
}

PmicGpioBase::Status AXP192Gpio::setPull(uint8_t pin, Pull pull)
{
    if (pin >= getPinCount()) return Status::InvalidPin;
    if (pin > 2) return Status::Failed;
    // REG 97H [2:0]: GPIO0-2 pull-down enable (1=enable, 0=disable)
    // Only pull-down is supported, pull-up returns false
    if (pull == Pull::Up) return Status::Failed;
    uint8_t bit = (1 << pin);
    if (pull == Pull::Down) {
        return _core.setRegBit(axp192_regs::gpio::GPIO012_PULLDOWN, bit) ? Status::Ok : Status::Failed;
    }
    // Pull::None
    return _core.clrRegBit(axp192_regs::gpio::GPIO012_PULLDOWN, bit) ? Status::Ok : Status::Failed;
}

PmicGpioBase::Pull AXP192Gpio::getPull(uint8_t pin)
{
    if (pin > 2) return Pull::None;
    // REG 97H [2:0]: GPIO0-2 pull-down enable
    return _core.getRegBit(axp192_regs::gpio::GPIO012_PULLDOWN, pin) ? Pull::Down : Pull::None;
}

bool AXP192Gpio::setPwmFrequency(uint8_t pin, uint8_t x)
{
    // PWM1 → GPIO1 (REG 0x98), PWM2 → GPIO2 (REG 0x9B)
    uint8_t reg;
    switch (pin) {
    case 1: reg = axp192_regs::gpio::PWM1_FREQ; break;
    case 2: reg = axp192_regs::gpio::PWM2_FREQ; break;
    default: return false;
    }
    return _core.writeReg(reg, x) >= 0;
}

bool AXP192Gpio::setPwmDutyCycle(uint8_t pin, uint8_t y1, uint8_t y2)
{
    // PWM1 → GPIO1 (REG 0x99=Y1, 0x9A=Y2), PWM2 → GPIO2 (REG 0x9C=Y1, 0x9D=Y2)
    uint8_t reg_y1, reg_y2;
    switch (pin) {
    case 1:
        reg_y1 = axp192_regs::gpio::PWM1_DUTY1;
        reg_y2 = axp192_regs::gpio::PWM1_DUTY2;
        break;
    case 2:
        reg_y1 = axp192_regs::gpio::PWM2_DUTY1;
        reg_y2 = axp192_regs::gpio::PWM2_DUTY2;
        break;
    default:
        return false;
    }
    if (y1 == 0) return false;  // Y1 must be > 0 to avoid division by zero
    if (_core.writeReg(reg_y1, y1) < 0) return false;
    return _core.writeReg(reg_y2, y2) >= 0;
}

bool AXP192Gpio::setPwm(uint8_t pin, uint8_t x, uint8_t y1, uint8_t y2)
{
    // Set pin function to PWM mode (0x02)
    if (!isOk(setFunction(pin, 0x02))) return false;
    // Set direction to output
    if (!isOk(setDirection(pin, Direction::Output))) return false;
    // Configure frequency
    if (!setPwmFrequency(pin, x)) return false;
    // Configure duty cycle
    return setPwmDutyCycle(pin, y1, y2);
}

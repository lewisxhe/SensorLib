/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
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
 * @file      ExtensionIOXL9555.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-12-27
 *
 */
#pragma once

#include "REG/XL9555Constants.h"
#include "SensorPlatform.hpp"
#include "ExtensionSPI.hpp"
#include "VirtualGpio.hpp"

typedef void (*gpio_event_t)(void *user_data);
typedef struct ioEvent {
    uint8_t mode;
    gpio_event_t cb;
    void *user_data;
    uint8_t last_state;
    ioEvent() :  mode(0), cb(NULL), user_data(NULL), last_state(LOW)
    {
    }
} ioEvent_t;



class ExtensionIOXL9555 :
    public VirtualGpio,
    public XL95xxConstants,
    public ExtensionSPI<ExtensionIOXL9555>
{
    friend class ExtensionSPI<ExtensionIOXL9555>;
public:


    enum {
        PORT0,
        PORT1,
    };

    enum ExtensionGPIO {
        IO0,
        IO1,
        IO2,
        IO3,
        IO4,
        IO5,
        IO6,
        IO7,
        IO8,
        IO9,
        IO10,
        IO11,
        IO12,
        IO13,
        IO14,
        IO15,
    };

    ExtensionIOXL9555() : comm(nullptr) {}

    ~ExtensionIOXL9555()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, uint8_t addr = XL9555_UNKOWN_ADDRESS, int sda = -1, int scl = -1)
    {
        comm = std::make_unique<SensorCommI2C>(wire, addr, sda, scl);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl(addr);
    }

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, uint8_t addr = XL9555_UNKOWN_ADDRESS, int sda = -1, int scl = -1)
    {
        comm = std::make_unique<SensorCommI2C>(port_num, addr, sda, scl);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl(addr);
    }
#else
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = XL9555_UNKOWN_ADDRESS)
    {
        comm = std::make_unique<SensorCommI2C>(handle, addr);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl(addr);
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    bool begin(SensorCommCustom::CustomCallback callback, uint8_t addr)
    {
        comm = std::make_unique<SensorCommCustom>(callback, addr);
        if (!comm) {
            return false;
        }
        comm->init();
        return initImpl(addr);
    }

    void deinit()
    {
        // end();
    }

    void pinMode(uint8_t pin, uint8_t mode)
    {
        uint8_t registers = 0;
        if (pin >= IO8) {
            pin -= IO8;
            registers = XL9555_CTRL_CFG1;
        } else {
            registers = XL9555_CTRL_CFG0;
        }
        switch (mode) {
        case INPUT:
            comm->setRegisterBit(registers, pin);
            break;
        case OUTPUT:
            comm->clrRegisterBit(registers, pin);
            break;
        default:
            break;
        }
    }

    int digitalRead(uint8_t pin)
    {
        uint8_t registers = 0;
        if (pin >= IO8) {
            pin -= IO8;
            registers = XL9555_CTRL_INP1;
        } else {
            registers = XL9555_CTRL_INP0;
        }
        return comm->getRegisterBit(registers, pin);
    }

    void digitalWrite(uint8_t pin, uint8_t val)
    {
        uint8_t registers = 0;
        if (pin >= IO8) {
            pin -= IO8;
            registers = XL9555_CTRL_OUTP1;
        } else {
            registers = XL9555_CTRL_OUTP0;
        }
        val ? comm->setRegisterBit(registers, pin) : comm->clrRegisterBit(registers,  pin);
    }

    // Invert gpio
    void digitalToggle(uint8_t pin)
    {
        int state = 1 - digitalRead(pin);
        digitalWrite(pin, state);
    }

    // Read 8-bit gpio input status
    int readPort(uint8_t port)
    {
        return comm->readRegister(port == PORT0 ? XL9555_CTRL_INP0 : XL9555_CTRL_INP1);
    }

    // Write 8 bits of data to the specified port
    int writePort(uint8_t port, uint8_t mask)
    {
        return comm->writeRegister(port == PORT0 ? XL9555_CTRL_OUTP0 : XL9555_CTRL_OUTP1, mask);
    }

    // Read 16-bit gpio input status
    uint16_t read()
    {
        uint16_t val = 0x0;
        comm->readRegister(XL9555_CTRL_INP0, (uint8_t *)&val, 2);
        return val;
    }

    // Write 16-bit data to gpio, low bit port 0
    void write(uint16_t value)
    {
        uint8_t buffer[2] = {highByte(value), lowByte(value)};
        comm->writeRegister(XL9555_CTRL_OUTP0, buffer, 2);
    }

    // Read the specified port configuration status
    int readConfig(uint8_t port)
    {
        return comm->readRegister(port == PORT0 ? XL9555_CTRL_CFG0 : XL9555_CTRL_CFG1);
    }

    // Configure the specified port as input or output , 0xFF = all pin input , 0x00 = all pin output
    int configPort(uint8_t port, uint8_t mask)
    {
        return comm->writeRegister(port == PORT0 ? XL9555_CTRL_CFG0 : XL9555_CTRL_CFG1, mask);
    }

    void setPinEvent(uint8_t pin, uint8_t mode, gpio_event_t event, void *user_data)
    {
        if (pin > XL9555_MAX_PIN) {
            log_e("XL9555 Max use io pin is 0 ~ 15 .");
            return;
        }
        this->event[pin].cb = event;
        this->event[pin].mode = mode;
        this->event[pin].user_data = user_data;
    }

    void removePinEvent(uint8_t pin)
    {
        if (pin > XL9555_MAX_PIN) {
            log_e("XL9555 Max use io pin is 0 ~ 15 .");
            return;
        }
        this->event[pin].cb = NULL;
    }

    void update()
    {
        uint16_t val = this->read();

        int i = XL9555_MAX_PIN;

        for (; i >= 0; i--) {

            uint8_t _index = XL9555_MAX_PIN - i;

            if (this->event[_index].cb != NULL) {

                if (!(val & 1)) {
                    if (this->event[_index].mode == LOW) {
                        this->event[_index].cb(this->event[_index].user_data);
                    }
                    this->event[_index].last_state = LOW;
                } else {
                    if (this->event[_index].mode == HIGH && this->event[_index].last_state == LOW) {
                        this->event[_index].cb(this->event[_index].user_data);
                    }
                    this->event[_index].last_state = HIGH;
                }
            }
            val >>= 1;
        }
    }

    void setClock(uint32_t frequency)
    {
        //TODO:
    }

    uint32_t getClock()
    {
        //TODO:
        return 0;
    }
private:

    bool initImpl(uint8_t addr)
    {
        if (addr == XL9555_UNKOWN_ADDRESS) {
            log_d("Try to automatically discover the device");
            for (uint8_t a = XL9555_SLAVE_ADDRESS0; a <= XL9555_SLAVE_ADDRESS7; ++a) {
                I2CParam params(I2CParam::I2C_SET_ADDR, a);
                comm->setParams(params);
                log_d("Try to use 0x%02x address.", a);
                if (comm->readRegister(XL9555_CTRL_INP0) != -1) {
                    log_d("Found the xl9555 chip address is 0x%X", a);
                    return true;
                }
            }
            log_e("No found xl9555 chip ...");
            return false;
        }
        if (comm->readRegister(XL9555_CTRL_INP0) < 0 ) {
            return false;
        }
        return true;
    }

    ioEvent_t event[16];
protected:
    uint32_t _frequency;
    std::unique_ptr<SensorCommBase> comm;
};

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
 * @file      PmicAXP517.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-03-12
 *
 */
#pragma once

#include "../../PmicBase.hpp"
#include "../../PmicAdcBase.hpp"
#include "../../PmicIrqBase.hpp"
#include "../../PmicPowerBase.hpp"

#include "AXP517Core.hpp"
#include "AXP517Adc.hpp"
#include "AXP517Irq.hpp"
#include "AXP517Charger.hpp"
#include "AXP517Bc12.hpp"
#include "AXP517Led.hpp"
#include "AXP517Power.hpp"
#include "AXP517Gpio.hpp"
#include "AXP517Bc12.hpp"
// #include "AXP517Tcpc.hpp"
// #include "AXP517PdNegotiator.hpp"

static constexpr uint8_t AXP517_SLAVE_ADDRESS = 0x34;

class PmicAXP517 : public PmicBase
{
public:
    using Module = AXP517Core::Module;

    PmicAXP517() : _core(), _adc(_core)
        , _irq(_core), _charger(_core)
        , _bc12(_core), _led(_core)
        , _power(_core), _gpio(_core),
        _bc12_2(_core)
    {}


#if defined(ARDUINO)
    /**
     * @brief  Initialization using the Arduino Wire Interface
     * @note   This function sets up the I2C communication parameters for the sensor.
     * @param  &wire: Reference to the TwoWire I2C interface.
     * @param  addr: I2C address of the sensor.
     * @param  sda: SDA pin number, default is -1 (use board default)
     * @param  scl: SCL pin number, default is -1 (use board default)
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(wire, addr, sda, scl);
    }
#elif defined(ESP_PLATFORM)
#if defined(USEING_I2C_LEGACY)

    /**
     * @brief  Initialization using the ESP-IDF I2C Legacy Interface
     * @note   This function sets up the I2C communication parameters for the sensor.
     * @param  port_num: I2C port number.
     * @param  addr: I2C address of the sensor.
     * @param  sda: SDA pin number, default is -1 (use board default)
     * @param  scl: SCL pin number, default is -1 (use board default)
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(port_num, addr, sda, scl);
    }
#else

    /**
     * @brief  Initialization using the ESP-IDF I2C LL Interface idf version > 5.0.0
     * @note   This function sets up the I2C communication parameters for the sensor.
     * @param  handle: I2C master bus handle.
     * @param  addr: I2C address of the sensor.
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr)
    {
        return _core.begin(handle, addr);
    }
#endif
#endif
    /**
     * @brief  Initialization using a custom communication interface.
     * @note   This function sets up the communication parameters for the sensor.
     * @param  callback: Custom callback function for communication.
     * @param  hal_cb: No use this callback,can be nullptr
     * @param  addr: I2C address of the sensor.
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_cb,
               uint8_t addr)
    {
        return _core.begin(callback, hal_cb, addr);
    }

    /**
     * @brief Deinitialize (optional).
     * @note  This method is not implemented.
     */
    void end()
    {
    }

    // ---------------- Capability accessors ----------------
    PmicCapability::Capability getCapabilities() const override
    {
        return PmicCapability::Capability::PmicSupportCharger | PmicCapability::Capability::PmicSupportPower | PmicCapability::Capability::PmicSupportAdc |
               PmicCapability::Capability::PmicSupportGpio | PmicCapability::Capability::PmicSupportLed | PmicCapability::Capability::PmicSupportIrq |
               PmicCapability::Capability::PmicSupportBc12;
    }

    AXP517Adc &chipAdc()
    {
        return _adc;
    }

    AXP517Irq &irq()
    {
        return _irq;
    }

    PmicPowerBase &power() override
    {
        return _power;
    }

    PmicChargerBase *getCharger() override
    {
        return &_charger;
    }

    PmicAdcBase &adc() override
    {
        return _adc;
    }

    // ---------------- Chip-specific module accessors ----------------
    AXP517Charger &charger()
    {
        return _charger;
    }

    AXP517Bc12 &bc12()
    {
        return _bc12;
    }

    AXP517Led &led()
    {
        return _led;
    }

    AXP517Gpio &gpio()
    {
        return _gpio;
    }

    AXP517Bc12 &bc12_2()
    {
        return _bc12_2;
    }

    // AXP517Tcpc &tcpc()
    // {
    //     return _tcpc;
    // }

    /**
    * @brief  Enable or disable a module.
    * @note   The master switch for each functional module (e.g. boost, buck, charger,
    *         LED, BC1.2, TCPC, gauge, watchdog, etc.).
    * @param  module: The module to enable/disable.
    * @param  enable: True to enable, false to disable.
    * @retval True on success, false on failure.
    */
    bool enableModule(Module module, bool enable)
    {
        return _core.enableModule(module, enable);
    }

    /**
     * @brief  Check if a module is enabled.
     * @note   This function checks the status of a specific module.
     * @param  module: The module to check.
     * @retval True if the module is enabled, false otherwise.
     */
    bool isModuleEnabled(Module module)
    {
        return _core.isModuleEnabled(module);
    }

    AXP517Core &core()
    {
        return _core;
    }

    const char *getChipName() const override
    {
        return "AXP517";
    }
private:


    const AXP517Core &core() const
    {
        return _core;
    }

private:
    AXP517Core _core;

    AXP517Adc _adc;
    AXP517Irq _irq;
    AXP517Charger _charger;
    AXP517Bc12 _bc12;
    AXP517Led _led;
    AXP517Power _power;
    AXP517Gpio _gpio;
    AXP517Bc12 _bc12_2;
    // AXP517Tcpc _tcpc;
};

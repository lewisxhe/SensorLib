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
 * @brief     High-level AXP517 PMIC facade.
 */
#pragma once

#include "../../../SensorBuildOpt.h"
#if !SENSORLIB_EXCLUDE_PMIC_AXP517

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
#include "AXP517Tcpc.hpp"
#include "AXP517Pd.hpp"
#include "AXP517PdNegotiator.hpp"

static constexpr uint8_t AXP517_SLAVE_ADDRESS = 0x34;

/**
 * @brief High-level driver facade for the X-Powers AXP517 PMIC.
 *
 * The facade owns the shared register transport and exposes charger, ADC,
 * power-path, GPIO, IRQ, LED, BC1.2, Type-C, and USB-PD helpers through a
 * single object.
 */
class PmicAXP517 : public PmicBase
{
public:
    using Module = AXP517Core::Module;

    /**
     * @brief Construct an uninitialized AXP517 facade.
     */
    PmicAXP517() : _core(), _adc(_core)
        , _irq(_core), _charger(_core)
        , _bc12(_core), _led(_core)
        , _power(_core), _gpio(_core),
        _bc12_2(_core), _tcpc(_core), _pd(_tcpc),
        _pdNegotiator(_tcpc, _pd)
    {}


#if defined(ARDUINO)
    /**
     * @brief  Initialize using the Arduino Wire interface.
     * @param  wire Reference to the TwoWire I2C interface.
     * @param  addr AXP517 I2C address.
     * @param  sda SDA pin number, or -1 to use the board default.
     * @param  scl SCL pin number, or -1 to use the board default.
     * @retval true on success, false on communication or device-ID failure.
     */
    bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(wire, addr, sda, scl);
    }

    /**
     * @brief Initialize using Arduino Wire and configure the PD IRQ pin.
     * @param wire Reference to the Arduino TwoWire bus.
     * @param addr AXP517 I2C address.
     * @param sda SDA pin number.
     * @param scl SCL pin number.
     * @param irqPin Platform GPIO connected to PMIC_IRQ.
     * @retval true on success, false on communication, device-ID, or IRQ configuration failure.
     */
    bool begin(TwoWire &wire, uint8_t addr, int sda, int scl, int irqPin)
    {
        if (!_core.begin(wire, addr, sda, scl)) return false;
        return configurePdIrqPin(irqPin);
    }
#elif defined(ESP_PLATFORM)
#if defined(SENSORLIB_USE_I2C_LEGACY)

    /**
     * @brief  Initialize using the ESP-IDF legacy I2C interface.
     * @param  port_num I2C port number.
     * @param  addr AXP517 I2C address.
     * @param  sda SDA pin number, or -1 to use the board default.
     * @param  scl SCL pin number, or -1 to use the board default.
     * @retval true on success, false on communication or device-ID failure.
     */
    bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1)
    {
        return _core.begin(port_num, addr, sda, scl);
    }

    /**
     * @brief Initialize using ESP-IDF legacy I2C and configure the PD IRQ pin.
     * @param port_num I2C port number.
     * @param addr AXP517 I2C address.
     * @param sda SDA pin number.
     * @param scl SCL pin number.
     * @param irqPin Platform GPIO connected to PMIC_IRQ.
     * @retval true on success, false on communication, device-ID, or IRQ configuration failure.
     */
    bool begin(i2c_port_t port_num, uint8_t addr, int sda, int scl, int irqPin)
    {
        if (!_core.begin(port_num, addr, sda, scl)) return false;
        return configurePdIrqPin(irqPin);
    }
#else

    /**
     * @brief  Initialize using the ESP-IDF I2C master-bus interface.
     * @param  handle I2C master bus handle.
     * @param  addr AXP517 I2C address.
     * @retval true on success, false on communication or device-ID failure.
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr)
    {
        return _core.begin(handle, addr);
    }

    /**
     * @brief Initialize using ESP-IDF I2C master bus and configure the PD IRQ pin.
     * @param handle ESP-IDF I2C master bus handle.
     * @param addr AXP517 I2C address.
     * @param irqPin Platform GPIO connected to PMIC_IRQ.
     * @retval true on success, false on communication, device-ID, or IRQ configuration failure.
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr, int irqPin)
    {
        if (!_core.begin(handle, addr)) return false;
        return configurePdIrqPin(irqPin);
    }
#endif
#endif
    /**
     * @brief  Initialization using a custom communication interface.
     * @param  callback Custom callback function for communication.
     * @param  hal_cb Optional custom HAL callback; may be nullptr.
     * @param  addr AXP517 I2C address.
     * @retval true on success, false on communication or device-ID failure.
     */
    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_cb,
               uint8_t addr)
    {
        return _core.begin(callback, hal_cb, addr);
    }

    /**
     * @brief Initialize using custom callbacks and configure the PD IRQ pin.
     * @param callback Custom communication callback.
     * @param hal_cb Optional custom HAL callback; may be nullptr.
     * @param addr AXP517 I2C address.
     * @param irqPin Platform GPIO connected to PMIC_IRQ.
     * @retval true on success, false on communication, device-ID, or IRQ configuration failure.
     */
    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_cb,
               uint8_t addr,
               int irqPin)
    {
        if (!_core.begin(callback, hal_cb, addr)) return false;
        return configurePdIrqPin(irqPin);
    }

    /**
     * @brief Deinitialize the driver.
     * @note  This facade currently has no owned platform resources to release.
     */
    void end()
    {
    }

    /**
     * @brief Return the capabilities supported by this driver.
     */
    PmicCapability::Capability getCapabilities() const override
    {
        return PmicCapability::Capability::PmicSupportCharger | PmicCapability::Capability::PmicSupportPower | PmicCapability::Capability::PmicSupportAdc |
               PmicCapability::Capability::PmicSupportGpio | PmicCapability::Capability::PmicSupportLed | PmicCapability::Capability::PmicSupportIrq |
               PmicCapability::Capability::PmicSupportBc12 | PmicCapability::Capability::PmicSupportTypeC;
    }

    /**
     * @brief Return static PMIC configuration metadata.
     */
    const PmicConfig &getConfig() const override
    {
        static const PmicConfig config = {
            .chipName = "AXP517",
            .i2cAddress = 0x34,
            .chipIdReg = 0x00,
            .chipIdValue = 0,
            .channelCount = 0,
            .capabilities = PmicCapability::Capability::PmicSupportCharger |
                            PmicCapability::Capability::PmicSupportPower |
                            PmicCapability::Capability::PmicSupportAdc |
                            PmicCapability::Capability::PmicSupportGpio |
                            PmicCapability::Capability::PmicSupportLed |
                            PmicCapability::Capability::PmicSupportIrq |
                            PmicCapability::Capability::PmicSupportBc12 |
                            PmicCapability::Capability::PmicSupportTypeC,
            // REG62H[6:0]: ICC = 64*N mA, N=0..80
            .chargeCurrentMin  = 0,
            .chargeCurrentMax  = 5120,
            .chargeCurrentStep = 64,
            .chargeCurrentSteps = 81,
        };
        return config;
    }

    /** @brief Access the AXP517 ADC driver. */
    AXP517Adc &adc()
    {
        return _adc;
    }

    /** @brief Access the AXP517 IRQ driver. */
    AXP517Irq &irq()
    {
        return _irq;
    }

    /** @brief Access the generic PMIC power-path interface. */
    PmicPowerBase &getPower() override
    {
        return _power;
    }

    /** @brief Access the generic PMIC charger interface. */
    PmicChargerBase *getCharger() override
    {
        return &_charger;
    }

    /** @brief Access the generic PMIC ADC interface. */
    PmicAdcBase &getAdc() override
    {
        return _adc;
    }

    /** @brief Access the generic PMIC GPIO interface. */
    PmicGpioBase *getGpio() override
    {
        return &_gpio;
    }

    /** @brief Access the generic PMIC IRQ interface. */
    PmicIrqBase *getIrq() override
    {
        return &_irq;
    }

    /** @brief Access the AXP517 charger driver. */
    AXP517Charger &charger()
    {
        return _charger;
    }

    /** @brief Access the AXP517 BC1.2 driver. */
    AXP517Bc12 &bc12()
    {
        return _bc12;
    }

    /** @brief Access the generic PMIC LED interface. */
    PmicLedBase &getLed() override
    {
        return _led;
    }

    /** @brief Access the AXP517 LED driver. */
    AXP517Led &led()
    {
        return _led;
    }

    /** @brief Access the AXP517 power-path driver. */
    AXP517Power &power()
    {
        return _power;
    }

    /** @brief Access the AXP517 GPIO driver. */
    AXP517Gpio &gpio()
    {
        return _gpio;
    }

    /** @brief Access the secondary BC1.2 driver alias. */
    AXP517Bc12 &bc12_2()
    {
        return _bc12_2;
    }

    /** @brief Access the low-level AXP517 TCPC helper. */
    AXP517Tcpc &tcpc()
    {
        return _tcpc;
    }

    /** @brief Access the low-level USB-PD protocol helper. */
    AXP517Pd &pdProtocol()
    {
        return _pd;
    }

    /** @brief Access the USB-PD sink negotiator. */
    AXP517PdNegotiator &pd()
    {
        return _pdNegotiator;
    }

    /** @brief Access the USB-PD sink negotiator. */
    AXP517PdNegotiator &pdNegotiator()
    {
        return _pdNegotiator;
    }

    /**
     * @brief Configure the IRQ pin required for USB-PD negotiation.
     * @param irqPin Platform GPIO connected to PMIC_IRQ.
     * @retval true on success, false when irqPin is invalid.
     */
    bool setPdIrqPin(int irqPin)
    {
        return _pdNegotiator.setIrqPin(irqPin);
    }

    /**
     * @brief Initialize the TCPC and PD sink receive path.
     * @param irqPin Platform GPIO connected to PMIC_IRQ.
     * @retval true on success, false on IRQ or TCPC initialization failure.
     */
    bool initPdSink(int irqPin)
    {
        return _pdNegotiator.initSink(irqPin);
    }

    /**
     * @brief Request a fixed USB-PD voltage from the attached source.
     * @note A valid PMIC_IRQ pin must be configured before this call can succeed.
     * @param targetMv Requested fixed PDO voltage in millivolts.
     * @param timeoutMs Negotiation timeout in milliseconds.
     * @param outCaps Optional destination for parsed source capabilities.
     * @retval true when the requested PD contract reaches PS_RDY.
     */
    bool requestPd(uint16_t targetMv, uint32_t timeoutMs = 6000,
                   AXP517PdNegotiator::SourceCaps *outCaps = nullptr)
    {
        AXP517PdNegotiator::RequestParams req{};
        req.targetMv = targetMv;
        return _pdNegotiator.negotiate(req, outCaps, timeoutMs);
    }

    /**
     * @brief  Enable or disable a functional module.
     * @note   This is the master switch for boost, buck, charger, LED, BC1.2,
     *         TCPC, gauge, watchdog, and related PMIC modules.
     * @param  module Module to enable or disable.
     * @param  enable true to enable, false to disable.
     * @retval true on success, false on register access failure.
     */
    bool enableModule(Module module, bool enable)
    {
        return _core.enableModule(module, enable);
    }

    /**
     * @brief  Check if a module is enabled.
     * @note   This function checks the status of a specific module.
     * @param  module Module to check.
     * @retval true if the module is enabled, false otherwise.
     */
    bool isModuleEnabled(Module module)
    {
        return _core.isModuleEnabled(module);
    }

    /**
     * @brief Access the shared low-level AXP517 core.
     */
    AXP517Core &core()
    {
        return _core;
    }

    /**
     * @brief Return the chip name string.
     * @return Constant string "AXP517".
     */
    const char *getChipName() const override
    {
        return "AXP517";
    }

private:
    /** @brief Access the shared low-level AXP517 core from const contexts. */
    const AXP517Core &core() const
    {
        return _core;
    }

    /**
     * @brief Configure the optional PD IRQ pin after core initialization.
     * @param irqPin Platform GPIO connected to PMIC_IRQ, or a negative value to skip.
     * @retval true on success, false when IRQ pin setup fails.
     */
    bool configurePdIrqPin(int irqPin)
    {
        if (irqPin < 0) return true;
        return _pdNegotiator.setIrqPin(irqPin);
    }

private:
    AXP517Core _core;                    ///< Shared register transport.
    AXP517Adc _adc;                      ///< ADC helper.
    AXP517Irq _irq;                      ///< IRQ helper.
    AXP517Charger _charger;              ///< Charger helper.
    AXP517Bc12 _bc12;                    ///< BC1.2 detection helper.
    AXP517Led _led;                      ///< Charge LED helper.
    AXP517Power _power;                  ///< Power-path helper.
    AXP517Gpio _gpio;                    ///< GPIO helper.
    AXP517Bc12 _bc12_2;                  ///< Secondary BC1.2 helper alias.
    AXP517Tcpc _tcpc;                    ///< Type-C Port Controller helper.
    AXP517Pd _pd;                        ///< USB-PD message helper.
    AXP517PdNegotiator _pdNegotiator;    ///< USB-PD sink negotiation helper.
};

#endif

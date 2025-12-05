/**
 *
 * @license MIT License
 *
 * Copyright (c) 2025 lewis he
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
 * @file      GaugeAXP2602.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-12-05
 */

#pragma once

#include "REG/AXP2602Constants.h"
#include "SensorPlatform.hpp"

// AXP2602 Unique device address
static constexpr uint8_t AXP2602_SLAVE_ADDRESS  = (0x62);

class GaugeAXP2602 : public AXP2602Constants
{
public:
    enum CurrentSenseResistor {
        SENSE_RESISTOR_10_MOHM = 0x00,
        SENSE_RESISTOR_5_MOHM = 0x01,
        SENSE_RESISTOR_20_MOHM = 0x02,
        SENSE_RESISTOR_40_MOHM = 0x03
    };

    enum  IRQStatus {
        IRQ_ERROR               = 0x00,
        IRQ_WDT_TIMEOUT         = 0x08,
        IRQ_OVER_TEMPERATURE    = 0x04,
        IRQ_SOC_UPDATE          = 0x02,
        IRQ_LOW_BATTERY         = 0x01
    };

    enum OperatingMode {
        OPERATING_MODE_HIGH_PRECISION = 0x00,
        OPERATING_MODE_NORMAL = 0x02,
        OPERATING_MODE_LOW_POWER = 0x06,
        OPERATING_MODE_ERROR = 0xFF
    };

    GaugeAXP2602() : comm(nullptr), hal(nullptr) {}

    ~GaugeAXP2602()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    /**
     * @brief  Begin with Arduino TwoWire instance.
     * @note   This function initializes the AXP2602 chip with the specified I2C parameters.
     * @param  &wire: The TwoWire instance to use for I2C communication.
     * @param  sda: The SDA pin number (default is -1).
     * @param  scl: The SCL pin number (default is -1).
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(TwoWire &wire, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, AXP2602_SLAVE_ADDRESS, sda, scl)) {
            return false;
        }
        return initImpl();
    }

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    /**
     * @brief  Begin with ESP-IDF I2C port number.
     * @note   This function initializes the AXP2602 chip with the specified I2C parameters.
     * @param  port_num: The I2C port number to use.
     * @param  sda: The SDA pin number (default is -1).
     * @param  scl: The SCL pin number (default is -1).
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(i2c_port_t port_num, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, AXP2602_SLAVE_ADDRESS, sda, scl)) {
            return false;
        }
        return initImpl();
    }
#else
    /**
     * @brief  Begin with ESP-IDF I2C master bus handle.
     * @note   This function initializes the AXP2602 chip with the specified I2C parameters.
     * @param  handle: The I2C master bus handle to use.
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(i2c_master_bus_handle_t handle)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, AXP2602_SLAVE_ADDRESS)) {
            return false;
        }
        return initImpl();
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    /**
     * @brief  Begin with custom callback functions.
     * @note   This function initializes the AXP2602 chip with user-defined callback functions.
     * @param  callback: The custom callback function for communication.
     * @param  hal_callback: The custom callback function for hardware abstraction layer.
     * @retval True if initialization is successful, false otherwise.
     */
    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
                callback, hal_callback, AXP2602_SLAVE_ADDRESS, comm, hal)) {
            return false;
        }
        return initImpl();
    }

    /**
     * @brief  Get the chip ID.
     * @note   This function retrieves the unique identifier for the AXP2602 chip.
     * @retval The chip ID as an integer. Default value is 0x1C for AXP2602.
     */
    int getChipID()
    {
        return comm->readRegister(REG_ID);
    }

    /**
     * @brief  Reset Gauge AXP2602.
     * @note   This function resets the AXP2602 chip.
     * @param  resetMcu: Retained, meaning unclear.
     * @retval None
     */
    void reset(bool resetMcu = false)
    {
        int val = comm->readRegister(REG_MODE);
        if (val < 0) {
            return;
        }
        if (resetMcu) {
            val |= _BV(5);
        } else {
            val &= ~_BV(5);
        }
        comm->writeRegister(REG_MODE, (val | 0x20));
        hal->delay(100);
    }

    /**
     * @brief  Put the AXP2602 chip into sleep mode.
     * @note   This function puts the AXP2602 chip into sleep mode to conserve power.
     * @retval None
     */
    void sleep()
    {
        int val = comm->readRegister(REG_MODE);
        if (val < 0) {
            return;
        }
        comm->writeRegister(REG_MODE, (val | 0x01));
    }

    /**
     * @brief  Wake up the AXP2602 chip.
     * @note   This function wakes up the AXP2602 chip from sleep mode.
     * @retval None
     */
    void wakeup()
    {
        int val = comm->readRegister(REG_MODE);
        if (val < 0) {
            return;
        }
        comm->writeRegister(REG_MODE, (val & ~0x01));
    }

    /**
     * @brief  Get the temperature.
     * @note   This function retrieves the temperature reading from the AXP2602 chip.
     * @retval The temperature in degrees Celsius (°C).
     * @note   IC temperature, 1 LSB = 1℃, range -128℃ to +127℃
     */
    int8_t getTemperature()
    {
        return comm->readRegister(REG_TEMP_RESULT);
    }

    /**
     * @brief  Get the battery health status.
     * @note   This function retrieves the battery health status from the AXP2602 chip.
     * @retval The battery health status as an 8-bit unsigned integer.
     */
    uint8_t getBatteryHealthStatus()
    {
        return comm->readRegister(REG_SOH);
    }

    /**
     * @brief  Get the battery percentage.
     * @note   This function retrieves the battery percentage from the AXP2602 chip.
     * @retval The battery percentage as an 8-bit unsigned integer.
     * @note   Battery meter result, remaining battery percentage, 1 LSB = 1%
     */
    uint8_t getBatteryPercentage()
    {
        return comm->readRegister(REG_SOC);
    }

    /**
     * @brief  Get the battery time to empty.
     * @note   This function retrieves the estimated time remaining until the battery is empty from the AXP2602 chip.
     * @retval The battery time to empty in minutes.
     * @note   The remaining time calculated by the fuel meter is the lower 8 bits, where 1 LSB = 1 minute.
     */
    uint16_t getBatteryTimeToEmpty()
    {
        int high = comm->readRegister(REG_TIME_TO_EMPTY_HIGH);
        int low = comm->readRegister(REG_TIME_TO_EMPTY_LOW);
        if (high < 0 || low < 0) {
            return 0;
        }
        return (high << 8) | low;
    }

    /**
     * @brief  Get the battery time to full.
     * @note   This function retrieves the estimated time remaining until the battery is fully charged from the AXP2602 chip.
     * @retval The battery time to full in minutes.
     * @note   The remaining time calculated by the fuel meter is the lower 8 bits, where 1 LSB = 1 minute.
     */
    uint16_t getBatteryTimeToFull()
    {
        int high = comm->readRegister(REG_TIME_TO_FULL_HIGH);
        int low = comm->readRegister(REG_TIME_TO_FULL_LOW);
        if (high < 0 || low < 0) {
            return 0;
        }
        return (high << 8) | low;
    }

    /**
     * @brief  Get the low battery SOC threshold.
     * @note   This function retrieves the user-defined low battery IRQ threshold from the AXP2602 chip.
     * @retval The low battery SOC threshold as an 8-bit unsigned integer.
     * @note   The valid range is 0% to 63%. If the SOC is below this value, a low battery alarm IRQ is triggered.
     */
    uint16_t getLowBatterySOCThreshold()
    {
        int val = comm->readRegister(REG_LOW_SOC_THLD);
        if (val < 0) {
            return 0;
        }
        return (val & 0x1F);
    }

    /**
     * @brief  Set the low battery SOC threshold.
     * @note   This function sets the user-defined low battery IRQ threshold in the AXP2602 chip.
     * @param  threshold The low battery SOC threshold as an 8-bit unsigned integer (0% to 63%).
     * @note   If the SOC is below this value, a low battery alarm IRQ is triggered.
     */
    void setLowBatterySOCThreshold(uint8_t threshold)
    {
        if (threshold > 63) {
            threshold = 63;
        }
        comm->writeRegister(REG_LOW_SOC_THLD, threshold & 0x1F);
    }

    /**
     * @brief  Get the over-temperature threshold.
     * @note   This function retrieves the user-defined over-temperature IRQ threshold from the AXP2602 chip.
     * @retval The over-temperature threshold as an 8-bit unsigned integer.
     * @note   The valid range is 0°C to 127°C. If the temperature exceeds this value, an over-temperature alarm IRQ is triggered.
     */
    uint16_t getOverTemperatureThreshold()
    {
        int val = comm->readRegister(REG_OT_THLD);
        if (val < 0) {
            return 0;
        }
        return (val & 0x7F);
    }

    /**
     * @brief  Set the over-temperature threshold.
     * @note   This function sets the user-defined over-temperature IRQ threshold in the AXP2602 chip.
     * @param  threshold The over-temperature threshold as an 8-bit unsigned integer (0°C to 127°C).
     * @note   If the temperature exceeds this value, an over-temperature alarm IRQ is triggered.
     */
    void setOverTemperatureThreshold(uint8_t threshold)
    {
        if (threshold > 127) {
            threshold = 127;
        }
        comm->writeRegister(REG_OT_THLD, threshold & 0x7F);
    }

    /**
     * @brief  Enable or disable the thermal die measurement.
     * @note   This function enables or disables the thermal die measurement feature in the AXP2602 chip.
     * @param  enable A boolean value to enable (true) or disable (false) the thermal die measurement.
     * @note   default is enabled.
     */
    void setThermalDie(bool enable)
    {
        int val = comm->readRegister(REG_COMM_CONFIG);
        if (val < 0) {
            return;
        }
        if (enable) {
            val |= _BV(6);
        } else {
            val &= ~_BV(6);
        }
        comm->writeRegister(REG_COMM_CONFIG, val);
    }

    /**
     * @brief  Get the thermal die temperature.
     * @note   This function retrieves the thermal die temperature from the AXP2602 chip.
     * @retval The thermal die temperature in degrees Celsius.
     */
    uint16_t getThermalDieTemperature()
    {
        int high = comm->readRegister(REG_TDIE_ADC_HIGH);
        int low = comm->readRegister(REG_TDIE_ADC_LOW);
        if (high < 0 || low < 0) {
            return 0;
        }
        return ((high & 0x1F) << 8) | low;
    }

    /**
     * @brief  Enable or disable battery detection.
     * @note   This function enables or disables the battery detection feature in the AXP2602 chip.
     * @param  enable A boolean value to enable (true) or disable (false) battery detection.
     * @note   default is enabled.
     */
    void setCurrentMeasurement(bool enable)
    {
        int val = comm->readRegister(REG_COMM_CONFIG);
        if (val < 0) {
            return;
        }
        if (enable) {
            val |= _BV(5);
        } else {
            val &= ~_BV(5);
        }
        comm->writeRegister(REG_COMM_CONFIG, val);
    }

    /**
     * @brief  Set the battery detection feature.
     * @note   This function enables or disables the battery detection feature in the AXP2602 chip.
     * @param  enable A boolean value to enable (true) or disable (false) the battery detection.
     * @note   default is enabled.
     */
    void setBatteryDetection(bool enable)
    {
        int val = comm->readRegister(REG_COMM_CONFIG);
        if (val < 0) {
            return;
        }
        if (enable) {
            val |= _BV(4);
        } else {
            val &= ~_BV(4);
        }
        comm->writeRegister(REG_COMM_CONFIG, val);
    }

    /**
     * @brief  Set the current sense resistor value.
     * @note   This function sets the current sense resistor value in the AXP2602 chip.
     * @param  resistorValue: The desired current sense resistor value.see enum CurrentSenseResistor.
     * @retval None
     */
    void setCurrentSenseResistor(CurrentSenseResistor resistorValue)
    {
        comm->writeRegister(REG_COMM_CONFIG, resistorValue & 0x02);
    }

    /**
     * @brief  Get the battery voltage.
     * @note   This function reads the battery voltage from the AXP2602 registers.
     * @retval The battery voltage in millivolts (mV).
     */
    uint16_t getBatteryVoltage()
    {
        int high = comm->readRegister(REG_VBAT_ADC_HIGH);
        int low = comm->readRegister(REG_VBAT_ADC_LOW);
        if (high < 0 || low < 0) {
            return 0;
        }
        return ((high & 0x3F) << 8) | low;
    }

    /**
     * @brief  Get the battery current.
     * @note   This function reads the battery current from the AXP2602 registers.
     * @retval The battery current in milliamperes (mA).
     */
    uint16_t getBatteryCurrent()
    {
        int high = comm->readRegister(REG_IBAT_ADC_HIGH);
        int low = comm->readRegister(REG_IBAT_ADC_LOW);
        if (high < 0 || low < 0) {
            return 0;
        }
        return ((high << 8) | low);
    }

    /**
     * @brief  Get the IRQ status.
     * @note   This function reads the IRQ status from the AXP2602 registers.
     * @retval The IRQ status,see enum IRQStatus.
     */
    IRQStatus getIRQStatus()
    {
        int val = comm->readRegister(REG_IRQ_STATUS);
        if (val < 0) {
            return IRQ_ERROR;
        }
        return static_cast<IRQStatus>(val & 0x0F);
    }

    /**
     * @brief  Enable or disable specific IRQs.
     * @note   This function enables or disables specific IRQs in the AXP2602 chip.
     * @param  irq The IRQ to enable or disable,see enum IRQStatus.
     * @param  enable A boolean value to enable (true) or disable (false) the specified IRQ.
     * @param  low_level_trigger A boolean value to set the IRQ trigger level. true for low level trigger, false for low pulse trigger.
     * @note   default is low level trigger.
     */
    void setIRQEnable(IRQStatus irq, bool enable, bool low_level_trigger = true)
    {
        int val = comm->readRegister(REG_IRQ_ENABLE);
        if (val < 0) {
            return;
        }
        if (low_level_trigger) {
            val |= 0x80;
        } else {
            val &= ~0x80;
        }
        if (enable) {
            val |= irq;
        } else {
            val &= ~irq;
        }
        comm->writeRegister(REG_IRQ_ENABLE, val);
    }

    /**
     * @brief  Get the IRQ status.
     * @note   This function reads the IRQ status from the AXP2602 registers.
     * @retval The IRQ status,see enum IRQStatus.
     */
    IRQStatus getIRQStatus()
    {
        int val = comm->readRegister(REG_IRQ_STATUS);
        if (val < 0) {
            return IRQ_ERROR;
        }
        return static_cast<IRQStatus>(val & 0x0F);
    }

    /**
     * @brief  Clear the IRQ status.
     * @note   This function clears all IRQ status flags in the AXP2602 chip.
     */
    void clearIRQStatus()
    {
        comm->writeRegister(REG_IRQ_ENABLE, 0xFF);
        hal->delay(2);
        comm->writeRegister(REG_IRQ_ENABLE, 0x00);
    }

    /**
     * @brief  Get the operating mode.
     * @note   This function retrieves the current operating mode of the AXP2602 chip.
     * @retval The current operating mode, see enum OperatingMode.
     */
    OperatingMode getOperatingMode()
    {
        int val = comm->readRegister(REG_OPERATING_MODE);
        if (val < 0) {
            return OPERATING_MODE_ERROR;
        }
        return static_cast<OperatingMode>(val & 0x07);
    }

    /**
     * @brief  Set the operating mode.
     * @note   This function sets the operating mode of the AXP2602 chip.
     * @param  mode: The operating mode to set, see enum OperatingMode.
     * @retval None
     */
    void setOperatingMode(OperatingMode mode)
    {
        switch (mode) {
        case OPERATING_MODE_HIGH_PRECISION:
        case OPERATING_MODE_NORMAL:
        case OPERATING_MODE_LOW_POWER:
            break;
        default:
            return;
        }
        comm->writeRegister(REG_OPERATING_MODE, mode);
    }

private:
    bool initImpl()
    {
        int chipID = getChipID();
        if (chipID != AXP2602_CHIP_ID) {
            log_e("Chip id not match : %02X\n", chipID);
            return false;
        }
        setCurrentSenseResistor(SENSE_RESISTOR_10_MOHM);
        setOperatingMode(OPERATING_MODE_NORMAL);
        return true;
    }
protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
};

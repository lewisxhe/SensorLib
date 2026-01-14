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
        IRQ_STATUS_NONE                = 0x00,
        IRQ_STATUS_LOW_BATTERY         = 0x01,
        IRQ_STATUS_SOC_UPDATE          = 0x02,
        IRQ_STATUS_OVER_TEMPERATURE    = 0x03,
        IRQ_STATUS_WDT_TIMEOUT         = 0x04
    };

    enum IRQEnable {
        IRQ_ENABLE_WDT_TIMEOUT   = 0x08,
        IRQ_ENABLE_OVER_TEMPERATURE = 0x04,
        IRQ_ENABLE_SOC_UPDATE    = 0x02,
        IRQ_ENABLE_LOW_BATTERY   = 0x01
    };
    enum OperatingMode {
        // It has relatively high power consumption, but can be used for both charging and discharging.
        OPERATING_MODE_HIGH_PRECISION = 0x00,
        // In default mode, it can be used for both charging and discharging.
        OPERATING_MODE_NORMAL = 0x02,
        // With relatively low power consumption, it can only be used in low-power scenarios during discharge.
        // In other scenarios, software switching to normal mode or high-precision mode is required.
        OPERATING_MODE_LOW_POWER = 0x06,
    };

    struct GaugeData {

        uint8_t chipID;              // Chip ID
        // Voltage, current, and temperature measurements
        uint16_t voltage;           // mV
        int16_t currentRaw;         // mA (Signed, two's complement representation)
        int8_t temperature;         // °C
        uint16_t tdieTemperatureRaw;   // Thermosensitive Temperature ADC Raw Value

        // Battery status information
        uint8_t soc;                // Battery percentage (0-100)
        uint8_t soh;                // Battery health status (0-100)
        uint16_t timeToEmpty;       // Time remaining (minutes)
        uint16_t timeToFull;        // Full time (minutes)

        // IRQ status
        IRQStatus irqStatus;

        // Operating mode
        OperatingMode operatingMode;

        // Configuration status
        bool thermalDieEnabled;
        bool currentMeasurementEnabled;
        bool batteryDetectionEnabled;
        bool sleepModeEnabled;
        CurrentSenseResistor senseResistor;

        float current;              // Actual current calculated from the sampling resistor（mA）
        float actualDieTemperature; // die temperature calculated according to the formula（℃）
        float power;                // Instantaneous power (W), positive indicates discharging, negative indicates charging.

        GaugeData() :
            chipID(0),
            voltage(0),
            currentRaw(0),
            temperature(0),
            tdieTemperatureRaw(0),
            soc(0),
            soh(0),
            timeToEmpty(0),
            timeToFull(0),
            irqStatus(IRQ_STATUS_NONE),
            operatingMode(OPERATING_MODE_NORMAL),
            thermalDieEnabled(true),
            currentMeasurementEnabled(true),
            batteryDetectionEnabled(true),
            sleepModeEnabled(false),
            senseResistor(SENSE_RESISTOR_10_MOHM),
            current(0.0f),
            actualDieTemperature(0.0f),
            power(0.0f)
        {}
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
     * @brief  Refresh the AXP2602 chip data.
     * @note   This function reads the latest data from the AXP2602 chip registers.
     * @retval True if refresh is successful, false otherwise.
     */
    bool refresh()
    {
        if (!comm) return false;

        if (data.chipID != AXP2602_CHIP_ID) {
            log_e("Chip id not match : %02X\n", data.chipID);
            return false;
        }

        if (data.sleepModeEnabled) {
            // Chip is in sleep mode, no need to read other registers
            return true;
        }

        // Read voltage (14-bit ADC, 0-4500mV)
        int vbatHigh = comm->readRegister(REG_VBAT_ADC_HIGH);
        int vbatLow = comm->readRegister(REG_VBAT_ADC_LOW);
        if (vbatHigh >= 0 && vbatLow >= 0) {
            data.voltage = ((vbatHigh & 0x3F) << 8) | vbatLow;  // High 6 bits + Low 8 bits
        }

        // Read current (16-bit ADC, two's complement)
        int ibatHigh = comm->readRegister(REG_IBAT_ADC_HIGH);
        int ibatLow = comm->readRegister(REG_IBAT_ADC_LOW);
        if (ibatHigh >= 0 && ibatLow >= 0) {
            uint16_t raw = (ibatHigh << 8) | ibatLow;
            data.currentRaw = static_cast<int16_t>(raw);  // Convert to signed 16-bit

            // Calculate actual current value (mA) based on sense resistor
            float resolution = getCurrentResolution(data.senseResistor);
            data.current = data.currentRaw * resolution;
        }

        // Read temperature
        int temp = comm->readRegister(REG_TEMP_RESULT);
        if (temp >= 0) {
            // 1 LSB = 1°C, range -128°C to +127°C
            data.temperature = static_cast<int8_t>(temp);
        }

        // Read die temperature (13-bit ADC)
        int tdieHigh = comm->readRegister(REG_TDIE_ADC_HIGH);
        int tdieLow = comm->readRegister(REG_TDIE_ADC_LOW);
        if (tdieHigh >= 0 && tdieLow >= 0) {
            data.tdieTemperatureRaw = ((tdieHigh & 0x1F) << 8) | tdieLow;  // High 5 bits + Low 8 bits
            // Formula on page 10 of the manual: Temperature (°C) = (3415 - xDie) * 10 / 95 + 26
            data.actualDieTemperature = (3415.0 - data.tdieTemperatureRaw) * 10.0 / 95.0 + 26.0;
            if (data.actualDieTemperature < -40 || data.actualDieTemperature > 85) {
                log_e("Die temperature out of range: %d\n", data.actualDieTemperature);
                data.actualDieTemperature = 0; // Reset to 0 if out of range
            }
        }

        // Read battery status
        data.soc = comm->readRegister(REG_SOC);
        data.soh = comm->readRegister(REG_SOH);

        // Read time to empty estimation
        int timeEmptyHigh = comm->readRegister(REG_TIME_TO_EMPTY_HIGH);
        int timeEmptyLow = comm->readRegister(REG_TIME_TO_EMPTY_LOW);
        if (timeEmptyHigh >= 0 && timeEmptyLow >= 0) {
            data.timeToEmpty = (timeEmptyHigh << 8) | timeEmptyLow;
        }

        int timeFullHigh = comm->readRegister(REG_TIME_TO_FULL_HIGH);
        int timeFullLow = comm->readRegister(REG_TIME_TO_FULL_LOW);
        if (timeFullHigh >= 0 && timeFullLow >= 0) {
            data.timeToFull = (timeFullHigh << 8) | timeFullLow;
        }

        // Read IRQ status
        int irqStatus = comm->readRegister(REG_IRQ_STATUS);
        if (irqStatus >= 0) {
            data.irqStatus = static_cast<IRQStatus>(irqStatus & 0x0F);
        }

        // Read configuration
        int config = comm->readRegister(REG_COMM_CONFIG);
        if (config >= 0) {
            data.thermalDieEnabled = (config & _BV(6)) != 0;
            data.currentMeasurementEnabled = (config & _BV(5)) != 0;
            data.batteryDetectionEnabled = (config & _BV(4)) != 0;
            data.senseResistor = static_cast<CurrentSenseResistor>(config & 0x03);
        }

        // Calculate instantaneous power (W): P = V * I
        data.power = (data.voltage * data.current) / 1000000.0f;

        return true;
    }

    /**
    * @brief Obtain current resolution based on the sampling resistor
    * @param resistor Sampling resistor value
    * @retval Current resolution (mA/LSB)
    */
    float getCurrentResolution(CurrentSenseResistor resistor)
    {
        switch (resistor) {
        case SENSE_RESISTOR_5_MOHM:
            return 0.5f;    // 500uA/LSB = 0.5mA/LSB
        case SENSE_RESISTOR_10_MOHM:
            return 0.25f;   // 250uA/LSB = 0.25mA/LSB
        case SENSE_RESISTOR_20_MOHM:
            return 0.125f;  // 125uA/LSB = 0.125mA/LSB
        case SENSE_RESISTOR_40_MOHM:
            return 0.0625f; // 62.5uA/LSB = 0.0625mA/LSB
        default:
            return 0.25f;   // Default 10mΩ
        }
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
     * @brief  Get the temperature.
     * @retval The temperature in degrees Celsius (°C).
     */
    int8_t getTemperature()
    {
        return data.temperature;
    }

    /**
     * @brief  Get the battery health status.
     * @retval The battery health status as an 8-bit unsigned integer.
     */
    uint8_t getBatteryStatus()
    {
        return data.soh;
    }

    /**
     * @brief  Get the battery percentage.
     * @retval The battery percentage as an 8-bit unsigned integer.
     */
    uint8_t getStateOfCharge()
    {
        return data.soc;
    }

    /**
     * @brief  Get the battery time to empty.
     * @retval The battery time to empty in minutes.
     */
    uint16_t getTimeToEmpty()
    {
        return data.timeToEmpty;
    }

    /**
     * @brief  Get the battery time to full.
     * @retval The battery time to full in minutes.
     */
    uint16_t getTimeToFull()
    {
        return data.timeToFull;
    }

    /**
     * @brief  Get the low battery SOC threshold.
     * @retval The low battery SOC threshold as an 8-bit unsigned integer.
     */
    uint8_t getLowBatterySOCThreshold()
    {
        int lowSoc = comm->readRegister(REG_LOW_SOC_THLD);
        if (lowSoc >= 0) {
            return lowSoc & 0x3F;  // Low 6 bits valid
        }
        return 0;
    }

    /**
     * @brief  Get the over-temperature threshold.
     * @retval The over-temperature threshold as an 8-bit unsigned integer.
     */
    uint8_t getOverTemperatureThreshold()
    {
        int otThld = comm->readRegister(REG_OT_THLD);
        if (otThld >= 0) {
            return otThld & 0x7F;  // Low 7 bits valid
        }
        return 0;
    }

    /**
    * @brief Get the actual die temperature
    * @retval Actual temperature value (°C)
    */
    float getThermalDieTemperature()
    {
        return data.actualDieTemperature;
    }

    /**
     * @brief  Get the thermal die temperature.
     * @retval The thermal die temperature ADC value.
     */
    uint16_t getThermalDieTemperatureRaw()
    {
        return data.tdieTemperatureRaw;
    }

    /**
     * @brief  Get the battery voltage.
     * @retval The battery voltage in millivolts (mV).
     */
    uint16_t getVoltage()
    {
        return data.voltage;
    }


    /**
    * @brief Checks if the battery is charging
    * @retval Returns true for charging, false for discharging
    */
    bool isCharging()
    {
        return data.current > 0;
    }

    /**
    * @brief Checks if the battery is discharging
    * @retval Returns true for discharging and false for charging
    */
    bool isDischarging()
    {
        return data.current < 0;
    }

    /**
    * @brief Get charging power (positive or 0)
    * @retval Charging power (W), returns 0 if not charging
    */
    float getChargingPower()
    {
        return (data.current > 0) ? data.power : 0.0f;
    }

    /**
     * @brief Get discharging power (positive or 0)
     * @retval Discharging power (W), returns 0 if not discharging
     */
    float getDischargingPower()
    {
        return (data.current < 0) ? -data.power : 0.0f;
    }

    /**
     * @brief Get the absolute power (always positive)
     * @retval The absolute power (W)
     */
    float getAbsolutePower()
    {
        return fabs(data.power);
    }

    /**
    * @brief Get the actual current value (considering the sampling resistor)
    * @retval Actual current value (mA)
    */
    float getCurrent()
    {
        return data.current;
    }

    /**
     * @brief  Get the battery current.
     * @retval The battery current ADC value.
     */
    int16_t  getCurrentRaw()
    {
        return data.currentRaw;
    }

    /**
     * @brief  Reset Gauge AXP2602.
     */
    void reset()
    {
        comm->setRegisterBit(REG_MODE, 5);
        hal->delay(50);
        comm->clrRegisterBit(REG_MODE, 5);
        hal->delay(100);
    }

    /**
     * @brief  Put the AXP2602 chip into sleep mode.
     */
    void sleep()
    {
        if (comm->setRegisterBit(REG_MODE, 0)) {
            data.sleepModeEnabled = true;
        }
    }

    /**
     * @brief  Check if the AXP2602 chip is in sleep mode.
     * @retval True if the chip is in sleep mode, false otherwise.
     */
    bool isSleepModeEnabled()
    {
        return comm->getRegisterBit(REG_MODE, 0);
    }

    /**
     * @brief  Wake up the AXP2602 chip.
     */
    void wakeup()
    {
        if (comm->clrRegisterBit(REG_MODE, 0)) {
            data.sleepModeEnabled = false;
        }
    }

    /**
     * @brief  Set the low battery SOC threshold.
     * @param  threshold The low battery SOC threshold (0% to 63%).
     */
    void setLowBatterySOCThreshold(uint8_t threshold)
    {
        if (threshold > 63) {
            threshold = 63;
        }
        // Read low SOC threshold
        int lowSoc = comm->readRegister(REG_LOW_SOC_THLD);
        if (lowSoc >= 0) {
            lowSoc &= 0xC0;
            lowSoc |= (threshold & 0x3F); // Low 6 bits valid
            comm->writeRegister(REG_LOW_SOC_THLD, lowSoc);
        }
    }

    /**
     * @brief  Set the over-temperature threshold.
     * @param  threshold The over-temperature threshold (0°C to 127°C).
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
     * @param  enable A boolean value to enable (true) or disable (false) the thermal die measurement.
     */
    void setThermalDieMeasurement(bool enable)
    {
        if (enable) {
            comm->setRegisterBit(REG_COMM_CONFIG, 6);
        } else {
            comm->clrRegisterBit(REG_COMM_CONFIG, 6);
        }
        data.thermalDieEnabled = enable;
    }

    /**
     * @brief  Check if thermal die measurement is enabled.
     * @retval True if thermal die measurement is enabled, false otherwise.
     */
    bool isThermalDieMeasurementEnabled()
    {
        return data.thermalDieEnabled;
    }

    /**
     * @brief  Enable or disable battery current detection.
     * @param  enable A boolean value to enable (true) or disable (false) battery current detection.
     */
    void setCurrentMeasurement(bool enable)
    {
        if (enable) {
            comm->setRegisterBit(REG_COMM_CONFIG, 5);
        } else {
            comm->clrRegisterBit(REG_COMM_CONFIG, 5);
        }
        data.currentMeasurementEnabled = enable;
    }

    /**
     * @brief  Check if battery current measurement is enabled.
     * @retval True if battery current measurement is enabled, false otherwise.
     */
    bool isCurrentMeasurementEnabled()
    {
        return data.currentMeasurementEnabled;
    }

    /**
     * @brief  Enable or disable battery voltage detection.
     * @param  enable A boolean value to enable (true) or disable (false) the battery detection.
     */
    void setBatteryDetection(bool enable)
    {
        if (enable) {
            comm->setRegisterBit(REG_COMM_CONFIG, 4);
        } else {
            comm->clrRegisterBit(REG_COMM_CONFIG, 4);
        }
        data.batteryDetectionEnabled = enable;
    }

    /**
     * @brief  Check if battery voltage measurement is enabled.
     * @retval True if battery voltage measurement is enabled, false otherwise.
     */
    bool isBatteryVoltageMeasurementEnabled()
    {
        return data.batteryDetectionEnabled;
    }

    /**
     * @brief  Set the current sense resistor value.
     * @param  resistorValue: The desired current sense resistor value.
     */
    void setCurrentSenseResistor(CurrentSenseResistor resistorValue)
    {
        int val = comm->readRegister(REG_COMM_CONFIG);
        if (val < 0) return;

        val = (val & ~0x03) | (resistorValue & 0x03);
        comm->writeRegister(REG_COMM_CONFIG, val);
        data.senseResistor = resistorValue;
    }

    /**
     * @brief  Get the current sense resistor value.
     * @retval The current sense resistor value.
     */
    CurrentSenseResistor getCurrentSenseResistor()
    {
        return data.senseResistor;
    }


    /**
     * @brief  Get the IRQ status.
     * @retval The IRQ status.
     */
    IRQStatus getIRQStatus()
    {
        return data.irqStatus;
    }

    /**
     * @brief  Enable a specific IRQ.
     * @param  irq The IRQ to enable.
     */
    void enableIRQ(IRQEnable irq)
    {
        enableIRQ(irq, true);
    }

    /**
     * @brief  Disable a specific IRQ.
     * @param  irq The IRQ to disable.
     */
    void disableIRQ(IRQEnable irq)
    {
        enableIRQ(irq, false);
    }

    /**
     * @brief  Disable all IRQs.
     * @note   This function disables all IRQs by writing 0x00 to the IRQ_ENABLE register.
     */
    void disableAllIRQ()
    {
        comm->writeRegister(REG_IRQ_ENABLE, 0x00);
    }

    /**
    * @brief  Enable or disable specific IRQs.
    * @param  irq The IRQ to enable or disable.
    * @param  enable A boolean value to enable (true) or disable (false) the specified IRQ.
    * @param  low_level_trigger A boolean value to set the IRQ trigger level.
    */
    void enableIRQ(IRQEnable irq, bool enable, bool low_level_trigger = true)
    {
        int val = comm->readRegister(REG_IRQ_ENABLE);
        if (val < 0) {
            return;
        }
        // Set trigger mode: bit7=0 for low level, bit7=1 for low pulse.
        if (low_level_trigger) {
            val &= ~0x80;  // Low level trigger
        } else {
            val |= 0x80;   // Low pulse trigger
        }
        // Set IRQ enable bit
        if (enable) {
            val |= irq;
        } else {
            val &= ~irq;
        }
        comm->writeRegister(REG_IRQ_ENABLE, val);
    }

    /**
     * @brief  Clear the IRQ status.
     */
    void clearIRQStatus()
    {
        comm->writeRegister(REG_IRQ_STATUS, 0x00);
        hal->delay(2);
        comm->writeRegister(REG_IRQ_STATUS, 0x0F);
        data.irqStatus = IRQ_STATUS_NONE;
    }

    /**
     * @brief  Set the operating mode.
     * @param  mode: The operating mode to set.
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
        data.operatingMode = mode;
    }

    /**
    * @brief  Get the operating mode.
    * @retval The current operating mode.
    */
    OperatingMode getOperatingMode()
    {
        // Read operating mode
        int opMode = comm->readRegister(REG_OPERATING_MODE);
        if (opMode >= 0) {
            data.operatingMode = static_cast<OperatingMode>(opMode & 0x07);
        }
        return data.operatingMode;
    }

    /**
    * @brief  Write Gauge Data
    * @note   This function writes the provided gauge data to battery parameter.
    * @param  *data: Pointer to the data buffer to compare.
    * @param  len: Length of the data buffer (should be 128 bytes).
    * @retval True if the data successfully wrote to ROM by comparing with the written data, false otherwise.
    */
    bool writeGaugeData(uint8_t *data, uint8_t len)
    {
        if (len != 128 || data == NULL)return false;
        // Reset gauge first
        comm->writeRegister(REG_MODE, 0x01);
        hal->delay(50);
        comm->writeRegister(REG_MODE, 0x00);
        hal->delay(50);

        // Enabled BROM register
        comm->clrRegisterBit(REG_PARA_CONFIG, 0);
        comm->setRegisterBit(REG_PARA_CONFIG, 0);
        // Write data to buffer
        for (uint8_t i = 0; i < 128; i++) {
            comm->writeRegister(REG_BROM, data[i]);
        }

        // Reenable ROM register
        comm->clrRegisterBit(REG_PARA_CONFIG, 0);
        comm->setRegisterBit(REG_PARA_CONFIG, 0);

        return compareGaugeData(data, len);
    }

    /**
    * @brief  Compare Gauge Data
    * @note   This function compares the provided gauge data with the data in the gauge.
    * @param  *data: Pointer to the data buffer to compare.
    * @param  len: Length of the data buffer (should be 128 bytes).
    * @retval True if the data matches, false otherwise.
    */
    bool compareGaugeData(uint8_t *data, uint8_t len)
    {
        if (len != 128 || data == NULL)return false;
        // Reset gauge to load new data
        uint8_t buffer[128];
        memset(buffer, 0, sizeof(buffer));
        // Reenable BROM register
        comm->clrRegisterBit(REG_PARA_CONFIG, 0);
        comm->setRegisterBit(REG_PARA_CONFIG, 0);
        for (uint8_t i = 0; i < 128; i++) {
            buffer[i] = comm->readRegister(REG_BROM);
        }
        // Disable BROM register
        comm->clrRegisterBit(REG_PARA_CONFIG, 0);
        // Set data interface
        comm->setRegisterBit(REG_PARA_CONFIG, 4);
        // Reset gauge
        comm->setRegisterBit(REG_MODE, 5);
        comm->clrRegisterBit(REG_MODE, 5);

        return memcmp(data, buffer, 128) == 0;
    }

    void enableWDT()
    {
        comm->setRegisterBit(REG_PARA_CONFIG, 5);
    }

    void disableWDT()
    {
        comm->clrRegisterBit(REG_PARA_CONFIG, 5);
    }

private:
    bool initImpl()
    {
        data.chipID = getChipID();
        if (data.chipID != AXP2602_CHIP_ID) {
            log_e("Chip id not match : %02X\n", data.chipID);
            return false;
        }
        reset();    // Reset Configuration
        hal->delay(100);
        wakeup();   // Wake Up Chip
        setCurrentSenseResistor(SENSE_RESISTOR_10_MOHM);
        setOperatingMode(OPERATING_MODE_NORMAL);
        data.sleepModeEnabled = isSleepModeEnabled();
        return true;
    }

protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
    GaugeData data;
};

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
 * @file      GaugeBQ27220.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-16
 *
 */
#pragma once

#include "REG/BQ27220Constants.h"
#include "SensorPlatform.hpp"

// BQ27220 Unique device address
static constexpr uint8_t BQ27220_SLAVE_ADDRESS  = (0x55);

typedef struct  {
    //* Reserved bits
    // bool RSV[6];
    //* The fuel gauge is in CONFIG UPDATE mode. Fuel gauging is suspended.
    bool CFGUPDATE;
    //* Reserved bits
    // bool RSV2;
    //* Flag indicating that the BTP threshold has been exceeded.
    bool BTPINT;
    //* Indicates that the RemainingCapacity() accumulation is currently being throttled by the smoothing engine.
    bool SMTH;
    //* Indicates whether the fuel gauge initialization is complete. This bit can only be set when a
    //* battery is present. True when set.
    bool INITCOMP;
    //* Indicates whether the current discharge cycle complies or does not comply with the requirements of the FCC update.
    //* The discharge cycle that is effective for the FCC update is set.
    bool VDQ;
    //* Indicates whether the measured battery voltage is above or below the EDV2 threshold. If set, indicates below.
    bool EDV2;
    //* Define current security access
    //* 11 = Sealed Access
    //* 10 = Unblock access
    //* 01 = Full access
    uint8_t SEC;
    //* Use the 0x2D command to toggle to enable/disable CALIBRATION mode
    bool CALMD;
} OperationStatus_t;

class BatteryStatus {
    protected:
        struct {
            //* Full discharge detected. This flag is set and cleared based on the SOC
            //* Flag Config B option selected.
            bool fullDischargeDetected;
            //* OCV measurement update is complete. True when set
            bool ocvMeasurementUpdateComplete;
            //* Status bit indicating that an OCV read failed due to current.
            //* This bit can only be set if a battery is present after receiving an OCV_CMD(). True when set
            bool ocvReadFailedDueToCurrent;
            //* The device operates in SLEEP mode when set.
            //* This bit will be temporarily cleared during AD measurements in SLEEP mode.
            bool inSleepMode;
            //* Over-temperature is detected during charging. If Operation Config B [INT_OT] bit = 1,
            //* the SOC_INT pin toggles once when the [OTC] bit is set.
            bool overTemperatureDuringCharging;
            //* Over-temperature detected during discharge condition. True when set. If Operation Config B [INT_OT] bit = 1,
            //* the SOC_INT pin toggles once when the [OTD] bit is set.
            bool overTemperatureDuringDischarge;
            //* Full charge detected. This flag is set and cleared based on the SOC Flag Config A and SOC Flag Config B options selected.
            bool fullChargeDetected;
            //* Charge Inhibit: If set, indicates that charging should not begin because the Temperature() is outside the range
            //* [Charge Inhibit Temp Low, Charge Inhibit Temp High]. True when set
            bool chargeInhibited;
            //* Termination of charging alarm. This flag is set and cleared based on the selected SOC Flag Config A option.
            bool chargingTerminationAlarm;
            //* A good OCV measurement was made. True when set
            bool goodOcvMeasurement;
            //* Detects inserted battery. True when set.
            bool batteryInserted;
            //* Battery presence detected. True when set.
            bool batteryPresent;
            //* Termination discharge alarm. This flag is set and cleared according to the selected SOC Flag Config A option.
            bool dischargeTerminationAlarm;
            //* System shutdown bit indicating that the system should be shut down. True when set. If set, the SOC_INT pin toggles once.
            bool systemShutdownRequired;
            //* When set, the device is in DISCHARGE mode; when cleared, the device is in CHARGING or RELAXATION mode.
            bool inDischargeMode;
        } status;
    public:
        BatteryStatus(uint16_t bitmaps){
            uint8_t hsb = (bitmaps >> 8) & 0xFF;
            uint8_t lsb = bitmaps & 0xFF;
            status.fullDischargeDetected = (hsb & _BV(7)) != 0;
            status.ocvMeasurementUpdateComplete = (hsb & _BV(6)) != 0;
            status.ocvReadFailedDueToCurrent = (hsb & _BV(5)) != 0;
            status.inSleepMode = (hsb & _BV(4)) != 0;
            status.overTemperatureDuringCharging = (hsb & _BV(3)) != 0;
            status.overTemperatureDuringDischarge = (hsb & _BV(2)) != 0;
            status.fullChargeDetected = (hsb & _BV(1)) != 0;
            status.chargeInhibited = (hsb & _BV(0)) != 0;
    
            // status.RSVD = (lsb & (1 << 7))!= 0;
            status.chargingTerminationAlarm = (lsb & _BV(6)) != 0;
            status.goodOcvMeasurement = (lsb & _BV(5)) != 0;
            status.batteryInserted = (lsb & _BV(4)) != 0;
            status.batteryPresent = (lsb & _BV(3)) != 0;
            status.dischargeTerminationAlarm = (lsb & _BV(2)) != 0;
            status.systemShutdownRequired = (lsb & _BV(1)) != 0;
            status.inDischargeMode = (lsb & _BV(0)) != 0;
        };
    
        bool isFullDischargeDetected() const { return status.fullDischargeDetected; }
        bool isOcvMeasurementUpdateComplete() const { return status.ocvMeasurementUpdateComplete; }
        bool isOcvReadFailedDueToCurrent() const { return status.ocvReadFailedDueToCurrent; }
        bool isInSleepMode() const { return status.inSleepMode; }
        bool isOverTemperatureDuringCharging() const { return status.overTemperatureDuringCharging; }
        bool isOverTemperatureDuringDischarge() const { return status.overTemperatureDuringDischarge; }
        bool isFullChargeDetected() const { return status.fullChargeDetected; }
        bool isChargeInhibited() const { return status.chargeInhibited; }
        bool isChargingTerminationAlarm() const { return status.chargingTerminationAlarm; }
        bool isGoodOcvMeasurement() const { return status.goodOcvMeasurement; }
        bool isBatteryInserted() const { return status.batteryInserted; }
        bool isBatteryPresent() const { return status.batteryPresent; }
        bool isDischargeTerminationAlarm() const { return status.dischargeTerminationAlarm; }
        bool isSystemShutdownRequired() const { return status.systemShutdownRequired; }
        bool isInDischargeMode() const { return status.inDischargeMode; }
    };

class GaugeBQ27220 : public BQ27220Constants
{
public:

    enum Access {
        FULL_ACCESS = 1,
        UNBLOCK_ACCESS,
        SEALED_ACCESS,
    };

    GaugeBQ27220() : comm(nullptr), hal(nullptr), accessKey(0xFFFFFFFF) {}

    ~GaugeBQ27220()
    {
        if (comm) {
            comm->deinit();
        }
    }

#if defined(ARDUINO)
    bool begin(TwoWire &wire, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalArduino>(comm, hal, wire, BQ27220_SLAVE_ADDRESS, sda, scl)) {
            return false;
        }
        return initImpl();
    }

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    bool begin(i2c_port_t port_num, int sda = -1, int scl = -1)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, port_num, BQ27220_SLAVE_ADDRESS, sda, scl)) {
            return false;
        }
        return initImpl();
    }
#else
    bool begin(i2c_master_bus_handle_t handle)
    {
        if (!beginCommon<SensorCommI2C, HalEspIDF>(comm, hal, handle, BQ27220_SLAVE_ADDRESS)) {
            return false;
        }
        return initImpl();
    }
#endif  //ESP_PLATFORM
#endif  //ARDUINO

    bool begin(SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback)
    {
        if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(COMM_CUSTOM,
                callback, hal_callback, BQ27220_SLAVE_ADDRESS, comm, hal)) {
            return false;
        }
        return initImpl();
    }

    // Negative value indicates discharge current value
    // The unit is mA.
    int getAtRate()
    {
        return getHalfWord(BQ27220_REG_STA_AT_RATE_TIME_TO_EMPTY);
    }

    // The unit is minutes
    uint16_t getAtRateTimeToEmpty()
    {
        return getHalfWord(BQ27220_REG_STA_AT_RATE);
    }

    // This read and write word function returns the unsigned integer value of
    // the temperature measured by the fuel gauge.
    // The unit is Celsius
    float getTemperature()
    {
        return (getHalfWord(BQ27220_REG_STA_NTC_TEMP) * 0.1f) - 273.15f;
    }

    // This value represents the measured battery pack voltage in mV, ranging from 0 to 6000mV.
    // The unit is mV.
    uint16_t getBatteryVoltage()
    {
        return getHalfWord(BQ27220_REG_STA_BAT_VOLTAGE);
    }

    // The Read Word function returns the contents of the Fuel Gauge
    // Status Register, describing the current battery status.
    // Returns the full two bytes
    uint16_t getBatteryStatusRaw()
    {
        return getHalfWord(BQ27220_REG_STA_BAT_STATUS);
    }

    BatteryStatus getBatteryStatus()
    {
        uint16_t value = getHalfWord(BQ27220_REG_STA_BAT_STATUS);
        return BatteryStatus(value);
    }

    // The instantaneous current in the sense resistor.
    // It is updated once per second. The unit is mA.
    int getInstantaneousCurrent()
    {
        return getHalfWord(BQ27220_REG_STA_CURRENT);
    }

    // The unit is mAh.
    uint16_t getRemainingCapacity()
    {
        return getHalfWord(BQ27220_REG_STA_REMAINING_CAPACITY);
    }

    // The unit is mAh
    uint16_t getFullChargeCapacity()
    {
        return getHalfWord(BQ27220_REG_STA_FULL_CHARGE_CAPACITY);
    }

    // The unit is minutes
    uint16_t getTimeToEmpty()
    {
        uint16_t value = getHalfWord(BQ27220_REG_STA_TIME_TO_EMPTY);
        if (value == 0xFFFF) {
            return 0;
        }
        return value;
    }

    // The unit is minutes
    uint16_t getTimeToFull()
    {
        uint16_t value = getHalfWord(BQ27220_REG_STA_TIME_TO_FULL);
        if (value == 0xFFFF) {
            return 0;
        }
        return value;
    }

    // Standby current measured by sense resistor
    // The unit is mA
    int getStandbyCurrent()
    {
        return getHalfWord(BQ27220_REG_STA_STANDBY_CURRENT);
    }

    // Predicted remaining battery life in minutes at standby discharge rate
    // The unit is minutes
    uint16_t getStandbyTimeToEmpty()
    {
        uint16_t value = getHalfWord(BQ27220_REG_STA_STANDBY_TIME_TO_EMPTY);
        if (value == 0xFFFF) {
            return 0;
        }
        return value;
    }

    // Signed integer value in mA at maximum load
    int getMaxLoadCurrent()
    {
        return getHalfWord(BQ27220_REG_STA_MAX_LOAD_CURRENT);
    }

    // Predicted remaining battery life in minutes at maximum load current discharge rate
    uint16_t getMaxLoadTimeToEmpty()
    {
        return getHalfWord(BQ27220_REG_STA_MAX_LOAD_TO_EMPTY);
    }

    // The amount of coulombs transferred from a battery during charge/discharge
    uint16_t getRawCoulombCount()
    {
        return getHalfWord(BQ27220_REG_STA_COULOMB_COUNT);
    }

    // The average power during battery charging and discharging.
    // Values ​​are negative during discharging and positive during charging.
    // A value of 0 means the battery is not discharging. This value is reported in mW.
    int16_t getAveragePower()
    {
        return getHalfWord(BQ27220_REG_STA_AVG_POWER);
    }

    // This read-only function returns the unsigned integer
    // value of the internal temperature sensor measured by the fuel gauge.
    // The unit is Celsius
    float getInternalTemperature()
    {
        return (getHalfWord(BQ27220_REG_STA_INTER_TEMP) * 0.1) - 273.15;
    }

    // The number of cycles the active battery has experienced, ranging from 0 to 65535.
    // A cycle occurs when the accumulated discharge ≥ the cycle threshold.
    uint16_t getCycleCount()
    {
        return getHalfWord(BQ27220_REG_STA_CYCLE_COUNT);
    }

    // This read-only function returns an unsigned integer value representing the predicted
    // remaining battery capacity as a percentage of FullChargeCapacity(), in the range 0 to 100%.
    uint16_t getStateOfCharge()
    {
        return getHalfWord(BQ27220_REG_STA_STATE_OF_CHARGE);
    }

    // This read-only function returns an unsigned integer value representing the percentage of
    // FullChargeCapacity() to DesignCapacity(), ranging from 0 to 100%
    uint16_t getStateOfHealth()
    {
        return getHalfWord(BQ27220_REG_STA_STATE_OF_HEALTH);
    }

    // This read-only function returns the unsigned integer value of the desired battery
    // charging voltage. A value of 65,535 indicates that the battery is requesting the
    // maximum voltage from the battery charger.
    uint16_t getChargingVoltage()
    {
        return getHalfWord(BQ27220_REG_STA_CHARGING_VOLTAGE);
    }

    // This read-only function returns an unsigned integer value for the desired battery
    // charging current. A value of 65,535 indicates that the battery is requesting the maximum
    // current from the battery charger.
    uint16_t getChargingCurrent()
    {
        return getHalfWord(BQ27220_REG_STA_CHARGING_CURRENT);
    }

    // This read/write word command updates the BTP setup threshold that triggers the BTP
    // interrupt in the discharge direction and sets the OperationStatus()[BTPINT] bit.
    uint16_t getBTPDischargeSet()
    {
        return getHalfWord(BQ27220_REG_STA_BTP_DISC_SET);
    }

    int setBTPDischarge(uint8_t hsb, uint8_t lsb)
    {
        uint8_t buffer[2] = {lsb, hsb};
        return comm->writeRegister(BQ27220_REG_STA_BTP_DISC_SET, buffer, arraySize(buffer));
    }

    // This read/write word command updates the BTP setup threshold that triggers the BTP
    // interrupt in the charge direction and sets the OperationStatus()[BTPINT] bit.
    uint16_t getBTPChargeSet()
    {
        return getHalfWord(BQ27220_REG_STA_BTP_CHARGE_SET);
    }

    int setBTPCharge(uint8_t hsb, uint8_t lsb)
    {
        uint8_t buffer[2] = {lsb, hsb};
        return comm->writeRegister(BQ27220_REG_STA_BTP_CHARGE_SET, buffer, arraySize(buffer));
    }

    // The Read Word function returns the contents of the internal status register
    // See datasheet page 27.
    uint16_t getOperationStatusRaw()
    {
        return getHalfWord(BQ27220_REG_STA_OPERATION_STATUS);
    }

    OperationStatus_t getOperationStatus()
    {
        OperationStatus_t status;
        uint16_t value = getOperationStatusRaw();
        uint8_t hsb = (value >> 8) & 0xFF;
        uint8_t lsb = value & 0xFF;
        // HIGH BYTE
        status.CFGUPDATE = (hsb & _BV(2)) != 0;
        // LOW BYTE
        status.BTPINT = (lsb & _BV(7)) != 0;
        status.SMTH = (lsb & _BV(6)) != 0;
        status.INITCOMP = (lsb & _BV(5)) != 0;
        status.VDQ = (lsb & _BV(4)) != 0;
        status.EDV2 = (lsb & _BV(3)) != 0;
        status.SEC = (lsb >> 1) & 0x03;
        status.CALMD = (lsb & _BV(0)) != 0;
        return status;
    }


    // This read-only function returns the value stored in Design Capacity mAh.
    // This value is used as the theoretical or nominal capacity of a new battery pack
    // and is used to calculate StateOfHealth().
    uint16_t getDesignCapacity()
    {
        return getHalfWord(BQ27220_REG_STA_DESIGN_CAPACITY);
    }

    // This read-write block will return the result data for the currently active subcommand.
    int getMACData(uint8_t *buffer, uint8_t request_len)
    {
        const uint8_t max_size = BQ27220_REG_MAC_BUFFER_END - BQ27220_REG_MAC_BUFFER_START + 1;
        if (request_len > max_size) {
            return -1;
        }
        if (!buffer) {
            return -1;
        }
        uint8_t reg = BQ27220_REG_MAC_BUFFER_START;
        return comm->writeThenRead(&reg, 1, buffer, request_len);
    }

    // This read and write function returns the checksum of the current subcommand and data block.
    // Writing to this register provides the checksum required to execute a subcommand that requires data.
    uint16_t getMACDataSum()
    {
        uint8_t sum = 0x00;
        if (comm->readRegister(BQ27220_REG_MAC_DATA_SUM, &sum, 1) < 0) {
            return 0;
        }
        return sum;
    }

    // This read and write function returns the number of MACData()
    // bytes contained in MACDataSum() as part of the response.
    uint16_t getMACDataLen()
    {
        uint8_t length = 0x00;
        if (comm->readRegister(BQ27220_REG_MAC_DATA_LEN, &length, 1) < 0) {
            return 0;
        }
        return length;
    }

    // This read-only function returns the analog counter.
    // This value is incremented each time the analog data used for calibration is updated.
    uint16_t getAnalogCount()
    {
        uint8_t count = 0x00;
        if (comm->readRegister(BQ27220_REG_ANALOG_COUNT, &count, 1) < 0) {
            return 0;
        }
        return count;
    }

    // This read-only function returns the raw data from the coulomb counter.
    uint16_t getRawCurrent()
    {
        return getHalfWord(BQ27220_REG_RAW_CURRENT);
    }

    // This read-only function returns the raw data from the battery voltage reading.
    uint16_t getRawVoltage()
    {
        return getHalfWord(BQ27220_REG_RAW_VOLTAGE);
    }

    // Subcommands
    int sendSubCommand(uint16_t subCmd, bool waitConfirm = false)
    {
        uint8_t buffer[3];
        buffer[0] = 0x00;
        buffer[1] = lowByte(subCmd);
        buffer[2] = highByte(subCmd);
        if (comm->writeBuffer(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        if (!waitConfirm) {
            hal->delay(10);
            return 0;
        }
        constexpr uint8_t statusReg = 0x00;
        int waitCount = 20;
        hal->delay(10);
        while (waitCount--) {
            comm->writeThenRead(&statusReg, 1, buffer, 2);
            uint16_t *value = reinterpret_cast<uint16_t *>(buffer);
            if (*value == 0xFFA5) {
                return 0;
            }
            hal->delay(100);
        }
        log_e("Subcommand failed!");
        return -1;
    }

    // Unlock Safe Mode
    int unsealDevice()
    {
        uint8_t cmd1[] = {0x00, 0x14, 0x04};
        if (comm->writeBuffer(cmd1, arraySize(cmd1)) < 0) {
            return -1;
        }
        hal->delay(10);
        uint8_t cmd2[] = {0x00, 0x72, 0x36};
        if (comm->writeBuffer(cmd2, arraySize(cmd2)) < 0) {
            return -1;
        }
        hal->delay(10);
        return 0;
    }

    // Set the access key. If not set, it is 0xFFFFFFFF
    void setAccessKey(uint32_t key)
    {
        accessKey = key;
    }

    // Full access key, 0xFFFFFFFF if not set
    int unsealFullAccess()
    {
        uint8_t buffer[3];
        buffer[0] = 0x00;
        buffer[1] = lowByte((accessKey >> 24));
        buffer[2] = lowByte((accessKey >> 16));
        if (comm->writeBuffer(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        hal->delay(10);
        buffer[1] = lowByte((accessKey >> 8));
        buffer[2] = lowByte((accessKey));
        if (comm->writeBuffer(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        hal->delay(10);
        return 0;
    }

    int exitSealMode()
    {
        return  sendSubCommand(BQ27220_SUB_CMD_SEALED);
    }


    /**
    * @brief Set the new design capacity and full charge capacity of the battery.
    *
    * This function is responsible for updating the design capacity and full charge capacity of the battery.
    * It first checks the device's access settings, enters the configuration update mode, writes the new capacity values
    * and checksums, and finally exits the configuration update mode. If the device was previously in a sealed state,
    * it will be restored to the sealed mode after the operation is completed.
    * For new devices, use the default key for access. If it is an encrypted device, use setAccessKey(uint32_t key) to set the key.
    * @param newDesignCapacity The new design capacity to be set, of type uint16_t.
    * @param newFullChargeCapacity The new full charge capacity to be set, of type uint16_t.
    * @return bool Returns true if the setting is successful, false otherwise.
    */
    bool setNewCapacity(uint16_t newDesignCapacity, uint16_t newFullChargeCapacity)
    {
        bool isSealed = false;

        // Check access settings
        OperationStatus_t operationStatus = getOperationStatus();
        if (operationStatus.SEC == SEALED_ACCESS) {
            isSealed = true;
            unsealDevice();
        }
        if (operationStatus.SEC != FULL_ACCESS) {
            unsealFullAccess();
        }

        // Send ENTER_CFG_UPDATE command (0x0090)
        sendSubCommand(BQ27220_SUB_CMD_ENTER_CFG_UPDATE);

        // Confirm CFUPDATE mode by polling the OperationStatus() register until Bit 2 is set.
        bool isConfigUpdate = false;
        uint32_t timeout = hal->millis() + 1500UL;
        while (timeout > hal->millis()) {
            operationStatus = getOperationStatus();
            if (operationStatus.CFGUPDATE) {
                isConfigUpdate = true;
                break;
            }
            hal->delay(100);
        }
        if (!isConfigUpdate) {
            log_e("The update mode has timed out. It may also be that the access key for full permissions is invalid!");
            return false;
        }

        // Set the design capacity
        constexpr uint8_t DesignCapacityMSB = 0x9F;
        constexpr uint8_t DesignCapacityLSB = 0x92;
        if (!setCapacity(newDesignCapacity, DesignCapacityMSB, DesignCapacityLSB)) {
            log_e("Failed to set design capacity!");
            return false;
        }
        hal->delay(10);

        // Set full charge capacity
        constexpr uint8_t FullChargeCapacityMSB = 0x9D;
        constexpr uint8_t FullChargeCapacityLSB = 0x92;
        if (!setCapacity(newFullChargeCapacity, FullChargeCapacityMSB, FullChargeCapacityLSB)) {
            log_e("Failed to set full charge capacity!");
            return false;
        }
        hal->delay(10);

        // Exit CFUPDATE mode by sending the EXIT_CFG_UPDATE_REINIT (0x0091) or EXIT_CFG_UPDATE (0x0092) command
        sendSubCommand(BQ27220_SUB_CMD_EXIT_CFG_UPDATE_REINIT);
        hal->delay(10);

        // Confirm that CFUPDATE mode has been exited by polling the OperationStatus() register until bit 2 is cleared
        timeout = hal->millis() + 3000UL;
        while (timeout > hal->millis()) {
            operationStatus = getOperationStatus();
            if (operationStatus.CFGUPDATE == 0x00) {
                timeout = hal->millis() + 1000;
                break;
            }
            hal->delay(100);
        }
        if (hal->millis() > timeout) {
            log_e("Timed out waiting to exit update mode.");
            return false;
        }

        // 13. If the device was previously in SEALED state, return to SEALED mode by sending the Control(0x0030) subcommand
        if (isSealed) {
            log_d("Restore Safe Mode!");
            exitSealMode();
        }

        return true;
    }

    // The fuel gauge enters CALIBRATION mode
    int enterCalibration()
    {
        return sendSubCommand(BQ27220_SUB_CMD_ENTER_CAL);
    }

    // The fuel gauge exit CALIBRATION mode
    int exitCalibration()
    {
        return sendSubCommand(BQ27220_SUB_CMD_EXIT_CAL);
    }

    int enterRomMode()
    {
        return sendSubCommand(BQ27220_SUB_CMD_RETURN_TO_ROM);
    }

    // Get chip ID
    uint16_t getChipID()
    {
        uint8_t buffer[2] = {0};
        if (sendSubCommand(BQ27220_SUB_CMD_DEVICE_NUMBER, true) < 0) {
            return 0;
        }
        if (this->getMACData(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        return static_cast<uint16_t>((buffer[1] << 8) | buffer[0]);
    }

    int getHardwareVersion()
    {
        uint8_t buffer[2];
        constexpr uint8_t value = 0x00;
        comm->writeRegister(BQ27220_SUB_CMD_HW_VERSION, value);
        if (this->getMACData(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        return static_cast<uint16_t>((buffer[1] << 8) | buffer[0]);
    }

    int getFirmwareVersion()
    {
        uint8_t buffer[4];
        if (sendSubCommand(BQ27220_SUB_CMD_FW_VERSION, true) < 0) {
            return -1;
        }
        if (this->getMACData(buffer, arraySize(buffer)) < 0) {
            return -1;
        }
        return  (buffer[3] << 24) | (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
    }

    void reset()
    {
        sendSubCommand(BQ27220_SUB_CMD_RESET);
    }


private:


    bool setCapacity(uint16_t newCapacity, uint8_t msbAccessValue, uint8_t lsbAccessValue)
    {
        constexpr uint8_t fixedDataLength = 0x06;

        // Write to access the MSB of Capacity
        comm->writeRegister(BQ27220_REG_STA_DESIGN_CAPACITY_MSB, msbAccessValue);
        hal->delay(10);

        // Write to access the LSB of Capacity
        comm->writeRegister(BQ27220_REG_STA_DESIGN_CAPACITY_LSB, lsbAccessValue);
        hal->delay(10);

        // Write two Capacity bytes starting from 0x40
        uint8_t newCapacityMsb = highByte(newCapacity);
        uint8_t newCapacityLsb = lowByte(newCapacity);
        uint8_t capacityRaw[] = {newCapacityMsb, newCapacityLsb};
        comm->writeRegister(BQ27220_REG_MAC_BUFFER_START, capacityRaw, 2);

        // Calculate new checksum
        uint8_t newChksum = 0xFF - ((msbAccessValue + lsbAccessValue + newCapacityMsb + newCapacityLsb) & 0xFF);

        // Write new checksum (0x60)
        comm->writeRegister(BQ27220_REG_MAC_DATA_SUM, newChksum);

        // Write the block length
        comm->writeRegister(BQ27220_REG_MAC_DATA_LEN, fixedDataLength);

        return true;
    }

    int getHalfWord(uint8_t reg)
    {
        uint8_t buffer[2] = {0};
        if (comm->writeThenRead(&reg, 1, buffer, arraySize(buffer)) < 0) {
            log_e("Read register %02X failed!", reg);
            return UINT16_MAX;
        }
        return (buffer[1] << 8) | buffer[0];
    }

    bool initImpl()
    {
        int chipID = getChipID();
        if (chipID != BQ27220_CHIP_ID) {
            log_e("Chip id not match : %02X\n", chipID);
            return false;
        }

        int sw = getFirmwareVersion();
        if (sw < 0) {
            log_e("Software version error!");
        } else {
            log_d("Software version 0x%04X", sw);
        }
        hal->delay(100);
        int hw = getHardwareVersion();
        if (hw < 0) {
            log_e("Hardware version error!");
        } else {
            log_d("Hardware version 0x%04X", hw);
        }

        return true;
    }

protected:
    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
    uint32_t accessKey;
};


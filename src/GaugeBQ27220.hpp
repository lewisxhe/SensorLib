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

typedef struct  {
    //* Full discharge detected. This flag is set and cleared based on the SOC
    //* Flag Config B option selected.
    bool FD;
    //* OCV measurement update is complete. True when set
    bool OCVCOMP;
    //* Status bit indicating that an OCV read failed due to current.
    //* This bit can only be set if a battery is present after receiving an OCV_CMD(). True when set
    bool OCVFAIL;
    //* The device operates in SLEEP mode when set.
    //* This bit will be temporarily cleared during AD measurements in SLEEP mode.
    bool SLEEP;
    //* Over-temperature is detected during charging. If Operation Config B [INT_OT] bit = 1,
    //* the SOC_INT pin toggles once when the [OTC] bit is set.
    bool OTC;
    //* Over-temperature detected during discharge condition. True when set. If Operation Config B [INT_OT] bit = 1,
    //* the SOC_INT pin toggles once when the [OTD] bit is set.
    bool OTD;
    //* Full charge detected. This flag is set and cleared based on the SOC Flag Config A and SOC Flag Config B options selected.
    bool FC;
    //* Charge Inhibit: If set, indicates that charging should not begin because the Temperature() is outside the range
    //* [Charge Inhibit Temp Low, Charge Inhibit Temp High]. True when set
    bool CHGINH;
    //* Reserve
    // bool RSVD;
    //* Termination of charging alarm. This flag is set and cleared based on the selected SOC Flag Config A option.
    bool TCA;
    //* A good OCV measurement was made. True when set
    bool OCVGD;
    //* Detects inserted battery. True when set.
    bool AUTH_GD;
    //* Battery presence detected. True when set.
    bool BATTPRES;
    //* Termination discharge alarm. This flag is set and cleared according to the selected SOC Flag Config A option.
    bool TDA;
    //* System shutdown bit indicating that the system should be shut down. True when set. If set, the SOC_INT pin toggles once.
    bool SYSDWN;
    //* When set, the device is in DISCHARGE mode; when cleared, the device is in CHARGING or RELAXATION mode.
    bool DSG;
} BatteryStatus_t;



class GaugeBQ27220 : public BQ27220Constants
{
public:

    enum Access {
        FULL_ACCESS = 1,
        UNBLOCK_ACCESS,
        SEALED_ACCESS,
    };

    GaugeBQ27220() : comm(nullptr), hal(nullptr) {}

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

    BatteryStatus_t getBatteryStatus()
    {
        BatteryStatus_t status;
        uint16_t value = getHalfWord(BQ27220_REG_STA_BAT_STATUS);
        uint8_t highByte = (value >> 8) & 0xFF;
        uint8_t lowByte = value & 0xFF;

        status.FD = (highByte & _BV(7)) != 0;
        status.OCVCOMP = (highByte & _BV(6)) != 0;
        status.OCVFAIL = (highByte & _BV(5)) != 0;
        status.SLEEP = (highByte & _BV(4)) != 0;
        status.OTC = (highByte & _BV(3)) != 0;
        status.OTD = (highByte & _BV(2)) != 0;
        status.FC = (highByte & _BV(1)) != 0;
        status.CHGINH = (highByte & _BV(0)) != 0;

        // status.RSVD = (lowByte & (1 << 7))!= 0;
        status.TCA = (lowByte & _BV(6)) != 0;
        status.OCVGD = (lowByte & _BV(5)) != 0;
        status.AUTH_GD = (lowByte & _BV(4)) != 0;
        status.BATTPRES = (lowByte & _BV(3)) != 0;
        status.TDA = (lowByte & _BV(2)) != 0;
        status.SYSDWN = (lowByte & _BV(1)) != 0;
        status.DSG = (lowByte & _BV(0)) != 0;
        return status;
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
    uint16_t getAveragePower()
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
        uint16_t value = comm->readRegister(BQ27220_REG_STA_OPERATION_STATUS);
        uint8_t highByte = (value >> 8) & 0xFF;
        uint8_t lowByte = value & 0xFF;
        // HIGH BYTE
        status.CFGUPDATE = (highByte & _BV(1)) != 0;

        // LOW BYTE
        status.BTPINT = (lowByte & _BV(7)) != 0;
        status.SMTH = (lowByte & _BV(6)) != 0;
        status.INITCOMP = (lowByte & _BV(5)) != 0;
        status.VDQ = (lowByte & _BV(4)) != 0;
        status.EDV2 = (lowByte & _BV(3)) != 0;
        status.SEC = (lowByte >> 1) & 0x03;
        status.CALMD = (lowByte & _BV(0)) != 0;

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

    //* Subcommands

    int sendSubCommand(uint16_t subCmd)
    {
        uint8_t buffer[3];
        buffer[0] = 0x00;
        buffer[1] = lowByte(subCmd);
        buffer[2] = highByte(subCmd);
        comm->writeBuffer(buffer, arraySize(buffer));
        constexpr uint8_t statusReg = 0x00;
        int waitCount = 20;
        hal->delay(10);
        while (waitCount--) {
            comm->writeThenRead(&statusReg, 1, buffer, 2);
            uint16_t *value = reinterpret_cast<uint16_t *>(buffer);
            if (*value == 0xFFA5) {
                return 0;
            }
            hal->delay(1);
        }
        log_e("Subcommand failed!");
        return -1;
    }

    // UNSEAL
    int unsealDevice()
    {
        uint8_t cmd1[] = {0x00, 0x14, 0x04};
        if (comm->writeBuffer(cmd1, arraySize(cmd1)) < 0) {
            return -1;
        }
        uint8_t cmd2[] = {0x00, 0x72, 0x36};
        if (comm->writeBuffer(cmd2, arraySize(cmd2)) < 0) {
            return -1;
        }
        return 0;
    }

    // UNSEAL FULL ACCESS
    int unsealFullAccess()
    {
        if (unsealDevice() < 0) {
            return -1;
        }
        uint8_t cmd1[] = {0x00, 0xFF, 0xFF};
        if (comm->writeBuffer(cmd1, arraySize(cmd1)) < 0) {
            return -1;
        }
        if (comm->writeBuffer(cmd1, arraySize(cmd1)) < 0) {
            return -1;
        }
        return 0;
    }

    int exitSealMode()
    {
        return  sendSubCommand(BQ27220_SUB_CMD_SEALED);
    }

    bool setDesignCapacity(uint16_t newDesignCapacity)
    {
        //* 1.Check access settings
        OperationStatus_t s;
        s = getOperationStatus();
        if (s.SEC == SEALED_ACCESS) {
            unsealDevice();
        }
        if (s.SEC != FULL_ACCESS) {
            unsealFullAccess();
        }

        //* 3.Send ENTER_CFG_UPDATE command (0x0090)
        sendSubCommand(BQ27220_SUB_CMD_ENTER_CFG_UPDATE);

        //* 4.Confirm CFUPDATE mode by polling the OperationStatus() register until Bit 2 is set.
        bool isConfigUpdate = false;
        uint32_t timeout = hal->millis() + 3000UL;
        while (true && timeout > hal->millis()) {
            s = getOperationStatus();
            if (s.CFGUPDATE) {
                isConfigUpdate = true;
                break;
            }
            hal->delay(100);
        }
        if (!isConfigUpdate) {
            log_e("Enter to CFUPDATE mode failed!");
            return false;
        }

        //* 5.Write 0x9F to 0x3E to access the MSB of Design Capacity
        constexpr uint8_t DesignCapacityMSB = 0x9F;
        comm->writeRegister(BQ27220_REG_STA_DESIGN_CAPACITY_MSB, DesignCapacityMSB);

        //* 6.Write 0x92 to 0x3F to access the LSB of Design Capacity
        constexpr uint8_t DesignCapacityLSB = 0x92;
        comm->writeRegister(BQ27220_REG_STA_DESIGN_CAPACITY_LSB, DesignCapacityLSB);

        //* 7.Use the MACDataSum() command (0x60) to read the 1-byte checksum
        uint8_t oldChksum = getMACDataSum();

        //* 8.Use the MACDataLen() command (0x61) to read the 1-byte block length
        uint8_t dataLen = getMACDataLen();

        //* 9.Read two Design Capacity bytes starting from 0x40
        uint8_t buffer[2];
        getMACData(buffer, arraySize(buffer));
        uint8_t oldDcMsb = buffer[0];
        uint8_t oldDcLsb = buffer[1];

        //* 10.Read and write two Design Capacity bytes starting from 0x40
        uint8_t newDcMsb = lowByte(newDesignCapacity);
        uint8_t newDcLsb = highByte(newDesignCapacity);

        //* 11.Calculate new checksum
        int temp = (255 - oldChksum - oldDcMsb - oldDcLsb + newDcMsb + newDcLsb) % 256;
        uint8_t newChksum = 255 - temp;

        //* 12.Write new checksum
        comm->writeRegister(BQ27220_REG_MAC_DATA_SUM, newChksum);

        //* 13.Write the block length, in this example, Data_Len is 0x24
        uint8_t dataLength = 0x24;
        comm->writeRegister(BQ27220_REG_MAC_DATA_LEN, dataLength);

        //* 13.1.Write new Design Capacity bytes
        comm->writeRegister(BQ27220_REG_STA_DESIGN_CAPACITY_MSB, newDcMsb);
        comm->writeRegister(BQ27220_REG_STA_DESIGN_CAPACITY_LSB, newDcLsb);

        //* 14.Exit CFUPDATE mode by sending the EXIT_CFG_UPDATE_REINIT (0x0091) or EXIT_CFG_UPDATE (0x0092) command
        sendSubCommand(BQ27220_SUB_CMD_EXIT_CFG_UPDATE_REINIT);

        //* 15.Confirm that CFUPDATE mode has been exited by polling the
        //* OperationStatus() register until bit 2 is cleared
        timeout = hal->millis() + 3000UL;
        while (true && timeout > hal->millis()) {
            s = getOperationStatus();
            if (s.CFGUPDATE == 0x00) {
                timeout = hal->millis() + 1000;
                break;
            }
            hal->delay(100);
        }
        if (hal->millis() > timeout) {
            log_e("Wait CFUPDATE exit timeout");
        }
        //* 16.If the device was previously in SEALED state, return
        //* to SEALED mode by sending the Control(0x0030) subcommand
        exitSealMode();

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
        if (sendSubCommand(BQ27220_SUB_CMD_DEVICE_NUMBER) < 0) {
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
        if (sendSubCommand(BQ27220_SUB_CMD_FW_VERSION) < 0) {
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

    int getHalfWord(uint8_t reg)
    {
        uint8_t buffer[2] = {0};
        if (comm->writeThenRead(&reg, 1, buffer, arraySize(buffer)) < 0) {
            return 0xFFFF;
        }
        return (buffer[1] << 8) | buffer[0];
    }

    bool initImpl()
    {
        reset();

        hal->delay(10);

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
};


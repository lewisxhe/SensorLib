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
 * @file      BoschSensorInfo.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-18
 *
 */

#pragma once

#include <memory>
#include <cstdint>
#include <vector>
#include <cstdio>
#include "bhi260x/bhy2_bsec.h"
#include "bhi260x/bhy2_parse.h"
#include "bhi260x/bhy2_defs.h"
#include "BoschSensorUtils.hpp"

/**
 * @class BoschSensorInfo
 * @brief Manages and displays sensor information for Bosch BHI260AP/BMI270 sensors
 * 
 * This class encapsulates all sensor information including version numbers,
 * status flags, and metadata for available virtual sensors. It provides
 * methods to display this information through a flexible output system.
 * 
 * The class uses RAII principles for memory management and supports move
 * semantics for efficient transfer of ownership.
 */
class BoschSensorInfo
{
public:
    /**
     * @brief Default constructor
     */
    BoschSensorInfo();
    
    /**
     * @brief Destructor
     */
    ~BoschSensorInfo();
    
    // Delete copy constructor and assignment operator
    BoschSensorInfo(const BoschSensorInfo&) = delete;
    BoschSensorInfo& operator=(const BoschSensorInfo&) = delete;
    
    /**
     * @brief Move constructor
     */
    BoschSensorInfo(BoschSensorInfo&& other) noexcept;
    
    /**
     * @brief Move assignment operator
     */
    BoschSensorInfo& operator=(BoschSensorInfo&& other) noexcept;
    
    // Getters
    uint16_t getKernelVersion() const { return kernel_version; }
    uint16_t getUserVersion() const { return user_version; }
    uint16_t getRomVersion() const { return rom_version; }
    uint8_t getProductId() const { return product_id; }
    uint8_t getHostStatus() const { return host_status; }
    uint8_t getFeatStatus() const { return feat_status; }
    uint8_t getBootStatus() const { return boot_status; }
    uint8_t getSensorError() const { return sensor_error; }
    bhy2_dev* getDevice() const { return dev; }
    const bhy2_sensor_info* getSensorInfo() const { return info.get(); }
    bhy2_sensor_info* getSensorInfo() { return info.get(); }
    
    // Setters
    void setKernelVersion(uint16_t version) { kernel_version = version; }
    void setUserVersion(uint16_t version) { user_version = version; }
    void setRomVersion(uint16_t version) { rom_version = version; }
    void setProductId(uint8_t id) { product_id = id; }
    void setHostStatus(uint8_t status) { host_status = status; }
    void setFeatStatus(uint8_t status) { feat_status = status; }
    void setBootStatus(uint8_t status) { boot_status = status; }
    void setSensorError(uint8_t error) { sensor_error = error; }
    void setDevice(bhy2_dev* device) { dev = device; }
    
    /**
     * @brief Check if the sensor information is valid
     */
    bool isValid() const { return dev != nullptr && info != nullptr; }
    
    /**
     * @brief Check if a specific sensor is available
     */
    bool isSensorAvailable(uint8_t sensorId) const;
    
    /**
     * @brief Get a list of all available sensor IDs
     */
    std::vector<uint8_t> getAvailableSensors() const;
    
    /**
     * @brief Get the number of available sensors
     */
    size_t getAvailableSensorCount() const;
    
    /**
     * @brief Get sensor name by ID
     */
    static const char* getSensorName(uint8_t sensorId);
    
    /**
     * @brief Print all sensor information
     * 
     * @tparam Printer A callable type that supports printf-style formatting
     * @param printer Output function (defaults to printf)
     */
    template<typename Printer = int (*)(const char*, ...)>
    void printInfo(Printer printer = printf) const;
    
    /**
     * @brief Print boot status information
     * 
     * @tparam Printer A callable type that supports printf-style formatting
     * @param printer Output function (defaults to printf)
     */
    template<typename Printer = int (*)(const char*, ...)>
    void printBootStatus(Printer printer = printf) const;
    
    /**
     * @brief Print list of virtual sensors
     * 
     * @tparam Printer A callable type that supports printf-style formatting
     * @param printer Output function (defaults to printf)
     */
    template<typename Printer = int (*)(const char*, ...)>
    void printVirtualSensorList(Printer printer = printf) const;
    
    /**
     * @brief Update sensor information from device
     */
    bool updateFromDevice();
    
protected:
    uint16_t kernel_version;
    uint16_t user_version;
    uint16_t rom_version;
    uint8_t product_id;
    uint8_t host_status;
    uint8_t feat_status;
    uint8_t boot_status;
    uint8_t sensor_error;
    
    bhy2_dev* dev;

private:
    std::unique_ptr<bhy2_sensor_info[]> info;
    
    /**
     * @brief Initialize sensor information array
     */
    void initializeSensorInfo();
    
    /**
     * @brief Clear all data members
     */
    void clear();
    
    /**
     * @brief Get formatted error text for sensor error code
     */
    static const char* getErrorText(uint8_t error);
};

// Template method implementations in header (required for templates)
template<typename Printer>
void BoschSensorInfo::printInfo(Printer printer) const {
    printer("Product ID     : %02x\n", product_id);
    printer("Kernel version : %04u\n", kernel_version);
    printer("User version   : %04u\n", user_version);
    printer("ROM version    : %04u\n", rom_version);
    printer("Power state    : %s\n", (host_status & BHY2_HST_POWER_STATE) ? "sleeping" : "active");
    printer("Host interface : %s\n", (host_status & BHY2_HST_HOST_PROTOCOL) ? "SPI" : "I2C");
    printer("Feature status : 0x%02x\n", feat_status);
    
    if (sensor_error) {
        printer("%s\n", getErrorText(sensor_error));
    }
    
    printBootStatus(printer);
    printVirtualSensorList(printer);
}

template<typename Printer>
void BoschSensorInfo::printBootStatus(Printer printer) const {
    printer("Boot Status : 0x%02x: ", boot_status);
    
    if (boot_status & BHY2_BST_FLASH_DETECTED) {
        printer("\tFlash detected.\n");
    }
    if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
        printer("\tFlash verify done.\n");
    }
    if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR) {
        printer("\tFlash verification failed.\n");
    }
    if (boot_status & BHY2_BST_NO_FLASH) {
        printer("\tNo flash installed.\n");
    }
    if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
        printer("\tHost interface ready.\n");
    }
    if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) {
        printer("\tFirmware verification done.\n");
    }
    if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR) {
        printer("\tFirmware verification error.\n");
    }
    if (boot_status & BHY2_BST_HOST_FW_IDLE) {
        printer("\tFirmware halted.\n");
    }
}

template<typename Printer>
void BoschSensorInfo::printVirtualSensorList(Printer printer) const {
    if (!dev || !info) {
        return;
    }
    
    if (feat_status & BHY2_FEAT_STATUS_OPEN_RTOS_MSK) {
        printer("Virtual sensor list.\n");
        printer("Sensor ID |                          Sensor Name |  ID | Ver |  Min rate |  Max rate |\n");
        printer("----------+--------------------------------------+-----+-----+-----------+-----------|\n");
        
        for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; ++i) {
            if (bhy2_is_sensor_available(i, dev)) {
                const char* sensorName = "";
                if (i < BHY2_SENSOR_ID_CUSTOM_START) {
                    sensorName = getSensorName(i);
                }
                
                printer(" %8u | %36s | %3u | %3u | %9.4f | %9.4f |\n",
                       i, sensorName,
                       static_cast<unsigned int>(info[i].driver_id),
                       static_cast<unsigned int>(info[i].driver_version),
                       info[i].min_rate.f_val,
                       info[i].max_rate.f_val);
            }
        }
    }
}
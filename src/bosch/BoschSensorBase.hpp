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
 * @file      BoschSensorBase.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-06
 * @note      Most source code references come from the https://github.com/boschsensortec/BHY2-Sensor-API
 *            Simplification for Arduino
 */
#pragma once
#include "../SensorPlatform.hpp"
#include "../sensor/SensorDefs.hpp"
#include "BoschParseCallbackManager.hpp"
#include "BoschInterruptConfig.hpp"
#include "BoschPhySensorInfo.hpp"
#include "BoschSensorInfo.hpp"
#include "BoschSensorID.hpp"
#include <functional>

/**
 * @brief Enumeration of Bosch sensor types supported by this library.
 */
enum class BoschSensorType {
    BOSCH_SENSORTEC_UNKNOWN,
    BOSCH_SENSORTEC_BHI260,
    BOSCH_SENSORTEC_BHI360,
};

/**
 * @brief Enumeration of meta event types that can be received from the sensor.
 */
enum class MetaEventType {
    BOSCH_META_EVENT_FLUSH_COMPLETE,           ///< Flush complete event
    BOSCH_META_EVENT_SAMPLE_RATE_CHANGED,      ///< Sample rate changed event
    BOSCH_META_EVENT_POWER_MODE_CHANGED,       ///< Power mode changed event
    BOSCH_META_EVENT_ALGORITHM_EVENTS,         ///< Algorithm events
    BOSCH_META_EVENT_SENSOR_STATUS,            ///< Sensor status event
    BOSCH_META_EVENT_BSX_DO_STEPS_MAIN,        ///< BSX do steps main event
    BOSCH_META_EVENT_BSX_DO_STEPS_CALIB,       ///< BSX do steps calibration event
    BOSCH_META_EVENT_BSX_GET_OUTPUT_SIGNAL,    ///< BSX get output signal event
    BOSCH_META_EVENT_RESERVED1,                ///< Reserved event
    BOSCH_META_EVENT_SENSOR_ERROR,             ///< Sensor error event
    BOSCH_META_EVENT_FIFO_OVERFLOW,            ///< FIFO overflow event
    BOSCH_META_EVENT_DYNAMIC_RANGE_CHANGED,    ///< Dynamic range changed event
    BOSCH_META_EVENT_FIFO_WATERMARK,           ///< FIFO watermark event
    BOSCH_META_EVENT_RESERVED2,                ///< Reserved event
    BOSCH_META_EVENT_INITIALIZED,              ///< Sensor initialized event
    BOSCH_META_TRANSFER_CAUSE,                 ///< Data transfer cause event
    BOSCH_META_EVENT_SENSOR_FRAMEWORK,         ///< Sensor framework event
    BOSCH_META_EVENT_RESET,                    ///< Sensor reset event
    BOSCH_META_EVENT_SPACER,                   ///< Spacer event
};

/**
 * @brief Base class for Bosch sensor devices (BHI260, BHI360).
 *
 * This class provides a unified interface for initializing, configuring, and reading data
 * from Bosch sensors. It supports multiple communication interfaces (I2C, SPI) and platforms
 * (Arduino, ESP-IDF).
 */
class BoschSensorBase
{
public:
    /**
     * @brief Callback type for firmware upgrade progress monitoring.
     */
    using FirmwareUpgradeProgressCb = void (*)(uint32_t total, uint32_t transferred, void *user_data);

    /**
     * @brief Callback type for kernel debug messages.
     */
    using KernelDebugCb = std::function<void(const char *message)>;

    /**
     * @brief Callback type for meta events from the sensor.
     */
    using MetaEventCb = std::function<void(MetaEventType event, uint8_t sensor_id, uint8_t data)>;

    /**
     * @brief Structure to hold event callback functions.
     */
    struct EventCallbacks {
        KernelDebugCb onDebug = nullptr;  ///< Callback for debug messages
        MetaEventCb onEvent = nullptr;    ///< Callback for meta events
    };


    /**
     * @brief Set the reset pin for the sensor.
     * @note This method must be called before calling begin(). Calling it after begin() will have no effect.
     * @param rst GPIO pin number used for hardware reset of the sensor.
     */
    void setPins(int rst);

#if defined(ARDUINO)
    /**
     * @brief Initialize the sensor using Arduino I2C interface.
     * @param wire Reference to TwoWire object.
     * @param addr I2C device address (default 0x28, can also be 0x29).
     * @param sda I2C SDA pin number (optional, uses default if -1).
     * @param scl I2C SCL pin number (optional, uses default if -1).
     * @return true if initialization succeeded, false otherwise.
     */
    bool begin(TwoWire &wire, uint8_t addr, int sda = -1, int scl = -1);

    /**
     * @brief Initialize the sensor using Arduino SPI interface.
     * @param spi Reference to SPIClass object.
     * @param csPin SPI chip select pin number.
     * @param mosi SPI MOSI pin number (optional, uses default if -1).
     * @param miso SPI MISO pin number (optional, uses default if -1).
     * @param sck SPI clock pin number (optional, uses default if -1).
     * @return true if initialization succeeded, false otherwise.
     */
    bool begin(SPIClass &spi, uint8_t csPin, int mosi = -1, int miso = -1, int sck = -1);

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    /**
     * @brief Initialize the sensor using ESP-IDF I2C legacy interface.
     * @param port_num I2C port number (I2C_NUM0 or I2C_NUM1).
     * @param addr I2C device address (default 0x28, can also be 0x29).
     * @param sda I2C SDA pin number (optional, uses default if -1).
     * @param scl I2C SCL pin number (optional, uses default if -1).
     * @return true if initialization succeeded, false otherwise.
     */
    bool begin(i2c_port_t port_num, uint8_t addr, int sda = -1, int scl = -1);
#else
    /**
     * @brief Initialize the sensor using ESP-IDF I2C LL interface (IDF version > 5.0.0).
     * @param handle I2C master bus handle.
     * @param addr I2C device address (default 0x28, can also be 0x29).
     * @return true if initialization succeeded, false otherwise.
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr);
#endif  // ESP_PLATFORM

    /**
     * @brief Initialize the sensor using ESP-IDF SPI interface.
     * @param host SPI host device.
     * @param handle SPI device handle.
     * @param csPin SPI chip select pin number.
     * @param mosi SPI MOSI pin number.
     * @param miso SPI MISO pin number.
     * @param sck SPI clock pin number.
     * @return true if initialization succeeded, false otherwise.
     */
    bool begin(spi_host_device_t host, spi_device_handle_t handle, uint8_t csPin, int mosi, int miso, int sck);

#endif  // ARDUINO

    /**
     * @brief Initialize the sensor using custom callback interface.
     * @note Suitable for other platforms not covered by standard implementations.
     * @param interface Communication interface type (COMM_SPI or COMM_I2C).
     * @param callback Register read/write callback function.
     * @param hal_callback Platform digital IO and delay callback function.
     * @param addr Device address (default 0x28, can also be 0x29).
     * @return true if initialization succeeded, false otherwise.
     */
    bool begin(CommInterface interface, SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr);

    /**
     * @brief Get the sensor model.
     * @return BoschSensorType representing the sensor model.
     */
    BoschSensorType getModel();

    /**
     * @brief Perform a hardware reset of the sensor.
     * @note If using an external flash for firmware storage, no additional action is required after reset.
     *       If firmware was uploaded to RAM (volatile memory), you must manually call uploadFirmware()
     *       after each reset to reload the firmware into RAM.
     *       The sensor will return to its default state after reset.
     */
    void reset();

    /**
     * @brief Update sensor data by processing the FIFO.
     * @note This function should be called periodically to read and process sensor data.
     *       It triggers registered callbacks for available sensor data.
     */
    void update();

    /**
     * @brief Get the native BHY API device handle.
     * @return Pointer to the underlying bhy2_dev structure.
     */
    bhy2_dev *getDev();

    /**
     * @brief Configure sensor interrupt settings.
     * @param config Interrupt configuration object.
     * @return true if configuration succeeded, false otherwise.
     */
    bool configureInterrupt(const InterruptConfig& config);

    /**
     * @brief Get the current interrupt configuration.
     * @return Current interrupt configuration as an InterruptConfig object.
     */
    InterruptConfig getInterruptConfig() const;

    /**
     * @brief Reads current value from interrupt status register.
     * @return 8-bit interrupt status register value.
     */
    uint8_t getInterruptRegisterValue() const;

    /**
     * @brief Manually sets interrupt register value (for testing/debugging).
     * @param value Value to set in interrupt register.
     * @return true if setting the register succeeded, false otherwise.
     */
    bool setInterruptRegisterValue(uint8_t data);

    /**
     * @brief Check if the sensor is ready for communication.
     * @return true if sensor is ready, false otherwise.
     */
    bool isReady();

    /**
     * @brief Get the sensor firmware kernel version.
     * @return Kernel version as a 16-bit value.
     */
    uint16_t getKernelVersion();

    /**
     * @brief Register a callback for sensor meta events.
     * @param callback Callback function to handle meta events.
     */
    void onEvent(MetaEventCb callback);

    /**
     * @brief Remove the registered meta event callback.
     */
    void removeEvent();

    /**
     * @brief Register a callback for sensor data results.
     * @note Multiple callbacks can be registered for the same sensor ID.
     *       Do not register the same callback function multiple times.
     * @param sensor_id Sensor ID (see BoschSensorID enum).
     * @param callback Callback function to handle sensor data.
     * @param user_data Optional user data passed to the callback.
     * @return true if registration succeeded, false otherwise.
     */
    bool onResultEvent(uint8_t sensor_id, SensorDataParseCallback callback, void *user_data = nullptr);

    /**
     * @brief Remove a registered result callback function.
     * @param sensor_id Sensor ID (see BoschSensorID enum).
     * @param callback Callback function to remove.
     * @return true if removal succeeded, false otherwise.
     */
    bool removeResultEvent(uint8_t sensor_id, SensorDataParseCallback callback);

    /**
     * @brief Set the size of the internal FIFO processing buffer.
     * @note This method must be called before calling begin(). If called after begin(), it will have no effect.
     *       The buffer is allocated during initialization and used for FIFO data processing.
     *       Default size is 512 bytes (BOSCH_SMART_SENSOR_FIFO_PARSE_BUFFER_SIZE).
     * @param size The desired buffer size in bytes.
     */
    void setProcessBufferSize(uint32_t size);

    /**
     * @brief Get the last error message.
     * @return Human-readable error message string.
     */
    const char *getError();

    /**
     * @brief Configure sensor parameters.
     * @param sensor_id Sensor ID (see BoschSensorID enum).
     * @param sample_rate Data output rate in Hz.
     * @param report_latency_ms Report interval in milliseconds.
     * @return true if configuration succeeded, false otherwise.
     */
    bool configure(uint8_t sensor_id, float sample_rate, uint32_t report_latency_ms);

    /**
     * @brief Configure sensor measurement range.
     * @param sensor_id Sensor ID (see BoschSensorID enum).
     * @param range Measurement range value (see sensor datasheet).
     * @return true if configuration succeeded, false otherwise.
     */
    bool configureRange(uint8_t sensor_id, uint16_t range);

    /**
     * @brief Get current sensor configuration.
     * @param sensor_id Sensor ID (see BoschSensorID enum).
     * @return Sensor configuration as a SensorConfig object.
     */
    SensorConfig getConfigure(uint8_t sensor_id);

    /**
     * @brief Get sensor scale factor.
     * @param sensor_id Sensor ID (see BoschSensorID enum).
     * @return Scale factor for the specified sensor.
     */
    float getScaling(uint8_t sensor_id);

    /**
     * @brief Get sensor name by ID.
     * @param sensor_id Sensor ID (see BoschSensorID enum).
     * @return Human-readable sensor name.
     */
    const char *getSensorName(uint8_t sensor_id);

    /**
     * @brief Get current sensor accuracy.
     * @return Accuracy value (0-3, where 3 is highest accuracy).
     */
    uint8_t getAccuracy();

    /**
     * @brief Enable or disable kernel debug message output.
     * @note This method must be called before calling begin(). If called after begin(), it will have no effect.
     *       Debug messages require firmware support and are not available in default firmware.
     * @param enable true to enable debug output, false to disable.
     */
    void setDebugKernel(bool enable);

    /**
     * @brief Register a callback function for receiving kernel debug messages.
     * @note This method must be called before calling begin(). If called after begin(), it will have no effect.
     *       Debug messages require firmware support and are not available in default firmware.
     * @param cb Callback function of type KernelDebugCb that will receive debug messages.
     */
    void setDebugCallback(KernelDebugCb cb);

    /**
     * @brief Get information about a physical sensor.
     * @param sensor_id Physical sensor ID.
     * @return BoschPhySensorInfo object containing sensor information.
     */
    BoschPhySensorInfo getPhySensorInfo(uint8_t sensor_id);

    /**
     * @brief Get comprehensive sensor information.
     * @return BoschSensorInfo object containing all sensor information.
     */
    BoschSensorInfo getSensorInfo();

    /**
     * @brief Set the maximum transfer size for the communication interface.
     * @note This method must be called before calling begin(). If called after begin(), it will have no effect.
     *       Different platforms have different maximum transfer sizes.
     *       Default is 32 bytes for I2C and 256 bytes for SPI.
     * @param size_of_bytes Maximum number of bytes per transfer.
     */
    void setMaxiTransferSize(uint16_t size_of_bytes);

    /**
     * @brief Register a callback function for monitoring firmware upgrade progress.
     * @note This method must be called before calling begin(). If called after begin(), it will have no effect.
     *       The callback will be invoked during firmware upload operations to report progress.
     * @param callback Callback function that receives progress updates (total bytes, transferred bytes, user data).
     * @param user_data Optional user-defined data passed to the callback.
     */
    void setUpdateProcessCallback(FirmwareUpgradeProgressCb callback, void *user_data = nullptr);

    /**
     * @brief Get the number of available sensors.
     * @return Number of sensors currently available.
     */
    uint8_t availableSensorNums();

    /**
     * @brief Get the sensor model name as a string.
     * @return Human-readable model name (e.g., "BHI260", "BHI360").
     */
    const char *getModelName();

    /**
     * @brief Set the axis remapping for the sensor based on the specified orientation.
     *
     * This function allows you to configure the sensor's axis remapping according to a specific
     * physical orientation of the chip. By passing one of the values from the SensorRemap enum,
     * you can ensure that the sensor data is correctly interpreted based on how the chip is placed.
     *
     * Reference: Bosch BHI260AB datasheet, Section 20.3 "Sensing axes and axes remapping"
     *
     * @param remap An enumeration value from SensorRemap that specifies the desired axis remapping.
     * @return true if axis remapping was successfully set, false otherwise.
     */
    bool setRemapAxes(SensorRemap remap);

    /**
     * @brief Upload firmware to the sensor.
     * @note This function is required if firmware is loaded to RAM (volatile memory). After calling reset(),
     *       firmware in RAM is lost and must be reloaded using this function.
     *       For sensors using external flash, firmware persists across resets.
     * @param firmware Pointer to the firmware binary data.
     * @param length Length of the firmware data in bytes.
     * @param write2Flash If true, write firmware to external flash; if false, load to RAM only.
     * @return true if firmware upload was successful, false otherwise.
     */
    bool uploadFirmware(const uint8_t *firmware, uint32_t length, bool write2Flash = false);

    /**
     * @brief Set firmware to be uploaded during initialization.
     * @note This method must be called before calling begin(). It pre-configures firmware
     *       that will be automatically uploaded during the initialization process.
     * @param image Pointer to the firmware binary data.
     * @param image_len Length of the firmware data in bytes.
     * @param write_flash If true, write firmware to external flash; if false, load to RAM only.
     * @param force_update If true, force firmware update even if already present.
     */
    void setFirmware(const uint8_t *image, size_t image_len, bool write_flash = false, bool force_update = false);

protected:
    /**
     * @brief Pure virtual function to get the confirmation ID for the specific sensor model.
     * @return Expected product ID for the sensor.
     */
    virtual uint16_t getConfirmationIDImpl() = 0;

    /**
     * @brief Parse sensor data from FIFO.
     * @param fifo Pointer to FIFO parse data information.
     * @param user_data User data pointer (should be 'this').
     */
    void parseData(const struct bhy2_fifo_parse_data_info *fifo, void *user_data);

    /**
     * @brief Parse meta events from FIFO.
     * @param callback_info Pointer to FIFO parse data information.
     * @param user_data User data pointer (should be 'this').
     */
    void parseMetaEvent(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data);

    /**
     * @brief Parse debug messages from FIFO.
     * @param callback_info Pointer to FIFO parse data information.
     * @param user_data User data pointer (should be 'this').
     */
    void parseDebugMessage(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data);

private:
    /**
     * @brief Static wrapper for parseData to be used as a C-style callback.
     */
    static void bosch_static_parse_data(const struct bhy2_fifo_parse_data_info *fifo, void *user_data);

    /**
     * @brief Static wrapper for parseMetaEvent to be used as a C-style callback.
     */
    static void bosch_static_parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data);

    /**
     * @brief Static wrapper for parseDebugMessage to be used as a C-style callback.
     */
    static void bosch_static_parse_debug_message(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data);

    /**
     * @brief Boot the sensor from external flash memory.
     * @return true if booting from flash succeeded, false otherwise.
     */
    bool bootFromFlash();

    /**
     * @brief Print boot status information for debugging.
     * @param boot_status Boot status register value.
     */
    void print_boot_status(uint8_t boot_status);

    /**
     * @brief Internal implementation of initialization.
     * @param interface Communication interface type.
     * @return true if initialization succeeded, false otherwise.
     */
    bool initImpl(CommInterface interface);

protected:
    // Disable copy constructor and assignment operator
    BoschSensorBase(const BoschSensorBase &) = delete;
    BoschSensorBase &operator=(const BoschSensorBase &) = delete;

    /**
     * @brief Destructor for BoschSensorBase.
     */
    ~BoschSensorBase();

    /**
     * @brief Constructor for BoschSensorBase.
     */
    BoschSensorBase();

    std::unique_ptr<SensorCommBase> comm;      ///< Communication interface
    std::unique_ptr<SensorHal> hal;            ///< Hardware abstraction layer
    std::unique_ptr<SensorCommStatic> staticComm; ///< Static communication wrapper
    std::unique_ptr<struct bhy2_dev> dev;      ///< Native BHY device handle
    uint8_t             _chipID;
    int                 _rst;                   ///< Reset pin number
    int8_t              _error_code;            ///< Last error code
    uint8_t            *_processBuffer;         ///< FIFO processing buffer
    size_t              _processBufferSize;     ///< Size of processing buffer
    const uint8_t      *_firmware_stream;       ///< Pointer to firmware data
    size_t              _firmware_size;         ///< Size of firmware data
    bool                _write_flash;           ///< Write firmware to flash flag
    bool                _boot_from_flash;       ///< Boot from flash flag
    bool                _force_update;          ///< Force firmware update flag
    int                 _max_rw_length;         ///< Maximum read/write length
    uint8_t             _accuracy;              ///< Current sensor accuracy
    bool                _debugKernel;           ///< Debug output enabled flag
    FirmwareUpgradeProgressCb _process_callback; ///< Firmware upgrade progress callback
    void               *_process_callback_user_data; ///< User data for progress callback
    BoschParseCallbackManager  _callback_manager; ///< Callback manager for sensor data
    uint8_t             _sensor_available_nums; ///< Number of available sensors
    char                _err_buffer[128];       ///< Buffer for error messages
    EventCallbacks cbs;                         ///< Event callback functions
};
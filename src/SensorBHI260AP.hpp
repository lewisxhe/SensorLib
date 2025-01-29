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
 * @file      SensorBHI260AP.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-09-06
 * @note      Most source code references come from the https://github.com/boschsensortec/BHY2-Sensor-API
 *            Simplification for Arduino
 */
#pragma once
#include "bosch/BoschParse.h"
#include "bosch/SensorBhy2Define.h"
#include "bosch/BoschSensorControl.hpp"
#include "bosch/BoschPhySensorInfo.hpp"
#include "bosch/BoschSensorInfo.hpp"
#include "SensorPlatform.hpp"

class BoschVirtualSensor
{
public:
    enum BoschSensorID {
        ACCEL_PASSTHROUGH                       = (1),   /*BHY2_SENSOR_ID_ACC_PASS: Accelerometer passthrough */
        ACCEL_UNCALIBRATED                      = (3),   /*BHY2_SENSOR_ID_ACC_RAW: Accelerometer uncalibrated */
        ACCEL_CORRECTED                         = (4),   /*BHY2_SENSOR_ID_ACC: Accelerometer corrected */
        ACCEL_OFFSET                            = (5),   /*BHY2_SENSOR_ID_ACC_BIAS: Accelerometer offset */
        ACCEL_CORRECTED_WAKE_UP                 = (6),   /*BHY2_SENSOR_ID_ACC_WU: Accelerometer corrected wake up */
        ACCEL_UNCALIBRATED_WAKE_UP              = (7),   /*BHY2_SENSOR_ID_ACC_RAW_WU: Accelerometer uncalibrated wake up */
        // VIRTUAL_SENSOR_ID_FOR_ACCEL             = (8),   /*BHY2_SENSOR_ID_SI_ACCEL: Virtual Sensor ID for Accelerometer */
        GYRO_PASSTHROUGH                        = (10),  /*BHY2_SENSOR_ID_GYRO_PASS: Gyroscope passthrough */
        GYRO_UNCALIBRATED                       = (12),  /*BHY2_SENSOR_ID_GYRO_RAW: Gyroscope uncalibrated */
        GYRO_CORRECTED                          = (13),  /*BHY2_SENSOR_ID_GYRO: Gyroscope corrected */
        GYRO_OFFSET                             = (14),  /*BHY2_SENSOR_ID_GYRO_BIAS: Gyroscope offset */
        GYRO_WAKE_UP                            = (15),  /*BHY2_SENSOR_ID_GYRO_WU: Gyroscope wake up */
        GYRO_UNCALIBRATED_WAKE_UP               = (16),  /*BHY2_SENSOR_ID_GYRO_RAW_WU: Gyroscope uncalibrated wake up */
        // VIRTUAL_SENSOR_ID_FOR_GYRO              = (17),  /*BHY2_SENSOR_ID_SI_GYROS: Virtual Sensor ID for Gyroscope */
        MAGNETOMETER_PASSTHROUGH                = (19),  /*BHY2_SENSOR_ID_MAG_PASS: Magnetometer passthrough */
        MAGNETOMETER_UNCALIBRATED               = (21),  /*BHY2_SENSOR_ID_MAG_RAW: Magnetometer uncalibrated */
        MAGNETOMETER_CORRECTED                  = (22),  /*BHY2_SENSOR_ID_MAG: Magnetometer corrected */
        MAGNETOMETER_OFFSET                     = (23),  /*BHY2_SENSOR_ID_MAG_BIAS: Magnetometer offset */
        MAGNETOMETER_WAKE_UP                    = (24),  /*BHY2_SENSOR_ID_MAG_WU: Magnetometer wake up */
        MAGNETOMETER_UNCALIBRATED_WAKE_UP       = (25),  /*BHY2_SENSOR_ID_MAG_RAW_WU: Magnetometer uncalibrated wake up */
        GRAVITY_VECTOR                          = (28),  /*BHY2_SENSOR_ID_GRA: Gravity vector */
        GRAVITY_VECTOR_WAKE_UP                  = (29),  /*BHY2_SENSOR_ID_GRA_WU: Gravity vector wake up */
        LINEAR_ACCELERATION                     = (31),  /*BHY2_SENSOR_ID_LACC: Linear acceleration */
        LINEAR_ACCELERATION_WAKE_UP             = (32),  /*BHY2_SENSOR_ID_LACC_WU: Linear acceleration wake up */
        ROTATION_VECTOR                         = (34),  /*BHY2_SENSOR_ID_RV: Rotation vector , quaternion*/
        ROTATION_VECTOR_WAKE_UP                 = (35),  /*BHY2_SENSOR_ID_RV_WU: Rotation vector wake up */
        GAME_ROTATION_VECTOR                    = (37),  /*BHY2_SENSOR_ID_GAMERV: Game rotation vector */
        GAME_ROTATION_VECTOR_WAKE_UP            = (38),  /*BHY2_SENSOR_ID_GAMERV_WU: Game rotation vector wake up */
        GEO_MAGNETIC_ROTATION_VECTOR            = (40),  /*BHY2_SENSOR_ID_GEORV: Geo-magnetic rotation vector */
        GEO_MAGNETIC_ROTATION_VECTOR_WAKE_UP    = (41),  /*BHY2_SENSOR_ID_GEORV_WU: Geo-magnetic rotation vector wake up */
        ORIENTATION                             = (43),  /*BHY2_SENSOR_ID_ORI: Orientation */
        ORIENTATION_WAKE_UP                     = (44),  /*BHY2_SENSOR_ID_ORI_WU: Orientation wake up , Euler*/
        TILT_DETECTOR                           = (48),  /*BHY2_SENSOR_ID_TILT_DETECTOR: Tilt detector */
        STEP_DETECTOR                           = (50),  /*BHY2_SENSOR_ID_STD: Step detector */
        STEP_COUNTER                            = (52),  /*BHY2_SENSOR_ID_STC: Step counter */
        STEP_COUNTER_WAKE_UP                    = (53),  /*BHY2_SENSOR_ID_STC_WU: Step counter wake up */
        SIGNIFICANT_MOTION                      = (55),  /*BHY2_SENSOR_ID_SIG: Significant motion */
        WAKE_GESTURE                            = (57),  /*BHY2_SENSOR_ID_WAKE_GESTURE: Wake gesture */
        GLANCE_GESTURE                          = (59),  /*BHY2_SENSOR_ID_GLANCE_GESTURE: Glance gesture */
        PICKUP_GESTURE                          = (61),  /*BHY2_SENSOR_ID_PICKUP_GESTURE: Pickup gesture */
        ACTIVITY_RECOGNITION                    = (63),  /*BHY2_SENSOR_ID_AR: Activity recognition */
        WRIST_TILT_GESTURE                      = (67),  /*BHY2_SENSOR_ID_WRIST_TILT_GESTURE: Wrist tilt gesture */
        DEVICE_ORIENTATION                      = (69),  /*BHY2_SENSOR_ID_DEVICE_ORI: Device orientation */
        DEVICE_ORIENTATION_WAKE_UP              = (70),  /*BHY2_SENSOR_ID_DEVICE_ORI_WU: Device orientation wake up */
        STATIONARY_DETECT                       = (75),  /*BHY2_SENSOR_ID_STATIONARY_DET: Stationary detect */
        MOTION_DETECT                           = (77),  /*BHY2_SENSOR_ID_MOTION_DET: Motion detect */
        ACCEL_OFFSET_WAKE_UP                    = (91),  /*BHY2_SENSOR_ID_ACC_BIAS_WU: Accelerometer offset wake up */
        GYRO_OFFSET_WAKE_UP                     = (92),  /*BHY2_SENSOR_ID_GYRO_BIAS_WU: Gyroscope offset wake up */
        MAGNETOMETER_OFFSET_WAKE_UP             = (93),  /*BHY2_SENSOR_ID_MAG_BIAS_WU: Magnetometer offset wake up */
        STEP_DETECTOR_WAKE_UP                   = (94),  /*BHY2_SENSOR_ID_STD_WU: Step detector wake up */
        TEMPERATURE                             = (128), /*BHY2_SENSOR_ID_TEMP: Temperature */
        BAROMETER                               = (129), /*BHY2_SENSOR_ID_BARO: Barometer */
        HUMIDITY                                = (130), /*BHY2_SENSOR_ID_HUM: Humidity */
        GAS                                     = (131), /*BHY2_SENSOR_ID_GAS: Gas */
        TEMPERATURE_WAKE_UP                     = (132), /*BHY2_SENSOR_ID_TEMP_WU: Temperature wake up */
        BAROMETER_WAKE_UP                       = (133), /*BHY2_SENSOR_ID_BARO_WU: Barometer wake up */
        HUMIDITY_WAKE_UP                        = (134), /*BHY2_SENSOR_ID_HUM_WU: Humidity wake up */
        GAS_WAKE_UP                             = (135), /*BHY2_SENSOR_ID_GAS_WU: Gas wake up */
        STEP_COUNTER_LOW_POWER                  = (136), /*BHY2_SENSOR_ID_STC_LP: Step counter Low Power */
        STEP_DETECTOR_LOW_POWER                 = (137), /*BHY2_SENSOR_ID_STD_LP: Step detector Low Power */
        SIGNIFICANT_MOTION_LOW_POWER            = (138), /*BHY2_SENSOR_ID_SIG_LP: Significant motion Low Power */
        STEP_COUNTER_LOW_POWER_WAKE_UP          = (139), /*BHY2_SENSOR_ID_STC_LP_WU: Step counter Low Power wake up */
        STEP_DETECTOR_LOW_POWER_WAKE_UP         = (140), /*BHY2_SENSOR_ID_STD_LP_WU: Step detector Low Power wake up */
        SIGNIFICANT_MOTION_LOW_POWER_WAKE_UP    = (141), /*BHY2_SENSOR_ID_SIG_LP_WU: Significant motion Low Power wake up */
        ANY_MOTION_LOW_POWER                    = (142), /*BHY2_SENSOR_ID_ANY_MOTION_LP: Any motion Low Power */
        ANY_MOTION_LOW_POWER_WAKE_UP            = (143), /*BHY2_SENSOR_ID_ANY_MOTION_LP_WU: Any motion Low Power wake up */
        EXTERNAL_CAMERA_TRIGGER                 = (144), /*BHY2_SENSOR_ID_EXCAMERA: External camera trigger */
        GPS                                     = (145), /*BHY2_SENSOR_ID_GPS: GPS */
        LIGHT                                   = (146), /*BHY2_SENSOR_ID_LIGHT: Light */
        PROXIMITY                               = (147), /*BHY2_SENSOR_ID_PROX: Proximity */
        LIGHT_WAKE_UP                           = (148), /*BHY2_SENSOR_ID_LIGHT_WU: Light wake up */
        PROXIMITY_WAKE_UP                       = (149), /*BHY2_SENSOR_ID_PROX_WU: Proximity wake up */
        GPIO_EXP                                = (151), /*BHY2_SENSOR_ID_GPIO_EXP: GPIO_EXP*/
    };
};




class SensorBHI260AP : public BoschVirtualSensor
{
public:

    using ProcessCallback = void (*)(void *user_data, uint32_t total, uint32_t transferred);

    // The pin names are named according to the sensor manual.
    enum BHI260AP_GPIO {
        MCSB1 = 1,
        RESV1 = 2,
        RESV2 = 3,
        MCSB2 = 4,  //It may be connected to the BMM150 sensor, select according to the actual situation
        MCSB3 = 5,
        MCSB4 = 6,

        QSPI_CLK = 8, // If BHI260 carries external flash, it is not available
        QSPI_CSN = 9, // If BHI260 carries external flash, it is not available
        QSPI_D0 = 10, // If BHI260 carries external flash, it is not available
        QSPI_D1 = 11, // If BHI260 carries external flash, it is not available
        QSPI_D2 = 12, // If BHI260 carries external flash, it is not available
        QSPI_D3 = 13, // If BHI260 carries external flash, it is not available

        M2SCX = 14,
        M2SDX = 15,
        M2SDI = 16,
        M3SCL = 17, //It may be connected to the BMM150 sensor, select according to the actual situation
        M3SDA = 18, //It may be connected to the BMM150 sensor, select according to the actual situation
        JTAG_CLK = 19,
        JTAG_DIO = 20,

        M1SCX = 127, // Invalid Pin
        M1SDX = 128, // Invalid Pin
        M1SDI = 129, // Invalid Pin
        RESV3 = 130, // Invalid Pin
    };

    ~SensorBHI260AP();

    SensorBHI260AP();

    /**
     * @brief  setPins
     * @note   Set the reset pin. reset pin is not set by default.
     * @param  rst:
     * @retval None
     */
    void setPins(int rst);

#if defined(ARDUINO)
    /**
     * @brief   begin
     * @note    Initialization using the Arduino Wire Interface
     * @param  &wire: TwoWire Class
     * @param  addr: Device address, default 0x28, can also be changed to 0x29
     * @param  sda: Set I2C SCL Pin, not set by default
     * @param  scl: Set I2C SDA Pin, not set by default
     * @retval bool true-> Success false-> failure
     */
    bool begin(TwoWire &wire, uint8_t addr = BHI260AP_SLAVE_ADDRESS_L, int sda = -1, int scl = -1);

    /**
     * @brief  begin
     * @note   Initialization using the Arduino SPI Interface
     * @param  &spi: SPIClass
     * @param  csPin: Set CS SCL Pin, not set by default
     * @param  mosi:  Set SPI MOSI SCL Pin, not set by default
     * @param  miso:  Set SPI MISO SCL Pin, not set by default
     * @param  sck:   Set SPI SCK SCL Pin, not set by default
     * @retval bool true-> Success false-> failure
     */
    bool begin(SPIClass &spi, uint8_t csPin, int mosi = -1, int miso = -1, int sck = -1);

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
    /**
     * @brief  begin
     * @note   Initialization using the ESP-IDF I2C Legacy Interface
     * @param  port_num: I2C_NUM0 or I2C_NUM1
     * @param  addr: Device address, default 0x28, can also be changed to 0x29
     * @param  sda: Set I2C SCL Pin, not set by default
     * @param  scl: Set I2C SDA Pin, not set by default
     * @retval bool true-> Success false-> failure
     */
    bool begin(i2c_port_t port_num, uint8_t addr = BHI260AP_SLAVE_ADDRESS_L, int sda = -1, int scl = -1);
#else
    /**
     * @brief begin
     * @note    Initialization using the ESP-IDF I2C LL Interface idf version > 5.0.0
     * @param  handle: I2C Handle
     * @param  addr: Device address, default 0x28, can also be changed to 0x29
     * @retval bool true-> Success false-> failure
     */
    bool begin(i2c_master_bus_handle_t handle, uint8_t addr = BHI260AP_SLAVE_ADDRESS_L);
#endif  //ESP_PLATFORM


    /**
     * @brief  begin
     * @note   Initialization using the ESP-IDF SPI Interface
     * @param  host: spi_host_device_t enum
     * @param  handle: spi_device_handle_t handle
     * @param  csPin: cs pin
     * @param  mosi: spi mosi pin
     * @param  miso: spi miso pin
     * @param  sck: spi sck pin
     * @retval bool true-> Success false-> failure
     */
    bool begin(spi_host_device_t host, spi_device_handle_t handle, uint8_t csPin, int mosi, int miso, int sck);

#endif  //ARDUINO

    /**
     * @brief  begin
     * @note   Custom callback interface, suitable for other platforms
     * @param  interface: Communication mode, COMM_SPI or COMM_I2C
     * @param  callback: Register read and write callback function
     * @param  hal_callback: Platform digital IO and delay callback function
     * @param  addr: Device address, default 0x28, can also be changed to 0x29
     * @retval bool true-> Success false-> failure
     */
    bool begin(CommInterface interface, SensorCommCustom::CustomCallback callback,
               SensorCommCustomHal::CustomHalCallback hal_callback,
               uint8_t addr = BHI260AP_SLAVE_ADDRESS_L);

    /**
     * @brief  reset
     * @note   Reset sensor
     * @retval None
     */
    void reset();

    /**
     * @brief  update
     * @note   Update sensor fifo data
     * @retval None
     */
    void update();

    /**
     * @brief  setBootFromFlash
     * @note   Set whether to start from external flash
     * @param  boot_from_flash: true boot form flash or boot form ram
     * @retval None
     */
    void setBootFromFlash(bool boot_from_flash);

    /**
     * @brief  getHandler
     * @note   Get the native BHI API handle
     * @retval handle
     */
    bhy2_dev *getHandler();

    /**
     * @brief  setInterruptCtrl
     * @note   Set the interrupt control mask
     * @param  data:
     *               BHY2_ICTL_DISABLE_FIFO_W
     *               BHY2_ICTL_DISABLE_FIFO_NW
     *               BHY2_ICTL_DISABLE_STATUS_FIFO
     *               BHY2_ICTL_DISABLE_DEBUG
     *               BHY2_ICTL_DISABLE_FAULT
     *               BHY2_ICTL_ACTIVE_LOW
     *               BHY2_ICTL_EDGE
     *               BHY2_ICTL_OPEN_DRAIN
     *
     * @retval true is success , false is failed
     */
    bool setInterruptCtrl(uint8_t data);

    /**
     * @brief  getInterruptCtrl
     * @note   Get interrupt control info
     * @retval SensorBHI260APControl class
     */
    SensorBHI260APControl getInterruptCtrl();


    /**
     * @brief  isReady
     * @note   Query whether the sensor is ready
     * @retval true OK , false Not ready
     */
    bool isReady();

    /**
     * @brief  getKernelVersion
     * @note   Get the sensor firmware kernel version
     * @retval 2 bytes
     */
    uint16_t getKernelVersion();


    /**
     * @brief  onEvent
     * @note   Registered sensor event callback function
     * @param  callback: Callback Function
     * @retval None
     */
    void onEvent(BhyEventCb callback);

    /**
     * @brief  removeEvent
     * @note   Remove sensor event callback function
     * @retval None
     */
    void removeEvent();

    /**
     * @brief  onResultEvent
     * @note   Registered sensor result callback function , The same sensor ID can register multiple event callbacks.
     *         Please note that you should not register the same event callback repeatedly.
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @param  callback: Callback Function
     * @retval bool true-> Success false-> failure
     */
    bool onResultEvent(BoschSensorID sensor_id, BhyParseDataCallback callback);

    /**
     * @brief  removeResultEvent
     * @note   Remove the registered result callback function
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @param  callback: Callback Function
     * @retval bool true-> Success false-> failure
     */
    bool removeResultEvent(BoschSensorID sensor_id, BhyParseDataCallback callback);

    /**
     * @brief  setProcessBufferSize
     * @note   The default value is 512Bytes , Must be called before initialization
     * @param  size: The set buffer size is requested by malloc, and if PSRAM is enabled, it is requested from PSRAM
     * @retval None
     */
    void setProcessBufferSize(uint32_t size);

    /**
     * @brief  uploadFirmware
     * @note   Update BHI sensor firmware
     * @param  *firmware: Firmware data address
     * @param  length: Firmware data length
     * @param  write2Flash: 1 is written to external flash, 0 is written to RAM
     * @retval bool true-> Success false-> failure
     */
    bool uploadFirmware(const uint8_t *firmware, uint32_t length, bool write2Flash = false);

    /**
     * @brief  getError
     * @note   Get the error status string
     * @retval string
     */
    const char *getError();

    /**
     * @brief  configure
     * @note   Sensor Configuration
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @param  sample_rate: Data output rate, unit: HZ
     * @param  report_latency_ms: Report interval in milliseconds
     * @return bool true-> Success false-> failure
     */
    bool configure(uint8_t sensor_id, float sample_rate, uint32_t report_latency_ms);

    /**
     * @brief  configureRange
     * @note   Set range of the sensor
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @param  range:     Range for selected SensorID. See Table 79 in BHY260 datasheet 109 page
     * @retval  bool true-> Success false-> failure
     */
    bool configureRange(uint8_t sensor_id, uint16_t range);


    /**
     * @brief  getConfigure
     * @note   Get sensor configuration
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @retval  struct bhy2_virt_sensor_conf
     */
    struct bhy2_virt_sensor_conf getConfigure(uint8_t sensor_id);

    /**
     * @brief  getScaling
     * @note   Get sensor scale factor
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @retval scale factor
     */
    float getScaling(uint8_t sensor_id);

    /**
     * @brief  setFirmware
     * @note   Set the firmware
     * @param  *image: firmware data address
     * @param  image_len: firmware length
     * @param  write_flash: true : write to flash otherwise ram
     * @param  force_update: true, rewrite to flash or ram regardless of whether there is firmware, false, do not write if firmware is detected
     * @retval None
     */
    void setFirmware(const uint8_t *image, size_t image_len, bool write_flash = false, bool force_update = false);

    /**
     * @brief  getSensorName
     * @note   Get sensor name
     * @param  sensor_id: Sensor ID , see enum BoschSensorID
     * @retval sensor name
     */
    const char *getSensorName(uint8_t sensor_id);

    /**
     * @brief  getAccuracy
     * @note   Get an accuracy report
     * @retval Current report accuracy
     */
    uint8_t getAccuracy();

    /**
     * @brief  digitalRead
     * @note   Read GPIO level, only for custom firmware
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @param  pullup: true is set pullup or input mode
     * @retval 1 is high ,0 is low
     */
    uint8_t digitalRead(uint8_t pin, bool pullup = false);

    /**
     * @brief  digitalWrite
     * @note   Write GPIO level, only for custom firmware
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @param  level: 1 is high ,0 is low
     * @retval None
     */
    void digitalWrite(uint8_t pin, uint8_t level);

    /**
     * @brief  disableGpio
     * @note   Disable GPIO function
     * @param  pin: see BHI260AP_aux_BMM150_BME280_Expand_GPIO example
     * @retval None
     */
    void disableGpio(uint8_t pin);

    /**
     * @brief  setDebug
     * @note   Whether to enable chip debug output
     * @param  enable: true Enable message debug , false disable debug , Requires firmware support, the default firmware will not output any messages
     * @param  &serial: Stream
     * @retval None
     */
    void setDebugKernel(bool enable);

    /**
     * @brief setDebugCallback
     * @param  cb: Sensor debug output callback function , Requires firmware support, the default firmware will not output any messages
     * @retval None
     */
    void setDebugCallback(BhyDebugMessageCallback cb);

    /**
     * @brief  getPhySensorInfo
     * @note   Get all information about physical sensors
     * @param  sens_id: ID of the physical sensor
     * @retval BoschPhySensorInfo Class
     */
    BoschPhySensorInfo getPhySensorInfo(uint8_t sens_id);

    /**
     * @brief  getSensorInfo
     * @note   Get all information about sensors
     * @retval BoschSensorInfo Class
     */
    BoschSensorInfo getSensorInfo();


    /**
     * @brief  setMaxiTransferSize
     * @note   Set the maximum number of bytes transmitted by the interface , Called before begin
     * @param  size_of_bytes: The maximum transmission bytes of different platforms are different.
     *                        Set it according to the platform. If not set, the default is I2C 32 bytes, SPI 256 bytes.
     * @retval None
     */
    void setMaxiTransferSize(uint16_t size_of_bytes);


    /**
     * @brief  setUpdateProcessCallback
     * @note   Set the callback function of the firmware update process to obtain the update progress
     * @param  callback: callback function
     * @param  *user_data: user data, can be nullptr
     * @retval None
     */
    void setUpdateProcessCallback(ProcessCallback callback, void *user_data = nullptr);

private:

    /**
     * @brief  bootFromFlash
     * @note   Boot from external flash
     * @retval bool true-> Success false-> failure
     */
    bool bootFromFlash();

    void print_boot_status(uint8_t boot_status);

    bool initImpl(bhy2_intf interface);

protected:

    std::unique_ptr<SensorCommBase> comm;
    std::unique_ptr<SensorHal> hal;
    std::unique_ptr<SensorCommStatic> staticComm;
    std::unique_ptr<struct bhy2_dev> _bhy2;
    int                 _rst;
    int8_t              _error_code;
    uint8_t            *_processBuffer;
    size_t              _processBufferSize;
    const uint8_t      *_firmware_stream;
    size_t              _firmware_size;
    bool                _write_flash;
    bool                _boot_from_flash;
    bool                _force_update;
    int                 _max_rw_length;
    uint8_t             _accuracy;      /* Accuracy is reported as a meta event. */
    bool                _debug;
    ProcessCallback     _process_callback;
    void               *_process_callback_user_data;
    char                _err_buffer[128];

};

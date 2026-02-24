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
 * @file      BoschSensorBase.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-06
 * @note      Most source code references come from the https://github.com/boschsensortec/BHY2-Sensor-API
 *            Simplification for Arduino
 */
#include "BoschSensorBase.hpp"

#ifndef BOSCH_SMART_SENSOR_FIFO_PARSE_BUFFER_SIZE
#define BOSCH_SMART_SENSOR_FIFO_PARSE_BUFFER_SIZE 512
#endif

#ifndef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH 32
#endif

#define INT4_TO_INT8(INT4)  ((int8_t)(((INT4) > 1) ? -1 : (INT4)))

BoschSensorBase::BoschSensorBase(): comm(nullptr),
    hal(nullptr),
    staticComm(nullptr),
    _rst(-1), _error_code(0),
    _processBuffer(nullptr),
    _processBufferSize(BOSCH_SMART_SENSOR_FIFO_PARSE_BUFFER_SIZE),
    _firmware_stream(nullptr),
    _firmware_size(0),
    _write_flash(false),
    _boot_from_flash(false),
    _force_update(false),
    _max_rw_length(-1),
    _accuracy(0),
    _debugKernel(false),
    _process_callback(nullptr),
    _process_callback_user_data(nullptr)
{
}

BoschSensorBase::~BoschSensorBase()
{
    if (_processBuffer) {
        free(_processBuffer);
    }
    _processBuffer = NULL;
}

void BoschSensorBase::setPins(int rst)
{
    _rst = rst;
}

#if defined(ARDUINO)
bool BoschSensorBase::begin(TwoWire &wire, uint8_t addr, int sda, int scl)
{
    if (!beginCommonStatic<SensorCommI2C, HalArduino>(comm, staticComm, hal, wire, addr, sda, scl)) {
        return false;
    }
    return initImpl(COMM_I2C);
}

bool BoschSensorBase::begin(SPIClass &spi, uint8_t csPin, int mosi, int miso, int sck)
{
    if (!beginCommonStatic<SensorCommSPI, HalArduino>(comm,
            staticComm, hal,
            spi, csPin, mosi, miso, sck)) {
        return false;
    }
    return initImpl(COMM_SPI);
}

#elif defined(ESP_PLATFORM)

#if defined(USEING_I2C_LEGACY)
bool BoschSensorBase::begin(i2c_port_t port_num, uint8_t addr, int sda, int scl)
{
    if (!beginCommonStatic<SensorCommI2C, HalEspIDF>(comm, staticComm, hal, port_num, addr, sda, scl)) {
        return false;
    }
    return initImpl(COMM_I2C);
}
#else   //USEING_I2C_LEGACY
bool BoschSensorBase::begin(i2c_master_bus_handle_t handle, uint8_t addr)
{
    if (!beginCommonStatic<SensorCommI2C, HalEspIDF>(comm, staticComm, hal, handle, addr)) {
        return false;
    }
    return initImpl(COMM_I2C);
}
#endif  //USEING_I2C_LEGACY


bool BoschSensorBase::begin(spi_host_device_t host, spi_device_handle_t handle, uint8_t csPin, int mosi, int miso, int sck)
{
    if (!beginCommonStatic<SensorCommSPI, HalEspIDF>(comm,
            staticComm, hal,
            host, handle, csPin, mosi, miso, sck)) {
        return false;
    }
    return initImpl(COMM_SPI);
}

#endif  //ARDUINO

bool BoschSensorBase::begin(CommInterface interface,
                            SensorCommCustom::CustomCallback callback,
                            SensorCommCustomHal::CustomHalCallback hal_callback,
                            uint8_t addr)
{
    if (!beginCommCustomCallback<SensorCommCustom, SensorCommCustomHal>(interface,
            callback, hal_callback, addr, comm, hal)) {
        return false;
    }
    return initImpl(interface);
}

BoschSensorType BoschSensorBase::getModel()
{
    if (dev) {
        switch (_chipID) {
        case BHI260_CHIP_ID:
            return BoschSensorType::BOSCH_SENSORTEC_BHI260;
            break;
        case BHI360_CHIP_ID:
            return BoschSensorType::BOSCH_SENSORTEC_BHI360;
        default:
            break;
        }
    }
    return BoschSensorType::BOSCH_SENSORTEC_UNKNOWN;
}

void BoschSensorBase::reset()
{
    if (_rst != -1) {
        hal->digitalWrite(_rst, HIGH);
        hal->delay(5);
        hal->digitalWrite(_rst, LOW);
        hal->delay(10);
        hal->digitalWrite(_rst, HIGH);
        hal->delay(5);
    }
}

void BoschSensorBase::update()
{
    if (!_processBuffer) {
        log_e("Process buffer is not allocated.");
        return;
    }
    bhy2_get_and_process_fifo(_processBuffer, _processBufferSize, dev.get());
}

bhy2_dev *BoschSensorBase::getDev()
{
    return dev.get();
}

bool BoschSensorBase::configureInterrupt(const InterruptConfig &config)
{
    uint8_t regValue = config.getRegisterValue();
    return setInterruptRegisterValue(regValue);
}

InterruptConfig BoschSensorBase::getInterruptConfig() const
{
    uint8_t regValue = getInterruptRegisterValue();
    return InterruptConfig::fromRegisterValue(regValue);
}

uint8_t BoschSensorBase::getInterruptRegisterValue() const
{
    uint8_t data = 0;
    bhy2_get_host_interrupt_ctrl(&data, dev.get());
    return data;
}

bool BoschSensorBase::setInterruptRegisterValue(uint8_t data)
{
    _error_code = bhy2_set_host_interrupt_ctrl(data, dev.get());
    return _error_code == BHY2_OK;
}

bool BoschSensorBase::isReady()
{
    uint8_t  boot_status = 0;
    _error_code = bhy2_get_boot_status(&boot_status, dev.get());
    if (_error_code != BHY2_OK) {
        return false;
    }
    return (boot_status & BHY2_BST_HOST_INTERFACE_READY);
}

uint16_t BoschSensorBase::getKernelVersion()
{
    uint16_t version = 0;
    _error_code = bhy2_get_kernel_version(&version, dev.get());
    if ((_error_code != BHY2_OK) && (version == 0)) {
        return 0;
    }
    return version;
}

void BoschSensorBase::onEvent(MetaEventCb callback)
{
    cbs.onEvent = std::move(callback);
}

void BoschSensorBase::removeEvent()
{
    cbs.onEvent = nullptr;
}

bool BoschSensorBase::onResultEvent(uint8_t sensor_id, SensorDataParseCallback callback, void *user_data)
{
    if (!bhy2_is_sensor_available(sensor_id, dev.get())) {
        log_e("%s not present", getSensorName(sensor_id)); return false;
    }
    return  _callback_manager.add(sensor_id, callback, user_data);
}

bool BoschSensorBase::removeResultEvent(uint8_t sensor_id, SensorDataParseCallback callback)
{
    if (!bhy2_is_sensor_available(sensor_id, dev.get())) {
        log_e("%s not present", getSensorName(sensor_id)); return false;
    }
    return  _callback_manager.remove(sensor_id, callback);
}

void BoschSensorBase::setProcessBufferSize(uint32_t size)
{
    if (_processBuffer) {
        log_e("Process buffer setting is invalid. You must call `setProcessBufferSize()` before calling `begin()`.");
        return;
    }
    _processBufferSize = size;
}

const char *BoschSensorBase::getError()
{
    snprintf(_err_buffer, 128, "API:%s\nSensor:%s\n", BoschSensorUtils::get_api_error(_error_code), BoschSensorUtils::get_sensor_error_text(_error_code));
    return static_cast<const char *>(_err_buffer);
}

bool BoschSensorBase::configure(uint8_t sensor_id, float sample_rate, uint32_t report_latency_ms)
{
    if (!bhy2_is_sensor_available(sensor_id, dev.get())) {
        log_e("%s not present", getSensorName(sensor_id)); return false;
    }
    _error_code = bhy2_set_virt_sensor_cfg(sensor_id, sample_rate, report_latency_ms, dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to set virt sensor cfg with error code %d", _error_code);
        return false;
    }
    log_d("Enable %s at %.2fHz.", BoschSensorUtils::get_sensor_name(sensor_id), sample_rate);
    return true;
}

bool BoschSensorBase::configureRange(uint8_t sensor_id, uint16_t range)
{
    _error_code = bhy2_set_virt_sensor_range(sensor_id, range, dev.get());
    return _error_code == BHY2_OK;
}

SensorConfig BoschSensorBase::getConfigure(uint8_t sensor_id)
{
    bhy2_virt_sensor_conf conf {};
    bhy2_get_virt_sensor_cfg(sensor_id, &conf, dev.get());
    log_d("Range:%u sample_rate:%f latency:%lu sensitivity:%u\n", conf.range, conf.sample_rate, conf.latency, conf.sensitivity);
    return SensorConfig{SensorType::MULTI_AXIS, static_cast<float>(conf.range), conf.sample_rate, conf.latency, OperationMode::NORMAL};
}

float BoschSensorBase::getScaling(uint8_t sensor_id)
{
    return BoschSensorUtils::get_sensor_default_scaling(sensor_id);
}

void BoschSensorBase::setFirmware(const uint8_t *image, size_t image_len, bool write_flash, bool force_update)
{
    _firmware_stream = image;
    _firmware_size = image_len;
    _write_flash = write_flash;
    _force_update = force_update;
}

uint8_t BoschSensorBase::getChipID()
{
    return getConfirmationIDImpl();
}

const char *BoschSensorBase::getSensorName(uint8_t sensor_id)
{
    return BoschSensorUtils::get_sensor_name(sensor_id);
}

uint8_t BoschSensorBase::getAccuracy()
{
    return _accuracy;
}

void BoschSensorBase::setDebugKernel(bool enable)
{
    _debugKernel = enable;
}

void BoschSensorBase::setDebugCallback(KernelDebugCb cb)
{
    cbs.onDebug = std::move(cb);
}

BoschPhySensorInfo BoschSensorBase::getPhySensorInfo(uint8_t sensor_id)
{
    BoschPhySensorInfo result;

    struct bhy2_phys_sensor_info psi;

    memset(&psi, 0, sizeof(psi));

    if (!dev.get()) return result;

    uint16_t param_id = (uint16_t)(0x0120 | sensor_id);
    if (param_id >= 0x0121 && param_id <= 0x0160) {
        int8_t assert_rslt = (bhy2_get_phys_sensor_info(sensor_id, &psi, dev.get()));
        if (assert_rslt == BHY2_OK) {
            result.sensor_type = psi.sensor_type;
            result.driver_id = psi.driver_id;
            result.driver_version = psi.driver_version;
            result.power_current = psi.power_current / 10.f;
            result.curr_range = psi.curr_range.u16_val;
            result.irq_status = psi.flags & 0x01;
            result.master_intf = (psi.flags >> 1) & 0x0F;
            result.power_mode = (psi.flags >> 5) & 0x07;
            result.slave_address = psi.slave_address;
            result.gpio_assignment = psi.gpio_assignment;
            result.curr_rate = psi.curr_rate.f_val;
            result.num_axis = psi.num_axis;
            struct bhy2_orient_matrix ort_mtx = { 0 };

            ort_mtx.c[0] = INT4_TO_INT8(psi.orientation_matrix[0] & 0x0F);
            ort_mtx.c[1] = INT4_TO_INT8(psi.orientation_matrix[0] >> 8);
            ort_mtx.c[2] = INT4_TO_INT8(psi.orientation_matrix[1] & 0x0F);
            ort_mtx.c[3] = INT4_TO_INT8(psi.orientation_matrix[1] >> 8);
            ort_mtx.c[4] = INT4_TO_INT8(psi.orientation_matrix[2] & 0x0F);
            ort_mtx.c[5] = INT4_TO_INT8(psi.orientation_matrix[2] >> 8);
            ort_mtx.c[6] = INT4_TO_INT8(psi.orientation_matrix[3] & 0x0F);
            ort_mtx.c[7] = INT4_TO_INT8(psi.orientation_matrix[3] >> 8);
            ort_mtx.c[8] = INT4_TO_INT8(psi.orientation_matrix[4] & 0x0F);
            for (int i = 0; i < 9; ++i) {
                result.orientation_matrix[i] = ort_mtx.c[i];
            }
            result.reserved = psi.reserved;
        }
    }
    return result;
}

BoschSensorInfo BoschSensorBase::getSensorInfo()
{
    BoschSensorInfo sensorInfo;
    uint8_t product_id = 0x00;
    uint16_t kernel_version;
    uint16_t user_version;
    uint16_t rom_version;
    uint8_t host_status;
    uint8_t feat_status;
    uint8_t boot_status;
    uint8_t sensor_error;

    if (bhy2_get_product_id(&product_id, dev.get()) != BHY2_OK) {
        log_e("Failed to get product id");
    }
    if (bhy2_get_kernel_version(&kernel_version, dev.get()) != BHY2_OK) {
        log_e("Failed to get kernel version");
    }
    if (bhy2_get_user_version(&user_version, dev.get()) != BHY2_OK) {
        log_e("Failed to get user version");
    }
    if (bhy2_get_rom_version(&rom_version, dev.get()) != BHY2_OK) {
        log_e("Failed to get ROM version");
    }
    if (bhy2_get_host_status(&host_status, dev.get()) != BHY2_OK) {
        log_e("Failed to get host status");
    }
    if (bhy2_get_feature_status(&feat_status, dev.get()) != BHY2_OK) {
        log_e("Failed to get feature status");
    }
    if (bhy2_get_boot_status(&boot_status, dev.get()) != BHY2_OK) {
        log_e("Failed to get boot status");
    }
    if (bhy2_get_error_value(&sensor_error, dev.get()) != BHY2_OK) {
        log_e("Failed to get error value");
    }
    sensorInfo.setProductId(product_id);
    sensorInfo.setKernelVersion(kernel_version);
    sensorInfo.setUserVersion(user_version);
    sensorInfo.setRomVersion(rom_version);
    sensorInfo.setHostStatus(host_status);
    sensorInfo.setFeatStatus(feat_status);
    sensorInfo.setBootStatus(boot_status);
    sensorInfo.setSensorError(sensor_error);
    sensorInfo.setDevice(dev.get());
    sensorInfo.updateFromDevice();
    return sensorInfo;
}

void BoschSensorBase::setMaxiTransferSize(uint16_t size_of_bytes)
{
    if (_processBuffer) {
        log_e("Must be called before begin");
        return;
    }
    _max_rw_length = size_of_bytes;
}

void BoschSensorBase::setUpdateProcessCallback(FirmwareUpgradeProgressCb callback, void *user_data)
{
    _process_callback = callback;
    _process_callback_user_data = user_data;
}

uint8_t BoschSensorBase::availableSensorNums()
{
    return _sensor_available_nums;
}

const char *BoschSensorBase::getModelName()
{
    BoschSensorType mode = getModel();
    switch (mode) {
    case BoschSensorType::BOSCH_SENSORTEC_BHI260:   return "BHI260";
    case BoschSensorType::BOSCH_SENSORTEC_BHI360:   return "BHI360";
    default: break;
    }
    return "UNKNOWN";
}

bool BoschSensorBase::setRemapAxes(SensorRemap remap)
{
    if (remap > SensorRemap::BOTTOM_LAYER_BOTTOM_LEFT_CORNER) {
        log_e("Invalid SensorRemap value passed to setRemapAxes!");
        return false;
    }

    // Acceleration - related orientation matrices for different mounting directions
    struct bhy2_orient_matrix acc_matrices[] = {
        // P0 mounting direction, default direction, axis direction is consistent with the default coordinate system
        {1, 0, 0, 0, 1, 0, 0, 0, 1},
        // P1 mounting direction, P0 is rotated 90° clockwise around the Z - axis
        {0, -1, 0, 1, 0, 0, 0, 0, 1},
        // P2 mounting direction, P0 is rotated 180° clockwise around the Z - axis
        {-1, 0, 0, 0, -1, 0, 0, 0, 1},
        // P3 mounting direction, P0 is rotated 270° clockwise around the Z - axis
        {0, 1, 0, -1, 0, 0, 0, 0, 1},
        // P4 mounting direction, P0 is flipped vertically (rotated 180° around the X - axis)
        {1, 0, 0, 0, -1, 0, 0, 0, -1},
        // P5 mounting direction, P4 is rotated 90° clockwise around the Z - axis
        {0, 1, 0, 1, 0, 0, 0, 0, -1},
        // P6 mounting direction, P4 is rotated 180° clockwise around the Z - axis
        {-1, 0, 0, 0, 1, 0, 0, 0, -1},
        // P7 mounting direction, P4 is rotated 270° clockwise around the Z - axis
        {0, -1, 0, -1, 0, 0, 0, 0, -1}
    };

    // Gyroscope - related orientation matrices for different mounting directions
    struct bhy2_orient_matrix gyro_matrices[] = {
        // P0 mounting direction, default direction, axis direction is consistent with the default coordinate system
        {1, 0, 0, 0, 1, 0, 0, 0, 1},
        // P1 mounting direction, P0 is rotated 90° clockwise around the Z - axis
        {0, -1, 0, 1, 0, 0, 0, 0, 1},
        // P2 mounting direction, P0 is rotated 180° clockwise around the Z - axis
        {-1, 0, 0, 0, -1, 0, 0, 0, 1},
        // P3 mounting direction, P0 is rotated 270° clockwise around the Z - axis
        {0, 1, 0, -1, 0, 0, 0, 0, 1},
        // P4 mounting direction, P0 is flipped vertically (rotated 180° around the X - axis)
        {1, 0, 0, 0, -1, 0, 0, 0, -1},
        // P5 mounting direction, P4 is rotated 90° clockwise around the Z - axis
        {0, 1, 0, 1, 0, 0, 0, 0, -1},
        // P6 mounting direction, P4 is rotated 180° clockwise around the Z - axis
        {-1, 0, 0, 0, 1, 0, 0, 0, -1},
        // P7 mounting direction, P4 is rotated 270° clockwise around the Z - axis
        {0, -1, 0, -1, 0, 0, 0, 0, -1}
    };

    uint8_t remap_index = static_cast<uint8_t>(remap);
    // Set the orientation matrix for the accelerometer
    _error_code = bhy2_set_orientation_matrix(BHY2_PHYS_SENSOR_ID_ACCELEROMETER, acc_matrices[remap_index], dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to set acceleration orientation matrix!");
        return false;
    }
    // Set the orientation matrix for the gyroscope
    _error_code = bhy2_set_orientation_matrix(BHY2_PHYS_SENSOR_ID_GYROSCOPE, gyro_matrices[remap_index], dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to set gyroscope orientation matrix!");
        return false;
    }
    return true;
}

bool BoschSensorBase::bootFromFlash()
{
    int8_t rslt;
    uint8_t boot_status, feat_status;
    uint8_t error_val = 0;
    uint16_t tries = 300; /* Wait for up to little over 3s */

    log_d("Waiting for firmware verification to complete");
    do {
        _error_code = bhy2_get_boot_status(&boot_status, dev.get());
        if (_error_code != BHY2_OK) {
            log_e("Failed to get boot status");
            return false;
        }
        hal->delay(10);
    } while (tries--);

    _error_code = bhy2_get_boot_status(&boot_status, dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to get boot status");
        return false;
    }
    print_boot_status(boot_status);

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {

        if (boot_status & BHY2_BST_FLASH_DETECTED) {

            // If no firmware is running, boot from Flash
            log_d("Booting from flash");
            rslt = bhy2_boot_from_flash(dev.get());
            if (rslt != BHY2_OK) {
                log_e("%s. Booting from flash failed.\r\n", BoschSensorUtils::get_api_error(rslt));
                _error_code = bhy2_get_regs(BHY2_REG_ERROR_VALUE, &error_val, 1, dev.get());
                if (_error_code != BHY2_OK) {
                    log_e("Failed to get error registers");
                    return false;
                }
                return false;
            }

            _error_code = bhy2_get_boot_status(&boot_status, dev.get());
            if (_error_code != BHY2_OK) {
                log_e("Failed to get boot status");
                return false;
            }
            print_boot_status(boot_status);

            if (!(boot_status & BHY2_BST_HOST_INTERFACE_READY)) {
                /* hub is not ready, need reset hub */
                log_d("Host interface is not ready, triggering a reset");
                _error_code = bhy2_soft_reset(dev.get());
                if (_error_code != BHY2_OK) {
                    log_e("Failed to reset device");
                    return false;
                }
            }

            _error_code = (bhy2_get_feature_status(&feat_status, dev.get()));
            if (_error_code != BHY2_OK) {
                log_e("Failed to get feature status");
                return false;
            }

        } else {
            log_e("Can't detect external flash");
            return false;
        }
    } else {
        log_e("Host interface is not ready");
        return false;
    }

    log_d("Booting from flash successful");
    return true;
}

void BoschSensorBase::print_boot_status(uint8_t boot_status)
{
    log_d("Boot Status : 0x%02x: ", boot_status);
    if (boot_status & BHY2_BST_FLASH_DETECTED) {
        log_d("Flash detected. ");
    }

    if (boot_status & BHY2_BST_FLASH_VERIFY_DONE) {
        log_d("Flash verify done. ");
    }

    if (boot_status & BHY2_BST_FLASH_VERIFY_ERROR) {
        log_d("Flash verification failed. ");
    }

    if (boot_status & BHY2_BST_NO_FLASH) {
        log_d("No flash installed. ");
    }

    if (boot_status & BHY2_BST_HOST_INTERFACE_READY) {
        log_d("Host interface ready. ");
    }

    if (boot_status & BHY2_BST_HOST_FW_VERIFY_DONE) {
        log_d("Firmware verification done. ");
    }

    if (boot_status & BHY2_BST_HOST_FW_VERIFY_ERROR) {
        log_d("Firmware verification error. ");
    }

    if (boot_status & BHY2_BST_HOST_FW_IDLE) {
        log_d("Firmware halted. ");
    }
}

void BoschSensorBase::bosch_static_parse_data(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data)
{
    auto *self = static_cast<BoschSensorBase *>(user_data);
    if (self && callback_info) {
        self->parseData(callback_info, user_data);
    }
}

void BoschSensorBase::bosch_static_parse_meta_event(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data)
{
    auto *self = static_cast<BoschSensorBase *>(user_data);
    if (self && callback_info) {
        self->parseMetaEvent(callback_info, user_data);
    }
}

void BoschSensorBase::bosch_static_parse_debug_message(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data)
{
    auto *self = static_cast<BoschSensorBase *>(user_data);
    if (self && callback_info) {
        self->parseDebugMessage(callback_info, user_data);
    }
}

void BoschSensorBase::parseData(const struct bhy2_fifo_parse_data_info *fifo, void *user_data)
{
    if (user_data != this) {
        return;
    }
#ifdef BOSCH_PARSE_DATA_DUMP
    log_i("ID:[%d]:%s: DATA LEN:%u", fifo->sensor_id, BoschSensorUtils::get_sensor_name(fifo->sensor_id), fifo->data_size);
    SensorLibDumpBuffer(fifo->data_ptr, fifo->data_size);
#endif
    _callback_manager.call(fifo->sensor_id, fifo->data_ptr, fifo->data_size, fifo->time_stamp);
}

void BoschSensorBase::parseMetaEvent(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data)
{
    if (user_data != this) {
        return;
    }
    uint8_t meta_event_type = callback_info->data_ptr[0];
    uint8_t byte1 = callback_info->data_ptr[1];
    uint8_t byte2 = callback_info->data_ptr[2];
    const char *event_text;

    UNUSED(byte1);
    UNUSED(byte2);
    UNUSED(event_text);

    if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT) {
        event_text = "[META EVENT]";
    } else if (callback_info->sensor_id == BHY2_SYS_ID_META_EVENT_WU) {
        event_text = "[META EVENT WAKE UP]";
    } else {
        return;
    }

    MetaEventType event = static_cast<MetaEventType>(meta_event_type);
    switch (event) {
    case MetaEventType::BOSCH_META_EVENT_FLUSH_COMPLETE:
        log_i("%s Flush complete for sensor id %u", event_text, byte1);
        break;
    case MetaEventType::BOSCH_META_EVENT_SAMPLE_RATE_CHANGED:
        log_i("%s Sample rate changed for sensor id %u", event_text, byte1);
        break;
    case MetaEventType::BOSCH_META_EVENT_POWER_MODE_CHANGED:
        log_i("%s Power mode changed for sensor id %u", event_text, byte1);
        break;
    case MetaEventType::BOSCH_META_EVENT_ALGORITHM_EVENTS:
        log_i("%s Algorithm event", event_text);
        break;
    case MetaEventType::BOSCH_META_EVENT_SENSOR_STATUS:
        log_i("%s Accuracy for sensor id %u changed to %u", event_text, byte1, byte2);
        _accuracy = byte2;
        break;
    case MetaEventType::BOSCH_META_EVENT_BSX_DO_STEPS_MAIN:
        log_i("%s BSX event (do steps main)", event_text);
        break;
    case MetaEventType::BOSCH_META_EVENT_BSX_DO_STEPS_CALIB:
        log_i("%s BSX event (do steps calibration)", event_text);
        break;
    case MetaEventType::BOSCH_META_EVENT_BSX_GET_OUTPUT_SIGNAL:
        log_i("%s BSX event (get output signal)", event_text);
        break;
    case MetaEventType::BOSCH_META_EVENT_SENSOR_ERROR:
        log_i("%s Sensor id %u reported error 0x%02X", event_text, byte1, byte2);
        break;
    case MetaEventType::BOSCH_META_EVENT_FIFO_OVERFLOW:
        log_i("%s FIFO overflow", event_text);
        break;
    case MetaEventType::BOSCH_META_EVENT_DYNAMIC_RANGE_CHANGED:
        log_i("%s Dynamic range changed for sensor id %u", event_text, byte1);
        break;
    case MetaEventType::BOSCH_META_EVENT_FIFO_WATERMARK:
        log_i("%s FIFO watermark reached", event_text);
        break;
    case MetaEventType::BOSCH_META_EVENT_INITIALIZED:
        log_i("%s Firmware initialized. Firmware version %u", event_text, ((uint16_t)byte2 << 8) | byte1);
        break;
    case MetaEventType::BOSCH_META_TRANSFER_CAUSE:
        log_i("%s Transfer cause for sensor id %u", event_text, byte1);
        break;
    case MetaEventType::BOSCH_META_EVENT_SENSOR_FRAMEWORK:
        log_i("%s Sensor framework event for sensor id %u", event_text, byte1);
        break;
    case MetaEventType::BOSCH_META_EVENT_RESET:
        log_i("%s Reset event", event_text);
        break;
    case MetaEventType::BOSCH_META_EVENT_SPACER:
        return;
    default:
        log_i("%s Unknown meta event with id: %u", event_text, meta_event_type);
        break;
    }

    if (cbs.onEvent) {
        cbs.onEvent(event, byte1, byte2);
    }
}

void BoschSensorBase::parseDebugMessage(const struct bhy2_fifo_parse_data_info *callback_info, void *user_data)
{
    if (user_data != this) {
        return;
    }
    uint8_t msg_length = 0;
    uint8_t debug_msg[17] = { 0 }; /* Max payload size is 16 bytes, adds a trailing zero if the payload is full */
    if (!callback_info) {
        log_i("Invalid debug message callback info");
        return;
    }
    msg_length = callback_info->data_ptr[0];
    memcpy(debug_msg, &callback_info->data_ptr[1], msg_length);
    debug_msg[msg_length] = '\0'; /* Terminate the string */
    log_d("[DEBUG MSG]: %s", debug_msg);

    if (cbs.onDebug) {
        cbs.onDebug((const char *)debug_msg);
    }
}

bool BoschSensorBase::uploadFirmware(const uint8_t *firmware, uint32_t length, bool write2Flash)
{
    uint8_t sensor_error;
    uint8_t boot_status;

    log_d("Upload Firmware ...");
    _error_code = bhy2_get_boot_status(&boot_status, dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to get boot status");
        return false;
    }
    if (write2Flash) {
        if (boot_status & BHY2_BST_FLASH_DETECTED) {
            uint32_t start_addr = BHY2_FLASH_SECTOR_START_ADDR;
            uint32_t end_addr = start_addr + length;
            log_d("Flash detected. Erasing flash to upload firmware");
            _error_code = bhy2_erase_flash(start_addr, end_addr, dev.get());
            if (_error_code != BHY2_OK) {
                log_e("Failed to erase flash");
                return false;
            }
        } else {
            log_e("Flash not detected");
            return false;
        }
        log_d("Loading firmware into FLASH.");
        _error_code = bhy2_upload_firmware_to_flash(firmware, length,
                      _process_callback,
                      _process_callback_user_data, dev.get());
        if (_error_code != BHY2_OK) {
            log_e("Failed to upload firmware to flash");
            return false;
        }
        log_d("Loading firmware into FLASH Done");
    } else {
        log_d("Loading firmware into RAM.");
        log_d("upload size = %lu", length);
        _error_code = bhy2_upload_firmware_to_ram(firmware, length,
                      _process_callback,
                      _process_callback_user_data, dev.get());
        if (_error_code != BHY2_OK) {
            log_e("Failed to upload firmware to ram");
            return false;
        }
        log_d("Loading firmware into RAM Done");
    }

    _error_code = bhy2_get_error_value(&sensor_error, dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to get error value");
        return false;
    }
    if (sensor_error != BHY2_OK) {
        _error_code = bhy2_get_error_value(&sensor_error, dev.get());
        log_e("%s", BoschSensorUtils::get_sensor_error_text(sensor_error));
        return false;
    }
    if (write2Flash) {
        log_d("Booting from FLASH.");
        _error_code = bhy2_boot_from_flash(dev.get());
    } else {
        log_d("Booting from RAM.");
        _error_code = bhy2_boot_from_ram(dev.get());
    }
    if (_error_code != BHY2_OK) {
        log_e("Failed to boot device!");
        return false;
    }
    _error_code = bhy2_get_error_value(&sensor_error, dev.get());
    if (sensor_error) {
        log_e("%s", BoschSensorUtils::get_sensor_error_text(sensor_error));
        return false;
    }
    return sensor_error == BHY2_OK;
}

bool BoschSensorBase::initImpl(CommInterface interface)
{
    uint8_t chip_id = 0, product_id = 0;

    if (_rst != -1) {
        hal->pinMode(_rst, OUTPUT);
    }

    reset();

    dev = std::make_unique<struct bhy2_dev>();
    if (!dev) {
        log_e("Failed to allocate device handler!");
        return false;
    }

    if (_max_rw_length == -1) {
        switch (interface) {
        case BHY2_I2C_INTERFACE:
            // esp32s3 test I2C maximum read and write is 64 bytes
            _max_rw_length = I2C_BUFFER_LENGTH;
#ifdef ARDUINO_ARCH_ESP32
            // The test data was obtained from tests conducted on the ESP32S3-R8.
            // ESP32 series has an issue with reading large data, so we need to reduce
            // the max read/write length by half as a workaround.
            _max_rw_length /= 2;
#endif
            break;
        case BHY2_SPI_INTERFACE:
            // esp32s3 test SPI maximum read and write is 256 bytes
            _max_rw_length = 256;
            break;
        default:
            return false;
        }
    }

    bhy2_intf intf = interface == COMM_I2C ? BHY2_I2C_INTERFACE : BHY2_SPI_INTERFACE;

    _error_code = bhy2_init(intf,
                            SensorCommStatic::sensor_static_read_data,
                            SensorCommStatic::sensor_static_write_data,
                            SensorCommStatic::sensor_static_delay_us,
                            _max_rw_length,
                            staticComm.get(), dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to initialize device");
        return false;
    }

    _error_code = bhy2_soft_reset(dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to reset device");
        return false;
    }

    _error_code = bhy2_get_chip_id(&chip_id, dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to get chip ID");
        return false;
    }

    bhy2_get_product_id(&product_id, dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to get product ID");
        return false;
    }

    // Check for a valid product ID
    if (chip_id != BHI260_CHIP_ID && chip_id != BHI360_CHIP_ID) {
        log_e("Chip ID read 0x%02X. Expected 0x%02X or 0x%02X", chip_id, BHI260_CHIP_ID, BHI360_CHIP_ID);
        return false;
    } else {
        log_i("%s found. Chip ID read 0x%02X, Product ID read 0x%02X", chip_id == BHI260_CHIP_ID ? "BHI260" : "BHI360", chip_id, product_id);
    }

    // Check confirmation ID
    if (getConfirmationIDImpl() != chip_id) {
        log_e("Confirmation ID mismatch: expected 0x%02X, got 0x%02X", chip_id, getConfirmationIDImpl());
        return false;
    }

    _chipID = chip_id;

    // Set default interrupt configuration
    uint8_t data = 0, data_exp;
    bhy2_get_host_interrupt_ctrl(&data, dev.get());
    data &= ~BHY2_ICTL_DISABLE_STATUS_FIFO; ///> Enable status interrupts
    if (_debugKernel) {
        data &= ~BHY2_ICTL_DISABLE_DEBUG;   ///> Enable debug interrupts
    } else {
        data |= BHY2_ICTL_DISABLE_DEBUG;    ///> Disable debug interrupts
    }
    data &= ~BHY2_ICTL_EDGE;                ///> Level
    data &= ~BHY2_ICTL_ACTIVE_LOW;          ///> Active high
    data &= ~BHY2_ICTL_OPEN_DRAIN;          ///> Push-pull
    data_exp = data;
    bhy2_set_host_interrupt_ctrl(data, dev.get());
    bhy2_get_host_interrupt_ctrl(&data, dev.get());
    if (data != data_exp) {
        log_d("Expected Host Interrupt Control (0x07) to have value 0x%x but instead read 0x%x\r\n", data_exp, data);
    }

    // Config status channel
    bhy2_set_host_intf_ctrl(BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL, dev.get());
    bhy2_get_host_intf_ctrl(&data, dev.get());
    if (!(data & BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL)) {
        log_d("Expected Host Interface Control (0x06) to have bit 0x%x to be set\r\n", BHY2_HIF_CTRL_ASYNC_STATUS_CHANNEL);
    }

    if (_boot_from_flash) {
        if (!bootFromFlash()) {
            return false;
        }
        if (_force_update) {
            if ((_firmware_stream == NULL) || (_firmware_size == 0)) {
                log_e("No valid firmware is set. Please use the \"setFirmware\" method to set the valid firmware.");
                return false;
            }
            _error_code = bhy2_soft_reset(dev.get());
            if (_error_code != BHY2_OK) {
                log_e("Failed to reset device");
                return false;
            }
            log_i("Force update firmware.");
            if (!uploadFirmware(_firmware_stream, _firmware_size, _write_flash)) {
                log_e("Failed to upload firmware to flash");
                return false;
            }
        }
    } else {
        if ((_firmware_stream == NULL) || (_firmware_size == 0)) {
            log_e("No valid firmware is set. Please use the \"setFirmware\" method to set the valid firmware.");
            return false;
        }

        // Upload firmware to RAM
        if (!uploadFirmware(_firmware_stream, _firmware_size, false)) {
            log_e("Failed to upload firmware to RAM");
            return false;
        }
    }

    uint16_t version = getKernelVersion();
    if (version == 0) {
        log_e("Failed to get kernel version");
        return false;
    }
    log_i("Boot successful. Kernel version %u.", version);

    _error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT, bosch_static_parse_data, this, dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to register FIFO parse callback for META_EVENT");
        return false;
    }

    _error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_META_EVENT_WU, bosch_static_parse_meta_event, this, dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to register FIFO parse callback for META_EVENT_WU");
        return false;
    }

    if (_debugKernel) {
        _error_code = bhy2_register_fifo_parse_callback(BHY2_SYS_ID_DEBUG_MSG, bosch_static_parse_debug_message, this, dev.get());
        if (_error_code != BHY2_OK) {
            log_e("Failed to register FIFO parse callback for DEBUG_MSG");
            return false;
        }
    }

#if defined(ARDUINO_ARCH_ESP32)
    if (psramFound()) {
        _processBuffer = (uint8_t *)ps_malloc(_processBufferSize);
        // In older versions of esp-core, even if psramFound returns true, it may not initialize psram correctly.
        // This situation is common when OPI type SPI-RAM is selected as QSPI, or QSPI is selected as OPI.
        if (!_processBuffer) {
            log_e("Failed to allocate PSRAM buffer, trying to allocate SRAM buffer!");
            _processBuffer = (uint8_t *)malloc(_processBufferSize);
        }
    } else {
        _processBuffer = (uint8_t *)malloc(_processBufferSize);
    }
#else
    _processBuffer = (uint8_t *)malloc(_processBufferSize);
#endif

    if (!_processBuffer) {
        log_e("Failed to allocate process buffer");
        return false;
    }

    _error_code = bhy2_get_and_process_fifo(_processBuffer, _processBufferSize, dev.get());
    if (_error_code != BHY2_OK) {
        log_e("Failed to get and process FIFO");
        free(_processBuffer);
        _processBuffer = nullptr;
        return false;
    }

    // Update the callback table to enable parsing of sensor hintr_ctrl
    bhy2_update_virtual_sensor_list(dev.get());

    // Get present virtual sensor
    bhy2_get_virt_sensor_list(dev.get());

    // Only register valid sensor IDs
    for (uint8_t i = 0; i < BHY2_SENSOR_ID_MAX; i++) {
        if (bhy2_is_sensor_available(i, dev.get())) {
            _sensor_available_nums++;
            bhy2_register_fifo_parse_callback(i, bosch_static_parse_data, this, dev.get());
        }
    }
    return _error_code == BHY2_OK;
}

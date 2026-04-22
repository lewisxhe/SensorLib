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
 * @file      SensorBMM150.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-17
 *
 */

#include "SensorBMM150.hpp"
#include <cstring>

SensorBMM150::SensorBMM150() :  _error_code(BMM150_OK)
{
}

bool SensorBMM150::readData(MagnetometerData &data)
{
    if (!_dev) {
        return false;
    }

    struct bmm150_mag_data mag_data = {0, 0, 0};
    if (bmm150_read_mag_data(&mag_data, _dev.get()) != BMM150_OK) {
        return false;
    }

    const float x_gauss =  MagnetometerUtils::microTeslaToGauss(mag_data.x);
    const float y_gauss =  MagnetometerUtils::microTeslaToGauss(mag_data.y);
    const float z_gauss =  MagnetometerUtils::microTeslaToGauss(mag_data.z);

    data.raw.x = static_cast<int16_t>(x_gauss) - _x_offset;
    data.raw.y = static_cast<int16_t>(y_gauss) - _y_offset;
    data.raw.z = static_cast<int16_t>(z_gauss) - _z_offset;

    data.magnetic_field.x = x_gauss * _sensitivity;
    data.magnetic_field.y = y_gauss * _sensitivity;
    data.magnetic_field.z = z_gauss * _sensitivity;
    data.heading = MagnetometerUtils::calculateHeading(data, _declination_rad);
    data.heading_degrees = data.heading * (180.0f / M_PI);
    data.skip_data = false;

    updateInterruptStatus();
    data.overflow = (_dev->int_status & BMM150_INT_DATA_OVERFLOW) != 0;

    return true;
}

bool SensorBMM150::isDataReady()
{
    if (!updateInterruptStatus()) {
        return false;
    }
    return (_dev->int_status & BMM150_INT_ASSERTED_DRDY) != 0;
}

bool SensorBMM150::reset()
{
    if (!_dev) {
        return false;
    }

    _error_code = bmm150_soft_reset(_dev.get());
    if (_error_code != BMM150_OK) {
        return false;
    }

    hal->delay(5);
    _error_code = bmm150_get_sensor_settings(&_settings, _dev.get());
    return _error_code == BMM150_OK;
}

bool SensorBMM150::selfTest()
{
    if (!_dev) {
        return false;
    }
    return bmm150_perform_self_test(BMM150_SELF_TEST_NORMAL, _dev.get()) == BMM150_OK;
}

bool SensorBMM150::setFullScaleRange(MagFullScaleRange range)
{
    switch (range) {
    case MagFullScaleRange::FS_16G:
        _config.range = 16.0f;
        return true;
    default:
        log_e("BMM150 does not support configurable full-scale range");
        return false;
    }
}

bool SensorBMM150::setOutputDataRate(float odr)
{
    if (!_dev) {
        return false;
    }

    const int odr_x100 = static_cast<int>(odr * 100.0f + 0.5f);
    uint8_t data_rate = 0;
    switch (odr_x100) {
    case 200:
        data_rate = BMM150_DATA_RATE_02HZ;
        break;
    case 600:
        data_rate = BMM150_DATA_RATE_06HZ;
        break;
    case 800:
        data_rate = BMM150_DATA_RATE_08HZ;
        break;
    case 1000:
        data_rate = BMM150_DATA_RATE_10HZ;
        break;
    case 1500:
        data_rate = BMM150_DATA_RATE_15HZ;
        break;
    case 2000:
        data_rate = BMM150_DATA_RATE_20HZ;
        break;
    case 2500:
        data_rate = BMM150_DATA_RATE_25HZ;
        break;
    case 3000:
        data_rate = BMM150_DATA_RATE_30HZ;
        break;
    default:
        log_e("Invalid BMM150 output data rate: %.2f", odr);
        return false;
    }

    _settings.data_rate = data_rate;
    _error_code = bmm150_set_sensor_settings(BMM150_SEL_DATA_RATE, &_settings, _dev.get());
    if (_error_code != BMM150_OK) {
        return false;
    }

    _config.sample_rate = odr;
    return true;
}

bool SensorBMM150::setOperationMode(OperationMode mode)
{
    if (!_dev) {
        return false;
    }

    switch (mode) {
    case OperationMode::NORMAL:
    case OperationMode::CONTINUOUS_MEASUREMENT:
        _settings.pwr_mode = BMM150_POWERMODE_NORMAL;
        break;
    case OperationMode::SINGLE_MEASUREMENT:
        _settings.pwr_mode = BMM150_POWERMODE_FORCED;
        break;
    case OperationMode::SUSPEND:
        _settings.pwr_mode = BMM150_POWERMODE_SLEEP;
        break;
    default:
        return false;
    }

    _error_code = bmm150_set_op_mode(&_settings, _dev.get());
    if (_error_code != BMM150_OK) {
        return false;
    }

    _config.mode = mode;
    return true;
}

bool SensorBMM150::setOversamplingRate(MagOverSampleRatio osr)
{
    if (!_dev) {
        return false;
    }

    uint8_t xy_rep = BMM150_REPXY_REGULAR;
    uint8_t z_rep = BMM150_REPZ_REGULAR;
    switch (osr) {
    case MagOverSampleRatio::OSR_8:
        xy_rep = BMM150_REPXY_HIGHACCURACY;
        z_rep = BMM150_REPZ_HIGHACCURACY;
        _oversampling_rate = 8;
        break;
    case MagOverSampleRatio::OSR_4:
        xy_rep = BMM150_REPXY_ENHANCED;
        z_rep = BMM150_REPZ_ENHANCED;
        _oversampling_rate = 4;
        break;
    case MagOverSampleRatio::OSR_2:
        xy_rep = BMM150_REPXY_REGULAR;
        z_rep = BMM150_REPZ_REGULAR;
        _oversampling_rate = 2;
        break;
    case MagOverSampleRatio::OSR_1:
        xy_rep = BMM150_REPXY_LOWPOWER;
        z_rep = BMM150_REPZ_LOWPOWER;
        _oversampling_rate = 1;
        break;
    default:
        return false;
    }

    _settings.xy_rep = xy_rep;
    _settings.z_rep = z_rep;
    _error_code = bmm150_set_sensor_settings(BMM150_SEL_XY_REP | BMM150_SEL_Z_REP, &_settings, _dev.get());
    return _error_code == BMM150_OK;
}

bool SensorBMM150::setDownsamplingRate(MagDownSampleRatio dsr)
{
    (void)dsr;
    log_e("BMM150 does not support downsampling rate setting");
    return false;
}

bool SensorBMM150::configMagnetometer(OperationMode mode, MagFullScaleRange range, float odr,
                                      MagOverSampleRatio osr, MagDownSampleRatio dsr)
{
    (void)dsr;
    if (!setOperationMode(mode)) {
        return false;
    }
    if (!setFullScaleRange(range)) {
        return false;
    }
    if (!setOutputDataRate(odr)) {
        return false;
    }
    if (!setOversamplingRate(osr)) {
        return false;
    }
    return true;
}

bool SensorBMM150::initImpl(uint8_t param)
{
    (void)param;

    if (!ensureValid()) {
        return false;
    }

    std::memset(&_settings, 0, sizeof(_settings));

    _dev = std::make_unique<struct bmm150_dev>();
    if (!_dev) {
        log_e("Device handler alloc failed");
        return false;
    }
    std::memset(_dev.get(), 0, sizeof(struct bmm150_dev));

    switch (_iface) {
    case COMM_I2C:
        _dev->intf = BMM150_I2C_INTF;
        break;
    case COMM_SPI:
        _dev->intf = BMM150_SPI_INTF;
        break;
    default:
        return false;
    }

    _dev->read = SensorCommStatic::sensor_static_read_data;
    _dev->write = SensorCommStatic::sensor_static_write_data;
    _dev->delay_us = SensorCommStatic::sensor_static_delay_us;
    _dev->intf_ptr = staticComm.get();

    _error_code = bmm150_init(_dev.get());
    if (_error_code != BMM150_OK) {
        return false;
    }

    _error_code = bmm150_get_sensor_settings(&_settings, _dev.get());
    if (_error_code != BMM150_OK) {
        return false;
    }

    _info.uid = _dev->chip_id;
    _info.manufacturer = "Bosch";
    _info.model = "BMM150";
    _info.type = SensorType::MAGNETOMETER;
    _info.i2c_address = _addr;
    _info.version = 1;

    _x_offset = 0;
    _y_offset = 0;
    _z_offset = 0;
    _sensitivity = 0.01f;

    _config.type = SensorType::MAGNETOMETER;
    _config.mode = OperationMode::SUSPEND;
    _config.range = 16.0f;
    _config.sample_rate = 10.0f;
    _config.latency = 0;

    return configMagnetometer(OperationMode::NORMAL,
                              MagFullScaleRange::FS_16G,
                              10.0f,
                              MagOverSampleRatio::OSR_2,
                              MagDownSampleRatio::DSR_1);
}

bool SensorBMM150::updateInterruptStatus()
{
    if (!_dev) {
        return false;
    }
    return bmm150_get_interrupt_status(_dev.get()) == BMM150_OK;
}

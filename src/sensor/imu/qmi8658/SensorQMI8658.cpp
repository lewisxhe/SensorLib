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
 * @file      SensorQMI8658.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-15
 *
 */
#include "SensorQMI8658.hpp"
#include "../../AccelerometerUtils.hpp"
#include <string.h>
#include <math.h>
#include "SensorQMI8658_Reg.hpp"

using namespace QMI8658Regs;

namespace
{

uint8_t toQmiLpfMode(SensorQMI8658::LpfMode mode)
{
    switch (mode) {
    case SensorQMI8658::LpfMode::MODE_0:
        return LPF_MODE_0;
    case SensorQMI8658::LpfMode::MODE_1:
        return LPF_MODE_1;
    case SensorQMI8658::LpfMode::MODE_2:
        return LPF_MODE_2;
    case SensorQMI8658::LpfMode::MODE_3:
        return LPF_MODE_3;
    default:
        return LPF_MODE_0;
    }
}

uint8_t toQmiFifoMode(SensorQMI8658::FifoMode mode)
{
    switch (mode) {
    case SensorQMI8658::FifoMode::BYPASS:
        return FIFO_MODE_BYPASS;
    case SensorQMI8658::FifoMode::FIFO:
        return FIFO_MODE_FIFO;
    case SensorQMI8658::FifoMode::STREAM:
        return FIFO_MODE_STREAM;
    default:
        return FIFO_MODE_BYPASS;
    }
}

uint8_t toQmiFifoSamples(SensorQMI8658::FifoSamples samples)
{
    switch (samples) {
    case SensorQMI8658::FifoSamples::SAMPLES_16:
        return FIFO_SAMPLES_16;
    case SensorQMI8658::FifoSamples::SAMPLES_32:
        return FIFO_SAMPLES_32;
    case SensorQMI8658::FifoSamples::SAMPLES_64:
        return FIFO_SAMPLES_64;
    case SensorQMI8658::FifoSamples::SAMPLES_128:
        return FIFO_SAMPLES_128;
    default:
        return FIFO_SAMPLES_16;
    }
}

uint8_t toQmiTapPriority(SensorQMI8658::TapPriority priority)
{
    switch (priority) {
    case SensorQMI8658::TapPriority::X_GT_Y_GT_Z:
        return TAP_PRIORITY_X_GT_Y_GT_Z;
    case SensorQMI8658::TapPriority::X_GT_Z_GT_Y:
        return TAP_PRIORITY_X_GT_Z_GT_Y;
    case SensorQMI8658::TapPriority::Y_GT_X_GT_Z:
        return TAP_PRIORITY_Y_GT_X_GT_Z;
    case SensorQMI8658::TapPriority::Y_GT_Z_GT_X:
        return TAP_PRIORITY_Y_GT_Z_GT_X;
    case SensorQMI8658::TapPriority::Z_GT_X_GT_Y:
        return TAP_PRIORITY_Z_GT_X_GT_Y;
    case SensorQMI8658::TapPriority::Z_GT_Y_GT_X:
        return TAP_PRIORITY_Z_GT_Y_GT_X;
    default:
        return TAP_PRIORITY_X_GT_Y_GT_Z;
    }
}

bool toQmiInterruptBit(SensorQMI8658::IntPin pin, uint8_t &bit_pos, uint8_t &mask)
{
    switch (pin) {
    case SensorQMI8658::IntPin::PIN1:
        bit_pos = 3;
        mask = 0x01;
        return true;
    case SensorQMI8658::IntPin::PIN2:
        bit_pos = 4;
        mask = 0x02;
        return true;
    default:
        return false;
    }
}

// ==================== Legacy Mapping ====================

float legacyAccelOdrToFloat(SensorQMI8658::AccelODR odr)
{
    switch (odr) {
    case SensorQMI8658::AccelODR::ODR_7Hz:
        return 7.006f;
    case SensorQMI8658::AccelODR::ODR_14Hz:
        return 14.0125f;
    case SensorQMI8658::AccelODR::ODR_28Hz:
        return 28.025f;
    case SensorQMI8658::AccelODR::ODR_56Hz:
        return 56.05f;
    case SensorQMI8658::AccelODR::ODR_112Hz:
        return 112.1f;
    case SensorQMI8658::AccelODR::ODR_224Hz:
        return 224.2f;
    case SensorQMI8658::AccelODR::ODR_448Hz:
        return 448.4f;
    case SensorQMI8658::AccelODR::ODR_896Hz:
        return 896.8f;
    case SensorQMI8658::AccelODR::ODR_1792Hz:
        return 1793.6f;
    case SensorQMI8658::AccelODR::ODR_3584Hz:
        return 3587.2f;
    case SensorQMI8658::AccelODR::ODR_7168Hz:
        return 7174.4f;
    case SensorQMI8658::AccelODR::ODR_LP_3Hz:
        return 3.0f;
    case SensorQMI8658::AccelODR::ODR_LP_11Hz:
        return 11.0f;
    case SensorQMI8658::AccelODR::ODR_LP_21Hz:
        return 21.0f;
    case SensorQMI8658::AccelODR::ODR_LP_128Hz:
        return 128.0f;
    default:
        return 112.0f;
    }
}

float legacyGyroOdrToFloat(SensorQMI8658::GyroODR odr)
{
    switch (odr) {
    case SensorQMI8658::GyroODR::ODR_7Hz:
        return 7.006f;
    case SensorQMI8658::GyroODR::ODR_14Hz:
        return 14.0125f;
    case SensorQMI8658::GyroODR::ODR_28Hz:
        return 28.025f;
    case SensorQMI8658::GyroODR::ODR_56Hz:
        return 56.05f;
    case SensorQMI8658::GyroODR::ODR_112Hz:
        return 112.1f;
    case SensorQMI8658::GyroODR::ODR_224Hz:
        return 224.2f;
    case SensorQMI8658::GyroODR::ODR_448Hz:
        return 448.4f;
    case SensorQMI8658::GyroODR::ODR_896Hz:
        return 896.8f;
    case SensorQMI8658::GyroODR::ODR_1792Hz:
        return 1793.6f;
    case SensorQMI8658::GyroODR::ODR_3584Hz:
        return 3587.2f;
    case SensorQMI8658::GyroODR::ODR_7168Hz:
        return 7174.4f;
    default:
        return 112.0f;
    }
}

} // namespace

SensorQMI8658::SensorQMI8658()
    : _accel_range(AccelFullScaleRange::FS_8G)
    , _gyro_range(GyroFullScaleRange::FS_1000_DPS)
    , _accel_scale(ACCEL_SCALE_8G)
    , _gyro_scale(GYRO_SCALE_1024DPS)
    , _accel_odr(896.8f)
    , _gyro_odr(896.8f)
    , _motion_any_configured(false)
    , _motion_no_configured(false)
    , _motion_sig_configured(false)
    , _motion_any_thr{0, 0, 0}
    , _motion_no_thr{0, 0, 0}
    , _motion_any_window(4)
    , _motion_no_window(10)
    , _motion_sig_wait_window(0)
    , _motion_sig_confirm_window(8)
    , _accel_enabled(false)
    , _gyro_enabled(false)
    , _sync_mode(false)
    , _fifo_enabled(false)
    , _fifo_mode(0)
    , _fifo_watermark(16)
    , _int_pin(-1)
    , _int_pin_mask(0)
    , _int_enabled(false)
    , _last_step_count(0)
    , _tap_event_active(false)
    , _last_tap_event(TapEvent::INVALID)
    , _last_tap_event_ms(0)
    , _last_timestamp(0)
    , _firmware_version(0)
    , _fifo_buffer(nullptr)
    , _fifo_buffer_size(0)
    , _axis_layout(Layout::LAYOUT_DEFAULT)
    , _static_cali_sample_count(0)
    , _static_cali_enabled(false)
    , _static_cali_complete(false)
    , _dyn_cal_enabled(false)
    , _dyn_static_delay(0)
    , _dyn_static_flag(0)
    , _dyn_cali_sample_count(0)
    , _gyro_static_deviation(0)
    , _accel_static_deviation(0)
    , _dyn_offset_applied(0)
{
    memset(_usid, 0, sizeof(_usid));
    memset(_accel_cali_sum, 0, sizeof(_accel_cali_sum));
    memset(_gyro_cali_sum, 0, sizeof(_gyro_cali_sum));
    memset(_accel_cali_offset, 0, sizeof(_accel_cali_offset));
    memset(_gyro_cali_offset, 0, sizeof(_gyro_cali_offset));
    memset(_gyro_speed_buffer, 0, sizeof(_gyro_speed_buffer));
    memset(_accel_speed_buffer, 0, sizeof(_accel_speed_buffer));
    memset(_dyn_gyro_offset, 0, sizeof(_dyn_gyro_offset));
    memset(_dyn_gyro_sum, 0, sizeof(_dyn_gyro_sum));
    memset(_hw_st_accel_result, 0, sizeof(_hw_st_accel_result));
    memset(_hw_st_gyro_result, 0, sizeof(_hw_st_gyro_result));
}

SensorQMI8658::~SensorQMI8658()
{
    if (_fifo_buffer) {
        free(_fifo_buffer);
        _fifo_buffer = nullptr;
        _fifo_buffer_size = 0;
    }
}

int SensorQMI8658::writeCommand(uint8_t cmd, uint32_t timeout_ms)
{
    if (!hal) return -1;

    if (writeReg(REG_CTRL9, cmd) != 0) {
        return -1;
    }

    uint32_t start = hal->millis();
    while (hal->millis() - start < timeout_ms) {
        uint8_t status = readReg(REG_STATUS_INT);
        if (status != 0xFF && (status & MASK_INT_CTRL9_DONE)) {
            break;
        }
        hal->delay(1);
    }

    if (writeReg(REG_CTRL9, CTRL_CMD_ACK) != 0) {
        return -1;
    }

    start = hal->millis();
    while (hal->millis() - start < timeout_ms) {
        uint8_t status = readReg(REG_STATUS_INT);
        if (status != 0xFF && !(status & MASK_INT_CTRL9_DONE)) {
            return 0;
        }
        hal->delay(1);
    }

    return -1;
}

bool SensorQMI8658::reset()
{
    if (!hal) return false;

    writeReg(REG_RESET, QMI8658_RESET_VAL);

    uint32_t start = hal->millis();
    while (hal->millis() - start < 500) {
        uint8_t val = readReg(REG_RST_RESULT);
        if (val != 0xFF && (val & MASK_RST_RESULT)) {
            setRegBit(REG_CTRL1, 6);
            return true;
        }
        hal->delay(10);
    }

    log_e("QMI8658 reset timeout");
    return false;
}

bool SensorQMI8658::selfTest()
{
    return selfTestAccel() && selfTestGyro();
}

bool SensorQMI8658::isPresent()
{
    return getChipID() == QMI8658_WHO_AM_I_VAL;
}

uint8_t SensorQMI8658::getChipID()
{
    return readReg(REG_WHO_AM_I);
}

uint32_t SensorQMI8658::getFirmwareVersion()
{
    return _firmware_version;
}

bool SensorQMI8658::setOperationMode(OperationMode mode)
{
    switch (mode) {
    case OperationMode::SUSPEND:
        disableAccel();
        disableGyro();
        return setRegBit(REG_CTRL1, SHIFT_SENSOR_DISABLE);
    case OperationMode::NORMAL:
        return clrRegBit(REG_CTRL1, SHIFT_SENSOR_DISABLE);
    default:
        return false;
    }
}

float SensorQMI8658::getTemperature()
{
    uint8_t buffer[2];
    if (readRegBuff(REG_TEMPERATURE_L, buffer, 2) == 0) {
        return static_cast<float>(buffer[1]) + (static_cast<float>(buffer[0]) / 256.0f);
    }
    return NAN;
}

uint32_t SensorQMI8658::getTimestamp()
{
    uint8_t buffer[3];
    if (readRegBuff(REG_TIMESTAMP_L, buffer, 3) == 0) {
        uint32_t ts = (uint32_t)(buffer[2] << 16) | (uint32_t)(buffer[1] << 8) | buffer[0];
        if (ts > _last_timestamp) {
            _last_timestamp = ts;
        } else {
            _last_timestamp = (ts + 0x1000000 - _last_timestamp);
        }
        return _last_timestamp;
    }
    return _last_timestamp;
}

bool SensorQMI8658::readAccel(AccelerometerData &out)
{
    int16_t x, y, z;
    if (!readAccelRaw(x, y, z)) {
        return false;
    }

    out.raw.x = x;
    out.raw.y = y;
    out.raw.z = z;
    out.mps2.x = AccelerometerUtils::gToMps2(x * _accel_scale);
    out.mps2.y = AccelerometerUtils::gToMps2(y * _accel_scale);
    out.mps2.z = AccelerometerUtils::gToMps2(z * _accel_scale);
    out.temperature = getTemperature();

    return true;
}

bool SensorQMI8658::readAccelRaw(int16_t &x, int16_t &y, int16_t &z)
{
    if (!_accel_enabled) return false;

    uint8_t buffer[6];
    if (readRegBuff(REG_AX_L, buffer, 6) != 0) {
        return false;
    }

    x = (int16_t)((buffer[1] << 8) | buffer[0]);
    y = (int16_t)((buffer[3] << 8) | buffer[2]);
    z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return true;
}

bool SensorQMI8658::enableAccel()
{
    if (setRegBit(REG_CTRL7, 0)) {
        _accel_enabled = true;
        return true;
    }
    return false;
}

bool SensorQMI8658::disableAccel()
{
    if (clrRegBit(REG_CTRL7, 0)) {
        _accel_enabled = false;
        return true;
    }
    return false;
}

bool SensorQMI8658::isAccelEnabled()
{
    return _accel_enabled;
}

bool SensorQMI8658::setAccelFullScaleRange(AccelFullScaleRange range)
{
    uint8_t reg_val = getAccelRangeRegValue(range);
    int ret = updateBits(REG_CTRL2, 0x70, (reg_val << 4));
    if (ret == 0) {
        _accel_range = range;
        _accel_scale = getAccelScaleFromRange(range);
        return true;
    }
    return false;
}

bool SensorQMI8658::setAccelOutputDataRate(float data_rate_hz)
{
    uint8_t reg_val = findClosestAccelOdr(data_rate_hz);
    if (updateBits(REG_CTRL2, 0x0F, reg_val) != 0) {
        return false;
    }
    _accel_odr = data_rate_hz;
    return true;
}

uint8_t SensorQMI8658::getAccelRangeRegValue(AccelFullScaleRange range)
{
    switch (range) {
    case AccelFullScaleRange::FS_2G:  return ACCEL_RANGE_2G;
    case AccelFullScaleRange::FS_4G:  return ACCEL_RANGE_4G;
    case AccelFullScaleRange::FS_8G:  return ACCEL_RANGE_8G;
    case AccelFullScaleRange::FS_16G: return ACCEL_RANGE_16G;
    default:                 return ACCEL_RANGE_8G;
    }
}

float SensorQMI8658::getAccelScaleFromRange(AccelFullScaleRange range)
{
    switch (range) {
    case AccelFullScaleRange::FS_2G:  return ACCEL_SCALE_2G;
    case AccelFullScaleRange::FS_4G:  return ACCEL_SCALE_4G;
    case AccelFullScaleRange::FS_8G:  return ACCEL_SCALE_8G;
    case AccelFullScaleRange::FS_16G: return ACCEL_SCALE_16G;
    default:                 return ACCEL_SCALE_8G;
    }
}

bool SensorQMI8658::readGyro(GyroscopeData &out)
{
    int16_t x, y, z;
    if (!readGyroRaw(x, y, z)) {
        return false;
    }

    out.raw.x = x;
    out.raw.y = y;
    out.raw.z = z;

    int16_t accel_tmp[3] = {0, 0, 0};
    int16_t gyro_tmp[3] = {x, y, z};
    if (_static_cali_enabled || _dyn_cal_enabled) {
        uint8_t abuf[6];
        if (readRegBuff(REG_AX_L, abuf, 6) == 0) {
            accel_tmp[0] = static_cast<int16_t>((abuf[1] << 8) | abuf[0]);
            accel_tmp[1] = static_cast<int16_t>((abuf[3] << 8) | abuf[2]);
            accel_tmp[2] = static_cast<int16_t>((abuf[5] << 8) | abuf[4]);
        }
    }
    axisConvert(accel_tmp, gyro_tmp);
    x = gyro_tmp[0];
    y = gyro_tmp[1];
    z = gyro_tmp[2];

    if (_static_cali_enabled) {
        processStaticCalibration(accel_tmp, gyro_tmp);
    }
    if (_dyn_cal_enabled) {
        processDynamicCalibration(accel_tmp, gyro_tmp);
    }

    int16_t bias_x = 0;
    int16_t bias_y = 0;
    int16_t bias_z = 0;
    if (_dyn_cal_enabled && _dyn_offset_applied == 1) {
        bias_x = _dyn_gyro_offset[0];
        bias_y = _dyn_gyro_offset[1];
        bias_z = _dyn_gyro_offset[2];
    } else if (_static_cali_complete) {
        bias_x = _gyro_cali_offset[0];
        bias_y = _gyro_cali_offset[1];
        bias_z = _gyro_cali_offset[2];
    }
    float x_corr = static_cast<float>(x - bias_x);
    float y_corr = static_cast<float>(y - bias_y);
    float z_corr = static_cast<float>(z - bias_z);
    out.dps.x = x_corr * _gyro_scale;
    out.dps.y = y_corr * _gyro_scale;
    out.dps.z = z_corr * _gyro_scale;
    out.temperature = getTemperature();

    return true;
}

bool SensorQMI8658::readGyroRaw(int16_t &x, int16_t &y, int16_t &z)
{
    if (!_gyro_enabled) return false;

    uint8_t buffer[6];
    if (readRegBuff(REG_GX_L, buffer, 6) != 0) {
        return false;
    }

    x = (int16_t)((buffer[1] << 8) | buffer[0]);
    y = (int16_t)((buffer[3] << 8) | buffer[2]);
    z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return true;
}

bool SensorQMI8658::enableGyro()
{
    if (setRegBit(REG_CTRL7, 1)) {
        _gyro_enabled = true;
        return true;
    }
    return false;
}

bool SensorQMI8658::disableGyro()
{
    if (clrRegBit(REG_CTRL7, 1)) {
        _gyro_enabled = false;
        return true;
    }
    return false;
}

bool SensorQMI8658::isGyroEnabled()
{
    return _gyro_enabled;
}

bool SensorQMI8658::setGyroFullScaleRange(GyroFullScaleRange range)
{
    uint8_t reg_val = getGyroRangeRegValue(range);
    int ret = updateBits(REG_CTRL3, 0x70, (reg_val << 4));
    if (ret == 0) {
        _gyro_range = range;
        _gyro_scale = getGyroScaleFromRange(range);
        return true;
    }
    return false;
}

bool SensorQMI8658::setGyroOutputDataRate(float data_rate_hz)
{
    uint8_t reg_val = findClosestGyroOdr(data_rate_hz);
    if (updateBits(REG_CTRL3, 0x0F, reg_val) != 0) {
        return false;
    }
    _gyro_odr = data_rate_hz;
    return true;
}

uint8_t SensorQMI8658::getGyroRangeRegValue(GyroFullScaleRange range)
{
    switch (range) {
    case GyroFullScaleRange::FS_125_DPS:  return GYRO_RANGE_128DPS;
    case GyroFullScaleRange::FS_250_DPS:  return GYRO_RANGE_256DPS;
    case GyroFullScaleRange::FS_500_DPS:  return GYRO_RANGE_512DPS;
    case GyroFullScaleRange::FS_1000_DPS: return GYRO_RANGE_1024DPS;
    case GyroFullScaleRange::FS_2000_DPS: return GYRO_RANGE_2048DPS;
    default:                    return GYRO_RANGE_1024DPS;
    }
}

float SensorQMI8658::getGyroScaleFromRange(GyroFullScaleRange range)
{
    switch (range) {
    case GyroFullScaleRange::FS_125_DPS:  return GYRO_SCALE_128DPS;
    case GyroFullScaleRange::FS_250_DPS:  return GYRO_SCALE_256DPS;
    case GyroFullScaleRange::FS_500_DPS:  return GYRO_SCALE_512DPS;
    case GyroFullScaleRange::FS_1000_DPS: return GYRO_SCALE_1024DPS;
    case GyroFullScaleRange::FS_2000_DPS: return GYRO_SCALE_2048DPS;
    default:                    return GYRO_SCALE_1024DPS;
    }
}

bool SensorQMI8658::enableSyncMode()
{
    if (setRegBit(REG_CTRL7, 7)) {
        _sync_mode = true;
        return true;
    }
    return false;
}

bool SensorQMI8658::disableSyncMode()
{
    if (clrRegBit(REG_CTRL7, 7)) {
        _sync_mode = false;
        return true;
    }
    return false;
}

bool SensorQMI8658::configAccel(AccelFullScaleRange range, float data_rate_hz, LpfMode lpf)
{
    bool was_enabled = _accel_enabled;
    if (was_enabled) {
        disableAccel();
    }

    uint8_t range_val = getAccelRangeRegValue(range);
    uint8_t odr_val = findClosestAccelOdr(data_rate_hz);
    uint8_t ctrl2 = static_cast<uint8_t>((range_val << 4) | odr_val);
    if (writeReg(REG_CTRL2, ctrl2) != 0) {
        return false;
    }
    _accel_range = range;
    _accel_scale = getAccelScaleFromRange(range);
    _accel_odr = data_rate_hz;

    if (lpf != LpfMode::OFF) {
        uint8_t lpf_val = toQmiLpfMode(lpf);
        updateBits(REG_CTRL5, 0x06, static_cast<uint8_t>(lpf_val << 1));
        setRegBit(REG_CTRL5, 0);
    } else {
        clrRegBit(REG_CTRL5, 0);
    }

    if (was_enabled) {
        enableAccel();
    }

    return true;
}

bool SensorQMI8658::configGyro(GyroFullScaleRange range, float data_rate_hz, LpfMode lpf)
{
    bool was_enabled = _gyro_enabled;
    if (was_enabled) {
        disableGyro();
    }

    uint8_t range_val = getGyroRangeRegValue(range);
    uint8_t odr_val = findClosestGyroOdr(data_rate_hz);
    uint8_t ctrl3 = static_cast<uint8_t>((range_val << 4) | odr_val);
    if (writeReg(REG_CTRL3, ctrl3) != 0) {
        return false;
    }
    _gyro_range = range;
    _gyro_scale = getGyroScaleFromRange(range);
    _gyro_odr = data_rate_hz;

    if (lpf != LpfMode::OFF) {
        uint8_t lpf_val = toQmiLpfMode(lpf);
        updateBits(REG_CTRL5, 0x60, static_cast<uint8_t>(lpf_val << 5));
        setRegBit(REG_CTRL5, 4);
    } else {
        clrRegBit(REG_CTRL5, 4);
    }

    if (was_enabled) {
        enableGyro();
    }

    return true;
}

float SensorQMI8658::odrToFloat(float odr)
{
    return odr;
}

uint8_t SensorQMI8658::findClosestAccelOdr(float hz)
{
    if (hz <= 28.0f)    return ACCEL_ODR_28_025HZ;
    if (hz <= 56.0f)    return ACCEL_ODR_56_05HZ;
    if (hz <= 112.0f)   return ACCEL_ODR_112_1HZ;
    if (hz <= 224.0f)   return ACCEL_ODR_224_2HZ;
    if (hz <= 448.0f)   return ACCEL_ODR_448_4HZ;
    if (hz <= 896.0f)   return ACCEL_ODR_896_8HZ;
    if (hz <= 1792.0f)  return ACCEL_ODR_1793_6HZ;
    if (hz <= 3584.0f)  return ACCEL_ODR_3587_2HZ;
    return ACCEL_ODR_7174_4HZ;
}

uint8_t SensorQMI8658::findClosestGyroOdr(float hz)
{
    if (hz <= 28.0f)    return GYRO_ODR_28_025HZ;
    if (hz <= 56.0f)    return GYRO_ODR_56_05HZ;
    if (hz <= 112.0f)   return GYRO_ODR_112_1HZ;
    if (hz <= 224.0f)   return GYRO_ODR_224_2HZ;
    if (hz <= 448.0f)   return GYRO_ODR_448_4HZ;
    if (hz <= 896.0f)   return GYRO_ODR_896_8HZ;
    if (hz <= 1792.0f)  return GYRO_ODR_1793_6HZ;
    if (hz <= 3584.0f)  return GYRO_ODR_3587_2HZ;
    return GYRO_ODR_7174_4HZ;
}

uint8_t SensorQMI8658::findClosestAccelLpOdr(float hz)
{
    if (hz <= 3.0f)   return ACCEL_ODR_LP_3HZ;
    if (hz <= 11.0f)  return ACCEL_ODR_LP_11HZ;
    if (hz <= 21.0f)  return ACCEL_ODR_LP_21HZ;
    return ACCEL_ODR_LP_128HZ;
}

bool SensorQMI8658::enableInterrupt(IntPin pin)
{
    uint8_t bit_pos = 0;
    uint8_t mask = 0;
    if (!toQmiInterruptBit(pin, bit_pos, mask)) {
        return false;
    }

    setRegBit(REG_CTRL1, bit_pos);
    _int_pin_mask |= mask;

    _int_enabled = true;
    return true;
}

bool SensorQMI8658::disableInterrupt(IntPin pin)
{
    uint8_t bit_pos = 0;
    uint8_t mask = 0;
    if (!toQmiInterruptBit(pin, bit_pos, mask)) {
        return false;
    }

    clrRegBit(REG_CTRL1, bit_pos);
    _int_pin_mask &= ~mask;

    if (_int_pin_mask == 0) {
        _int_enabled = false;
    }
    return true;
}

bool SensorQMI8658::enableDataReadyInterrupt(IntPin pin)
{
    if (pin != IntPin::PIN2) {
        return false;
    }
    enableInterrupt(pin);
    return clrRegBit(REG_CTRL7, 5);
}

uint8_t SensorQMI8658::getInterruptStatus()
{
    return readReg(REG_STATUS_INT);
}

bool SensorQMI8658::isDataReady(uint8_t mask)
{
    if (_int_enabled && _int_pin >= 0 && hal) {
        if (hal->digitalRead(_int_pin) == 0) {
            return false;
        }
    }

    if (_sync_mode) {
        return getRegBit(REG_STATUS_INT, 0);
    }

    uint8_t status0 = readReg(REG_STATUS0);
    uint8_t req = mask & 0x03;
    if (req == 0) {
        bool accel_enabled = _accel_enabled;
        bool gyro_enabled = _gyro_enabled;
        if (accel_enabled && gyro_enabled) {
            req = 0x03;
        } else if (accel_enabled) {
            req = 0x01;
        } else if (gyro_enabled) {
            req = 0x02;
        } else {
            req = 0x03;
        }
    }
    return (status0 & req) == req;
}

void SensorQMI8658::setPins(int pin)
{
    _int_pin = pin;
    if (hal && pin >= 0) {
        hal->pinMode(pin, INPUT);
    }
}

void SensorQMI8658::setIntPin(int pin)
{
    setPins(pin);
}

bool SensorQMI8658::configureFifo(bool enable, uint8_t watermark_samples)
{
    bool gyro_en = _gyro_enabled;
    bool accel_en = _accel_enabled;

    if (gyro_en) disableGyro();
    if (accel_en) disableAccel();

    if (!enable) {
        resetFifo();
        _fifo_enabled = false;
        if (gyro_en) enableGyro();
        if (accel_en) enableAccel();
        return true;
    }

    if (writeCommand(CTRL_CMD_RST_FIFO) != 0) {
        log_e("Failed to reset FIFO");
        return false;
    }

    _fifo_watermark = watermark_samples;
    if (writeReg(REG_FIFO_WTM_TH, watermark_samples) != 0) {
        return false;
    }

    _fifo_mode = (FIFO_SAMPLES_16 << SHIFT_FIFO_SAMPLES) | FIFO_MODE_STREAM;
    if (writeReg(REG_FIFO_CTRL, _fifo_mode) != 0) {
        return false;
    }

    _fifo_enabled = true;

    if (gyro_en) enableGyro();
    if (accel_en) enableAccel();

    return true;
}

bool SensorQMI8658::configFifo(FifoMode mode, FifoSamples samples, uint8_t watermark_samples)
{
    bool gyro_en = _gyro_enabled;
    bool accel_en = _accel_enabled;

    if (gyro_en) disableGyro();
    if (accel_en) disableAccel();

    if (mode == FifoMode::BYPASS) {
        resetFifo();
        _fifo_enabled = false;
        if (gyro_en) enableGyro();
        if (accel_en) enableAccel();
        return true;
    }

    if (writeCommand(CTRL_CMD_RST_FIFO) != 0) {
        return false;
    }

    uint8_t wm = watermark_samples;
    switch (samples) {
    case FifoSamples::SAMPLES_16:
        wm = 16;
        break;
    case FifoSamples::SAMPLES_32:
        wm = 32;
        break;
    case FifoSamples::SAMPLES_64:
        wm = 64;
        break;
    case FifoSamples::SAMPLES_128:
        wm = 128;
        break;
    }

    _fifo_watermark = wm;
    if (writeReg(REG_FIFO_WTM_TH, wm) != 0) {
        return false;
    }

    uint8_t mode_val = toQmiFifoMode(mode);
    uint8_t samples_val = toQmiFifoSamples(samples);
    _fifo_mode = (samples_val << 2) | mode_val;
    if (writeReg(REG_FIFO_CTRL, _fifo_mode) != 0) {
        return false;
    }

    _fifo_enabled = true;

    if (gyro_en) enableGyro();
    if (accel_en) enableAccel();

    return true;
}

bool SensorQMI8658::resetFifo()
{
    return writeCommand(CTRL_CMD_RST_FIFO) == 0;
}

uint8_t SensorQMI8658::getFifoStatus()
{
    return readReg(REG_FIFO_STATUS);
}

uint16_t SensorQMI8658::readFromFifo(AccelerometerData *accel_data, uint16_t accel_count,
                                     GyroscopeData *gyro_data, uint16_t gyro_count)
{
    if (!_fifo_enabled || !hal) return 0;

    if (!_accel_enabled && !_gyro_enabled) return 0;

    uint8_t status = getFifoStatus();
    if (!(status & MASK_FIFO_EMPTY)) {
        log_d("FIFO is empty");
        return 0;
    }

    uint8_t count_buf[2];
    if (readRegBuff(REG_FIFO_SMPL_CNT_L, count_buf, 2) != 0) {
        return 0;
    }
    // FIFO sample count is expressed in bytes:
    // FIFO_Bytes = 2 * ((FIFO_SMPL_CNT_H[1:0] << 8) | FIFO_SMPL_CNT_L)
    uint16_t fifo_bytes = static_cast<uint16_t>(2 * (((count_buf[1] & 0x03) << 8) | count_buf[0]));

    if (fifo_bytes == 0 || fifo_bytes > 1536 || (fifo_bytes % 6) != 0) {
        return 0;
    }

    size_t buffer_size = fifo_bytes;
    if (!_fifo_buffer || buffer_size > _fifo_buffer_size) {
        if (_fifo_buffer) free(_fifo_buffer);
        _fifo_buffer = (uint8_t *)malloc(buffer_size);
        if (!_fifo_buffer) {
            log_e("Failed to allocate FIFO buffer");
            return 0;
        }
        _fifo_buffer_size = buffer_size;
    }

    if (writeCommand(CTRL_CMD_REQ_FIFO) != 0) {
        log_e("Failed to request FIFO");
        return 0;
    }

    // Some I2C implementations cannot read >255 bytes in a single request.
    // Read FIFO payload in chunks to avoid transport-level truncation.
    static constexpr uint16_t kFifoReadChunk = 240;
    uint16_t remaining = fifo_bytes;
    uint16_t offset = 0;
    while (remaining > 0) {
        uint16_t chunk = remaining > kFifoReadChunk ? kFifoReadChunk : remaining;
        if (readRegBuff(REG_FIFO_DATA, _fifo_buffer + offset, chunk) != 0) {
            log_e("Failed to read FIFO data");
            return 0;
        }
        offset += chunk;
        remaining -= chunk;
    }

    writeReg(REG_FIFO_CTRL, _fifo_mode);

    uint16_t accel_idx = 0;
    uint16_t gyro_idx = 0;

    if (_accel_enabled && _gyro_enabled) {
        // FIFO frame layout in 6DOF mode is [acc_xyz(6 bytes), gyro_xyz(6 bytes)] per sample set.
        uint16_t sample_sets = static_cast<uint16_t>(fifo_bytes / 12);
        uint16_t usable = sample_sets;
        if (accel_data && usable > accel_count) usable = accel_count;
        if (gyro_data && usable > gyro_count) usable = gyro_count;

        for (uint16_t i = 0; i < usable; ++i) {
            uint16_t off = static_cast<uint16_t>(i * 12);

            int16_t ax = (int16_t)((_fifo_buffer[off + 1] << 8) | _fifo_buffer[off + 0]);
            int16_t ay = (int16_t)((_fifo_buffer[off + 3] << 8) | _fifo_buffer[off + 2]);
            int16_t az = (int16_t)((_fifo_buffer[off + 5] << 8) | _fifo_buffer[off + 4]);

            int16_t gx = (int16_t)((_fifo_buffer[off + 7] << 8) | _fifo_buffer[off + 6]);
            int16_t gy = (int16_t)((_fifo_buffer[off + 9] << 8) | _fifo_buffer[off + 8]);
            int16_t gz = (int16_t)((_fifo_buffer[off + 11] << 8) | _fifo_buffer[off + 10]);

            if (accel_data) {
                accel_data[accel_idx].raw.x = ax;
                accel_data[accel_idx].raw.y = ay;
                accel_data[accel_idx].raw.z = az;
                accel_data[accel_idx].mps2.x = AccelerometerUtils::gToMps2(ax * _accel_scale);
                accel_data[accel_idx].mps2.y = AccelerometerUtils::gToMps2(ay * _accel_scale);
                accel_data[accel_idx].mps2.z = AccelerometerUtils::gToMps2(az * _accel_scale);
                accel_idx++;
            }

            if (gyro_data) {
                gyro_data[gyro_idx].raw.x = gx;
                gyro_data[gyro_idx].raw.y = gy;
                gyro_data[gyro_idx].raw.z = gz;
                int16_t bias_x = 0;
                int16_t bias_y = 0;
                int16_t bias_z = 0;
                if (_dyn_cal_enabled && _dyn_offset_applied == 1) {
                    bias_x = _dyn_gyro_offset[0];
                    bias_y = _dyn_gyro_offset[1];
                    bias_z = _dyn_gyro_offset[2];
                } else if (_static_cali_complete) {
                    bias_x = _gyro_cali_offset[0];
                    bias_y = _gyro_cali_offset[1];
                    bias_z = _gyro_cali_offset[2];
                }
                float gx_corr = static_cast<float>(gx - bias_x);
                float gy_corr = static_cast<float>(gy - bias_y);
                float gz_corr = static_cast<float>(gz - bias_z);
                gyro_data[gyro_idx].dps.x = gx_corr * _gyro_scale;
                gyro_data[gyro_idx].dps.y = gy_corr * _gyro_scale;
                gyro_data[gyro_idx].dps.z = gz_corr * _gyro_scale;
                gyro_idx++;
            }
        }
        return usable;
    }

    uint16_t samples_read = static_cast<uint16_t>(fifo_bytes / 6);
    for (uint16_t i = 0; i < samples_read; ++i) {
        int16_t x = (int16_t)((_fifo_buffer[i * 6 + 1] << 8) | _fifo_buffer[i * 6 + 0]);
        int16_t y = (int16_t)((_fifo_buffer[i * 6 + 3] << 8) | _fifo_buffer[i * 6 + 2]);
        int16_t z = (int16_t)((_fifo_buffer[i * 6 + 5] << 8) | _fifo_buffer[i * 6 + 4]);

        if (_accel_enabled && accel_idx < accel_count && accel_data) {
            accel_data[accel_idx].raw.x = x;
            accel_data[accel_idx].raw.y = y;
            accel_data[accel_idx].raw.z = z;
            accel_data[accel_idx].mps2.x = AccelerometerUtils::gToMps2(x * _accel_scale);
            accel_data[accel_idx].mps2.y = AccelerometerUtils::gToMps2(y * _accel_scale);
            accel_data[accel_idx].mps2.z = AccelerometerUtils::gToMps2(z * _accel_scale);
            accel_idx++;
        } else if (_gyro_enabled && gyro_idx < gyro_count && gyro_data) {
            gyro_data[gyro_idx].raw.x = x;
            gyro_data[gyro_idx].raw.y = y;
            gyro_data[gyro_idx].raw.z = z;
            int16_t bias_x = 0;
            int16_t bias_y = 0;
            int16_t bias_z = 0;
            if (_dyn_cal_enabled && _dyn_offset_applied == 1) {
                bias_x = _dyn_gyro_offset[0];
                bias_y = _dyn_gyro_offset[1];
                bias_z = _dyn_gyro_offset[2];
            } else if (_static_cali_complete) {
                bias_x = _gyro_cali_offset[0];
                bias_y = _gyro_cali_offset[1];
                bias_z = _gyro_cali_offset[2];
            }
            float x_corr = static_cast<float>(x - bias_x);
            float y_corr = static_cast<float>(y - bias_y);
            float z_corr = static_cast<float>(z - bias_z);
            gyro_data[gyro_idx].dps.x = x_corr * _gyro_scale;
            gyro_data[gyro_idx].dps.y = y_corr * _gyro_scale;
            gyro_data[gyro_idx].dps.z = z_corr * _gyro_scale;
            gyro_idx++;
        }
    }

    return samples_read;
}

bool SensorQMI8658::enableLockingMechanism()
{
    enableSyncMode();
    if (writeReg(REG_CAL1_L, 0x01) != 0) {
        return false;
    }
    return writeCommand(CTRL_CMD_AHB_CLOCK_GATING) == 0;
}

bool SensorQMI8658::disableLockingMechanism()
{
    disableSyncMode();
    if (writeReg(REG_CAL1_L, 0x00) != 0) {
        return false;
    }
    return writeCommand(CTRL_CMD_AHB_CLOCK_GATING) == 0;
}

bool SensorQMI8658::calibrate(uint16_t *gyro_x_gain, uint16_t *gyro_y_gain, uint16_t *gyro_z_gain)
{
    if (!hal) return false;

    if (_accel_enabled || _gyro_enabled) {
        disableAccel();
        disableGyro();
    }

    if (writeCommand(CTRL_CMD_ON_DEMAND_CALIBRATION, 3000) != 0) {
        log_e("Calibration command failed");
        return false;
    }

    hal->delay(1600);

    uint8_t status = readReg(REG_COD_STATUS);
    if (status & MASK_COD_FAIL) {
        log_e("Calibration failed");
        return false;
    }
    if (status & MASK_COD_GYRO_ENABLED_ERR) {
        log_e("Gyro enabled during calibration");
        return false;
    }
    if (status & MASK_COD_GYRO_STARTUP_ERR) {
        log_e("Gyro startup error during calibration");
        return false;
    }
    if (status & MASK_COD_ACCEL_ERR) {
        log_e("Accelerometer error during calibration");
        return false;
    }

    uint8_t buffer[6];
    if (readRegBuff(REG_DVX_L, buffer, 6) == 0) {
        if (gyro_x_gain) *gyro_x_gain = (uint16_t)buffer[0] | ((uint16_t)buffer[1] << 8);
        if (gyro_y_gain) *gyro_y_gain = (uint16_t)buffer[2] | ((uint16_t)buffer[3] << 8);
        if (gyro_z_gain) *gyro_z_gain = (uint16_t)buffer[4] | ((uint16_t)buffer[5] << 8);
    }

    return true;
}

bool SensorQMI8658::writeCalibration(uint16_t gyro_x_gain, uint16_t gyro_y_gain, uint16_t gyro_z_gain)
{
    if (_accel_enabled || _gyro_enabled) {
        disableAccel();
        disableGyro();
    }

    uint8_t buffer[6] = {
        (uint8_t)(gyro_x_gain & 0xFF),
        (uint8_t)((gyro_x_gain >> 8) & 0xFF),
        (uint8_t)(gyro_y_gain & 0xFF),
        (uint8_t)((gyro_y_gain >> 8) & 0xFF),
        (uint8_t)(gyro_z_gain & 0xFF),
        (uint8_t)((gyro_z_gain >> 8) & 0xFF),
    };

    if (writeRegBuff(REG_CAL1_L, buffer, 6) != 0) {
        return false;
    }

    return writeCommand(CTRL_CMD_APPLY_GYRO_GAINS) == 0;
}

void SensorQMI8658::setAccelOffset(int16_t x, int16_t y, int16_t z)
{
    uint8_t data[2];
    data[0] = (uint8_t)(x & 0xFF);
    data[1] = (uint8_t)((x >> 8) & 0xFF);
    writeRegBuff(REG_CAL1_L, data, 2);
    data[0] = (uint8_t)(y & 0xFF);
    data[1] = (uint8_t)((y >> 8) & 0xFF);
    writeRegBuff(REG_CAL2_L, data, 2);
    data[0] = (uint8_t)(z & 0xFF);
    data[1] = (uint8_t)((z >> 8) & 0xFF);
    writeRegBuff(REG_CAL3_L, data, 2);
    writeCommand(CTRL_CMD_ACCEL_HOST_DELTA_OFFSET);
}

void SensorQMI8658::setGyroOffset(int16_t x, int16_t y, int16_t z)
{
    uint8_t data[2];
    data[0] = (uint8_t)(x & 0xFF);
    data[1] = (uint8_t)((x >> 8) & 0xFF);
    writeRegBuff(REG_CAL1_L, data, 2);
    data[0] = (uint8_t)(y & 0xFF);
    data[1] = (uint8_t)((y >> 8) & 0xFF);
    writeRegBuff(REG_CAL2_L, data, 2);
    data[0] = (uint8_t)(z & 0xFF);
    data[1] = (uint8_t)((z >> 8) & 0xFF);
    writeRegBuff(REG_CAL3_L, data, 2);
    writeCommand(CTRL_CMD_GYRO_HOST_DELTA_OFFSET);
}

bool SensorQMI8658::selfTestAccel()
{
    if (!hal) return false;

    disableAccel();
    disableGyro();

    if (writeReg(REG_CTRL7, 0x00) != 0) {
        return false;
    }

    if (updateBits(REG_CTRL2, 0xF0, (ACCEL_ODR_7174_4HZ | 0x80)) != 0) {
        return false;
    }

    uint8_t retry = 50;
    while (retry-- > 0) {
        uint8_t status = readReg(REG_STATUS_INT);
        if (status != 0xFF && (status & MASK_INT_AVAIL)) {
            break;
        }
        hal->delay(20);
    }

    if (retry == 0) {
        log_e("Accelerometer self-test timeout");
        return false;
    }

    clrRegBit(REG_CTRL2, 7);

    uint8_t buffer[6];
    if (readRegBuff(REG_DVX_L, buffer, 6) != 0) {
        return false;
    }

    int16_t dVX = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t dVY = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t dVZ = (int16_t)((buffer[5] << 8) | buffer[4]);

    float dVX_mg = dVX * 0.5f;
    float dVY_mg = dVY * 0.5f;
    float dVZ_mg = dVZ * 0.5f;

    if (abs(dVX_mg) > 200.0f && abs(dVY_mg) > 200.0f && abs(dVZ_mg) > 200.0f) {
        return true;
    }

    log_e("Accelerometer self-test failed: dVX=%.1f, dVY=%.1f, dVZ=%.1f", dVX_mg, dVY_mg, dVZ_mg);
    return false;
}

bool SensorQMI8658::selfTestGyro()
{
    if (!hal) return false;

    disableAccel();
    disableGyro();

    if (writeReg(REG_CTRL7, 0x00) != 0) {
        return false;
    }

    setRegBit(REG_CTRL3, 7);

    uint8_t retry = 50;
    while (retry-- > 0) {
        uint8_t status = readReg(REG_STATUS_INT);
        if (status != 0xFF && (status & MASK_INT_AVAIL)) {
            break;
        }
        hal->delay(20);
    }

    if (retry == 0) {
        log_e("Gyroscope self-test timeout");
        return false;
    }

    clrRegBit(REG_CTRL3, 7);

    uint8_t buffer[6];
    if (readRegBuff(REG_DVX_L, buffer, 6) != 0) {
        return false;
    }

    float dVX = (float)(((int16_t)buffer[0] << 12) | (buffer[1] >> 4));
    float dVY = (float)(((int16_t)buffer[2] << 12) | (buffer[3] >> 4));
    float dVZ = (float)(((int16_t)buffer[4] << 12) | (buffer[5] >> 4));

    dVX *= (1.0f / 16.0f);
    dVY *= (1.0f / 16.0f);
    dVZ *= (1.0f / 16.0f);

    if (abs(dVX) > 300.0f && abs(dVY) > 300.0f && abs(dVZ) > 300.0f) {
        return true;
    }

    log_e("Gyroscope self-test failed: dVX=%.1f, dVY=%.1f, dVZ=%.1f", dVX, dVY, dVZ);
    return false;
}

bool SensorQMI8658::configMotionDetect(MotionType type, float threshold_x, float threshold_y,
                                       float threshold_z, uint8_t duration)
{
    uint8_t thr_x = mgToBytes(threshold_x);
    uint8_t thr_y = mgToBytes(threshold_y);
    uint8_t thr_z = mgToBytes(threshold_z);

    switch (type) {
    case MotionType::ANY_MOTION:
        _motion_any_thr[0] = thr_x;
        _motion_any_thr[1] = thr_y;
        _motion_any_thr[2] = thr_z;
        _motion_any_window = duration;
        _motion_any_configured = true;
        break;
    case MotionType::NO_MOTION:
        _motion_no_thr[0] = thr_x;
        _motion_no_thr[1] = thr_y;
        _motion_no_thr[2] = thr_z;
        _motion_no_window = duration;
        _motion_no_configured = true;
        break;
    case MotionType::SIGNIFICANT:
        _motion_sig_confirm_window = duration;
        _motion_sig_configured = true;
        break;
    default:
        return false;
    }

    if (_motion_sig_configured) {
        if (!_motion_any_configured) {
            _motion_any_thr[0] = mgToBytes(100.0f);
            _motion_any_thr[1] = mgToBytes(100.0f);
            _motion_any_thr[2] = mgToBytes(100.0f);
            _motion_any_window = 4;
            _motion_any_configured = true;
        }
        if (!_motion_no_configured) {
            _motion_no_thr[0] = mgToBytes(100.0f);
            _motion_no_thr[1] = mgToBytes(100.0f);
            _motion_no_thr[2] = mgToBytes(100.0f);
            _motion_no_window = 10;
            _motion_no_configured = true;
        }
    }

    return applyMotionConfig();
}

bool SensorQMI8658::applyMotionConfig()
{
    disableSyncMode();

    bool gyro_en = _gyro_enabled;
    bool accel_en = _accel_enabled;
    if (gyro_en) disableGyro();
    if (accel_en) disableAccel();

    if (!_motion_any_configured) {
        _motion_any_thr[0] = mgToBytes(100.0f);
        _motion_any_thr[1] = mgToBytes(100.0f);
        _motion_any_thr[2] = mgToBytes(100.0f);
        _motion_any_window = 4;
    }
    if (!_motion_no_configured) {
        _motion_no_thr[0] = mgToBytes(100.0f);
        _motion_no_thr[1] = mgToBytes(100.0f);
        _motion_no_thr[2] = mgToBytes(100.0f);
        _motion_no_window = 10;
    }

    bool ok = true;

    ok = ok && (writeReg(REG_CAL1_L, _motion_any_thr[0]) == 0);
    ok = ok && (writeReg(REG_CAL1_H, _motion_any_thr[1]) == 0);
    ok = ok && (writeReg(REG_CAL2_L, _motion_any_thr[2]) == 0);
    ok = ok && (writeReg(REG_CAL2_H, _motion_no_thr[0]) == 0);
    ok = ok && (writeReg(REG_CAL3_L, _motion_no_thr[1]) == 0);
    ok = ok && (writeReg(REG_CAL3_H, _motion_no_thr[2]) == 0);

    uint8_t mode_ctrl = 0;
    if (_motion_any_configured || _motion_sig_configured) {
        mode_ctrl |= MASK_ANY_MOTION_X_EN | MASK_ANY_MOTION_Y_EN | MASK_ANY_MOTION_Z_EN;
    }
    if (_motion_no_configured || _motion_sig_configured) {
        mode_ctrl |= MASK_NO_MOTION_X_EN | MASK_NO_MOTION_Y_EN | MASK_NO_MOTION_Z_EN;
        // Use stricter No-Motion axis logic to avoid false positives while moving.
        // Requiring all enabled axes to satisfy No-Motion keeps bit6 from staying high.
        mode_ctrl |= MASK_NO_MOTION_LOGIC;
    }

    ok = ok && (writeReg(REG_CAL4_L, mode_ctrl) == 0);
    ok = ok && (writeReg(REG_CAL4_H, 0x01) == 0);
    ok = ok && (writeCommand(CTRL_CMD_CONFIGURE_MOTION) == 0);

    ok = ok && (writeReg(REG_CAL1_L, _motion_any_window) == 0);
    ok = ok && (writeReg(REG_CAL1_H, _motion_no_window) == 0);
    ok = ok && (writeReg(REG_CAL2_L, static_cast<uint8_t>(_motion_sig_wait_window & 0xFF)) == 0);
    ok = ok && (writeReg(REG_CAL2_H, static_cast<uint8_t>((_motion_sig_wait_window >> 8) & 0xFF)) == 0);
    ok = ok && (writeReg(REG_CAL3_L, static_cast<uint8_t>(_motion_sig_confirm_window & 0xFF)) == 0);
    ok = ok && (writeReg(REG_CAL3_H, static_cast<uint8_t>((_motion_sig_confirm_window >> 8) & 0xFF)) == 0);
    ok = ok && (writeReg(REG_CAL4_H, 0x02) == 0);
    ok = ok && (writeCommand(CTRL_CMD_CONFIGURE_MOTION) == 0);

    if (gyro_en) enableGyro();
    if (accel_en) enableAccel();

    return ok;
}

bool SensorQMI8658::configMotionDetectDefault(MotionType type)
{
    switch (type) {
    case MotionType::ANY_MOTION:
        return configMotionDetect(type, 100.0f, 100.0f, 100.0f, 4);
    case MotionType::NO_MOTION:
        return configMotionDetect(type, 100.0f, 100.0f, 100.0f, 10);
    case MotionType::SIGNIFICANT:
        return configMotionDetect(type, 200.0f, 200.0f, 200.0f, 8);
    default:
        break;
    }
    return false;
}

bool SensorQMI8658::enableMotionDetect(IntPin pin)
{
    if (!_accel_enabled) return false;

    switch (pin) {
    case IntPin::PIN1:
    case IntPin::PIN2:
        enableInterrupt(pin);
        setRegBit(REG_CTRL8, 6);
        break;
    default:
        break;
    }

    const bool enable_any = _motion_any_configured || _motion_sig_configured;
    const bool enable_no = _motion_no_configured || _motion_sig_configured;
    const bool enable_sig = _motion_sig_configured;

    if (enable_any) {
        setRegBit(REG_CTRL8, SHIFT_ANY_MOTION_ENABLE);
    } else {
        clrRegBit(REG_CTRL8, SHIFT_ANY_MOTION_ENABLE);
    }

    if (enable_no) {
        setRegBit(REG_CTRL8, SHIFT_NO_MOTION_ENABLE);
    } else {
        clrRegBit(REG_CTRL8, SHIFT_NO_MOTION_ENABLE);
    }

    if (enable_sig) {
        setRegBit(REG_CTRL8, SHIFT_SIGNIFICANT_MOTION_EN);
    } else {
        clrRegBit(REG_CTRL8, SHIFT_SIGNIFICANT_MOTION_EN);
    }

    return true;
}

bool SensorQMI8658::disableMotionDetect()
{
    clrRegBit(REG_CTRL8, SHIFT_ANY_MOTION_ENABLE);
    clrRegBit(REG_CTRL8, SHIFT_NO_MOTION_ENABLE);
    clrRegBit(REG_CTRL8, SHIFT_SIGNIFICANT_MOTION_EN);

    return true;
}

bool SensorQMI8658::configWakeOnMotion(uint8_t threshold, float odr_hz, IntPin pin)
{
    return configWakeOnMotionAdvanced(threshold, odr_hz, pin, 1, 0x20, AccelFullScaleRange::FS_8G);
}

bool SensorQMI8658::configWakeOnMotionAdvanced(uint8_t threshold, float odr_hz, IntPin pin,
        uint8_t default_pin_value, uint8_t blanking_time,
        AccelFullScaleRange acc_range)
{
    if (!hal) return false;

    reset();

    clrRegBit(REG_CTRL7, 0);

    uint8_t range_val = getAccelRangeRegValue(acc_range);
    if (updateBits(REG_CTRL2, 0x70, static_cast<uint8_t>(range_val << 4)) != 0) {
        return false;
    }

    uint8_t odr_val = findClosestAccelLpOdr(odr_hz);
    if (updateBits(REG_CTRL2, 0x0F, odr_val) != 0) {
        return false;
    }

    if (writeReg(REG_CAL1_L, threshold) != 0) {
        return false;
    }

    uint8_t cal1_h = 0;
    if (pin == IntPin::PIN1) {
        cal1_h = default_pin_value ? 0x02 : 0x00;
    } else if (pin == IntPin::PIN2) {
        cal1_h = default_pin_value ? 0x03 : 0x01;
    } else {
        return false;
    }
    cal1_h = static_cast<uint8_t>((cal1_h << 6) | (blanking_time & 0x3F));
    if (writeReg(REG_CAL1_H, cal1_h) != 0) {
        return false;
    }

    if (writeCommand(CTRL_CMD_WRITE_WOM_SETTING) != 0) {
        return false;
    }

    enableAccel();
    enableInterrupt(pin);

    return true;
}

bool SensorQMI8658::configAccel(AccelRange range, AccelODR odr, LpfMode lpf)
{
    AccelFullScaleRange common_range = AccelFullScaleRange::FS_8G;
    switch (range) {
    case AccelRange::FS_2G:
        common_range = AccelFullScaleRange::FS_2G;
        break;
    case AccelRange::FS_4G:
        common_range = AccelFullScaleRange::FS_4G;
        break;
    case AccelRange::FS_8G:
        common_range = AccelFullScaleRange::FS_8G;
        break;
    case AccelRange::FS_16G:
        common_range = AccelFullScaleRange::FS_16G;
        break;
    }
    return configAccel(common_range, legacyAccelOdrToFloat(odr), lpf);
}

bool SensorQMI8658::configAccel(AccelRange range, float data_rate_hz, LpfMode lpf)
{
    AccelFullScaleRange common_range = AccelFullScaleRange::FS_8G;
    switch (range) {
    case AccelRange::FS_2G:
        common_range = AccelFullScaleRange::FS_2G;
        break;
    case AccelRange::FS_4G:
        common_range = AccelFullScaleRange::FS_4G;
        break;
    case AccelRange::FS_8G:
        common_range = AccelFullScaleRange::FS_8G;
        break;
    case AccelRange::FS_16G:
        common_range = AccelFullScaleRange::FS_16G;
        break;
    }
    return configAccel(common_range, data_rate_hz, lpf);
}

bool SensorQMI8658::configGyro(GyroRange range, GyroODR odr, LpfMode lpf)
{
    GyroFullScaleRange common_range = GyroFullScaleRange::FS_1000_DPS;
    switch (range) {
    case GyroRange::FS_128DPS:
        common_range = GyroFullScaleRange::FS_125_DPS;
        break;
    case GyroRange::FS_256DPS:
        common_range = GyroFullScaleRange::FS_250_DPS;
        break;
    case GyroRange::FS_512DPS:
        common_range = GyroFullScaleRange::FS_500_DPS;
        break;
    case GyroRange::FS_1024DPS:
        common_range = GyroFullScaleRange::FS_1000_DPS;
        break;
    case GyroRange::FS_2048DPS:
        common_range = GyroFullScaleRange::FS_2000_DPS;
        break;
    default:
        common_range = GyroFullScaleRange::FS_1000_DPS;
        break;
    }
    return configGyro(common_range, legacyGyroOdrToFloat(odr), lpf);
}

bool SensorQMI8658::configGyro(GyroRange range, float data_rate_hz, LpfMode lpf)
{
    GyroFullScaleRange common_range = GyroFullScaleRange::FS_1000_DPS;
    switch (range) {
    case GyroRange::FS_128DPS:
        common_range = GyroFullScaleRange::FS_125_DPS;
        break;
    case GyroRange::FS_256DPS:
        common_range = GyroFullScaleRange::FS_250_DPS;
        break;
    case GyroRange::FS_512DPS:
        common_range = GyroFullScaleRange::FS_500_DPS;
        break;
    case GyroRange::FS_1024DPS:
        common_range = GyroFullScaleRange::FS_1000_DPS;
        break;
    case GyroRange::FS_2048DPS:
        common_range = GyroFullScaleRange::FS_2000_DPS;
        break;
    default:
        common_range = GyroFullScaleRange::FS_1000_DPS;
        break;
    }
    return configGyro(common_range, data_rate_hz, lpf);
}

bool SensorQMI8658::configWakeOnMotion(uint8_t threshold, AccelODR odr, IntPin pin)
{
    return configWakeOnMotion(threshold, legacyAccelOdrToFloat(odr), pin);
}

bool SensorQMI8658::configWakeOnMotion(uint8_t threshold, AccelODR odr, IntPin pin,
                                       uint8_t default_pin_value, uint8_t blanking_time,
                                       AccelRange acc_range)
{
    AccelFullScaleRange common_range = AccelFullScaleRange::FS_8G;
    switch (acc_range) {
    case AccelRange::FS_2G:
        common_range = AccelFullScaleRange::FS_2G;
        break;
    case AccelRange::FS_4G:
        common_range = AccelFullScaleRange::FS_4G;
        break;
    case AccelRange::FS_8G:
        common_range = AccelFullScaleRange::FS_8G;
        break;
    case AccelRange::FS_16G:
        common_range = AccelFullScaleRange::FS_16G;
        break;
    }
    return configWakeOnMotionAdvanced(threshold, legacyAccelOdrToFloat(odr), pin,
                                      default_pin_value, blanking_time, common_range);
}

uint8_t SensorQMI8658::mgToBytes(float mg)
{
    float g = mg / 1000.0f;
    int units = static_cast<int>(roundf(g / 0.03125f));
    if (units < 0) {
        units = 0;
    } else if (units > 0xFF) {
        units = 0xFF;
    }
    return static_cast<uint8_t>(units);
}

bool SensorQMI8658::configTap(TapPriority priority, uint8_t peak_window, uint16_t tap_window,
                              uint16_t double_tap_window, float peak_mag_threshold, float quiet_threshold)
{
    disableSyncMode();

    bool gyro_en = _gyro_enabled;
    bool accel_en = _accel_enabled;
    if (gyro_en) disableGyro();
    if (accel_en) disableAccel();

    writeReg(REG_CAL1_L, peak_window);
    writeReg(REG_CAL1_H, toQmiTapPriority(priority));
    writeReg(REG_CAL2_L, (uint8_t)(tap_window & 0xFF));
    writeReg(REG_CAL2_H, (uint8_t)((tap_window >> 8) & 0xFF));
    writeReg(REG_CAL3_L, (uint8_t)(double_tap_window & 0xFF));
    writeReg(REG_CAL3_H, (uint8_t)((double_tap_window >> 8) & 0xFF));
    writeReg(REG_CAL4_H, 0x01);

    writeCommand(CTRL_CMD_CONFIGURE_TAP);

    const float alpha = 0.0625f;
    const float gamma = 0.25f;
    uint8_t alpha_hex = static_cast<uint8_t>(alpha * 128.0f);
    uint8_t gamma_hex = static_cast<uint8_t>(gamma * 128.0f);
    writeReg(REG_CAL1_L, alpha_hex);
    writeReg(REG_CAL1_H, gamma_hex);

    const float g = QMI8658_CONSTANT_ONE_G;
    float resolution = 0.001f * g * g;

    uint16_t peak_val = (uint16_t)((peak_mag_threshold * g * g) / resolution);
    uint16_t quiet_val = (uint16_t)((quiet_threshold * g * g) / resolution);

    writeReg(REG_CAL2_L, (uint8_t)(peak_val & 0xFF));
    writeReg(REG_CAL2_H, (uint8_t)((peak_val >> 8) & 0xFF));
    writeReg(REG_CAL3_L, (uint8_t)(quiet_val & 0xFF));
    writeReg(REG_CAL3_H, (uint8_t)((quiet_val >> 8) & 0xFF));
    writeReg(REG_CAL4_H, 0x02);

    writeCommand(CTRL_CMD_CONFIGURE_TAP);

    if (gyro_en) enableGyro();
    if (accel_en) enableAccel();

    return true;
}

bool SensorQMI8658::configTapDefault(TapPriority priority)
{
    return configTap(priority,
                     30,         // peak_window: 30 samples
                     100,        // tap_window: 100 samples
                     500,        // double_tap_window: 500 samples
                     1.5f,       // peak_mag_threshold: 1.5 g^2
                     0.5f);      // quiet_threshold: 0.5 g^2
}

bool SensorQMI8658::enableTap(IntPin pin)
{
    if (!_accel_enabled) return false;
    switch (pin) {
    case IntPin::PIN1:
    case IntPin::PIN2:
        enableInterrupt(pin);
        setRegBit(REG_CTRL8, 6);
        break;
    default:
        break;
    }
    return setRegBit(REG_CTRL8, 0);
}

bool SensorQMI8658::disableTap()
{
    return clrRegBit(REG_CTRL8, 0);
}

SensorQMI8658::TapEvent SensorQMI8658::getTapStatus()
{
    uint8_t status = readReg(REG_TAP_STATUS);
    uint8_t tap_type = status & MASK_TAP_TYPE;

    switch (tap_type) {
    case TAP_TYPE_SINGLE:
        return TapEvent::SINGLE;
    case TAP_TYPE_DOUBLE:
        return TapEvent::DOUBLE;
    default:
        return TapEvent::INVALID;
    }
}

bool SensorQMI8658::configPedometer(uint16_t sample_count, uint16_t peak_to_peak, uint16_t peak_threshold,
                                    uint16_t time_up, uint8_t time_low, uint8_t entry_count,
                                    uint8_t fix_precision, uint8_t sig_count)
{
    disableSyncMode();

    bool gyro_en = _gyro_enabled;
    bool accel_en = _accel_enabled;
    if (gyro_en) disableGyro();
    if (accel_en) disableAccel();

    uint8_t data[2];
    data[0] = (uint8_t)(sample_count & 0xFF);
    data[1] = (uint8_t)((sample_count >> 8) & 0xFF);
    if (writeRegBuff(REG_CAL1_L, data, 2) != 0) {
        return false;
    }
    data[0] = (uint8_t)(peak_to_peak & 0xFF);
    data[1] = (uint8_t)((peak_to_peak >> 8) & 0xFF);
    if (writeRegBuff(REG_CAL2_L, data, 2) != 0) {
        return false;
    }
    data[0] = (uint8_t)(peak_threshold & 0xFF);
    data[1] = (uint8_t)((peak_threshold >> 8) & 0xFF);
    if (writeRegBuff(REG_CAL3_L, data, 2) != 0) {
        return false;
    }
    if (writeReg(REG_CAL4_H, 0x01) != 0 || writeReg(REG_CAL4_L, 0x02) != 0) {
        return false;
    }

    if (writeCommand(CTRL_CMD_CONFIGURE_PEDOMETER) != 0) {
        return false;
    }

    data[0] = (uint8_t)(time_up & 0xFF);
    data[1] = (uint8_t)((time_up >> 8) & 0xFF);
    if (writeRegBuff(REG_CAL1_L, data, 2) != 0) {
        return false;
    }
    if (writeReg(REG_CAL2_L, time_low) != 0 || writeReg(REG_CAL2_H, entry_count) != 0 ||
            writeReg(REG_CAL3_L, fix_precision) != 0 || writeReg(REG_CAL3_H, sig_count) != 0 ||
            writeReg(REG_CAL4_H, 0x02) != 0 || writeReg(REG_CAL4_L, 0x02) != 0) {
        return false;
    }

    if (writeCommand(CTRL_CMD_CONFIGURE_PEDOMETER) != 0) {
        return false;
    }

    if (gyro_en) enableGyro();
    if (accel_en) enableAccel();

    return true;
}

bool SensorQMI8658::configPedometerDefault(float odr)
{
    float rate = 1000.0f / odr;
    uint16_t sample_count = 50;
    uint16_t peak_to_peak = 100;
    uint16_t peak_threshold = 116;
    uint16_t time_up = static_cast<uint16_t>(2000.0f / rate);
    uint8_t time_low = static_cast<uint8_t>(300.0f / rate);
    uint8_t entry_count = 8;
    uint8_t sig_count = 1;

    return configPedometer(sample_count, peak_to_peak, peak_threshold,
                           time_up, time_low, entry_count, 0, sig_count);
}

bool SensorQMI8658::enablePedometer(IntPin pin)
{
    if (!_accel_enabled) return false;

    disableSyncMode();

    _last_step_count = getStepCount();

    switch (pin) {
    case IntPin::PIN1:
    case IntPin::PIN2:
        enableInterrupt(pin);
        setRegBit(REG_CTRL8, 6);
        break;
    default:
        break;
    }

    // Datasheet: toggling Pedo_EN from 0->1 resets step counter and restarts engine.
    clrRegBit(REG_CTRL8, 4);
    bool ok = setRegBit(REG_CTRL8, 4);
    if (ok) {
        _last_step_count = 0;
        enableAccel();
        if (_gyro_enabled) {
            enableGyro();
        }
    }
    return ok;
}

bool SensorQMI8658::disablePedometer()
{
    return clrRegBit(REG_CTRL8, 4);
}

uint32_t SensorQMI8658::getStepCount()
{
    uint8_t buffer[3];
    if (readRegBuff(REG_STEP_CNT_L, buffer, 3) == 0) {
        return (uint32_t)(buffer[2] << 16) | (uint32_t)(buffer[1] << 8) | buffer[0];
    }
    return 0;
}

bool SensorQMI8658::resetStepCount()
{
    return writeCommand(CTRL_CMD_RESET_PEDOMETER) == 0;
}

void SensorQMI8658::setAnyMotionCallback(MotionCallback callback)
{
    callbacks.onAnyMotion = callback;
}

void SensorQMI8658::setNoMotionCallback(MotionCallback callback)
{
    callbacks.onNoMotion = callback;
}

void SensorQMI8658::setSignificantMotionCallback(MotionCallback callback)
{
    callbacks.onSignificantMotion = callback;
}

void SensorQMI8658::setWakeOnMotionCallback(MotionCallback callback)
{
    callbacks.onWakeOnMotion = callback;
}

void SensorQMI8658::setTapCallback(TapCallback callback)
{
    callbacks.onTap = callback;
}

void SensorQMI8658::setPedometerCallback(MotionCallback callback)
{
    callbacks.onPedometer = callback;
}

void SensorQMI8658::setAccelDataReadyCallback(DataReadyCallback callback)
{
    callbacks.onAccelDataReady = callback;
}

void SensorQMI8658::setGyroDataReadyCallback(DataReadyCallback callback)
{
    callbacks.onGyroDataReady = callback;
}

void SensorQMI8658::setDataLockingCallback(DataReadyCallback callback)
{
    callbacks.onDataLocking = callback;
}

void SensorQMI8658::setCallbacks(const EventCallbacks &cbs)
{
    callbacks = cbs;
}

SensorQMI8658::EventCallbacks &SensorQMI8658::getCallbacks()
{
    return callbacks;
}

const SensorQMI8658::EventCallbacks &SensorQMI8658::getCallbacks() const
{
    return callbacks;
}

uint16_t SensorQMI8658::update()
{
    uint16_t result = 0;

    uint8_t status[3];
    if (readRegBuff(REG_STATUS_INT, status, 3) != 0) {
        return 0;
    }

    if (status[0] & MASK_INT_CTRL9_DONE) {
        result |= 0x80;
    }

    if (status[0] & MASK_INT_AVAIL) {
        result |= 0x01;
    }

    if ((status[0] & 0x03) == 0x03) {
        result |= 0x100;
        if (callbacks.onDataLocking) {
            callbacks.onDataLocking();
        }
    }

    if (!_sync_mode) {
        if (status[1] & MASK_GYRO_DATA_RDY) {
            result |= 0x02;
            if (callbacks.onGyroDataReady) {
                callbacks.onGyroDataReady();
            }
        }

        if (status[1] & MASK_ACCEL_DATA_RDY) {
            result |= 0x04;
            if (callbacks.onAccelDataReady) {
                callbacks.onAccelDataReady();
            }
        }
    }

    if (status[2] & STATUS_SIGNIFICANT_MOTION) {
        result |= STATUS_SIGNIFICANT_MOTION;
        if (callbacks.onSignificantMotion) {
            callbacks.onSignificantMotion();
        }
    }

    if (status[2] & STATUS_NO_MOTION_EVENT) {
        result |= STATUS_NO_MOTION_EVENT;
        if (callbacks.onNoMotion) {
            callbacks.onNoMotion();
        }
    }

    if (status[2] & STATUS_ANY_MOTION_EVENT) {
        result |= STATUS_ANY_MOTION_EVENT;
        if (callbacks.onAnyMotion) {
            callbacks.onAnyMotion();
        }
    }

    bool pedometer_event = (status[2] & STATUS_PEDOMETER_EVENT) != 0;
    uint32_t step_count = getStepCount();
    if (step_count != _last_step_count) {
        pedometer_event = true;
        _last_step_count = step_count;
    }
    if (pedometer_event) {
        result |= STATUS_PEDOMETER_EVENT;
        if (callbacks.onPedometer) {
            callbacks.onPedometer();
        }
    }

    if (status[2] & STATUS_WOM_EVENT) {
        result |= STATUS_WOM_EVENT;
        if (callbacks.onWakeOnMotion) {
            callbacks.onWakeOnMotion();
        }
    }

    bool tap_now = (status[2] & STATUS_TAP_EVENT) != 0;
    if (tap_now) {
        TapEvent tap_event = getTapStatus();
        if (tap_event != TapEvent::INVALID) {
            uint32_t now_ms = 0;
            now_ms = hal->millis();
            bool in_cooldown = (now_ms - _last_tap_event_ms) < 500;
            bool duplicate = (tap_event == _last_tap_event) && in_cooldown;
            if ((!duplicate && !in_cooldown) || !_tap_event_active) {
                result |= STATUS_TAP_EVENT;
                if (callbacks.onTap) {
                    callbacks.onTap(tap_event);
                }
                _last_tap_event = tap_event;
                _last_tap_event_ms = now_ms;
            }
        }
    }
    _tap_event_active = tap_now;

    return result;
}

void SensorQMI8658::dumpRegisters()
{
    uint8_t buffer[9];
    if (readRegBuff(REG_CTRL1, buffer, 9) == 0) {
        for (int i = 0; i < 9; ++i) {
            log_i("CTRL%d: 0x%02X", i + 1, buffer[i]);
        }
    }

    log_i("FIFO_CTRL: 0x%02X", readReg(REG_FIFO_CTRL));
    log_i("STATUS_INT: 0x%02X", readReg(REG_STATUS_INT));
    log_i("STATUS0: 0x%02X", readReg(REG_STATUS0));
    log_i("STATUS1: 0x%02X", readReg(REG_STATUS1));
}

void SensorQMI8658::getChipUsid(uint8_t *buffer, uint8_t length)
{
    if (length > 6) {
        length = 6;
    }
    memcpy(buffer, _usid, length);
}

float SensorQMI8658::getAccelScale()
{
    return _accel_scale;
}

float SensorQMI8658::getGyroScale()
{
    return _gyro_scale;
}

void SensorQMI8658::setAxisLayout(Layout layout)
{
    _axis_layout = layout;
}

SensorQMI8658::Layout SensorQMI8658::getAxisLayout() const
{
    return _axis_layout;
}

void SensorQMI8658::axisConvert(int16_t accel_data[3], int16_t gyro_data[3])
{
    int16_t raw_a[3], raw_g[3];

    raw_a[0] = accel_data[0];
    raw_a[1] = accel_data[1];
    raw_a[2] = accel_data[2];
    raw_g[0] = gyro_data[0];
    raw_g[1] = gyro_data[1];
    raw_g[2] = gyro_data[2];

    uint8_t layout = static_cast<uint8_t>(_axis_layout);

    if (layout >= 4 && layout <= 7) {
        accel_data[2] = -accel_data[2];
        gyro_data[2] = -gyro_data[2];
    }

    if (layout & 0x01) {
        accel_data[0] = raw_a[1];
        accel_data[1] = raw_a[0];
        gyro_data[0] = raw_g[1];
        gyro_data[1] = raw_g[0];
    } else {
        accel_data[0] = raw_a[0];
        accel_data[1] = raw_a[1];
        gyro_data[0] = raw_g[0];
        gyro_data[1] = raw_g[1];
    }

    if (layout == 1 || layout == 2 || layout == 4 || layout == 7) {
        accel_data[0] = -accel_data[0];
        gyro_data[0] = -gyro_data[0];
    }
    if (layout == 2 || layout == 3 || layout == 6 || layout == 7) {
        accel_data[1] = -accel_data[1];
        gyro_data[1] = -gyro_data[1];
    }
}

void SensorQMI8658::enableStaticCalibration(bool enable)
{
    _static_cali_enabled = enable;
    if (!enable) {
        resetStaticCalibration();
    }
}

bool SensorQMI8658::isStaticCalibrationEnabled() const
{
    return _static_cali_enabled;
}

bool SensorQMI8658::isStaticCalibrationComplete() const
{
    return _static_cali_complete;
}

bool SensorQMI8658::processStaticCalibration(int16_t accel_data[3], int16_t gyro_data[3])
{
    if (!_static_cali_enabled || _static_cali_complete) {
        return _static_cali_complete;
    }

    if (_static_cali_sample_count == 0) {
        memset(_accel_cali_sum, 0, sizeof(_accel_cali_sum));
        memset(_gyro_cali_sum, 0, sizeof(_gyro_cali_sum));
    }

    if (_static_cali_sample_count < MAX_STATIC_CALI_SAMPLES) {
        for (int i = 0; i < 3; i++) {
            _gyro_cali_sum[i] += gyro_data[i];
            if (i == 2) {
                _accel_cali_sum[i] += (accel_data[i] - (1 << 14));
            } else {
                _accel_cali_sum[i] += accel_data[i];
            }
        }
        _static_cali_sample_count++;
    }

    if (_static_cali_sample_count >= MAX_STATIC_CALI_SAMPLES) {
        for (int i = 0; i < 3; i++) {
            _accel_cali_offset[i] = _accel_cali_sum[i] / MAX_STATIC_CALI_SAMPLES;
            _gyro_cali_offset[i] = _gyro_cali_sum[i] / MAX_STATIC_CALI_SAMPLES;
        }
        _static_cali_sample_count = 0;
        _static_cali_complete = true;
        return true;
    }

    return false;
}

void SensorQMI8658::getStaticCalibrationOffsets(int16_t accel_offset[3], int16_t gyro_offset[3])
{
    if (accel_offset) {
        memcpy(accel_offset, _accel_cali_offset, sizeof(_accel_cali_offset));
    }
    if (gyro_offset) {
        memcpy(gyro_offset, _gyro_cali_offset, sizeof(_gyro_cali_offset));
    }
}

void SensorQMI8658::resetStaticCalibration()
{
    _static_cali_sample_count = 0;
    _static_cali_complete = false;
    memset(_accel_cali_sum, 0, sizeof(_accel_cali_sum));
    memset(_gyro_cali_sum, 0, sizeof(_gyro_cali_sum));
    memset(_accel_cali_offset, 0, sizeof(_accel_cali_offset));
    memset(_gyro_cali_offset, 0, sizeof(_gyro_cali_offset));
}

void SensorQMI8658::enableDynamicGyroCalibration(bool enable)
{
    _dyn_cal_enabled = enable;
    if (enable) {
        _gyro_static_deviation = 0.005f * 57.29578f * (1 << 15);
        _accel_static_deviation = 0.04f / 9.8f * (1 << 14);
    } else {
        memset(_dyn_gyro_offset, 0, sizeof(_dyn_gyro_offset));
        _dyn_static_delay = 0;
        _dyn_static_flag = 0;
        _dyn_cali_sample_count = 0;
        _dyn_offset_applied = 0;
    }
}

bool SensorQMI8658::isDynamicGyroCalibrationEnabled() const
{
    return _dyn_cal_enabled;
}

void SensorQMI8658::getDynamicGyroCalibrationOffsets(int16_t gyro_offset[3])
{
    if (gyro_offset) {
        memcpy(gyro_offset, _dyn_gyro_offset, sizeof(_dyn_gyro_offset));
    }
}

void SensorQMI8658::processDynamicCalibration(int16_t accel_data[3], int16_t gyro_data[3])
{
    if (!_dyn_cal_enabled) {
        return;
    }

    float gyro_magnitude = sqrtf(
                               static_cast<float>(gyro_data[0]) * gyro_data[0] +
                               static_cast<float>(gyro_data[1]) * gyro_data[1] +
                               static_cast<float>(gyro_data[2]) * gyro_data[2]
                           );
    float accel_magnitude = sqrtf(
                                static_cast<float>(accel_data[0]) * accel_data[0] +
                                static_cast<float>(accel_data[1]) * accel_data[1] +
                                static_cast<float>(accel_data[2]) * accel_data[2]
                            );

    float gyro_sum = 0;
    float accel_sum = 0;

    for (int i = 0; i < DYN_CAL_BUFFER_SIZE - 1; i++) {
        _accel_speed_buffer[i] = _accel_speed_buffer[i + 1];
        accel_sum += _accel_speed_buffer[i];
        _gyro_speed_buffer[i] = _gyro_speed_buffer[i + 1];
        gyro_sum += _gyro_speed_buffer[i];
    }

    _accel_speed_buffer[DYN_CAL_BUFFER_SIZE - 1] = accel_magnitude;
    accel_sum += _accel_speed_buffer[DYN_CAL_BUFFER_SIZE - 1];
    _gyro_speed_buffer[DYN_CAL_BUFFER_SIZE - 1] = gyro_magnitude;
    gyro_sum += _gyro_speed_buffer[DYN_CAL_BUFFER_SIZE - 1];

    float accel_avg = accel_sum / DYN_CAL_BUFFER_SIZE;
    float gyro_avg = gyro_sum / DYN_CAL_BUFFER_SIZE;

    gyro_sum = 0;
    accel_sum = 0;

    for (int i = 0; i < DYN_CAL_BUFFER_SIZE; i++) {
        float diff = _accel_speed_buffer[i] - accel_avg;
        accel_sum += diff * diff;
        diff = _gyro_speed_buffer[i] - gyro_avg;
        gyro_sum += diff * diff;
    }

    float accel_std_dev = sqrtf(accel_sum / DYN_CAL_BUFFER_SIZE);
    float gyro_std_dev = sqrtf(gyro_sum / DYN_CAL_BUFFER_SIZE);

    if (accel_std_dev < _accel_static_deviation && gyro_std_dev < _gyro_static_deviation) {
        if (_dyn_static_flag == 0) {
            if (_dyn_static_delay < 12) {
                _dyn_static_delay++;
            } else {
                _dyn_static_delay = 0;
                _dyn_static_flag = 1;
            }
        } else {
            if (_dyn_offset_applied != 1) {
                if (_dyn_cali_sample_count == 0) {
                    for (int i = 0; i < 3; i++) {
                        _dyn_gyro_sum[i] = 0;
                    }
                    _dyn_cali_sample_count++;
                } else if (_dyn_cali_sample_count < DYN_CAL_STATIC_COUNT + 1) {
                    for (int i = 0; i < 3; i++) {
                        _dyn_gyro_sum[i] += gyro_data[i];
                    }
                    _dyn_cali_sample_count++;
                } else if (_dyn_cali_sample_count == DYN_CAL_STATIC_COUNT + 1) {
                    for (int i = 0; i < 3; i++) {
                        _dyn_gyro_offset[i] = _dyn_gyro_sum[i] / DYN_CAL_STATIC_COUNT;
                    }
                    _dyn_offset_applied = 1;
                    _dyn_cali_sample_count = 0;
                }
            }
        }
    } else {
        _dyn_static_delay = 0;
        _dyn_static_flag = 0;
        _dyn_offset_applied = 0;
        _dyn_cali_sample_count = 0;
    }
}

bool SensorQMI8658::hardwareSelfTest(bool includeAccel, bool includeGyro)
{
    if (!hal) return false;

    bool gyro_en = _gyro_enabled;
    bool accel_en = _accel_enabled;
    if (gyro_en) disableGyro();
    if (accel_en) disableAccel();

    writeReg(REG_CTRL7, 0x00);

    if (includeAccel) {
        writeReg(REG_CTRL2, static_cast<uint8_t>((ACCEL_RANGE_8G << 4) | ACCEL_ODR_224_2HZ | 0x80));

        uint8_t retry = 50;
        while (retry-- > 0) {
            uint8_t status = readReg(REG_STATUS_INT);
            if (status != 0xFF && (status & MASK_INT_AVAIL)) {
                break;
            }
            hal->delay(1);
        }

        writeReg(REG_CTRL2, static_cast<uint8_t>((ACCEL_RANGE_8G << 4) | ACCEL_ODR_224_2HZ));

        retry = 50;
        while (retry-- > 0) {
            uint8_t status = readReg(REG_STATUS_INT);
            if (status != 0xFF && !(status & MASK_INT_AVAIL)) {
                break;
            }
            hal->delay(1);
        }

        uint8_t reg_data[6];
        if (readRegBuff(REG_DVX_L, reg_data, 6) != 0) {
            if (accel_en) enableAccel();
            if (gyro_en) enableGyro();
            return false;
        }

        int16_t raw_x = (int16_t)((reg_data[1] << 8) | reg_data[0]);
        int16_t raw_y = (int16_t)((reg_data[3] << 8) | reg_data[2]);
        int16_t raw_z = (int16_t)((reg_data[5] << 8) | reg_data[4]);

        _hw_st_accel_result[0] = static_cast<float>(raw_x) * 1000.0f / 2048.0f;
        _hw_st_accel_result[1] = static_cast<float>(raw_y) * 1000.0f / 2048.0f;
        _hw_st_accel_result[2] = static_cast<float>(raw_z) * 1000.0f / 2048.0f;

        if (fabs(_hw_st_accel_result[0]) <= HW_ST_ACCEL_THRESHOLD_MG ||
                fabs(_hw_st_accel_result[1]) <= HW_ST_ACCEL_THRESHOLD_MG ||
                fabs(_hw_st_accel_result[2]) <= HW_ST_ACCEL_THRESHOLD_MG) {
            log_e("HW Self-test Accel failed: X=%.1f Y=%.1f Z=%.1f mg",
                  _hw_st_accel_result[0], _hw_st_accel_result[1], _hw_st_accel_result[2]);
            if (accel_en) enableAccel();
            if (gyro_en) enableGyro();
            return false;
        }
    }

    if (includeGyro) {
        writeReg(REG_CTRL3, static_cast<uint8_t>((GYRO_RANGE_1024DPS << 4) | GYRO_ODR_224_2HZ | 0x80));

        uint8_t retry = 50;
        while (retry-- > 0) {
            uint8_t status = readReg(REG_STATUS_INT);
            if (status != 0xFF && (status & MASK_INT_AVAIL)) {
                break;
            }
            hal->delay(1);
        }

        writeReg(REG_CTRL3, static_cast<uint8_t>((GYRO_RANGE_1024DPS << 4) | GYRO_ODR_224_2HZ));

        retry = 50;
        while (retry-- > 0) {
            uint8_t status = readReg(REG_STATUS_INT);
            if (status != 0xFF && !(status & MASK_INT_AVAIL)) {
                break;
            }
            hal->delay(1);
        }

        uint8_t reg_data[6];
        if (readRegBuff(REG_DVX_L, reg_data, 6) != 0) {
            if (accel_en) enableAccel();
            if (gyro_en) enableGyro();
            return false;
        }

        int16_t raw_x = (int16_t)((reg_data[1] << 8) | reg_data[0]);
        int16_t raw_y = (int16_t)((reg_data[3] << 8) | reg_data[2]);
        int16_t raw_z = (int16_t)((reg_data[5] << 8) | reg_data[4]);

        _hw_st_gyro_result[0] = static_cast<float>(raw_x) / 16.0f;
        _hw_st_gyro_result[1] = static_cast<float>(raw_y) / 16.0f;
        _hw_st_gyro_result[2] = static_cast<float>(raw_z) / 16.0f;

        if (fabs(_hw_st_gyro_result[0]) <= HW_ST_GYRO_THRESHOLD_DPS ||
                fabs(_hw_st_gyro_result[1]) <= HW_ST_GYRO_THRESHOLD_DPS ||
                fabs(_hw_st_gyro_result[2]) <= HW_ST_GYRO_THRESHOLD_DPS) {
            log_e("HW Self-test Gyro failed: X=%.1f Y=%.1f Z=%.1f dps",
                  _hw_st_gyro_result[0], _hw_st_gyro_result[1], _hw_st_gyro_result[2]);
            if (accel_en) enableAccel();
            if (gyro_en) enableGyro();
            return false;
        }
    }

    if (accel_en) enableAccel();
    if (gyro_en) enableGyro();

    return true;
}

void SensorQMI8658::getHardwareSelfTestResults(float accel_result[3], float gyro_result[3])
{
    if (accel_result) {
        memcpy(accel_result, _hw_st_accel_result, sizeof(_hw_st_accel_result));
    }
    if (gyro_result) {
        memcpy(gyro_result, _hw_st_gyro_result, sizeof(_hw_st_gyro_result));
    }
}


bool SensorQMI8658::initImpl(uint8_t param)
{
    if (_int_pin >= 0 && hal) {
        hal->pinMode(_int_pin, INPUT);
    }

    if (!reset()) {
        return false;
    }

    uint8_t id = getChipID();
    if (id != QMI8658_WHO_AM_I_VAL) {
        log_e("QMI8658 ID mismatch: expected 0x%02X, got 0x%02X", QMI8658_WHO_AM_I_VAL, id);
        return false;
    }

    writeReg(REG_CTRL8, 0x80);

    if (writeCommand(CTRL_CMD_COPY_USID, 100) != 0) {
        log_e("Failed to copy USID");
        return false;
    }

    uint8_t buffer[3] = {0};
    if (readRegBuff(REG_DQW_L, buffer, 3) == 0) {
        _firmware_version = buffer[0] | (uint32_t)(buffer[1] << 8) | (uint32_t)(buffer[2] << 16);
    }

    if (readRegBuff(REG_DVX_L, _usid, 6) != 0) {
        log_e("Failed to read USID");
        return false;
    }

    return true;
}

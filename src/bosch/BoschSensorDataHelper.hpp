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
 * @file      BoschSensorDataHelper.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-02-04
 *
 */
#pragma once
#include "BoschSensorBase.hpp"
#include "bhi260x/bhy2_defs.h"
#include "bosch/bhi36x/bhi360_event_data.h"
#include "bosch/bhi36x/bhi360_multi_tap_param_defs.h"

class BoschSensorDataHelperBase
{
public:
    typedef union {
        float temperature;
        float humidity;
        float pressure;
        float altitude;
        unified_air_quality_t iaq;
        bhy2_data_quaternion quaternion;
        bhy2_data_orientation orientation;
        bhy2_data_xyz vector;
        uint32_t step_counter;
        uint32_t gas;
        uint16_t activity_bitmap;
        uint8_t dev_ori;
        bhi360_event_data_multi_tap multi_tap;
        bool detected;
    } SensorData;

    BoschSensorDataHelperBase(BoschSensorID sensor_id, BoschSensorBase &handle)
        : _sensor_id(sensor_id), _handle(handle)
    {
        _scaling_factor = _handle.getScaling(_sensor_id);
    }

    virtual ~BoschSensorDataHelperBase() = default;

    uint8_t id() const
    {
        return _sensor_id;
    }

    bool configure(float sample_rate, uint32_t report_latency_ms)
    {
        return _handle.configure(_sensor_id, sample_rate, report_latency_ms);
    }

    bool setRange(uint16_t range)
    {
        return _handle.configureRange(_sensor_id, range);
    }

    const SensorConfig getConfiguration()
    {
        return _handle.getConfigure(_sensor_id);
    }

    float getScaling()
    {
        return _scaling_factor;
    }

protected:
    SensorData parse_data(uint8_t sensor_id, const uint8_t *data)
    {
        SensorData result;
        switch (sensor_id) {
        case BHY2_SENSOR_ID_TEMP:
        case BHY2_SENSOR_ID_TEMP_WU:
            bhy2_parse_temperature_celsius(data, &result.temperature);
            break;
        case BHY2_SENSOR_ID_HUM:
        case BHY2_SENSOR_ID_HUM_WU:
            bhy2_parse_humidity(data, &result.humidity);
            break;
        case BHY2_SENSOR_ID_BARO:
        case BHY2_SENSOR_ID_BARO_WU:
            bhy2_parse_pressure(data, &result.pressure);
            break;
        case BHY2_SENSOR_ID_GAS:
        case BHY2_SENSOR_ID_GAS_WU:
            result.gas = BHY2_LE2U32(data);
            break;
        case BHY2_SENSOR_ID_RV:
        case BHY2_SENSOR_ID_RV_WU:
        case BHY2_SENSOR_ID_GAMERV:
        case BHY2_SENSOR_ID_GAMERV_WU:
        case BHY2_SENSOR_ID_GEORV:
        case BHY2_SENSOR_ID_GEORV_WU:
            bhy2_parse_quaternion(data, &result.quaternion);
            break;
        case BHY2_SENSOR_ID_ORI:
        case BHY2_SENSOR_ID_ORI_WU:
            bhy2_parse_orientation(data, &result.orientation);
            break;
        case BHY2_SENSOR_ID_DEVICE_ORI:
        case BHY2_SENSOR_ID_DEVICE_ORI_WU:
            result.dev_ori = data[0];
            break;
        case BHY2_SENSOR_ID_ACC_PASS:
        case BHY2_SENSOR_ID_ACC_RAW:
        case BHY2_SENSOR_ID_ACC:
        case BHY2_SENSOR_ID_ACC_BIAS:
        case BHY2_SENSOR_ID_ACC_WU:
        case BHY2_SENSOR_ID_ACC_RAW_WU:
        case BHY2_SENSOR_ID_GRA:
        case BHY2_SENSOR_ID_GRA_WU:
        case BHY2_SENSOR_ID_LACC:
        case BHY2_SENSOR_ID_LACC_WU:
        case BHY2_SENSOR_ID_ACC_BIAS_WU:
        case BHY2_SENSOR_ID_GYRO_PASS:
        case BHY2_SENSOR_ID_GYRO_RAW:
        case BHY2_SENSOR_ID_GYRO:
        case BHY2_SENSOR_ID_GYRO_BIAS:
        case BHY2_SENSOR_ID_GYRO_WU:
        case BHY2_SENSOR_ID_GYRO_RAW_WU:
        case BHY2_SENSOR_ID_GYRO_BIAS_WU:
        case BHY2_SENSOR_ID_MAG_PASS:
        case BHY2_SENSOR_ID_MAG_RAW:
        case BHY2_SENSOR_ID_MAG:
        case BHY2_SENSOR_ID_MAG_BIAS:
        case BHY2_SENSOR_ID_MAG_WU:
        case BHY2_SENSOR_ID_MAG_RAW_WU:
        case BHY2_SENSOR_ID_MAG_BIAS_WU:
            bhy2_parse_xyz(data, &result.vector);
            break;
        case BHY2_SENSOR_ID_STC:
        case BHY2_SENSOR_ID_STC_WU:
        case BHY2_SENSOR_ID_STC_LP:
            result.step_counter = bhy2_parse_step_counter(data);
            break;
        case  BHY2_SENSOR_ID_STD:
        case BHY2_SENSOR_ID_STD_LP:
            result.detected = true;
            break;
        case BHY2_SENSOR_ID_AR:
            result.activity_bitmap = (data[1] << 8) | data[0];
            break;
        case BHY2_SENSOR_ID_AIR_QUALITY:
            if (_handle.getChipID() == BHI260_CHIP_ID) {
                bhy2_bsec_air_quality raw;
                bhy2_bsec_parse_air_quality(data, &raw);
                result.iaq.co2 = raw.e_co2;
                result.iaq.humidity = raw.comp_hum;
                result.iaq.iaq_accuracy = raw.iaq_accuracy;
                result.iaq.temperature = raw.comp_temp;
                result.iaq.voc = raw.voc;
                result.iaq.static_iaq = raw.static_iaq;
                result.iaq.gas_resistance = raw.comp_gas;
            } else {
                /*SCALE FACTOR from BHI3 data sheet, IAQ data format*/
                const float  SCALE_IAQ_VOC  = 100.0;
                const float  SCALE_IAQ_TEMP = 256.0;
                const float  SCALE_IAQ_HUMI = 500.0;
                bhi360_event_data_iaq_output_t raw;
                bhi360_event_data_parse_air_quality(data, &raw);
                result.iaq.co2 = raw.co2;
                result.iaq.humidity = raw.comp_humidity / SCALE_IAQ_HUMI;
                result.iaq.iaq_accuracy = raw.iaq_accuracy;
                result.iaq.temperature = raw.comp_temperature / SCALE_IAQ_TEMP;
                result.iaq.voc = raw.voc / SCALE_IAQ_VOC;
                result.iaq.static_iaq = raw.siaq;
                result.iaq.gas_resistance = raw.raw_gas;
            }
            break;
        case BHI360_SENSOR_ID_MULTI_TAP:
            result.multi_tap = static_cast<bhi360_event_data_multi_tap>(data[0]);
            break;
        default:
            log_e("Sensor ID Undefined");
            break;
        }
        return result;
    }
    BoschSensorID _sensor_id;
    float _scaling_factor;
    BoschSensorBase &_handle;
};

template <typename DataType>
class SensorTemplateBase : public BoschSensorDataHelperBase
{
public:
    SensorTemplateBase(BoschSensorID sensor_id, BoschSensorBase &handle)
        : BoschSensorDataHelperBase(sensor_id, handle)
    {
    }

    ~SensorTemplateBase() override
    {
    }

    const DataType &getValue() const
    {
        return _value;
    }

    bool enable(float sample_rate, uint32_t report_latency_ms)
    {
        _handle.onResultEvent(_sensor_id, staticCallback, this);
        return configure(sample_rate, report_latency_ms);
    }

    void disable()
    {
        _handle.removeResultEvent(_sensor_id, staticCallback);
        _handle.configure(_sensor_id, 0, 0);
    }

    bool hasUpdated()
    {
        bool result =  _lastUpdateTime != _currentTime;
        _lastUpdateTime = _currentTime;
        return result;
    }

    uint64_t getTimestamp()
    {
        return _currentTime;
    }

    uint32_t getLastUpdateSecond()
    {
        uint64_t tns = getNanosecondsFromCurrentTime();
        return static_cast<uint32_t>(tns / UINT64_C(1000000000));
    }

    uint64_t getLastUpdateNanoseconds()
    {
        uint64_t tns = getNanosecondsFromCurrentTime();
        uint32_t s = static_cast<uint32_t>(tns / UINT64_C(1000000000));
        return tns - (static_cast<uint64_t>(s) * UINT64_C(1000000000));
    }

    void getLastTime(uint32_t &s, uint32_t &ns)
    {
        uint64_t  tns = getNanosecondsFromCurrentTime(); /* timestamp is now in nanoseconds */
        s = (uint32_t)(tns / UINT64_C(1000000000));
        ns = (uint32_t)(tns - ((s) * UINT64_C(1000000000)));
    }

protected:
    virtual void updateValue(const SensorData &data) = 0;
    DataType _value;
    uint64_t _lastUpdateTime;
    uint64_t _currentTime;
private:
    uint64_t getNanosecondsFromCurrentTime() const
    {
        return _currentTime * 15625;
    }
    static void staticCallback(uint8_t sensor_id, const uint8_t *data, uint32_t size, uint64_t *timestamp, void *user_data)
    {
        auto self = static_cast<SensorTemplateBase<DataType>*>(user_data);
        SensorData parsedData = self->parse_data(sensor_id, data);
        self->updateValue(parsedData);
        self->_lastUpdateTime = self->_currentTime;
        self->_currentTime = *timestamp;
    }
};

class SensorTemperature : public SensorTemplateBase<float>
{
public:
    SensorTemperature(BoschSensorBase &handle)
        : SensorTemplateBase<float>(BoschSensorID::TEMPERATURE, handle) {}

    float getCelsius() const
    {
        return getValue();
    }

    float getFahrenheit()
    {
        return getCelsius() * (9.0 / 5.0) + 32;
    }

    float getKelvin()
    {
        return getCelsius() + 273.15;
    }

    bool enable()
    {
        return SensorTemplateBase::enable(1, 0);
    }

protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.temperature;
    }
};

class SensorHumidity : public SensorTemplateBase<float>
{
public:
    SensorHumidity(BoschSensorBase &handle)
        : SensorTemplateBase<float>(BoschSensorID::HUMIDITY, handle) {}

    float getHumidity() const
    {
        return getValue();
    }

    bool enable()
    {
        return SensorTemplateBase::enable(1, 0);
    }

protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.humidity;
    }
};

class SensorPressure : public SensorTemplateBase<float>
{
public:
    SensorPressure(BoschSensorBase &handle)
        : SensorTemplateBase<float>(BoschSensorID::BAROMETER, handle) {}

    // Pressure in Pa (pascals)
    float getPressure() const
    {
        return getValue();
    }

    // Pressure in hPa (hectopascals)
    float getPressureHPa() const
    {
        return getPressure() / 100.0f;
    }

    // Pressure in kPa (kilopascals)
    float getPressureKPa() const
    {
        return getPressure() / 1000.0f;
    }

protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.pressure;
    }
};


class SensorGas : public SensorTemplateBase<uint32_t>
{
public:
    SensorGas(BoschSensorBase &handle)
        : SensorTemplateBase<uint32_t>(BoschSensorID::GAS, handle) {}

    uint32_t getGas() const
    {
        return getValue() * _scaling_factor;
    }
protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.gas;
    }
};


class SensorOrientation : public SensorTemplateBase<uint8_t>
{
public:
    SensorOrientation(BoschSensorBase &handle)
        : SensorTemplateBase<uint8_t>(BoschSensorID::DEVICE_ORIENTATION, handle) {}

    uint32_t getOrientation() const
    {
        return getValue();
    }
protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.dev_ori;
    }
};

class SensorEuler : public SensorTemplateBase<bhy2_data_orientation>
{
public:
    SensorEuler(BoschSensorBase &handle)
        : SensorTemplateBase<bhy2_data_orientation>(BoschSensorID::ORIENTATION, handle) {}

    float getHeading() const
    {
        return getValue().heading * _scaling_factor;
    }

    float getPitch() const
    {
        return getValue().pitch * _scaling_factor;
    }

    float getRoll() const
    {
        return getValue().roll * _scaling_factor;
    }

protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.orientation;
    }
};

class SensorQuaternion : public SensorTemplateBase<bhy2_data_quaternion>
{
public:
    SensorQuaternion(BoschSensorBase &handle)
        : SensorTemplateBase<bhy2_data_quaternion>(BoschSensorID::GAME_ROTATION_VECTOR, handle) {}

    float getX() const
    {
        return getValue().x * _scaling_factor;
    }
    float getY() const
    {
        return getValue().y * _scaling_factor;
    }
    float getZ() const
    {
        return getValue().z * _scaling_factor;
    }
    float getW() const
    {
        return getValue().w * _scaling_factor;
    }
    uint16_t getAccuracy() const
    {
        return getValue().accuracy;
    }
    float getHeading() const
    {
        return heading;
    }
    float getPitch() const
    {
        return pitch;
    }
    float getRoll() const
    {
        return roll;
    }
    void toEuler()
    {
        float w,  x,  y,  z;
        w = _value.w / 16384.0f;
        x = _value.x / 16384.0f;
        y = _value.y / 16384.0f;
        z = _value.z / 16384.0f;
        roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
        pitch = asin(2 * (w * y - z * x));
        heading = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
        // Convert radians to degrees
        roll = roll * (180.0 / M_PI);
        pitch = pitch * (180.0 / M_PI);
        heading = heading * (180.0 / M_PI);
    }

protected:
    float heading;
    float pitch;
    float roll;
    void updateValue(const SensorData &data) override
    {
        _value = data.quaternion;
    }
};

class SensorStepCounter : public SensorTemplateBase<uint32_t>
{
public:
    SensorStepCounter(BoschSensorBase &handle)
        : SensorTemplateBase<uint32_t>(BoschSensorID::STEP_COUNTER, handle)
    {
        if (_handle.getChipID() == BHI360_CHIP_ID) {
            _sensor_id = BoschSensorID::STEP_COUNTER_LOW_POWER;
        } else if (_handle.getChipID() == BHI260_CHIP_ID) {
            _sensor_id = BoschSensorID::STEP_COUNTER;
        } else {
            log_e("Unknown chip ID 0x%02X. Step Counter sensor may not function correctly.", _handle.getChipID());
        }
    }

    uint32_t getStepCount() const
    {
        return getValue();
    }

protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.step_counter;
    }
};

class SensorStepDetector : public SensorTemplateBase<bool>
{
public:
    SensorStepDetector(BoschSensorBase &handle)
        : SensorTemplateBase<bool>(BoschSensorID::STEP_DETECTOR, handle)
    {
        if (_handle.getChipID() == BHI360_CHIP_ID) {
            _sensor_id = BoschSensorID::STEP_DETECTOR_LOW_POWER;
        } else if (_handle.getChipID() == BHI260_CHIP_ID) {
            _sensor_id = BoschSensorID::STEP_DETECTOR;
        } else {
            log_e("Unknown chip ID 0x%02X. Step Detector sensor may not function correctly.", _handle.getChipID());
        }
    }

    bool isDetected()
    {
        bool tmp = getValue();
        _value = false;
        return tmp;
    }

protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.detected;
    }
};

class SensorXYZ : public SensorTemplateBase<bhy2_data_xyz>
{
public:
    SensorXYZ(BoschSensorID sensor_id, BoschSensorBase &handle)
        : SensorTemplateBase<bhy2_data_xyz>(sensor_id, handle) {}

    float getX() const
    {
        return getValue().x * _scaling_factor;
    }
    float getY() const
    {
        return getValue().y * _scaling_factor;
    }
    float getZ() const
    {
        return getValue().z * _scaling_factor;
    }
protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.vector;
    }
};


class SensorAcceleration : public SensorXYZ
{
public:
    SensorAcceleration(BoschSensorBase &handle)
        : SensorXYZ(BoschSensorID::ACCEL_PASSTHROUGH, handle) {}

    bool readData(AccelerometerData &data)
    {
        if (hasUpdated()) {
            data.temperature = NAN;
            data.mps2.x = getX();
            data.mps2.y = getY();
            data.mps2.z = getZ();
            data.raw.x = getValue().x;
            data.raw.y = getValue().y;
            data.raw.z = getValue().z;
            return true;
        }
        return false;
    }
};

class SensorGyroscope : public SensorXYZ
{
public:
    SensorGyroscope(BoschSensorBase &handle)
        : SensorXYZ(BoschSensorID::GYRO_PASSTHROUGH, handle) {}

    bool readData(GyroscopeData &data)
    {
        if (hasUpdated()) {
            data.temperature = NAN;
            data.dps.x = getX();
            data.dps.y = getY();
            data.dps.z = getZ();
            data.raw.x = getValue().x;
            data.raw.y = getValue().y;
            data.raw.z = getValue().z;
            return true;
        }
        return false;
    }
};

class SensorMagnetometer : public SensorXYZ
{
public:
    SensorMagnetometer(BoschSensorBase &handle)
        : SensorXYZ(BoschSensorID::MAGNETOMETER_PASSTHROUGH, handle) {}

    bool readData(MagnetometerData &data)
    {
        if (hasUpdated()) {
            data.temperature = NAN;
            data.magnetic_field.x = getX();
            data.magnetic_field.y = getY();
            data.magnetic_field.z = getZ();
            data.raw.x = getValue().x;
            data.raw.y = getValue().y;
            data.raw.z = getValue().z;
            return true;
        }
        return false;
    }
};

class SensorActivity : public SensorTemplateBase<uint16_t>
{
public:
    // See [bst-bhi260ab-ds000.pdf](https://www.mouser.com/datasheet/2/783/bst-bhi260ab-ds000-1816249.pdf)
    // 15.1.4 Format "Activity"
    enum ActivityStatus {
        STILL_ACTIVITY_ENDED,
        WALKING_ACTIVITY_ENDED,
        RUNNING_ACTIVITY_ENDED,
        ON_BICYCLE_ACTIVITY_ENDED,
        IN_VEHICLE_ACTIVITY_ENDED,
        TILTING_ACTIVITY_ENDED,
        IN_VEHICLE_STILL_ENDED,
        RESERVED_LOW,
        STILL_ACTIVITY_STARTED,
        WALKING_ACTIVITY_STARTED,
        RUNNING_ACTIVITY_STARTED,
        ON_BICYCLE_ACTIVITY_STARTED,
        IN_VEHICLE_ACTIVITY_STARTED,
        TILTING_ACTIVITY_STARTED,
        IN_VEHICLE_STILL_STARTED,
        RESERVED_HIGH
    };
    SensorActivity(BoschSensorBase &handle)
        : SensorTemplateBase<uint16_t>(BoschSensorID::ACTIVITY_RECOGNITION, handle) {}

    /**
     * @brief Check if a specific activity status is set in the activity bitmap.
     *
     * This function checks if the bit corresponding to the given activity status is set.
     *
     * @param status The activity status to check. It is an enumeration value of ActivityStatus.
     * @return bool Returns true if the bit corresponding to the given activity status is set in the bitmap,
     *              false otherwise.
     */
    bool isActivitySet(ActivityStatus status)
    {
        bool bitStatus = (_value & (1 << static_cast<uint16_t>(status))) != 0;
        _value &= ~(1 << static_cast<uint16_t>(status));
        return bitStatus;
    }

    /**
     * @brief  Manually clear all motion status
     * @retval None
     */
    void clear()
    {
        _value = 0;
    }

protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.activity_bitmap;
    }
};

class SensorIAQ : public SensorTemplateBase<unified_air_quality_t>
{
public:
    SensorIAQ(BoschSensorBase &handle)
        : SensorTemplateBase<unified_air_quality_t>(BoschSensorID::IAQ, handle) {}

    float getCelsius() const
    {
        return getValue().temperature;
    }

    float getFahrenheit() const
    {
        return getCelsius() * 9.0f / 5.0f + 32;
    }
    float getKelvin()     const
    {
        return getCelsius() + 273.15f;
    }

    float getVOC() const
    {
        return getValue().voc;
    }

    float getHumidity() const
    {
        return getValue().humidity;
    }

    uint16_t getIAQ() const
    {
        return getValue().iaq;
    }

    uint16_t getSIAQ() const
    {
        return getValue().static_iaq;
    }

    uint32_t getCO2() const
    {
        return getValue().co2;
    }

    uint8_t getIAQAccuracy() const
    {
        return getValue().iaq_accuracy;
    }

    uint32_t getRawGas() const
    {
        return getValue().gas_resistance;
    }

    uint8_t getIAQLevel() const
    {
        uint16_t iaq = getIAQ();
        if (iaq <= 50) {
            return 0; // Excellent
        } else if (iaq <= 100) {
            return 1; // Good
        } else if (iaq <= 150) {
            return 2; // Lightly Polluted
        } else if (iaq <= 200) {
            return 3; // Moderately Polluted
        } else if (iaq <= 300) {
            return 4; // Heavily Polluted
        } else {
            return 5; // Severely Polluted
        }
    }

    uint8_t getIAQLevelText(char *buffer, size_t buffer_size) const
    {
        uint16_t iaq = getIAQ();
        const char *level_text;
        if (iaq <= 50) {
            level_text = "Excellent";
        } else if (iaq <= 100) {
            level_text = "Good";
        } else if (iaq <= 150) {
            level_text = "Lightly Polluted";
        } else if (iaq <= 200) {
            level_text = "Moderately Polluted";
        } else if (iaq <= 300) {
            level_text = "Heavily Polluted";
        } else {
            level_text = "Severely Polluted";
        }
        strncpy(buffer, level_text, buffer_size - 1);
        buffer[buffer_size - 1] = '\0'; // Ensure null-termination
        return getIAQLevel();
    }

    uint8_t getIAQAccuracyText(char *buffer, size_t buffer_size) const
    {
        uint8_t accuracy = getIAQAccuracy();
        const char *accuracy_text;
        switch (accuracy) {
        case 0:
            accuracy_text = "Unreliable";
            break;
        case 1:
            accuracy_text = "Low Accuracy";
            break;
        case 2:
            accuracy_text = "Medium Accuracy";
            break;
        case 3:
            accuracy_text = "High Accuracy";
            break;
        default:
            accuracy_text = "Unknown Accuracy";
            break;
        }
        strncpy(buffer, accuracy_text, buffer_size - 1);
        buffer[buffer_size - 1] = '\0'; // Ensure null-termination
        return accuracy;
    }

    bool enable()
    {
        return SensorTemplateBase::enable(1, 0);
    }

protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.iaq;
    }
};


class SensorMultiTap : public SensorTemplateBase<bhi360_event_data_multi_tap>
{
public:
    SensorMultiTap(BoschSensorBase &handle)
        : SensorTemplateBase<bhi360_event_data_multi_tap>(BoschSensorID::MULTI_TAP, handle)
    {
        if (_handle.getChipID() == BHI260_CHIP_ID) {
            log_e("BHI260 does not support Multi Tap sensor.");
        }
    }
protected:
    void updateValue(const SensorData &data) override
    {
        _value = data.multi_tap;
    }
};
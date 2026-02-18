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
 * @file      SensorDefs.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-02-04
 *
 */

#pragma once

#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/**
* @enum SensorRemap
* @brief Enumeration representing different remapping options for the sensor's orientation.
*
* This enum defines various positions and orientations of the sensor chip. Each value corresponds
* to a specific corner or location of the chip, which can be used to remap the axes of the sensor
* according to its physical placement.
*
* Top view of the chip, where 'T' stands for top,
* 'B' stands for bottom,
* 'L' stands for left, and 'R' stands for right
*  -------------
* | TL         TR |
* |               |
* |               |
* |               |
* | BL         BR |
*  -------------
*
* There is also a bottom view of the chip：
*
*  -------------
* | BT         BB |
* |               |
* |               |
* |               |
* | LT         RT |
*  -------------
*/
enum class SensorRemap {
    // Chip top view, upper left corner
    //  -------------
    // | *             |
    // |               |
    // |               |
    // |               |
    // |               |
    //  -------------
    TOP_LAYER_LEFT_CORNER,
    // Chip top view, upper right corner
    //  -------------
    // |             * |
    // |               |
    // |               |
    // |               |
    // |               |
    //  -------------
    TOP_LAYER_RIGHT_CORNER,
    // Chip top view, bottom right corner of the top
    //  -------------
    // |               |
    // |               |
    // |               |
    // |               |
    // |             * |
    //  -------------
    TOP_LAYER_BOTTOM_RIGHT_CORNER,
    // The top view of the chip, the lower left corner of the front bottom
    //  -------------
    // |               |
    // |               |
    // |               |
    // |               |
    // | *             |
    //  -------------
    TOP_LAYER_BOTTOM_LEFT_CORNER,
    // The bottom view of the chip, the upper left corner of the top
    //  -------------
    // | *             |
    // |               |
    // |               |
    // |               |
    // |               |
    //  -------------
    BOTTOM_LAYER_TOP_LEFT_CORNER,
    // The bottom view of the chip, the upper right corner of the top
    //  -------------
    // |             * |
    // |               |
    // |               |
    // |               |
    // |               |
    //  -------------
    BOTTOM_LAYER_TOP_RIGHT_CORNER,
    // The bottom view of the chip, the lower right corner of the bottom
    //  -------------
    // |               |
    // |               |
    // |               |
    // |               |
    // |             * |
    //  -------------
    BOTTOM_LAYER_BOTTOM_RIGHT_CORNER,
    // Chip bottom view, bottom left corner
    //  -------------
    // |               |
    // |               |
    // |               |
    // |               |
    // | *             |
    //  -------------
    BOTTOM_LAYER_BOTTOM_LEFT_CORNER,
};


enum SensorDirection {
    //  ________________
    // |                |
    // |             *  |
    // |                |
    // |                |
    // |                |
    // |                |
    // |________________|
    DIRECTION_TOP_RIGHT,
    
    //  ________________
    // |                |
    // |  *             |
    // |                |
    // |                |
    // |                |
    // |                |
    // |________________|
    DIRECTION_TOP_LEFT,

    //  ________________
    // |                |
    // |                |
    // |                |
    // |                |
    // |                |
    // |  *             |
    // |________________|
    DIRECTION_BOTTOM_LEFT,

    //  ________________
    // |                |
    // |                |
    // |                |
    // |                |
    // |                |
    // |             *  |
    // |________________|
    DIRECTION_BOTTOM_RIGHT,

    //  ________________
    // |________________|
    //    *
    DIRECTION_BOTTOM,

    //  __*_____________
    // |________________|
    DIRECTION_TOP,
};

/**
 * @brief Enumeration of sensor types
 */
enum class SensorType {
    UNKNOWN = 0,     ///< Unknown sensor type
    ACCELEROMETER,   ///< Accelerometer sensor
    GYROSCOPE,       ///< Gyroscope sensor
    MAGNETOMETER,    ///< Magnetometer sensor
    PRESSURE,        ///< Pressure sensor
    TEMPERATURE,     ///< Temperature sensor
    HUMIDITY,        ///< Humidity sensor
    MULTI_AXIS       ///< Multi-axis sensor
};

/**
 * @brief Structure representing a 3D vector with floating-point components
 */
struct SensorVector {
    float x;  ///< X-axis component
    float y;  ///< Y-axis component
    float z;  ///< Z-axis component
};

/**
 * @brief Structure representing a raw 3D vector with 16-bit integer components
 */
struct RawVector {
    int16_t x;  ///< X-axis raw value
    int16_t y;  ///< Y-axis raw value
    int16_t z;  ///< Z-axis raw value
};

/**
 * @brief Enumeration of sensor operation modes
 */
enum class OperationMode {
    SUSPEND = 0,
    NORMAL,
    SINGLE_MEASUREMENT,
    CONTINUOUS_MEASUREMENT,
};

/**
 * @brief Enumeration of accelerometer full-scale range settings
 */
enum class AccelFullScaleRange {
    FS_2G = 0,   ///< ±2g range
    FS_4G,       ///< ±4g range
    FS_8G,       ///< ±8g range
    FS_16G,      ///< ±16g range
    FS_32G,      ///< ±32g range
    FS_64G,      ///< ±64g range
    FS_128G      ///< ±128g range
};

/**
 * @brief Enumeration of interrupt pin mapping options
 */
enum class InterruptPinMap {
    PIN1 = 1,   ///< Interrupt pin 1
    PIN2,       ///< Interrupt pin 2
};


/**
* @brief Enumeration defining full-scale range settings for the sensor.
*/
enum class MagFullScaleRange {
    FS_2G = 0,      // ±2 Gauss
    FS_4G,          // ±4 Gauss
    FS_8G,          // ±8 Gauss
    FS_12G,         // ±12 Gauss
    FS_16G,         // ±16 Gauss
    FS_20G,         // ±20 Gauss
    FS_25G,         // ±25 Gauss
    FS_30G,         // ±30 Gauss
    FS_32G          // ±32 Gauss
};

/**
* @brief Enumeration defining over-sample ratios for the sensor.
*/
enum class MagOverSampleRatio {
    OSR_8,   ///< 8x over-sample ratio
    OSR_4,   ///< 4x over-sample ratio
    OSR_2,   ///< 2x over-sample ratio
    OSR_1    ///< 1x over-sample ratio
};

/**
 * @brief Enumeration defining down-sample ratios for the sensor.
 */
enum class MagDownSampleRatio {
    DSR_1,   ///< 1x down-sample ratio
    DSR_2,   ///< 2x down-sample ratio
    DSR_4,   ///< 4x down-sample ratio
    DSR_8,   ///< 8x down-sample ratio
};

enum class MagLowPassFilter {
    LPF_1,  ///< 1 Hz low-pass filter
    LPF_2,  ///< 2 Hz low-pass filter
    LPF_4,  ///< 4 Hz low-pass filter
    LPF_8,  ///< 8 Hz low-pass filter
    LPF_16  ///< 16 Hz low-pass filter
};

/**
 * @brief Enumeration of gyroscope full-scale range settings
 */
enum class GyroFullScaleRange {
    FS_125_DPS = 0,  ///< ±125 °/s
    FS_250_DPS,   ///< ±250 °/s
    FS_500_DPS,        ///< ±500 °/s
    FS_1000_DPS,       ///< ±1000 °/s
    FS_2000_DPS,       ///< ±2000 °/s
    FS_4000_DPS        ///< ±4000 °/s (if supported)
};

/**
 * @brief Structure representing accelerometer data
 *
 * Contains acceleration values in both raw format (LSB) and processed format (m/s²),
 * as well as temperature if the sensor supports it.
 */
struct AccelerometerData {
    SensorVector mps2;       ///< Acceleration in meters per second squared (m/s²)
    RawVector raw;           ///< Raw sensor values in LSB (least significant bits)
    float temperature;       ///< Temperature in degrees Celsius (if available)

    AccelerometerData() : mps2{0, 0, 0}, raw{0, 0, 0}, temperature{0} {}
};

/**
 * @brief Structure representing magnetometer data
 *
 * Contains magnetic field values in both raw format (LSB) and processed format (Gauss),
 * as well as temperature if the sensor supports it.
 */
struct MagnetometerData {
    SensorVector magnetic_field;  ///< magnetic field strength (Gauss)
    RawVector raw;                ///< raw data
    float temperature;            ///< temperature (°C)
    float heading;                ///< heading angle (radians)
    float heading_degrees;        ///< heading angle (degrees)
    bool overflow;                ///< data overflow flag
    bool skip_data;               ///< skip data flag

    MagnetometerData() : magnetic_field{0, 0, 0}, raw{0, 0, 0}, temperature{0}, heading{0}, heading_degrees{0}, overflow{false}, skip_data{false} {}
};

/**
 * @brief Structure representing gyroscope data
 */
struct GyroscopeData {
    SensorVector dps;           ///< Angular velocity in degrees per second (°/s)
    RawVector    raw;           ///< Raw sensor values in LSB
    float        temperature;   ///< Temperature in °C (if available)

    GyroscopeData() : dps{0, 0, 0}, raw{0, 0, 0}, temperature{0} {}
};

/**
 * @brief Structure representing sensor configuration parameters
 */
struct SensorConfig {

    SensorType type;            ///< Type of the sensor
    float range;                ///< Measurement range
    float sample_rate;          ///< Sample rate in Hz
    uint32_t latency;           ///< Reporting latency in milliseconds
    OperationMode mode;         ///< Operation mode
    SensorConfig() : type(SensorType::UNKNOWN), range(0), sample_rate(0), latency(0), mode(OperationMode::SUSPEND) {}

    SensorConfig(SensorType type, float range, float sample_rate, uint32_t latency, OperationMode mode)
        : type(type), range(range), sample_rate(sample_rate), latency(latency), mode(mode) {}
};

/**
 * @brief Structure containing sensor identification information
 */
struct SensorInfo {
    const char *manufacturer;       ///< Manufacturer name
    const char *model;              ///< Model name/identifier
    uint8_t i2c_address;            ///< Default I2C address
    uint8_t version;                ///< Hardware/firmware version
    uint32_t uid;                   ///< Unique identifier
    SensorType type;                ///< Sensor type

    SensorInfo() : manufacturer("Unknown"), model("Unknown"), i2c_address(0), version(0), uid(0), type(SensorType::UNKNOWN) {}
};

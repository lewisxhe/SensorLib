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

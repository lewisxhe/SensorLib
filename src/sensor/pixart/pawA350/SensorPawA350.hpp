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
 * @file      SensorPawA350.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-11
 *
 * @brief     PAW-A350 Optical Finger Navigation (OFN) Sensor Driver
 *
 * @section   Overview
 *
 * The PAW-A350 is an optical finger navigation sensor from PixArt Imaging.
 * It provides motion detection and finger tracking capabilities through
 * an I2C interface. The sensor measures surface motion by comparing
 * successive images of the fingerprint and calculating the displacement.
 *
 * @section   Key Features
 *
 * - I2C interface (slave address 0x33)
 * - Count per inch (CPI) resolution: 125, 250, 500, 750, 1000, 1250
 * - 8-bit two's complement motion delta output (-128 to +127)
 * - Multiple power management states: Run, Rest1, Rest2, Rest3
 * - Configurable LED drive current and shutter control
 * - Motion interrupt support
 *
 * @section   Register Map Overview
 *
 * | Address | Register Name    | Description                                           |
 * |---------|-----------------|------------------------------------------------------|
 * | 0x00    | PRODUCT_ID      | Unique chip identification (default: 0x88)          |
 * | 0x01    | REVISION_ID     | IC revision number                                   |
 * | 0x02    | EVENT          | Event status register (motion, reset, FPD)         |
 * | 0x03    | Delta_X        | X movement delta (8-bit signed)                      |
 * | 0x04    | Delta_Y        | Y movement delta (8-bit signed)                      |
 * | 0x11    | Rest1_Period   | Rest1 frame period                                 |
 * | 0x13    | Run_Downshift  | Run to Rest1 downshift time                        |
 * | 0x15    | Rest1_Downshift| Rest1 to Rest2 downshift time                     |
 * | 0x16    | Rest2_Period   | Rest2 frame period                                 |
 * | 0x17    | Rest2_Downshift| Rest2 to Rest3 downshift time                     |
 * | 0x18    | Rest3_Period   | Rest3 frame period                                 |
 * | 0x1A    | LED_CTRL       | LED control and drive current                      |
 * | 0x3A    | SOFT_RESET    | Software reset (write 0x5A)                         |
 * | 0x3B    | Shutter_Max_Hi| Shutter max open time (upper byte)                |
 * | 0x3C    | Shutter_Max_Lo| Shutter max open time (lower byte)                |
 * | 0x62    | CPI_SEL       | Count per inch resolution setting               |
 * | 0x74    | MOTION_CTRL   | Motion interrupt threshold control               |
 * | 0x7A    | FPD_FLAG      | Finger presence detect flag                      |
 *
 * @section   Power States
 *
 * The sensor supports 4 power states to reduce power consumption:
 *
 * 1. Run State    - Full operation, highest power consumption
 * 2. Rest1 State - Reduced frame rate after Run_Downshift time
 * 3. Rest2 State - Further reduced frame rate after Rest1_Downshift time
 * 4. Rest3 State - Lowest frame rate, minimal power
 *
 * Downshift timing formulas:
 * - Run_Downshift:      RD[7:0] x 8 x 8ms    (default: 256ms)
 * - Rest1_Period:      (R1R[7:0] + 1) x 10ms  (default: 20ms)
 * - Rest1_Downshift:   R1D[7:0] x 16 x Rest1_Period  (default: 9.92s)
 * - Rest2_Period:      (R2R[7:0] + 1) x 10ms  (default: 100ms)
 * - Rest2_Downshift:   R2D[7:0] x 128 x Rest2_Period (default: 10min)
 * - Rest3_Period:     (R3R[7:0] + 1) x 10ms  (default: 500ms)
 */

#pragma once

#include "platform/comm/I2CDeviceWithHal.hpp"

/**
 * @brief PAW-A350 I2C slave address
 */
static constexpr uint8_t PAW_A350_SLAVE_ADDRESS = 0x33;

/**
 * @brief PAW-A350 Optical Finger Navigation Sensor Driver
 *
 * Provides interface for Configuring and reading motion data from the
 * PixArt PAW-A350 OFN sensor.
 *
 * @section   CpiResolution Count Per Inch (CPI) Resolution Options
 *
 * | Value   | CPI  | Description                           |
 * |---------|------|---------------------------------------|
 * | 0       | 125  | Lowest resolution, highest precision   |
 * | 1       | 250  | 2x base resolution                    |
 * | 2       | 500  | Default resolution                    |
 * | 3       | 750  | 1.5x default                         |
 * | 4       | 1000 | High resolution                       |
 * | 5       | 1250 | Highest resolution                    |
 *
 * @section   MotionStatus Motion Detection Status
 *
 * | Value           | Description                                      |
 * |-----------------|-------------------------------------------------|
 * | NO_MOTION       | No motion detected since last read                |
 * | MOTION_DETECTED | Motion detected, data ready in Delta_X/Y registers |
 *
 * @section   MotionData Motion Delta Structure
 *
 * Contains signed movement deltas in counts. The actual distance in
 * inches is calculated as: delta / CPI
 *
 * | Member   | Type  | Description                    |
 * |----------|-------|--------------------------------|
 * | delta_x  | int8_t| X axis movement (-128 to 127) |
 * | delta_y  | int8_t| Y axis movement (-128 to 127) |
 *
 */
class SensorPawA350 : public I2CDeviceWithHal
{
public:

    /**
     * @brief Count Per Inch (CPI) Resolution
     *
     * Defines the sensor resolution in counts per inch.
     * Higher CPI means more sensitivity but smaller range.
     *
     * @note The actual movement in inches = delta / CPI
     */
    enum class CpiResolution : uint8_t {
        CPI_125  = 0,    /**< 125 CPI  - Lowest resolution, largest range */
        CPI_250  = 1,    /**< 250 CPI  */
        CPI_500  = 2,    /**< 500 CPI  - Default resolution */
        CPI_750  = 3,    /**< 750 CPI  */
        CPI_1000 = 4,    /**< 1000 CPI - Recommended for most applications */
        CPI_1250 = 5     /**< 1250 CPI - Highest resolution */
    };

    /**
     * @brief Motion Detection Status
     *
     * Indicates whether motion has been detected since
     * the last time the EVENT register was read.
     *
     * @note Reading Delta_X and Delta_Y registers clears
     *       the MOTION bit in the EVENT register.
     */
    enum class MotionStatus : uint8_t {
        NO_MOTION = 0,       /**< No motion detected */
        MOTION_DETECTED = 1   /**< Motion occurred, data ready for reading */
    };

    /**
     * @brief Motion Delta Data
     *
     * Stores the movement delta in X and Y directions.
     * Values are 8-bit signed integers in two's complement,
     * representing the number of counts moved since the
     * last reading.
     *
     * @note The deltas are cleared when read. PixArt recommends
     *       reading Delta_X (0x03) and Delta_Y (0x04) registers
     *       sequentially in a single I2C transaction.
     */
    struct MotionData {
        int8_t delta_x;  /**< X-axis movement (-128 to +127 counts) */
        int8_t delta_y;  /**< Y-axis movement (-128 to +127 counts) */
    };

    SensorPawA350() = default;

    /**
     * @brief Get chip name
     * @return Chip name string "PAW-A350"
     */
    const char *getChipName() const;

    /**
     * @brief Get Product ID
     *
     * Reads the PRODUCT_ID register (0x00) which contains
     * a unique identification number assigned to the PAW-A350.
     * The value is fixed at 0x88 and can be used to verify
     * I2C communication is working correctly.
     *
     * @return Product ID (default: 0x88)
     *
     * @note Use this function in initImpl() to verify sensor
     *       is responding correctly on the I2C bus.
     */
    int getProductID() const;

    /**
     * @brief Get Revision ID
     *
     * Reads the REVISION_ID register (0x01) which contains
     * the IC revision number. This value may change when
     * new silicon versions are released.
     *
     * @return Revision ID
     */
    int getRevisionID() const;

    /**
     * @brief Perform software reset
     *
     * Writes to the SOFT_RESET register (0x3A) with value 0x5A
     * to trigger a software reset. After reset, all registers
     * return to their default values.
     *
     * @return true if reset successful and Product ID verified
     *
     * @note This function verifies the reset was successful
     *       by checking the Product ID matches the expected
     *       value (0x88).
     */
    bool softReset();

    /**
     * @brief Check motion status
     *
     * Reads the EVENT register (0x02) to check if motion has
     * been detected. The EVENT register contains multiple
     * status bits:
     *   - Bit 7 (MOTION): Motion detected flag
     *   - Bit 3 (RESET_ST): Reset occurred flag
     *   - Bit 0 (FPD_ST): Finger state change flag
     *
     * @param[out] status Reference to store motion status
     * @return true if register read successful
     *
     * @note The MOTION bit is cleared when Delta_X and
     *       Delta_Y registers are read.
     * @note WRITE 0x00 to EVENT register to clear status bits.
     *
     */
    bool checkMotion(MotionStatus &status);

    /**
     * @brief Get motion delta data
     *
     * Reads Delta_X (0x03) and Delta_Y (0x04) registers
     * sequentially to get motion deltas. Values are
     * 8-bit signed integers in two's complement format.
     *
     * @param[out] data Reference to store motion data
     * @return true if read successful
     *
     * @note Registers are cleared after reading
     * @note Recommend reading both registers in one I2C transaction
     */
    bool getMotionData(MotionData &data);

    /**
     * @brief Set CPI resolution
     *
     * Configures the CPI_SEL register (0x62) to set the
     * count per inch resolution. Higher CPI provides
     * more sensitivity but smaller usable range.
     *
     * @param cpi Resolution setting
     * @return true if write successful
     *
     * @note Resolution affects delta output interpretation:
     *       - 125 CPI:  125 counts = 1 inch
     *       - 250 CPI:  250 counts = 1 inch
     *       - 500 CPI: 500 counts = 1 inch
     *       - 750 CPI: 750 counts = 1 inch
     *       - 1000 CPI: 1000 counts = 1 inch
     *       - 1250 CPI: 1250 counts = 1 inch
     *
     */
    bool setCpi(CpiResolution cpi);

    /**
     * @brief Get current CPI resolution
     * @return Current CPI resolution setting
     */
    CpiResolution getCpi() const;

    /**
     * @brief Set Run to Rest1 downshift time
     *
     * Writes to Run_Downshift register (0x13) to set the
     * time before transitioning from Run state to
     * Rest1 state when no motion is detected.
     *
     * @param rd Downshift time value (2-242, default: 4)
     * @return true if write successful
     *
     * @note Time formula: RD[7:0] x 8 x 8ms
     *       - Min: 2 x 8 x 8ms = 128ms
     *       - Default: 4 x 8 x 8ms = 256ms
     *       - Max: 242 x 8 x 8ms = 15,488ms
     *
     */
    bool setRunDownshiftTime(uint8_t rd);

    /**
     * @brief Get Run to Rest1 downshift time
     * @return Raw downshift register value
     */
    int getRunDownshiftTime() const;

    /**
     * @brief Set Rest1 frame period
     *
     * Writes to Rest1_Period register (0x11) to set
     * the frame period in Rest1 power state.
     *
     * @param period Period value (1-256, default: 1)
     * @return true if write successful
     *
     * @note Time formula: (R1R[7:0] + 1) x 10ms
     *       - Min: 2 x 10ms = 20ms
     *       - Default: (1+1) x 10ms = 20ms
     *       - Max: 256 x 10ms = 2,560ms
     *
     */
    bool setRest1Period(uint8_t period);

    /**
     * @brief Get Rest1 frame period
     * @return Raw period register value
     */
    int getRest1Period() const;

    /**
     * @brief Set Rest1 to Rest2 downshift time
     *
     * Writes to Rest1_Downshift register (0x15) to set
     * the time before transitioning from Rest1 to Rest2.
     *
     * @param r1d Downshift time value (1-242, default: 31)
     * @return true if write successful
     *
     * @note Time formula: R1D[7:0] x 16 x Rest1_Period
     *       - Min: 1 x 16 x 20ms = 320ms
     *       - Default: 31 x 16 x 20ms = 9,920ms
     *       - Max: 242 x 16 x 2.56s = 9,912s
     *
     */
    bool setRest1DownshiftTime(uint8_t r1d);

    /**
     * @brief Get Rest1 to Rest2 downshift time
     * @return Raw downshift register value
     */
    int getRest1DownshiftTime() const;

    /**
     * @brief Set Rest2 frame period
     *
     * Writes to Rest2_Period register (0x16) to set
     * the frame period in Rest2 power state.
     *
     * @param period Period value (1-256, default: 9)
     * @return true if write successful
     *
     * @note Time formula: (R2R[7:0] + 1) x 10ms
     *       - Min: 2 x 10ms = 20ms
     *       - Default: (9+1) x 10ms = 100ms
     *       - Max: 256 x 10ms = 2,560ms
     */
    bool setRest2Period(uint8_t period);

    /**
     * @brief Get Rest2 frame period
     * @return Raw period register value
     */
    int getRest2Period() const;

    /**
     * @brief Set Rest2 to Rest3 downshift time
     *
     * Writes to Rest2_Downshift register (0x17) to set
     * the time before transitioning from Rest2 to Rest3.
     *
     * @param r2d Downshift time value (1-242, default: 47)
     * @return true if write successful
     *
     * @note Time formula: R2D[7:0] x 128 x Rest2_Period
     *       - Min: 1 x 128 x 20ms = 2,560ms
     *       - Default: 47 x 128 x 100ms = 601,600ms
     *       - Max: 242 x 128 x 2.56s = 79,298s
     */
    bool setRest2DownshiftTime(uint8_t r2d);

    /**
     * @brief Get Rest2 to Rest3 downshift time
     * @return Raw downshift register value
     */
    int getRest2DownshiftTime() const;

    /**
     * @brief Set Rest3 frame period
     *
     * Writes to Rest3_Period register (0x18) to set
     * the frame period in Rest3 (lowest power) state.
     *
     * @param period Period value (1-256, default: 49)
     * @return true if write successful
     *
     * @note Time formula: (R3R[7:0] + 1) x 10ms
     *       - Min: 2 x 10ms = 20ms
     *       - Default: (49+1) x 10ms = 500ms
     *       - Max: 256 x 10ms = 2,560ms
     */
    bool setRest3Period(uint8_t period);

    /**
     * @brief Get Rest3 frame period
     * @return Raw period register value
     */
    int getRest3Period() const;

    /**
     * @brief Enable/disable LED always-on mode
     *
     * Controls the LED_On bit in LED_CTRL register (0x1A).
     * When disabled (default), LED operates normally.
     * When enabled, LED stays always on.
     *
     * @param enable true to keep LED on, false for normal operation
     * @return true if write successful
     */
    bool setLedOn(bool enable);

    /**
     * @brief Get LED always-on status
     * @return true if LED is in always-on mode
     */
    bool getLedOn() const;

    /**
     * @brief Set LED drive current
     *
     * Configures LED drive current in LED_CTRL register (0x1A).
     *
     * @param current Drive current (0=13mA, 2=9.6mA, 7=27mA)
     * @return true if write successful
     *
     * @note Available values:
     *       - 0: 13mA (default)
     *       - 2: 9.6mA
     *       - 7: 27mA
     */
    bool setLedDriveCurrent(uint8_t current);

    /**
     * @brief Get LED drive current setting
     * @return Current drive current value
     */
    int getLedDriveCurrent() const;

    /**
     * @brief Set shutter maximum open time
     *
     * Writes to Shutter_Max_Hi (0x3B) and Shutter_Max_Lo (0x3C)
     * registers to set the maximum exposure time.
     *
     * @param value Shutter max value (0-2929, default: 2929)
     * @return true if write successful
     *
     * @note Shutter value represents pixel array exposure time
     *       in multiples of internal clock cycles.
     *       Max value: 2929 decimal (0x0B71)
     *
     */
    bool setShutterMax(uint16_t value);

    /**
     * @brief Get shutter maximum open time
     * @return Current shutter max value
     */
    int getShutterMax() const;

    /**
     * @brief Set motion interrupt threshold
     *
     * Configures MOTION_CTRL register (0x74) to set the
     * motion interrupt threshold.
     *
     * @param threshold Threshold value (0-7, default: 0)
     * @return true if write successful
     *
     * @note Motion interrupt triggers only when:
     *       |Delta_X| + |Delta_Y| > Threshold
     *       Default (0) triggers on any motion.
     *
     */
    bool setMotionInterruptThreshold(uint8_t threshold);

    /**
     * @brief Get motion interrupt threshold
     * @return Current threshold value
     */
    int getMotionInterruptThreshold() const;

    /**
     * @brief Clear finger presence detect flag
     *
     * Reads the FPD_FLAG register (0x7A) to clear
     * the finger state change flag (FPD_ST) in the
     * EVENT register.
     *
     * @return true if read successful
     *
     * @note FPD_ST bit in EVENT register indicates
     *       finger state change (finger on/off).
     *       This function clears that flag.
     *
     */
    bool clearFingerState();

    /**
     * @brief Quick check if motion detected
     *
     * Convenience function to check if motion flag
     * is set in EVENT register.
     *
     * @return true if motion detected
     */
    bool isMotionDetected();

    /**
     * @brief Get Run downshift time in milliseconds
     *
     * Calculates actual downshift time in ms based on
     * Run_Downshift register value.
     *
     * @return Time in milliseconds
     *
     * @note Formula: RD x 8 x 8ms
     */
    uint32_t getRunDownshiftTimeMs() const;

    /**
     * @brief Get Rest1 period in milliseconds
     * @return Rest1 period in ms
     */
    uint32_t getRest1PeriodMs() const;

    /**
     * @brief Get Rest1 downshift time in milliseconds
     * @return Rest1 to Rest2 downshift time in ms
     */
    uint32_t getRest1DownshiftTimeMs() const;

    /**
     * @brief Get Rest2 period in milliseconds
     * @return Rest2 period in ms
     */
    uint32_t getRest2PeriodMs() const;

    /**
     * @brief Get Rest2 downshift time in milliseconds
     * @return Rest2 to Rest3 downshift time in ms
     */
    uint32_t getRest2DownshiftTimeMs() const;

    /**
     * @brief Get Rest3 period in milliseconds
     * @return Rest3 period in ms
     */
    uint32_t getRest3PeriodMs() const;

protected:
    bool initImpl(uint8_t param) override;
    uint8_t cached_rest1_period = 1;
};

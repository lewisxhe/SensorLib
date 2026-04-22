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
 * @file      IMUDriver_QMI8658_Reg.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2026-04-15
 *
 */
#pragma once


#include <stdint.h>

namespace QMI8658Regs
{

#ifdef _BV
#undef _BV
#endif
#define _BV(bit) (1U << (bit))

static constexpr uint8_t QMI8658_WHO_AM_I_VAL          = 0x05;
static constexpr uint8_t QMI8658_RESET_VAL             = 0xB0;
static constexpr uint8_t QMI8658_RST_RESULT_VAL         = 0x80;

static constexpr float QMI8658_CONSTANT_ONE_G = 9.80665f;

// ==============================================
// 0x00 (RO) - WHO_AM_I
// ==============================================
static constexpr uint8_t REG_WHO_AM_I    = 0x00;
static constexpr uint8_t MASK_WHO_AM_I   = 0xFF;
static constexpr uint8_t SHIFT_WHO_AM_I  = 0;

// ==============================================
// 0x01 (RO) - REVISION
// ==============================================
static constexpr uint8_t REG_REVISION    = 0x01;
static constexpr uint8_t MASK_REVISION  = 0xFF;
static constexpr uint8_t SHIFT_REVISION = 0;

// ==============================================
// 0x02 (RW) - CTRL1 - Interface and Reset Control
// ==============================================
static constexpr uint8_t REG_CTRL1            = 0x02;
static constexpr uint8_t MASK_SIM             = _BV(7);
static constexpr uint8_t MASK_ADDR_AI         = _BV(6);
static constexpr uint8_t MASK_BIG_ENDIAN      = _BV(5);
static constexpr uint8_t MASK_I2C_WDT         = _BV(2);
static constexpr uint8_t MASK_I2C_INT2        = _BV(4);
static constexpr uint8_t MASK_I2C_INT1        = _BV(3);
static constexpr uint8_t MASK_SENSOR_DISABLE  = _BV(0);
static constexpr uint8_t MASK_SPI_MODE        = MASK_SIM;
static constexpr uint8_t MASK_SOFT_RESET      = MASK_SENSOR_DISABLE;
static constexpr uint8_t MASK_CTRL1_RESERVED  = _BV(1);
static constexpr uint8_t SHIFT_SIM            = 7;
static constexpr uint8_t SHIFT_ADDR_AI        = 6;
static constexpr uint8_t SHIFT_BIG_ENDIAN     = 5;
static constexpr uint8_t SHIFT_I2C_WDT        = 2;
static constexpr uint8_t SHIFT_I2C_INT2       = 4;
static constexpr uint8_t SHIFT_I2C_INT1       = 3;
static constexpr uint8_t SHIFT_SENSOR_DISABLE = 0;
static constexpr uint8_t SHIFT_CTRL1_RESERVED = 1;
static constexpr uint8_t SHIFT_SPI_MODE       = SHIFT_SIM;
static constexpr uint8_t SHIFT_SOFT_RESET     = 0;

// ==============================================
// 0x03 (RW) - CTRL2 - Accelerometer Control
// ==============================================
static constexpr uint8_t REG_CTRL2                 = 0x03;
static constexpr uint8_t MASK_ACCEL_SELF_TEST     = _BV(7);
static constexpr uint8_t MASK_ACCEL_RANGE        = 0x70;
static constexpr uint8_t MASK_ACCEL_ODR           = 0x0F;
static constexpr uint8_t SHIFT_ACCEL_SELF_TEST    = 7;
static constexpr uint8_t SHIFT_ACCEL_RANGE       = 4;
static constexpr uint8_t SHIFT_ACCEL_ODR          = 0;

// Accelerometer Full Scale Range
static constexpr uint8_t ACCEL_RANGE_2G   = 0x00;
static constexpr uint8_t ACCEL_RANGE_4G   = 0x01;
static constexpr uint8_t ACCEL_RANGE_8G   = 0x02;
static constexpr uint8_t ACCEL_RANGE_16G  = 0x03;

// Accelerometer Output Data Rate
static constexpr uint8_t ACCEL_ODR_7174_4HZ  = 0x00;
static constexpr uint8_t ACCEL_ODR_3587_2HZ  = 0x01;
static constexpr uint8_t ACCEL_ODR_1793_6HZ  = 0x02;
static constexpr uint8_t ACCEL_ODR_896_8HZ   = 0x03;
static constexpr uint8_t ACCEL_ODR_448_4HZ   = 0x04;
static constexpr uint8_t ACCEL_ODR_224_2HZ   = 0x05;
static constexpr uint8_t ACCEL_ODR_112_1HZ   = 0x06;
static constexpr uint8_t ACCEL_ODR_56_05HZ   = 0x07;
static constexpr uint8_t ACCEL_ODR_28_025HZ  = 0x08;
static constexpr uint8_t ACCEL_ODR_14_0125HZ = 0x09;
static constexpr uint8_t ACCEL_ODR_7_006HZ   = 0x0A;
static constexpr uint8_t ACCEL_ODR_LP_128HZ  = 0x0C;
static constexpr uint8_t ACCEL_ODR_LP_21HZ   = 0x0D;
static constexpr uint8_t ACCEL_ODR_LP_11HZ   = 0x0E;
static constexpr uint8_t ACCEL_ODR_LP_3HZ    = 0x0F;

// ==============================================
// 0x04 (RW) - CTRL3 - Gyroscope Control
// ==============================================
static constexpr uint8_t REG_CTRL3                = 0x04;
static constexpr uint8_t MASK_GYRO_SELF_TEST      = _BV(7);
static constexpr uint8_t MASK_GYRO_RANGE          = 0x70;
static constexpr uint8_t MASK_GYRO_ODR           = 0x0F;
static constexpr uint8_t SHIFT_GYRO_SELF_TEST    = 7;
static constexpr uint8_t SHIFT_GYRO_RANGE        = 4;
static constexpr uint8_t SHIFT_GYRO_ODR          = 0;

// Gyroscope Full Scale Range
static constexpr uint8_t GYRO_RANGE_16DPS   = 0x00;
static constexpr uint8_t GYRO_RANGE_32DPS   = 0x01;
static constexpr uint8_t GYRO_RANGE_64DPS   = 0x02;
static constexpr uint8_t GYRO_RANGE_128DPS  = 0x03;
static constexpr uint8_t GYRO_RANGE_256DPS  = 0x04;
static constexpr uint8_t GYRO_RANGE_512DPS  = 0x05;
static constexpr uint8_t GYRO_RANGE_1024DPS = 0x06;
static constexpr uint8_t GYRO_RANGE_2048DPS = 0x07;

// Gyroscope Output Data Rate
static constexpr uint8_t GYRO_ODR_7174_4HZ  = 0x00;
static constexpr uint8_t GYRO_ODR_3587_2HZ  = 0x01;
static constexpr uint8_t GYRO_ODR_1793_6HZ  = 0x02;
static constexpr uint8_t GYRO_ODR_896_8HZ   = 0x03;
static constexpr uint8_t GYRO_ODR_448_4HZ   = 0x04;
static constexpr uint8_t GYRO_ODR_224_2HZ   = 0x05;
static constexpr uint8_t GYRO_ODR_112_1HZ   = 0x06;
static constexpr uint8_t GYRO_ODR_56_05HZ   = 0x07;
static constexpr uint8_t GYRO_ODR_28_025HZ  = 0x08;
static constexpr uint8_t GYRO_ODR_14_0125HZ = 0x09;
static constexpr uint8_t GYRO_ODR_7_006HZ   = 0x0A;

// ==============================================
// 0x06 (RW) - CTRL5 - Low Pass Filter Configuration
// ==============================================
static constexpr uint8_t REG_CTRL5                  = 0x06;
static constexpr uint8_t MASK_ACCEL_LPF_EN           = _BV(0);
static constexpr uint8_t MASK_ACCEL_LPF_ODR          = 0x06;
static constexpr uint8_t MASK_GYRO_LPF_EN            = _BV(4);
static constexpr uint8_t MASK_GYRO_LPF_ODR           = 0x60;
static constexpr uint8_t SHIFT_ACCEL_LPF_EN          = 0;
static constexpr uint8_t SHIFT_ACCEL_LPF_ODR         = 1;
static constexpr uint8_t SHIFT_GYRO_LPF_EN           = 4;
static constexpr uint8_t SHIFT_GYRO_LPF_ODR           = 5;

// LPF Mode (percentage of output data rate)
static constexpr uint8_t LPF_MODE_0 = 0x00;  // 2.66%
static constexpr uint8_t LPF_MODE_1 = 0x01;  // 3.63%
static constexpr uint8_t LPF_MODE_2 = 0x02;  // 5.39%
static constexpr uint8_t LPF_MODE_3 = 0x03;  // 13.37%

// ==============================================
// 0x08 (RW) - CTRL7 - Sensor Enable
// ==============================================
static constexpr uint8_t REG_CTRL7               = 0x08;
static constexpr uint8_t MASK_ACCEL_ENABLE        = _BV(0);
static constexpr uint8_t MASK_GYRO_ENABLE         = _BV(1);
static constexpr uint8_t MASK_SYNC_MODE           = _BV(7);
static constexpr uint8_t MASK_SYNC_DATA_SELECT    = _BV(6);
static constexpr uint8_t MASK_SYNC_DATA_LATCH     = _BV(5);
static constexpr uint8_t MASK_I3C_ENABLE          = _BV(4);
static constexpr uint8_t SHIFT_ACCEL_ENABLE       = 0;
static constexpr uint8_t SHIFT_GYRO_ENABLE        = 1;
static constexpr uint8_t SHIFT_SYNC_MODE          = 7;
static constexpr uint8_t SHIFT_SYNC_DATA_SELECT   = 6;
static constexpr uint8_t SHIFT_SYNC_DATA_LATCH    = 5;
static constexpr uint8_t SHIFT_I3C_ENABLE         = 4;

// ==============================================
// 0x09 (RW) - CTRL8 - Interrupt and Activity Configuration
// ==============================================
static constexpr uint8_t REG_CTRL8                     = 0x09;
static constexpr uint8_t MASK_ACTIVITY_INT_MAP         = _BV(6);
static constexpr uint8_t MASK_PEDOMETER_ENABLE         = _BV(4);
static constexpr uint8_t MASK_SIGNIFICANT_MOTION_EN    = _BV(3);
static constexpr uint8_t MASK_NO_MOTION_ENABLE         = _BV(2);
static constexpr uint8_t MASK_ANY_MOTION_ENABLE        = _BV(1);
static constexpr uint8_t MASK_TAP_DETECTION_ENABLE     = _BV(0);
static constexpr uint8_t SHIFT_ACTIVITY_INT_MAP        = 6;
static constexpr uint8_t SHIFT_PEDOMETER_ENABLE        = 4;
static constexpr uint8_t SHIFT_SIGNIFICANT_MOTION_EN   = 3;
static constexpr uint8_t SHIFT_NO_MOTION_ENABLE        = 2;
static constexpr uint8_t SHIFT_ANY_MOTION_ENABLE       = 1;
static constexpr uint8_t SHIFT_TAP_DETECTION_ENABLE   = 0;

// ==============================================
// 0x0A (WO) - CTRL9 - Command Register
// ==============================================
static constexpr uint8_t REG_CTRL9          = 0x0A;
static constexpr uint8_t MASK_CTRL9_COMMAND = 0xFF;
static constexpr uint8_t SHIFT_CTRL9_COMMAND = 0;

// CTRL9 Commands
static constexpr uint8_t CTRL_CMD_ACK                         = 0x00;
static constexpr uint8_t CTRL_CMD_RST_FIFO                   = 0x04;
static constexpr uint8_t CTRL_CMD_REQ_FIFO                   = 0x05;
static constexpr uint8_t CTRL_CMD_WRITE_WOM_SETTING          = 0x08;
static constexpr uint8_t CTRL_CMD_ACCEL_HOST_DELTA_OFFSET    = 0x09;
static constexpr uint8_t CTRL_CMD_GYRO_HOST_DELTA_OFFSET     = 0x0A;
static constexpr uint8_t CTRL_CMD_CONFIGURE_TAP             = 0x0C;
static constexpr uint8_t CTRL_CMD_CONFIGURE_PEDOMETER        = 0x0D;
static constexpr uint8_t CTRL_CMD_CONFIGURE_MOTION          = 0x0E;
static constexpr uint8_t CTRL_CMD_RESET_PEDOMETER           = 0x0F;
static constexpr uint8_t CTRL_CMD_COPY_USID                  = 0x10;
static constexpr uint8_t CTRL_CMD_SET_RPU                    = 0x11;
static constexpr uint8_t CTRL_CMD_AHB_CLOCK_GATING           = 0x12;
static constexpr uint8_t CTRL_CMD_ON_DEMAND_CALIBRATION      = 0xA2;
static constexpr uint8_t CTRL_CMD_APPLY_GYRO_GAINS           = 0xAA;

// ==============================================
// 0x0B-0x12 - Calibration Registers
// ==============================================
static constexpr uint8_t REG_CAL1_L      = 0x0B;
static constexpr uint8_t REG_CAL1_H      = 0x0C;
static constexpr uint8_t REG_CAL2_L      = 0x0D;
static constexpr uint8_t REG_CAL2_H      = 0x0E;
static constexpr uint8_t REG_CAL3_L      = 0x0F;
static constexpr uint8_t REG_CAL3_H      = 0x10;
static constexpr uint8_t REG_CAL4_L      = 0x11;
static constexpr uint8_t REG_CAL4_H      = 0x12;

// ==============================================
// 0x13 (RW) - FIFO_WTM_TH - FIFO Watermark Threshold
// ==============================================
static constexpr uint8_t REG_FIFO_WTM_TH    = 0x13;
static constexpr uint8_t MASK_FIFO_WTM_TH  = 0xFF;
static constexpr uint8_t SHIFT_FIFO_WTM_TH = 0;

// ==============================================
// 0x14 (RW) - FIFO_CTRL - FIFO Control
// ==============================================
static constexpr uint8_t REG_FIFO_CTRL              = 0x14;
static constexpr uint8_t MASK_FIFO_MODE             = 0x03;
static constexpr uint8_t MASK_FIFO_SAMPLES          = 0x0C;
static constexpr uint8_t SHIFT_FIFO_MODE            = 0;
static constexpr uint8_t SHIFT_FIFO_SAMPLES         = 2;

static constexpr uint8_t FIFO_MODE_BYPASS  = 0x00;
static constexpr uint8_t FIFO_MODE_FIFO    = 0x01;
static constexpr uint8_t FIFO_MODE_STREAM  = 0x02;

static constexpr uint8_t FIFO_SAMPLES_16 = 0x00;
static constexpr uint8_t FIFO_SAMPLES_32 = 0x01;
static constexpr uint8_t FIFO_SAMPLES_64 = 0x02;
static constexpr uint8_t FIFO_SAMPLES_128 = 0x03;

// ==============================================
// 0x15-0x16 - FIFO Status Registers
// ==============================================
static constexpr uint8_t REG_FIFO_SMPL_CNT_L  = 0x15;
static constexpr uint8_t REG_FIFO_SMPL_CNT_H  = 0x16;
static constexpr uint8_t REG_FIFO_STATUS       = 0x16;
static constexpr uint8_t MASK_FIFO_EMPTY       = _BV(4);
static constexpr uint8_t MASK_FIFO_OVERFLOW    = _BV(5);
static constexpr uint8_t MASK_FIFO_WTM_HIT     = _BV(6);
static constexpr uint8_t MASK_FIFO_FULL        = _BV(7);

// ==============================================
// 0x18 (RO) - FIFO_DATA - FIFO Data Output
// ==============================================
static constexpr uint8_t REG_FIFO_DATA = 0x17;

// ==============================================
// 0x2D (RO) - STATUS_INT - Interrupt Status
// ==============================================
static constexpr uint8_t REG_STATUS_INT        = 0x2D;
static constexpr uint8_t MASK_INT_AVAIL        = _BV(0);
static constexpr uint8_t MASK_INT_LOCKED       = _BV(1);
static constexpr uint8_t MASK_INT_CTRL9_DONE   = _BV(7);
static constexpr uint8_t SHIFT_INT_AVAIL       = 0;
static constexpr uint8_t SHIFT_INT_LOCKED      = 1;
static constexpr uint8_t SHIFT_INT_CTRL9_DONE  = 7;

// ==============================================
// 0x2E (RO) - STATUS0 - Sensor Data Status
// ==============================================
static constexpr uint8_t REG_STATUS0          = 0x2E;
static constexpr uint8_t MASK_ACCEL_DATA_RDY  = _BV(0);
static constexpr uint8_t MASK_GYRO_DATA_RDY   = _BV(1);
static constexpr uint8_t SHIFT_ACCEL_DATA_RDY = 0;
static constexpr uint8_t SHIFT_GYRO_DATA_RDY  = 1;

// ==============================================
// 0x2F (RO) - STATUS1 - Activity Status
// ==============================================
static constexpr uint8_t REG_STATUS1                   = 0x2F;
static constexpr uint8_t MASK_TAP_EVENT                 = _BV(1);
static constexpr uint8_t MASK_WOM_EVENT                 = _BV(2);
static constexpr uint8_t MASK_PEDOMETER_EVENT          = _BV(4);
static constexpr uint8_t MASK_ANY_MOTION_EVENT          = _BV(5);
static constexpr uint8_t MASK_NO_MOTION_EVENT          = _BV(6);
static constexpr uint8_t MASK_SIGNIFICANT_MOTION_EVENT = _BV(7);
static constexpr uint8_t SHIFT_TAP_EVENT                = 1;
static constexpr uint8_t SHIFT_WOM_EVENT                = 2;
static constexpr uint8_t SHIFT_PEDOMETER_EVENT         = 4;
static constexpr uint8_t SHIFT_ANY_MOTION_EVENT        = 5;
static constexpr uint8_t SHIFT_NO_MOTION_EVENT         = 6;
static constexpr uint8_t SHIFT_SIGNIFICANT_MOTION_EVENT = 7;

// ==============================================
// 0x30-0x32 - Timestamp Registers
// ==============================================
static constexpr uint8_t REG_TIMESTAMP_L  = 0x30;
static constexpr uint8_t REG_TIMESTAMP_M  = 0x31;
static constexpr uint8_t REG_TIMESTAMP_H  = 0x32;

// ==============================================
// 0x33-0x40 - Sensor Data Registers
// ==============================================
static constexpr uint8_t REG_TEMPERATURE_L = 0x33;
static constexpr uint8_t REG_TEMPERATURE_H = 0x34;
static constexpr uint8_t REG_AX_L         = 0x35;
static constexpr uint8_t REG_AX_H         = 0x36;
static constexpr uint8_t REG_AY_L         = 0x37;
static constexpr uint8_t REG_AY_H         = 0x38;
static constexpr uint8_t REG_AZ_L         = 0x39;
static constexpr uint8_t REG_AZ_H         = 0x3A;
static constexpr uint8_t REG_GX_L         = 0x3B;
static constexpr uint8_t REG_GX_H         = 0x3C;
static constexpr uint8_t REG_GY_L         = 0x3D;
static constexpr uint8_t REG_GY_H         = 0x3E;
static constexpr uint8_t REG_GZ_L         = 0x3F;
static constexpr uint8_t REG_GZ_H         = 0x40;

// ==============================================
// 0x46 (RO) - COD_STATUS - Calibration On Demand Status
// ==============================================
static constexpr uint8_t REG_COD_STATUS            = 0x46;
static constexpr uint8_t MASK_COD_FAIL             = _BV(0);
static constexpr uint8_t MASK_COD_GYRO_ENABLED_ERR  = _BV(1);
static constexpr uint8_t MASK_COD_GYRO_STARTUP_ERR  = _BV(2);
static constexpr uint8_t MASK_COD_ACCEL_ERR        = _BV(3);
static constexpr uint8_t MASK_COD_GYRO_Z_HIGH_ERR  = _BV(4);
static constexpr uint8_t MASK_COD_GYRO_Z_LOW_ERR   = _BV(5);
static constexpr uint8_t MASK_COD_GYRO_Y_HIGH_ERR   = _BV(6);
static constexpr uint8_t MASK_COD_GYRO_Y_LOW_ERR    = _BV(7);

// ==============================================
// 0x49-0x56 - Self-Test / Calibration Data Registers
// ==============================================
static constexpr uint8_t REG_DQW_L = 0x49;
static constexpr uint8_t REG_DQW_H = 0x4A;
static constexpr uint8_t REG_DQX_L = 0x4B;
static constexpr uint8_t REG_DQX_H = 0x4C;
static constexpr uint8_t REG_DQY_L = 0x4D;
static constexpr uint8_t REG_DQY_H = 0x4E;
static constexpr uint8_t REG_DQZ_L = 0x4F;
static constexpr uint8_t REG_DQZ_H = 0x50;
static constexpr uint8_t REG_DVX_L = 0x51;
static constexpr uint8_t REG_DVX_H = 0x52;
static constexpr uint8_t REG_DVY_L = 0x53;
static constexpr uint8_t REG_DVY_H = 0x54;
static constexpr uint8_t REG_DVZ_L = 0x55;
static constexpr uint8_t REG_DVZ_H = 0x56;

// ==============================================
// 0x59 (RO) - TAP_STATUS - Tap Detection Status
// ==============================================
static constexpr uint8_t REG_TAP_STATUS         = 0x59;
static constexpr uint8_t MASK_TAP_TYPE          = 0x03;
static constexpr uint8_t MASK_TAP_AXIS          = 0x30;
static constexpr uint8_t MASK_TAP_DIRECTION     = _BV(7);
static constexpr uint8_t SHIFT_TAP_TYPE         = 0;
static constexpr uint8_t SHIFT_TAP_AXIS         = 4;
static constexpr uint8_t SHIFT_TAP_DIRECTION    = 7;

static constexpr uint8_t TAP_TYPE_NONE     = 0x00;
static constexpr uint8_t TAP_TYPE_SINGLE  = 0x01;
static constexpr uint8_t TAP_TYPE_DOUBLE  = 0x02;

// ==============================================
// 0x5A-0x5C - Step Counter Registers
// ==============================================
static constexpr uint8_t REG_STEP_CNT_L    = 0x5A;
static constexpr uint8_t REG_STEP_CNT_M    = 0x5B;
static constexpr uint8_t REG_STEP_CNT_H    = 0x5C;

// ==============================================
// 0x60 (WO) - RESET - Reset Register
// ==============================================
static constexpr uint8_t REG_RESET         = 0x60;
static constexpr uint8_t MASK_RESET        = 0xFF;

// ==============================================
// 0x4D (RO) - RST_RESULT - Reset Result
// ==============================================
static constexpr uint8_t REG_RST_RESULT    = 0x4D;
static constexpr uint8_t MASK_RST_RESULT   = _BV(7);

// ==============================================
// Constants for scale calculations
// ==============================================
static constexpr float ACCEL_SCALE_2G    = 2.0f / 32768.0f;
static constexpr float ACCEL_SCALE_4G    = 4.0f / 32768.0f;
static constexpr float ACCEL_SCALE_8G    = 8.0f / 32768.0f;
static constexpr float ACCEL_SCALE_16G   = 16.0f / 32768.0f;

static constexpr float GYRO_SCALE_16DPS   = 16.0f / 32768.0f;
static constexpr float GYRO_SCALE_32DPS   = 32.0f / 32768.0f;
static constexpr float GYRO_SCALE_64DPS   = 64.0f / 32768.0f;
static constexpr float GYRO_SCALE_128DPS  = 128.0f / 32768.0f;
static constexpr float GYRO_SCALE_256DPS  = 256.0f / 32768.0f;
static constexpr float GYRO_SCALE_512DPS  = 512.0f / 32768.0f;
static constexpr float GYRO_SCALE_1024DPS = 1024.0f / 32768.0f;
static constexpr float GYRO_SCALE_2048DPS = 2048.0f / 32768.0f;

// ==============================================
// Motion Control Configuration Masks
// ==============================================
static constexpr uint8_t MASK_ANY_MOTION_X_EN  = _BV(0);
static constexpr uint8_t MASK_ANY_MOTION_Y_EN  = _BV(1);
static constexpr uint8_t MASK_ANY_MOTION_Z_EN  = _BV(2);
static constexpr uint8_t MASK_ANY_MOTION_LOGIC  = _BV(3);
static constexpr uint8_t MASK_NO_MOTION_X_EN   = _BV(4);
static constexpr uint8_t MASK_NO_MOTION_Y_EN   = _BV(5);
static constexpr uint8_t MASK_NO_MOTION_Z_EN   = _BV(6);
static constexpr uint8_t MASK_NO_MOTION_LOGIC  = _BV(7);

// ==============================================
// Tap Detection Priority
// ==============================================
static constexpr uint8_t TAP_PRIORITY_X_GT_Y_GT_Z = 0x00;
static constexpr uint8_t TAP_PRIORITY_X_GT_Z_GT_Y = 0x01;
static constexpr uint8_t TAP_PRIORITY_Y_GT_X_GT_Z = 0x02;
static constexpr uint8_t TAP_PRIORITY_Y_GT_Z_GT_X = 0x03;
static constexpr uint8_t TAP_PRIORITY_Z_GT_X_GT_Y = 0x04;
static constexpr uint8_t TAP_PRIORITY_Z_GT_Y_GT_X = 0x05;
} // namespace QMI8658Regs

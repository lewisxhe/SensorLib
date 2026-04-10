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
 * @file      BoschSensorID.hpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-30
 * @note      Raw API sensor IDs in bhy2_defs.h
 */
#pragma once

#include <stdint.h>

namespace BoschSensorID {

  static const uint16_t  ACCEL_PASSTHROUGH                       = (1);   /*BHY2_SENSOR_ID_ACC_PASS: Accelerometer passthrough */
  static const uint16_t  ACCEL_UNCALIBRATED                      = (3);   /*BHY2_SENSOR_ID_ACC_RAW: Accelerometer uncalibrated */
  static const uint16_t  ACCEL_CORRECTED                         = (4);   /*BHY2_SENSOR_ID_ACC: Accelerometer corrected */
  static const uint16_t  ACCEL_OFFSET                            = (5);   /*BHY2_SENSOR_ID_ACC_BIAS: Accelerometer offset */
  static const uint16_t  ACCEL_CORRECTED_WAKE_UP                 = (6);   /*BHY2_SENSOR_ID_ACC_WU: Accelerometer corrected wake up */
  static const uint16_t  ACCEL_UNCALIBRATED_WAKE_UP              = (7);   /*BHY2_SENSOR_ID_ACC_RAW_WU: Accelerometer uncalibrated wake up */
//   static const uint16_t  VIRTUAL_SENSOR_ID_FOR_ACCEL             = (8);   /*BHY2_SENSOR_ID_SI_ACCEL: Virtual Sensor ID for Accelerometer */
  static const uint16_t  GYRO_PASSTHROUGH                        = (10);  /*BHY2_SENSOR_ID_GYRO_PASS: Gyroscope passthrough */
  static const uint16_t  GYRO_UNCALIBRATED                       = (12);  /*BHY2_SENSOR_ID_GYRO_RAW: Gyroscope uncalibrated */
  static const uint16_t  GYRO_CORRECTED                          = (13);  /*BHY2_SENSOR_ID_GYRO: Gyroscope corrected */
  static const uint16_t  GYRO_OFFSET                             = (14);  /*BHY2_SENSOR_ID_GYRO_BIAS: Gyroscope offset */
  static const uint16_t  GYRO_WAKE_UP                            = (15);  /*BHY2_SENSOR_ID_GYRO_WU: Gyroscope wake up */
  static const uint16_t  GYRO_UNCALIBRATED_WAKE_UP               = (16);  /*BHY2_SENSOR_ID_GYRO_RAW_WU: Gyroscope uncalibrated wake up */
//   static const uint16_t  VIRTUAL_SENSOR_ID_FOR_GYRO              = (17);  /*BHY2_SENSOR_ID_SI_GYROS: Virtual Sensor ID for Gyroscope */
  static const uint16_t  MAGNETOMETER_PASSTHROUGH                = (19);  /*BHY2_SENSOR_ID_MAG_PASS: Magnetometer passthrough */
  static const uint16_t  MAGNETOMETER_UNCALIBRATED               = (21);  /*BHY2_SENSOR_ID_MAG_RAW: Magnetometer uncalibrated */
  static const uint16_t  MAGNETOMETER_CORRECTED                  = (22);  /*BHY2_SENSOR_ID_MAG: Magnetometer corrected */
  static const uint16_t  MAGNETOMETER_OFFSET                     = (23);  /*BHY2_SENSOR_ID_MAG_BIAS: Magnetometer offset */
  static const uint16_t  MAGNETOMETER_WAKE_UP                    = (24);  /*BHY2_SENSOR_ID_MAG_WU: Magnetometer wake up */
  static const uint16_t  MAGNETOMETER_UNCALIBRATED_WAKE_UP       = (25);  /*BHY2_SENSOR_ID_MAG_RAW_WU: Magnetometer uncalibrated wake up */
  static const uint16_t  GRAVITY_VECTOR                          = (28);  /*BHY2_SENSOR_ID_GRA: Gravity vector */
  static const uint16_t  GRAVITY_VECTOR_WAKE_UP                  = (29);  /*BHY2_SENSOR_ID_GRA_WU: Gravity vector wake up */
  static const uint16_t  LINEAR_ACCELERATION                     = (31);  /*BHY2_SENSOR_ID_LACC: Linear acceleration */
  static const uint16_t  LINEAR_ACCELERATION_WAKE_UP             = (32);  /*BHY2_SENSOR_ID_LACC_WU: Linear acceleration wake up */
  static const uint16_t  ROTATION_VECTOR                         = (34);  /*BHY2_SENSOR_ID_RV: Rotation vector ; quaternion*/
  static const uint16_t  ROTATION_VECTOR_WAKE_UP                 = (35);  /*BHY2_SENSOR_ID_RV_WU: Rotation vector wake up */
  static const uint16_t  GAME_ROTATION_VECTOR                    = (37);  /*BHY2_SENSOR_ID_GAMERV: Game rotation vector */
  static const uint16_t  GAME_ROTATION_VECTOR_WAKE_UP            = (38);  /*BHY2_SENSOR_ID_GAMERV_WU: Game rotation vector wake up */
  static const uint16_t  GEO_MAGNETIC_ROTATION_VECTOR            = (40);  /*BHY2_SENSOR_ID_GEORV: Geo-magnetic rotation vector */
  static const uint16_t  GEO_MAGNETIC_ROTATION_VECTOR_WAKE_UP    = (41);  /*BHY2_SENSOR_ID_GEORV_WU: Geo-magnetic rotation vector wake up */
  static const uint16_t  ORIENTATION                             = (43);  /*BHY2_SENSOR_ID_ORI: Orientation */
  static const uint16_t  ORIENTATION_WAKE_UP                     = (44);  /*BHY2_SENSOR_ID_ORI_WU: Orientation wake up ; Euler*/
  static const uint16_t  TILT_DETECTOR                           = (48);  /*BHY2_SENSOR_ID_TILT_DETECTOR: Tilt detector */
  static const uint16_t  STEP_DETECTOR                           = (50);  /*BHY2_SENSOR_ID_STD: Step detector */
  static const uint16_t  STEP_COUNTER                            = (52);  /*BHY2_SENSOR_ID_STC: Step counter */
  static const uint16_t  STEP_COUNTER_WAKE_UP                    = (53);  /*BHY2_SENSOR_ID_STC_WU: Step counter wake up */
  static const uint16_t  SIGNIFICANT_MOTION                      = (55);  /*BHY2_SENSOR_ID_SIG: Significant motion */
  static const uint16_t  WAKE_GESTURE                            = (57);  /*BHY2_SENSOR_ID_WAKE_GESTURE: Wake gesture */
  static const uint16_t  GLANCE_GESTURE                          = (59);  /*BHY2_SENSOR_ID_GLANCE_GESTURE: Glance gesture */
  static const uint16_t  PICKUP_GESTURE                          = (61);  /*BHY2_SENSOR_ID_PICKUP_GESTURE: Pickup gesture */
  static const uint16_t  ACTIVITY_RECOGNITION                    = (63);  /*BHY2_SENSOR_ID_AR: Activity recognition */
  static const uint16_t  WRIST_TILT_GESTURE                      = (67);  /*BHY2_SENSOR_ID_WRIST_TILT_GESTURE: Wrist tilt gesture */
  static const uint16_t  DEVICE_ORIENTATION                      = (69);  /*BHY2_SENSOR_ID_DEVICE_ORI: Device orientation */
  static const uint16_t  DEVICE_ORIENTATION_WAKE_UP              = (70);  /*BHY2_SENSOR_ID_DEVICE_ORI_WU: Device orientation wake up */
  static const uint16_t  STATIONARY_DETECT                       = (75);  /*BHY2_SENSOR_ID_STATIONARY_DET: Stationary detect */
  static const uint16_t  MOTION_DETECT                           = (77);  /*BHY2_SENSOR_ID_MOTION_DET: Motion detect */
  static const uint16_t  ACCEL_OFFSET_WAKE_UP                    = (91);  /*BHY2_SENSOR_ID_ACC_BIAS_WU: Accelerometer offset wake up */
  static const uint16_t  GYRO_OFFSET_WAKE_UP                     = (92);  /*BHY2_SENSOR_ID_GYRO_BIAS_WU: Gyroscope offset wake up */
  static const uint16_t  MAGNETOMETER_OFFSET_WAKE_UP             = (93);  /*BHY2_SENSOR_ID_MAG_BIAS_WU: Magnetometer offset wake up */
  static const uint16_t  STEP_DETECTOR_WAKE_UP                   = (94);  /*BHY2_SENSOR_ID_STD_WU: Step detector wake up */
  static const uint16_t  KLIO                                    = (112); /*BHY2_SENSOR_ID_KLIO Supported by klio firmware ;defined in bhy2_klio_defs.h */
  static const uint16_t  SWIM                                    = (114); /*BHY2_SENSOR_ID_SWIM:Supported by swim firmware ;defined in bhy2_swim_defs.h */
  static const uint16_t  IAQ                                     = (115); /*BHY2_SENSOR_ID_AIR_QUALITY: IAQ */
  static const uint16_t  KLIO_LOG                                = (127); /*BHY2_SENSOR_ID_KLIO_LOG Supported by klio firmware ;defined in bhy2_klio_defs.h */
  static const uint16_t  TEMPERATURE                             = (128); /*BHY2_SENSOR_ID_TEMP: Temperature */
  static const uint16_t  BAROMETER                               = (129); /*BHY2_SENSOR_ID_BARO: Barometer */
  static const uint16_t  HUMIDITY                                = (130); /*BHY2_SENSOR_ID_HUM: Humidity */
  static const uint16_t  GAS                                     = (131); /*BHY2_SENSOR_ID_GAS: Gas */
  static const uint16_t  TEMPERATURE_WAKE_UP                     = (132); /*BHY2_SENSOR_ID_TEMP_WU: Temperature wake up */
  static const uint16_t  BAROMETER_WAKE_UP                       = (133); /*BHY2_SENSOR_ID_BARO_WU: Barometer wake up */
  static const uint16_t  HUMIDITY_WAKE_UP                        = (134); /*BHY2_SENSOR_ID_HUM_WU: Humidity wake up */
  static const uint16_t  GAS_WAKE_UP                             = (135); /*BHY2_SENSOR_ID_GAS_WU: Gas wake up */
  static const uint16_t  STEP_COUNTER_LOW_POWER                  = (136); /*BHY2_SENSOR_ID_STC_LP: Step counter Low Power */
  static const uint16_t  STEP_DETECTOR_LOW_POWER                 = (137); /*BHY2_SENSOR_ID_STD_LP: Step detector Low Power */
  static const uint16_t  SIGNIFICANT_MOTION_LOW_POWER            = (138); /*BHY2_SENSOR_ID_SIG_LP: Significant motion Low Power */
  static const uint16_t  STEP_COUNTER_LOW_POWER_WAKE_UP          = (139); /*BHY2_SENSOR_ID_STC_LP_WU: Step counter Low Power wake up */
  static const uint16_t  STEP_DETECTOR_LOW_POWER_WAKE_UP         = (140); /*BHY2_SENSOR_ID_STD_LP_WU: Step detector Low Power wake up */
  static const uint16_t  SIGNIFICANT_MOTION_LOW_POWER_WAKE_UP    = (141); /*BHY2_SENSOR_ID_SIG_LP_WU: Significant motion Low Power wake up */
  static const uint16_t  ANY_MOTION_LOW_POWER                    = (142); /*BHY2_SENSOR_ID_ANY_MOTION_LP: Any motion Low Power */
  static const uint16_t  ANY_MOTION_LOW_POWER_WAKE_UP            = (143); /*BHY2_SENSOR_ID_ANY_MOTION_LP_WU: Any motion Low Power wake up */
  static const uint16_t  EXTERNAL_CAMERA_TRIGGER                 = (144); /*BHY2_SENSOR_ID_EXCAMERA: External camera trigger */
  static const uint16_t  GPS                                     = (145); /*BHY2_SENSOR_ID_GPS: GPS */
  static const uint16_t  LIGHT                                   = (146); /*BHY2_SENSOR_ID_LIGHT: Light */
  static const uint16_t  PROXIMITY                               = (147); /*BHY2_SENSOR_ID_PROX: Proximity */
  static const uint16_t  LIGHT_WAKE_UP                           = (148); /*BHY2_SENSOR_ID_LIGHT_WU: Light wake up */
  static const uint16_t  PROXIMITY_WAKE_UP                       = (149); /*BHY2_SENSOR_ID_PROX_WU: Proximity wake up */
  static const uint16_t  GPIO_EXP                                = (151); /*BHY2_SENSOR_ID_GPIO_EXP: GPIO_EXP*/
  static const uint16_t  MULTI_TAP                               = (153); /*BHY2_SENSOR_ID_MULTI_TAP: Multi tap only supported by BHI360 */
  static const uint16_t  NO_MOTION_LOW_POWER_WAKE_UP             = (159); /*BHY2_SENSOR_ID_NO_MOTION_DET: No motion detect only supported by BHI360 */

} // namespace BoschSmartSensorID

/**
* Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bma4.c
* @date       2023-07-05
* @version    V2.29.0
*
*/

/*
 * @file       bma4.c
 * @brief   Source file for the BMA4 Sensor API
 */

/***************************************************************************/

/*!
 * @defgroup bma4 BMA4
 */

/**\name        Header files
 ****************************************************************************/
#include "bma4.h"

/***************************************************************************/

/**\name        Local structures
 ****************************************************************************/

/*!
 * @brief Accel data deviation from ideal value
 */
struct bma4_offset_delta
{
    /*! X axis */
    int16_t x;

    /*! Y axis */
    int16_t y;

    /*! Z axis */
    int16_t z;
};

/*!
 * @brief Accel offset xyz structure
 */
struct bma4_accel_offset
{
    /*! Accel offset X  data */
    uint8_t x;

    /*! Accel offset Y  data */
    uint8_t y;

    /*! Accel offset Z  data */
    uint8_t z;
};

/***************************************************************************/

/*! Static Function Declarations
 ****************************************************************************/

/*!
 *  @brief This API validates the bandwidth, odr and perf_mode
 *  combinations set by the user.
 *
 *  param bandwidth[in] : bandwidth value set by the user.
 *  param perf_mode[in] : perf_mode value set by the user.
 *  param odr[in]       : odr value set by the user.
 */
static int8_t validate_odr_bandwidth_perfmode(uint8_t odr, uint8_t bandwidth, uint8_t perf_mode);

/*!
 *  @brief This API validates the bandwidth and perfmode
 *  value set by the user.
 *
 *  param bandwidth[in] : bandwidth value set by the user.
 *  param perf_mode[in] : perf_mode value set by the user.
 */
static int8_t validate_bandwidth_perfmode(uint8_t bandwidth, uint8_t perf_mode);

/*!
 *  @brief @brief This API validates the ODR value set by the user.
 *
 *  param odr[in] : odr value set by the user.
 */
static int8_t validate_odr(uint8_t odr);

/*!
 *  @brief This API validates the bandwidth and odr when perf_mode = BMA4_CIC_AVG_MODE
 *
 *  param bandwidth[in] : bandwidth value set by the user.
 *  param odr[in]       : odr value set by the user.
 */
static int8_t validate_bandwidth_odr(uint8_t bandwidth, uint8_t odr);

/*!
 *  @brief This API is used to reset the FIFO related configurations
 *  in the fifo_frame structure.
 *
 *  @param fifo[in,out]  : Structure instance of bma4_fifo_frame
 *
 */
static void reset_fifo_data_structure(struct bma4_fifo_frame *fifo);

/*!
 *  @brief This API computes the number of bytes of accel FIFO data
 *  which is to be parsed in header-less mode
 *
 *  @param[out] start_idx   : The start index for parsing data
 *  @param[out] len         : Number of bytes to be parsed
 *  @param[in]  acc_count   : Number of accelerometer frames to be read
 *  @param[in]  fifo        : Structure instance of bma4_fifo_frame.
 *
 */
static void get_accel_len_to_parse(uint16_t *start_idx,
                                   uint16_t *len,
                                   const uint16_t *acc_count,
                                   const struct bma4_fifo_frame *fifo);

/*!
 *  @brief This API checks the fifo read data as empty frame, if it
 *  is empty frame then moves the index to last byte.
 *
 *  @param[in,out] data_index   : The index of the current data to
 *                                be parsed from fifo data
 *  @param[in]  fifo            : Structure instance of bma4_fifo_frame.
 */
static void check_empty_fifo(uint16_t *data_index, const struct bma4_fifo_frame *fifo);

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in header mode.
 *
 *  @param[in,out] accel_data   : Structure instance of bma4_accel where
 *                                the accelerometer data in FIFO is stored.
 *  @param[in,out] accel_length : Number of accelerometer frames
 *                                (x,y,z axes data)
 *  @param[in,out] fifo         : Structure instance of bma4_fifo_frame
 *  @param[in,out] dev          : Structure instance of bma4_dev.
 *
 */
static void extract_accel_header_mode(struct bma4_accel *accel_data,
                                      uint16_t *accel_length,
                                      struct bma4_fifo_frame *fifo,
                                      const struct bma4_dev *dev);

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in both header mode and header-less mode.
 *  It update the idx value which is used to store the index of
 *  the current data byte which is parsed.
 *
 *  @param[in,out] acc       : Structure instance of bma4_accel.
 *  @param[in,out] idx       : Index value of number of bytes parsed
 *  @param[in,out] acc_idx   : Index value of accelerometer data
 *                             (x,y,z axes) frame to be parsed
 *  @param[in] frm           : It consists of either fifo_data_enable
 *                             parameter (Accel and/or mag data enabled in FIFO)
 *                             in header-less mode or frame header data
 *                             in header mode
 *  @param[in]  fifo         : Structure instance of bma4_fifo_frame.
 *  @param[in]  dev          : Structure instance of bma4_dev.
 *
 */
static void unpack_acc_frm(struct bma4_accel *acc,
                           uint16_t *idx,
                           uint16_t *acc_idx,
                           uint8_t frm,
                           const struct bma4_fifo_frame *fifo,
                           const struct bma4_dev *dev);

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data and store it in the instance of the structure bma4_accel.
 *
 *  @param[out] accel_data        : Structure instance of bma4_accel where
 *                                  the parsed accel data bytes are stored.
 *  @param[in] data_start_index   : Index value of the accel data bytes
 *                                  which is to be parsed from the fifo data.
 *  @param[in] fifo               :  Structure instance of bma4_fifo_frame.
 *  @param[in] dev                :  Structure instance of bma4_dev.
 *
 */
static void unpack_accel_data(struct bma4_accel *accel_data,
                              uint16_t data_start_index,
                              const struct bma4_fifo_frame *fifo,
                              const struct bma4_dev *dev);

/*!
 *  @brief This API computes the number of bytes of Mag FIFO data which is
 *  to be parsed in header-less mode
 *
 *  @param[out] start_idx   : The start index for parsing data
 *  @param[out] len         : Number of bytes to be parsed
 *  @param[in]  mag_count   : Number of magnetometer frames to be read
 *  @param[in]  fifo        : Structure instance of bma4_fifo_frame.
 *
 */
static void get_mag_len_to_parse(uint16_t *start_idx,
                                 uint16_t *len,
                                 const uint16_t *mag_count,
                                 const struct bma4_fifo_frame *fifo);

/*!
 *  @brief This API is used to parse the magnetometer data from the
 *  FIFO data in header mode.
 *
 *  @param[in,out] data     : Structure instance of bma4_mag_xyzr where
 *                            the magnetometer data in FIFO is extracted
 *                            and stored.
 *  @param[in,out] len      : Number of magnetometer frames
 *                           (x,y,z,r data)
 *  @param[in,out] fifo     : Structure instance of bma4_fifo_frame.
 *  @param[in] dev          : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 *
 */
static int8_t extract_mag_header_mode(const struct bma4_mag *data,
                                      uint16_t *len,
                                      struct bma4_fifo_frame *fifo,
                                      const struct bma4_dev *dev);

/*!
 *  @brief This API is used to parse the magnetometer data from the
 *  FIFO data in both header mode and header-less mode and update the
 *  idx value which is used to store the index of the current
 *  data byte which is parsed.
 *
 *  @param data     : Structure instance of bma4_mag_xyzr.
 *  @param idx      : Index value of number of bytes parsed
 *  @param mag_idx  : Index value magnetometer data frame (x,y,z,r)
 *                    to be parsed
 *  @param frm      : It consists of either the fifo_data_enable parameter
 *                    (Accel and/or mag data enabled in FIFO) in
 *                    header-less mode and frame header data in header mode
 *  @param fifo     : Structure instance of bma4_fifo_frame.
 *  @param dev      : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 *
 */
static int8_t unpack_mag_frm(const struct bma4_mag *data,
                             uint16_t *idx,
                             uint16_t *mag_idx,
                             uint8_t frm,
                             const struct bma4_fifo_frame *fifo,
                             const struct bma4_dev *dev);

/*!
 *  @brief This API is used to parse the auxiliary magnetometer data from
 *  the FIFO data and store it in the instance of the structure mag_data.
 *
 *  @param mag_data         : Structure instance of bma4_mag_xyzr where the
 *                            parsed magnetometer data bytes are stored.
 *  @param start_idx        : Index value of the magnetometer data bytes
 *                            which is to be parsed from the FIFO data
 *  @param fifo             : Structure instance of bma4_fifo_frame.
 *  @param dev              : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 *
 */
static int8_t unpack_mag_data(const struct bma4_mag *mag_data,
                              uint16_t start_idx,
                              const struct bma4_fifo_frame *fifo,
                              const struct bma4_dev *dev);

/*!
 *  @brief This API is used to parse and store the sensor time from the
 *  FIFO data in the structure instance dev.
 *
 *  @param[in,out] data_index : Index of the FIFO data which
 *                              has the sensor time.
 *  @param[in,out] fifo       : Structure instance of bma4_fifo_frame.
 *
 */
static void unpack_sensortime_frame(uint16_t *data_index, struct bma4_fifo_frame *fifo);

/*!
 *  @brief This API is used to parse and store the skipped_frame_count from
 *  the FIFO data in the structure instance dev.
 *
 *  @param[in,out] data_index       : Index of the FIFO data which
 *                                    has the skipped frame count.
 *  @param[in,out] fifo             : Structure instance of bma4_fifo_frame.
 *
 */
static void unpack_skipped_frame(uint16_t *data_index, struct bma4_fifo_frame *fifo);

/*!
 *  @brief This API is used to parse and store the dropped_frame_count from
 *  the FIFO data in the structure instance dev.
 *
 *  @param[in,out] data_index       : Index of the FIFO data which
 *                                    has the dropped frame data.
 *  @param[in,out] fifo             : Structure instance of bma4_fifo_frame.
 *
 */
static void unpack_dropped_frame(uint16_t *data_index, struct bma4_fifo_frame *fifo);

/*!
 *  @brief This API is used to move the data index ahead of the
 *  current_frame_length parameter when unnecessary FIFO data appears while
 *  extracting the user specified data.
 *
 *  @param[in,out] data_index       : Index of the FIFO data which
 *                                    is to be moved ahead of the
 *                                    current_frame_length
 *  @param[in] current_frame_length : Number of bytes in a particular frame
 *  @param[in] fifo                 : Structure instance of bma4_fifo_frame.
 *
 */
static void move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bma4_fifo_frame *fifo);

/*!
 *  @brief This API writes the config stream data in memory using burst mode
 *
 *  @param[in] stream_data : Pointer to store data of 32 bytes
 *  @param[in] index : Represents value in multiple of 32 bytes
 *  @param[in] dev : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t stream_transfer_write(const uint8_t *stream_data, uint16_t index, struct bma4_dev *dev);

/*!
 *  @brief This API enables or disables the Accel self-test feature in the
 *  sensor.
 *
 *  @param[in] accel_self-test_enable : Variable used to enable or disable
 *  the Accel self-test feature
 *  Value   |  Description
 *  --------|---------------
 *  0x00    | BMA4_DISABLE
 *  0x01    | BMA4_ENABLE
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 *
 */
static int8_t set_accel_selftest_enable(uint8_t accel_selftest_axis, struct bma4_dev *dev);

/*!
 *  @brief This API selects the sign of Accel self-test excitation
 *
 *  @param[in] accel_selftest_sign: Variable used to select the Accel
 *  self-test sign
 *  Value   |  Description
 *  --------|--------------------------
 *  0x00    | BMA4_DISABLE (negative)
 *  0x01    | BMA4_ENABLE (positive)
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 *
 */
static int8_t set_accel_selftest_sign(uint8_t accel_selftest_sign, struct bma4_dev *dev);

/*!
 *  @brief This API sets the Accel self-test amplitude in the sensor.
 *
 *  @param[in] accel_selftest_amp : Variable used to specify the Accel self
 *  test amplitude
 *  Value   |  Description
 *  --------|------------------------------------
 *  0x00    | BMA4_SELFTEST_AMP_LOW
 *  0x01    | BMA4_SELFTEST_AMP_HIGH
 *
 *  @param[in] dev : structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 *
 */
static int8_t set_accel_selftest_amp(uint8_t accel_selftest_amp, struct bma4_dev *dev);

/*!
 *  @brief This function enables and configures the Accel which is needed
 *  for self-test operation.
 *
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return results of self-test
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 *
 */
static int8_t set_accel_selftest_config(struct bma4_dev *dev);

/*!
 *  @brief This function validates the Accel self-test data and decides the
 *  result of self-test operation.
 *
 *  @param[in] accel_data_diff : Pointer to structure variable which holds
 *  the Accel data difference of self-test operation
 *  @param[in] dev : Structure instance of bma4_dev
 *
 *  @return results of self-test operation
 *  @retval 0 -> Success
 *  @retval Any non zero value -> Fail
 *
 */
static int8_t validate_selftest(const struct bma4_selftest_delta_limit *accel_data_diff, const struct bma4_dev *dev);

/*!
 *  @brief This API converts lsb value of axes to mg for self-test
 *
 *  @param[in] accel_data_diff : Pointer variable used to pass accel difference
 *  values in g
 *  @param[out] accel_data_diff_mg : Pointer variable used to store accel
 *  difference values in mg
 *  @param[out] dev : Structure instance of bma4_dev
 *
 */
static void convert_lsb_g(const struct bma4_selftest_delta_limit *accel_data_diff,
                          struct bma4_selftest_delta_limit *accel_data_diff_mg,
                          const struct bma4_dev *dev);

/*!
 *  @brief This API sets the feature config. data start address in the sensor.
 *
 *  @param[in] dev  : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t set_feature_config_start_addr(struct bma4_dev *dev);

/*!
 *  @brief This API increments the feature config. data address according to the user
 *  provided read/write length in the dev structure.
 *
 *  @param[in] dev  : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t increment_feature_config_addr(struct bma4_dev *dev);

/*!
 *  @brief This API reads the 8-bit data from the given register
 *  in the sensor.
 *
 *  @param[in] addr : Register address.
 *  @param[in] data : Read data buffer.
 *  @param[in] len  : No of bytes to read.
 *  @param[in] dev  : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t read_regs(uint8_t addr, uint8_t *data, uint32_t len, struct bma4_dev *dev);

/*!
 *  @brief This API reads the 8-bit data from the given register
 *  when length is less than read write length
 *
 *  @param[in] data : Read data buffer.
 *  @param[in] len  : No of bytes to read.
 *  @param[in] dev  : Structure instance of bma4_dev
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t read_within_len_regs(uint8_t *data, uint32_t len, struct bma4_dev *dev);

/*!
 *  @brief This API writes the 8-bit data to the given register
 *  in the sensor.
 *
 *  @param[in] addr : Register address.
 *  @param[in] data : Write data buffer
 *  @param[in] len  : No of bytes to write
 *  @param[in] dev  : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t write_regs(uint8_t addr, const uint8_t *data, uint32_t len, struct bma4_dev *dev);

/*!
 *  @brief This API writes the 8-bit data to the given register
 *  in the sensor for length less than read write length
 *
 *  @param[in] data : Write data buffer
 *  @param[in] len  : No of bytes to write
 *  @param[in] dev  : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t write_within_len_regs(const uint8_t *data, uint32_t len, struct bma4_dev *dev);

/*!
 *  @brief This API sets the feature config. data start address in the sensor.
 *
 *  @param[in] dev  : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t get_feature_config_start_addr(struct bma4_dev *dev);

/*!
 * @brief This API is used to calculate the power of given
 * base value.
 *
 * @param[in] base : value of base
 * @param[in] resolution : resolution of the sensor
 *
 * @return : Return the value of base^resolution
 */
static int32_t power(int16_t base, uint8_t resolution);

/*!
 * @brief This API finds the the null error of the device pointer structure
 *
 * @param[in] dev : Structure instance of bma4_dev.
 *
 *  @return Result of API execution status
 *  @retval BMA4_OK -> Success
 *  @retval BMA4_E_NULL_PTR -> Null pointer Error
 */
static int8_t null_pointer_check(const struct bma4_dev *dev);

/*!
 * @brief This internal API gets the re-mapped x, y and z axes from the sensor.
 *
 * @param[out] remap_data      : Structure instance of bma4_axes_remap
 * @param[in] feature_config   : Array data containing feature configurations
 * @param[in] index            : Index to which mapping has to be applied
 * @param[in] feature_len      : Length of feature configurations
 * @param[in, out] dev         : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t get_remap_axes(struct bma4_axes_remap *remap_data,
                             uint8_t *feature_config,
                             uint8_t index,
                             uint8_t feature_len,
                             struct bma4_dev *dev);

/*!
 * @brief This internal API sets the re-mapped x, y and z axes in the sensor.
 *
 * @param[in] remap_data       : Structure instance of bma4_axes_remap
 * @param[in] feature_config   : Array data containing feature configurations
 * @param[in] index            : Index to which mapping has to be applied
 * @param[in] feature_len      : Length of feature configurations
 * @param[in, out] dev         : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t set_remap_axes(const struct bma4_axes_remap *remap_data,
                             uint8_t *feature_config,
                             uint8_t index,
                             uint8_t feature_len,
                             struct bma4_dev *dev);

/*!
 * @brief This internal API is to store re-mapped axis and sign values
 * in device structure
 *
 * @param[in] remap_axis      : Value of re-mapped axis
 * @param[out]  axis          : Re-mapped axis value stored in device structure
 * @param[out]  sign          : Re-mapped axis sign stored in device structure
 *
 * @return None
 *
 * @retval None
 */
static void assign_remap_axis(uint8_t remap_axis, uint8_t *axis, uint8_t *sign);

/*!
 * @brief This internal API is to receive re-mapped axis and sign values
 * in device structure
 *
 * @param[in] remap_axis      : Re-mapped axis value
 * @param[in]  remap_sign     : Re-mapped axis sign value
 * @param[out]  axis          : Re-mapped axis stored in local structure
 *
 * @return None
 *
 * @retval None
 */
static void receive_remap_axis(uint8_t remap_axis, uint8_t remap_sign, uint8_t *axis);

/*!
 * @brief This internal API gets the re-mapped accelerometer/gyroscope data.
 *
 * @param[out] data         : Structure instance of bma4_accel.
 * @param[in]  dev          : Structure instance of bma4_dev.
 *
 * @return None
 *
 * @retval None
 */
static void get_remapped_data(struct bma4_accel *data, const struct bma4_dev *dev);

/*!
 * @brief This API performs the steps needed for Self test operation
 *  before reading the Accel Self test data.
 *
 * @param[in] sign: Variable used to specify the self test sign
 * @param[in] dev : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval 0 -> Success
 * @retval < 0 -> Fail
 */
static int8_t selftest_config(uint8_t sign, struct bma4_dev *dev);

/*!
 * @brief This internal API brings up the secondary interface to access
 *          auxiliary sensor
 *
 * @param[in] dev : Structure instance of bma4_dev.
 *
 * @return Result of API execution status
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t set_aux_interface_config(struct bma4_dev *dev);

/*!
 * @brief This internal API reads the data from the auxiliary sensor
 * depending on burst length configured
 *
 * @param[in] dev  : Structure instance of bma4_dev.
 * @param[out] aux_data  : Pointer variable to store data read
 * @param[in] aux_reg_addr  : Variable to pass address from where
 *                            data is to be read
 *
 * @return Result of API execution status
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t extract_aux_data(uint8_t aux_reg_addr, uint8_t *aux_data, uint16_t len, struct bma4_dev *dev);

/*!
 * @brief This internal API maps the actual burst read length with user length set.
 *
 * @param[in] dev  : Structure instance of bma4_dev.
 * @param[out] len : Pointer variable to store mapped length
 *
 * @return Result of API execution status
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t map_read_len(uint8_t *len, const struct bma4_dev *dev);

/*!
 *  @brief This internal API validates accel self-test status from positive and negative axes input
 *
 * @param[in] positive  : Positive accel data.
 * @param[in] negative  : Negative accel data.
 * @param[in/out] accel_data_diff_mg  : accel data difference data between positive and negative in mg.
 * @param[in] dev  : Structure instance of bma4_dev.
 *
 * @return Result of API execution status
 *
 *  @return Result of API execution status
 *  @retval 0 -> Success
 *  @retval < 0 -> Fail
 */
static int8_t get_accel_data_difference_and_validate(struct bma4_accel positive,
                                                     struct bma4_accel negative,
                                                     struct bma4_selftest_delta_limit *accel_data_diff_mg,
                                                     const struct bma4_dev *dev);

/*!
 * @brief This internal API saves the configurations before performing FOC.
 *
 * @param[out] acc_cfg      : Accelerometer configuration value
 * @param[out] aps          : Advance power mode value
 * @param[out] acc_en       : Accelerometer enable value
 * @param[in] dev           : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval BMA4_OK - Success.
 * @retval BMA4_E_COM_FAIL - Error: Communication fail
 * @retval BMA4_E_INVALID_SENSOR - Error: Invalid sensor
 */
static int8_t save_accel_foc_config(struct bma4_accel_config *acc_cfg,
                                    uint8_t *aps,
                                    uint8_t *acc_en,
                                    struct bma4_dev *dev);

/*!
 * @brief This internal API sets configurations for performing accelerometer FOC.
 *
 * @param[in] dev       : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval BMA4_OK - Success.
 * @retval BMA4_E_COM_FAIL - Error: Communication fail
 * @retval BMA4_E_INVALID_SENSOR - Error: Invalid sensor
 */
static int8_t set_accel_foc_config(struct bma4_dev *dev);

/*!
 * @brief This internal API enables/disables the offset compensation for
 * filtered and un-filtered accelerometer data.
 *
 * @param[in] offset_en     : Enables/Disables offset compensation.
 * @param[in] dev           : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval BMA4_OK - Success.
 * @retval BMA4_E_COM_FAIL - Error: Communication fail
 */
static int8_t set_bma4_accel_offset_comp(uint8_t offset_en, struct bma4_dev *dev);

/*!
 * @brief This internal API performs Fast Offset Compensation for accelerometer.
 *
 * @param[in] accel_g_value : This parameter selects the accel FOC
 * axis to be performed
 *
 * Input format is {x, y, z, sign}. '1' to enable. '0' to disable
 *
 * Eg:- To choose x axis  {1, 0, 0, 0}
 * Eg:- To choose -x axis {1, 0, 0, 1}
 *
 * @param[in] acc_cfg       : Accelerometer configuration value
 * @param[in] dev           : Structure instance of bma4_dev.
 *
 * @return Result of API execution status
 *
 * @retval BMA4_OK - Success.
 * @retval BMA4_E_NULL_PTR - Error: Null pointer error
 * @retval BMA4_E_COM_FAIL - Error: Communication fail
 */
static int8_t perform_accel_foc(const struct bma4_accel_foc_g_value *accel_g_value,
                                const struct bma4_accel_config *acc_cfg,
                                struct bma4_dev *dev);

/*!
 * @brief This internal API converts the range value into accelerometer
 * corresponding integer value.
 *
 * @param[in] range_in      : Input range value.
 * @param[out] range_out    : Stores the integer value of range.
 *
 * @return None
 * @retval None
 */
static void map_accel_range(uint8_t range_in, uint8_t *range_out);

/*!
 * @brief This internal API compensate the accelerometer data against gravity.
 *
 * @param[in] lsb_per_g     : LSB value per 1g.
 * @param[in] g_val         : Gravity reference value of all axes.
 * @param[in] data          : Accelerometer data
 * @param[out] comp_data    : Stores the data that is compensated by taking the
 *                            difference in accelerometer data and lsb_per_g
 *                            value.
 *
 * @return None
 * @retval None
 */
static void comp_for_gravity(uint16_t lsb_per_g,
                             const struct bma4_accel_foc_g_value *g_val,
                             const struct bma4_accel *data,
                             struct bma4_offset_delta *comp_data);

/*!
 * @brief This internal API scales the compensated accelerometer data according
 * to the offset register resolution.
 *
 * @param[in] range         : Gravity range of the accelerometer.
 * @param[out] comp_data    : Data that is compensated by taking the
 *                            difference in accelerometer data and lsb_per_g
 *                            value.
 * @param[out] data         : Stores offset data
 * @param[in] resolution    : Resolution of bma4 sensor
 *
 * @return None
 * @retval None
 */
static void scale_bma4_accel_offset(uint8_t range,
                                    const struct bma4_offset_delta *comp_data,
                                    struct bma4_accel_offset *data,
                                    uint8_t resolution);

/*!
 * @brief This internal API inverts the accelerometer offset data.
 *
 * @param[out] offset_data  : Stores the inverted offset data
 *
 * @return None
 * @retval None
 */
static void invert_bma4_accel_offset(struct bma4_accel_offset *offset_data);

/*!
 * @brief This internal API writes the offset data in the offset compensation
 * register.
 *
 * @param[in] offset        : Offset data
 * @param[in] dev           : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval BMA4_OK - Success.
 * @retval BMA4_E_COM_FAIL - Error: Communication fail
 */
static int8_t write_bma4_accel_offset(const struct bma4_accel_offset *offset, struct bma4_dev *dev);

/*!
 * @brief This internal API finds the bit position of 3.9mg according to given
 * range and resolution.
 *
 * @param[in] range        : Gravity range of the accelerometer.
 * @param[in] resolution   : Resolution of sensor
 *
 * @return Result of API execution status
 * @retval Bit position of 3.9mg
 */
static int8_t get_bit_pos_3_9mg(uint8_t range, uint8_t resolution);

/*!
 * @brief This internal API restores the configurations saved before performing
 * accelerometer FOC.
 *
 * @param[in] acc_cfg       : Accelerometer configuration value
 * @param[in] aps           : Advance power mode value
 * @param[in] acc_en        : Accelerometer enable value
 * @param[in] dev           : Structure instance of bma4_dev
 *
 * @return Result of API execution status
 * @retval BMA4_OK - Success.
 * @retval BMA4_E_COM_FAIL - Error: Communication fail
 * @retval BMA4_E_INVALID_SENSOR - Error: Invalid sensor
 * @retval BMA4_E_SET_APS_FAIL - Error: Set Advance Power Save Fail
 */
static int8_t restore_accel_foc_config(const struct bma4_accel_config *acc_cfg,
                                       uint8_t aps,
                                       uint8_t acc_en,
                                       struct bma4_dev *dev);

/***************************************************************************/

/**\name        Extern Declarations
 ****************************************************************************/

/***************************************************************************/

/**\name        Globals
 ****************************************************************************/

/***************************************************************************/

/**\name        Function definitions
 ****************************************************************************/

/*!
 *  @brief This API is the entry point.
 *  Call this API before using all other APIs.
 *  This API reads the chip-id of the sensor which is the first step to
 *  verify the sensor and also it configures the read mechanism of SPI and
 *  I2C interface.
 */
int8_t bma4_init(struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;
    uint8_t dummy_read = 0;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        if (dev->intf == BMA4_SPI_INTF)
        {
            dev->dummy_byte = 1;
            rslt = bma4_read_regs(BMA4_CHIP_ID_ADDR, &dummy_read, 1, dev);
        }
        else
        {
            dev->dummy_byte = 0;
        }

        if (rslt == BMA4_OK)
        {
            rslt = bma4_read_regs(BMA4_CHIP_ID_ADDR, &data, 1, dev);
            if (rslt == BMA4_OK)
            {
                /* Assign Chip Id */
                dev->chip_id = data;
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API is used to write the binary configuration in the sensor
 */
int8_t bma4_write_config_file(struct bma4_dev *dev)
{
    int8_t rslt;

    /* Config loading disable*/
    uint8_t config_load = 0;
    uint16_t index = 0;
    uint8_t config_stream_status = 0;

    /* Disable advanced power save */
    rslt = bma4_set_advance_power_save(BMA4_DISABLE, dev);

    if (rslt == BMA4_OK)
    {
        /* Disable config loading*/
        rslt = bma4_write_regs(BMA4_INIT_CTRL_ADDR, &config_load, 1, dev);

        if (rslt == BMA4_OK)
        {
            /* Write the config stream */
            for (index = 0; index < dev->config_size; index += dev->read_write_len)
            {
                rslt = stream_transfer_write((dev->config_file_ptr + index), index, dev);
            }

            if (rslt == BMA4_OK)
            {
                /* Enable config loading and FIFO mode */
                config_load = 0x01;
                rslt = bma4_write_regs(BMA4_INIT_CTRL_ADDR, &config_load, 1, dev);

                if (rslt == BMA4_OK)
                {
                    /* Wait till ASIC is initialized. Refer the data-sheet for
                     * more information
                     */
                    dev->delay_us(BMA4_MS_TO_US(150), dev->intf_ptr);

                    /* Read the status of config stream operation */
                    rslt = bma4_read_regs(BMA4_INTERNAL_STAT, &config_stream_status, 1, dev);
                    config_stream_status = config_stream_status & BMA4_CONFIG_STREAM_MESSAGE_MSK;

                    if (rslt == BMA4_OK)
                    {
                        if (config_stream_status != BMA4_ASIC_INITIALIZED)
                        {
                            rslt = BMA4_E_CONFIG_STREAM_ERROR;
                        }
                        else
                        {
                            /* Enable advanced power save */
                            rslt = bma4_set_advance_power_save(BMA4_ENABLE, dev);
                            if (rslt == BMA4_OK)
                            {
                                rslt = get_feature_config_start_addr(dev);
                            }
                        }
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the write operation requested is for feature
 *  config or register write and accordingly writes the data in the sensor.
 */
int8_t bma4_write_regs(uint8_t addr, const uint8_t *data, uint32_t len, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t adv_pwr_save = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (data != NULL))
    {
        if (addr == BMA4_FEATURE_CONFIG_ADDR)
        {
            /* Disable APS if enabled before writing the feature
             * config register
             */
            rslt = bma4_get_advance_power_save(&adv_pwr_save, dev);
            if ((adv_pwr_save == BMA4_ENABLE) && (rslt == BMA4_OK))
            {
                rslt = bma4_set_advance_power_save(BMA4_DISABLE, dev);
            }

            if (((len % 2) == 0) && (len <= dev->feature_len) && (rslt == BMA4_OK))
            {
                if (dev->read_write_len < len)
                {
                    rslt = write_within_len_regs(data, len, dev);
                }
                else
                {
                    rslt = write_regs(BMA4_FEATURE_CONFIG_ADDR, data, len, dev);
                }
            }
            else
            {
                rslt = BMA4_E_RD_WR_LENGTH_INVALID;
            }

            if (rslt == BMA4_OK)
            {
                /* Enable APS if previously enabled */
                if (adv_pwr_save == BMA4_ENABLE)
                {
                    rslt = bma4_set_advance_power_save(BMA4_ENABLE, dev);
                }
            }
        }
        else
        {
            rslt = write_regs(addr, data, len, dev);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*! @cond DOXYGEN_SUPRESS */

/* Suppressing doxygen warnings triggered for same static function names present across various sensor variant
 * directories */

/*!
 *  @brief This API writes the 8-bit data to the given register
 *  in the sensor.
 */
static int8_t write_regs(uint8_t addr, const uint8_t *data, uint32_t len, struct bma4_dev *dev)
{
    int8_t rslt;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (data != NULL))
    {
        if (dev->intf == BMA4_SPI_INTF)
        {
            addr = addr & BMA4_SPI_WR_MASK;
        }

        /* write data in the register*/
        dev->intf_rslt = dev->bus_write(addr, data, len, dev->intf_ptr);

        if (dev->intf_rslt == BMA4_INTF_RET_SUCCESS)
        {
            /* After write operation 2us delay is required when
             * device operates in performance mode whereas
             * 450us is required when the device operates in suspend and low power mode.
             * NOTE: For more information refer datasheet */
            if (dev->perf_mode_status == BMA4_ENABLE)
            {
                dev->delay_us(2, dev->intf_ptr);
            }
            else
            {
                dev->delay_us(450, dev->intf_ptr);
            }
        }
        else
        {
            rslt = BMA4_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API writes the 8-bit data to the given register
 *  in the sensor for length less than read write length
 */
static int8_t write_within_len_regs(const uint8_t *data, uint32_t len, struct bma4_dev *dev)
{
    int8_t rslt;
    uint32_t loop_count;
    uint16_t overflow;
    uint16_t index;
    uint32_t i;

    /* Calculate the no of writes to be
      * performed according to the read/write
      * length
      */
    loop_count = len / dev->read_write_len;
    overflow = len % dev->read_write_len;
    index = 0;
    rslt = set_feature_config_start_addr(dev);

    if (rslt == BMA4_OK)
    {
        for (i = 0; i < loop_count; i++)
        {
            rslt = write_regs(BMA4_FEATURE_CONFIG_ADDR, data + index, dev->read_write_len, dev);
            if (rslt == BMA4_OK)
            {
                rslt = increment_feature_config_addr(dev);
                if (rslt == BMA4_OK)
                {
                    index = index + dev->read_write_len;
                }
            }
        }

        if ((overflow) && (rslt == BMA4_OK))
        {
            rslt = write_regs(BMA4_FEATURE_CONFIG_ADDR, data + index, overflow, dev);
        }

        if (rslt == BMA4_OK)
        {
            rslt = set_feature_config_start_addr(dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API sets the feature config. data start address in the sensor.
 */
static int8_t get_feature_config_start_addr(struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t asic_lsb = 0;
    uint8_t asic_msb = 0;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = read_regs(BMA4_RESERVED_REG_5B_ADDR, &asic_lsb, 1, dev);
        if (rslt == BMA4_OK)
        {
            rslt = read_regs(BMA4_RESERVED_REG_5C_ADDR, &asic_msb, 1, dev);
        }

        if (rslt == BMA4_OK)
        {
            /* Store asic info in dev structure */
            dev->asic_data.asic_lsb = asic_lsb & 0x0F;
            dev->asic_data.asic_msb = asic_msb;
        }
    }

    return rslt;
}

/*!
 *  @brief This API sets the feature config. data start address in the sensor.
 */
static int8_t set_feature_config_start_addr(struct bma4_dev *dev)
{
    int8_t rslt;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = write_regs(BMA4_RESERVED_REG_5B_ADDR, &dev->asic_data.asic_lsb, 1, dev);
        if (rslt == BMA4_OK)
        {
            rslt = write_regs(BMA4_RESERVED_REG_5C_ADDR, &dev->asic_data.asic_msb, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API increments the feature config. data address according to the user
 *  provided read/write length in the dev structure.
 */
static int8_t increment_feature_config_addr(struct bma4_dev *dev)
{
    int8_t rslt;
    uint16_t asic_addr;
    uint8_t asic_lsb = 0;
    uint8_t asic_msb = 0;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Read the asic address from the sensor */
        rslt = read_regs(BMA4_RESERVED_REG_5B_ADDR, &asic_lsb, 1, dev);
        if (rslt == BMA4_OK)
        {
            rslt = read_regs(BMA4_RESERVED_REG_5C_ADDR, &asic_msb, 1, dev);
        }
        else
        {
            rslt = BMA4_E_COM_FAIL;
        }

        if (rslt == BMA4_OK)
        {
            /* Get the asic address */
            asic_addr = (asic_msb << 4) | (asic_lsb & 0x0F);

            /* Sum the asic address with read/write length after converting from
             * byte to word
             */
            asic_addr = asic_addr + (dev->read_write_len / 2);

            /* Split the asic address */
            asic_lsb = asic_addr & 0x0F;
            asic_msb = (uint8_t)(asic_addr >> 4);

            /* Write the asic address in the sensor */
            rslt = write_regs(BMA4_RESERVED_REG_5B_ADDR, &asic_lsb, 1, dev);
            if (rslt == BMA4_OK)
            {
                rslt = write_regs(BMA4_RESERVED_REG_5C_ADDR, &asic_msb, 1, dev);
            }
        }
        else
        {
            rslt = BMA4_E_COM_FAIL;
        }
    }

    return rslt;
}

/*! @endcond */

/*!
 *  @brief This API checks whether the read operation requested is for feature
 *  or register read and accordingly reads the data from the sensor.
 */
int8_t bma4_read_regs(uint8_t addr, uint8_t *data, uint32_t len, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t adv_pwr_save = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (data != NULL))
    {
        if (addr == BMA4_FEATURE_CONFIG_ADDR)
        {
            /* Disable APS if enabled before reading the feature
             * config register
             */
            rslt = bma4_get_advance_power_save(&adv_pwr_save, dev);
            if (adv_pwr_save == BMA4_ENABLE)
            {
                rslt = bma4_set_advance_power_save(BMA4_DISABLE, dev);
            }

            if (((len % 2) == 0) && (len <= dev->feature_len) && (rslt == BMA4_OK))
            {
                if (dev->read_write_len < len)
                {
                    rslt = read_within_len_regs(data, len, dev);
                }
                else
                {
                    rslt = read_regs(BMA4_FEATURE_CONFIG_ADDR, data, len, dev);
                }
            }
            else
            {
                rslt = BMA4_E_RD_WR_LENGTH_INVALID;
            }

            if (rslt == BMA4_OK)
            {
                /* Enable APS if previously enabled */
                if (adv_pwr_save == BMA4_ENABLE)
                {
                    rslt = bma4_set_advance_power_save(BMA4_ENABLE, dev);
                }
            }
        }
        else
        {
            rslt = read_regs(addr, data, len, dev);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*! @cond DOXYGEN_SUPRESS */

/* Suppressing doxygen warnings triggered for same static function names present across various sensor variant
 * directories */

/*!
 *  @brief This API reads the 8-bit data from the given register
 *  in the sensor.
 */
static int8_t read_regs(uint8_t addr, uint8_t *data, uint32_t len, struct bma4_dev *dev)
{
    int8_t rslt;
    uint16_t indx;
    uint8_t temp_buff[BMA4_MAX_LEN];

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (data != NULL))
    {
        if (dev->intf == BMA4_SPI_INTF)
        {
            /* SPI mask added */
            addr = addr | BMA4_SPI_RD_MASK;
        }

        /* Read the data from the register */
        dev->intf_rslt = dev->bus_read(addr, temp_buff, (len + dev->dummy_byte), dev->intf_ptr);

        if (dev->intf_rslt == BMA4_INTF_RET_SUCCESS)
        {
            for (indx = 0; indx < len; indx++)
            {
                /* Parsing and storing the valid data */
                data[indx] = temp_buff[indx + dev->dummy_byte];
            }
        }
        else
        {
            rslt = BMA4_E_COM_FAIL;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the 8-bit data when length is less than read write length
 */
static int8_t read_within_len_regs(uint8_t *data, uint32_t len, struct bma4_dev *dev)
{
    uint32_t idx;
    int8_t rslt;
    uint32_t loop_count;
    uint16_t overflow;
    uint16_t index;

    /* Calculate the no of writes to be
     * performed according to the read/write
     * length
     */
    loop_count = len / dev->read_write_len;
    overflow = len % dev->read_write_len;
    index = 0;
    rslt = set_feature_config_start_addr(dev);

    if (rslt == BMA4_OK)
    {
        for (idx = 0; idx < loop_count; idx++)
        {
            rslt = read_regs(BMA4_FEATURE_CONFIG_ADDR, data + index, dev->read_write_len, dev);

            if (rslt == BMA4_OK)
            {
                rslt = increment_feature_config_addr(dev);

                if (rslt == BMA4_OK)
                {
                    index = index + dev->read_write_len;
                }
            }
        }

        if ((overflow) && (rslt == BMA4_OK))
        {
            rslt = read_regs(BMA4_FEATURE_CONFIG_ADDR, data + index, overflow, dev);
        }

        if (rslt == BMA4_OK)
        {
            rslt = set_feature_config_start_addr(dev);
        }
    }

    return rslt;
}

/*! @endcond */

/*!
 *  @brief This API reads the error status from the sensor.
 */
int8_t bma4_get_error_status(struct bma4_err_reg *err_reg, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (err_reg != NULL))
    {
        /* Read the error codes*/
        rslt = bma4_read_regs(BMA4_ERROR_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            /* Fatal error*/
            err_reg->fatal_err = BMA4_GET_BITS_POS_0(data, BMA4_FATAL_ERR);

            /* Cmd error*/
            err_reg->cmd_err = BMA4_GET_BITSLICE(data, BMA4_CMD_ERR);

            /* User error*/
            err_reg->err_code = BMA4_GET_BITSLICE(data, BMA4_ERR_CODE);

            /* FIFO error*/
            err_reg->fifo_err = BMA4_GET_BITSLICE(data, BMA4_FIFO_ERR);

            /* Mag data ready error*/
            err_reg->aux_err = BMA4_GET_BITSLICE(data, BMA4_AUX_ERR);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the Accel data for x,y and z axis from the sensor.
 *  The data units is in LSB format.
 */
int8_t bma4_read_accel_xyz(struct bma4_accel *accel, struct bma4_dev *dev)
{
    int8_t rslt;
    uint16_t lsb = 0;
    uint16_t msb = 0;
    uint8_t data[BMA4_ACCEL_DATA_LENGTH] = { 0 };

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (accel != NULL))
    {
        rslt = bma4_read_regs(BMA4_DATA_8_ADDR, data, BMA4_ACCEL_DATA_LENGTH, dev);
        if (rslt == BMA4_OK)
        {
            msb = data[1];
            lsb = data[0];

            /* Accel data x axis */
            accel->x = (int16_t)((msb << 8) | lsb);
            msb = data[3];
            lsb = data[2];

            /* Accel data y axis */
            accel->y = (int16_t)((msb << 8) | lsb);
            msb = data[5];
            lsb = data[4];

            /* Accel data z axis */
            accel->z = (int16_t)((msb << 8) | lsb);
            if (dev->resolution == BMA4_12_BIT_RESOLUTION)
            {
                accel->x = (accel->x / 0x10);
                accel->y = (accel->y / 0x10);
                accel->z = (accel->z / 0x10);
            }
            else if (dev->resolution == BMA4_14_BIT_RESOLUTION)
            {
                accel->x = (accel->x / 0x04);
                accel->y = (accel->y / 0x04);
                accel->z = (accel->z / 0x04);
            }

            /* Get the re-mapped accelerometer data */
            get_remapped_data(accel, dev);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the sensor time of Sensor time gets updated
 *  with every update of data register or FIFO.
 */
int8_t bma4_get_sensor_time(uint32_t *sensor_time, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data[BMA4_SENSOR_TIME_LENGTH] = { 0 };
    uint8_t msb = 0;
    uint8_t xlsb = 0;
    uint8_t lsb = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (sensor_time != NULL))
    {
        rslt = bma4_read_regs(BMA4_SENSORTIME_0_ADDR, data, BMA4_SENSOR_TIME_LENGTH, dev);
        if (rslt == BMA4_OK)
        {
            msb = data[BMA4_SENSOR_TIME_MSB_BYTE];
            xlsb = data[BMA4_SENSOR_TIME_XLSB_BYTE];
            lsb = data[BMA4_SENSOR_TIME_LSB_BYTE];
            *sensor_time = (uint32_t)((msb << 16) | (xlsb << 8) | lsb);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the chip temperature of sensor.
 *
 *  @note Using a scaling factor of 1000, to obtain integer values, which
 *  at the user end, are used to get accurate temperature value .
 *  BMA4_FAHREN_SCALED = 1.8 * 1000, BMA4_KELVIN_SCALED = 273.15 * 1000
 */
int8_t bma4_get_temperature(int32_t *temp, uint8_t temp_unit, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data[BMA4_TEMP_DATA_SIZE] = { 0 };
    int32_t temp_raw_scaled = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (temp != NULL))
    {
        /* Read temperature value from the register */
        rslt = bma4_read_regs(BMA4_TEMPERATURE_ADDR, data, BMA4_TEMP_DATA_SIZE, dev);
        if (rslt == BMA4_OK)
        {
            temp_raw_scaled = (int32_t)data[BMA4_TEMP_BYTE] * BMA4_SCALE_TEMP;
        }

        /* '0' value read from the register corresponds to 23 degree C */
        (*temp) = temp_raw_scaled + (BMA4_OFFSET_TEMP * BMA4_SCALE_TEMP);
        switch (temp_unit)
        {
            case BMA4_DEG:
                break;
            case BMA4_FAHREN:

                /* Temperature in degree Fahrenheit */
                (*temp) = (((*temp) / BMA4_SCALE_TEMP) * BMA4_FAHREN_SCALED) + (32 * BMA4_SCALE_TEMP);
                break;
            case BMA4_KELVIN:

                /* Temperature in degree Kelvin */
                (*temp) = (*temp) + BMA4_KELVIN_SCALED;
                break;
            default:
                break;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the Output data rate, Bandwidth, perf_mode
 *  and Range of accel.
 */
int8_t bma4_get_accel_config(struct bma4_accel_config *accel, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (accel != NULL))
    {
        rslt = bma4_read_regs(BMA4_ACCEL_CONFIG_ADDR, data, BMA4_ACCEL_CONFIG_LENGTH, dev);
        if (rslt == BMA4_OK)
        {
            /* To get the ODR */
            accel->odr = BMA4_GET_BITS_POS_0(data[0], BMA4_ACCEL_ODR);

            /* To get the bandwidth */
            accel->bandwidth = BMA4_GET_BITSLICE(data[0], BMA4_ACCEL_BW);

            /* To get the under sampling mode */
            accel->perf_mode = BMA4_GET_BITSLICE(data[0], BMA4_ACCEL_PERFMODE);

            /* Read the Accel range */
            accel->range = BMA4_GET_BITS_POS_0(data[1], BMA4_ACCEL_RANGE);

            /* Flag bit to store the performance mode status */
            dev->perf_mode_status = accel->perf_mode;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the output_data_rate, bandwidth, perf_mode
 *  and range of Accel.
 */
int8_t bma4_set_accel_config(const struct bma4_accel_config *accel, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t accel_config_data[2] = { 0, 0 };

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (accel != NULL))
    {
        /* Check whether the bandwidth , odr and perf_mode combinations are valid */
        rslt = validate_odr_bandwidth_perfmode(accel->odr, accel->bandwidth, accel->perf_mode);

        if (rslt == BMA4_OK)
        {
            accel_config_data[0] = accel->odr & BMA4_ACCEL_ODR_MSK;
            accel_config_data[0] |= (uint8_t)(accel->bandwidth << BMA4_ACCEL_BW_POS);
            accel_config_data[0] |= (uint8_t)(accel->perf_mode << BMA4_ACCEL_PERFMODE_POS);
            accel_config_data[1] = accel->range & BMA4_ACCEL_RANGE_MSK;

            rslt = bma4_write_regs(BMA4_ACCEL_RANGE_ADDR, &accel_config_data[1], 1, dev);

            if (rslt == BMA4_OK)
            {
                rslt = bma4_write_regs(BMA4_ACCEL_CONFIG_ADDR, &accel_config_data[0], 1, dev);

                /* Flag bit to store the performance mode status */
                dev->perf_mode_status = ((accel_config_data[0] & BMA4_ACCEL_PERFMODE_MSK) >> BMA4_ACCEL_PERFMODE_POS);
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*! @cond DOXYGEN_SUPRESS */

/* Suppressing doxygen warnings triggered for same static function names present across various sensor variant
 * directories */

/*!
 *  @brief This API validates the bandwidth, odr and perf_mode
 *  combinations set by the user.
 */
static int8_t validate_odr_bandwidth_perfmode(uint8_t odr, uint8_t bandwidth, uint8_t perf_mode)
{
    int8_t rslt;

    /* Check bandwidth and perf_mode combinations */
    rslt = validate_bandwidth_perfmode(bandwidth, perf_mode);

    if (rslt == BMA4_OK)
    {
        /* Check ODR validity */
        rslt = validate_odr(odr);
    }

    if (rslt == BMA4_OK)
    {
        /* Validate ODR and bandwidth combinations when perf_mode = BMA4_CIC_AVG_MODE */
        if (perf_mode == BMA4_CIC_AVG_MODE)
        {
            rslt = validate_bandwidth_odr(bandwidth, odr);
        }
    }

    return rslt;

}

/*!
 *  @brief This API validates the bandwidth and perf_mode
 *  value set by the user.
 */
static int8_t validate_bandwidth_perfmode(uint8_t bandwidth, uint8_t perf_mode)
{
    int8_t rslt = BMA4_OK;

    if (perf_mode == BMA4_CONTINUOUS_MODE)
    {
        if (bandwidth > BMA4_ACCEL_NORMAL_AVG4)
        {
            /* Invalid bandwidth error for continuous mode */
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }
    else if (perf_mode == BMA4_CIC_AVG_MODE)
    {
        if (bandwidth > BMA4_ACCEL_RES_AVG128)
        {
            /* Invalid bandwidth error for CIC avg. mode */
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }
    else
    {
        rslt = BMA4_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 *  @brief This API validates the ODR value set by the user.
 */
static int8_t validate_odr(uint8_t odr)
{
    int8_t rslt = BMA4_OK;

    if ((odr < BMA4_OUTPUT_DATA_RATE_0_78HZ) || (odr > BMA4_OUTPUT_DATA_RATE_1600HZ))
    {
        /* If odr is not valid return error */
        rslt = BMA4_E_OUT_OF_RANGE;
    }

    return rslt;
}

/*!
 *  @brief This API validates the bandwidth and odr when perf_mode = BMA4_CIC_AVG_MODE
 */
static int8_t validate_bandwidth_odr(uint8_t bandwidth, uint8_t odr)
{
    int8_t rslt = BMA4_OK;
    uint8_t index;

    /*
     * Array contains ODR values
     * From BMA4_OUTPUT_DATA_RATE_0_78HZ (value = 1)
     * To BMA4_OUTPUT_DATA_RATE_400HZ (value = 10)
     */
    uint8_t accel_odr[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

    /*
     * Array contains valid bandwidth values for each ODR
     */
    uint8_t valid_bw[10] = { 7, 7, 7, 7, 6, 5, 4, 3, 2, 1 };

    /*
     * For ODR - 800Hz and 1600Hz, none of the bandwidth is applicable
     */
    if ((odr == BMA4_OUTPUT_DATA_RATE_800HZ) || (odr == BMA4_OUTPUT_DATA_RATE_1600HZ))
    {
        rslt = BMA4_E_AVG_MODE_INVALID_CONF;
    }

    /*
     * Maximum valid bandwidth value for accel_odr[index] is the
     * corresponding value in valid_bw[index]
     */
    for (index = 0; index < 10; index++)
    {
        if (odr == accel_odr[index])
        {
            if (bandwidth > valid_bw[index])
            {
                rslt = BMA4_E_AVG_MODE_INVALID_CONF;
            }
        }
    }

    return rslt;
}

/*! @endcond */

/*!
 *  @brief This API sets the advance power save mode in the sensor.
 */
int8_t bma4_set_advance_power_save(uint8_t adv_pwr_save, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = bma4_read_regs(BMA4_POWER_CONF_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            data = BMA4_SET_BITS_POS_0(data, BMA4_ADVANCE_POWER_SAVE, adv_pwr_save);
            rslt = bma4_write_regs(BMA4_POWER_CONF_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the status of advance power save mode
 *  from the sensor.
 */
int8_t bma4_get_advance_power_save(uint8_t *adv_pwr_save, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (adv_pwr_save != NULL))
    {
        rslt = bma4_read_regs(BMA4_POWER_CONF_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *adv_pwr_save = BMA4_GET_BITS_POS_0(data, BMA4_ADVANCE_POWER_SAVE);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the FIFO self wake up functionality in the sensor.
 */
int8_t bma4_set_fifo_self_wakeup(uint8_t fifo_self_wakeup, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = bma4_read_regs(BMA4_POWER_CONF_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            data = BMA4_SET_BITSLICE(data, BMA4_FIFO_SELF_WAKE_UP, fifo_self_wakeup);
            rslt = bma4_write_regs(BMA4_POWER_CONF_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API gets the status of FIFO self wake up functionality from
 *  the sensor.
 */
int8_t bma4_get_fifo_self_wakeup(uint8_t *fifo_self_wake_up, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (fifo_self_wake_up != NULL))
    {
        rslt = bma4_read_regs(BMA4_POWER_CONF_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *fifo_self_wake_up = BMA4_GET_BITSLICE(data, BMA4_FIFO_SELF_WAKE_UP);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API enables or disables the Accel in the sensor.
 */
int8_t bma4_set_accel_enable(uint8_t accel_en, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = bma4_read_regs(BMA4_POWER_CTRL_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            data = BMA4_SET_BITSLICE(data, BMA4_ACCEL_ENABLE, accel_en);

            rslt = bma4_write_regs(BMA4_POWER_CTRL_ADDR, &data, 1, dev);
            dev->delay_us(25000, dev->intf_ptr);
        }
    }

    return rslt;
}

/*!
 *  @brief This API checks whether Accel is enabled or not in the sensor.
 */
int8_t bma4_get_accel_enable(uint8_t *accel_en, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (accel_en != NULL))
    {
        rslt = bma4_read_regs(BMA4_POWER_CTRL_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *accel_en = BMA4_GET_BITSLICE(data, BMA4_ACCEL_ENABLE);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API is used to enable or disable auxiliary Mag
 *  in the sensor.
 */
int8_t bma4_set_mag_enable(uint8_t mag_en, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = bma4_read_regs(BMA4_POWER_CTRL_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            data = BMA4_SET_BITS_POS_0(data, BMA4_MAG_ENABLE, mag_en);
            rslt = bma4_write_regs(BMA4_POWER_CTRL_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API is used to check whether the auxiliary Mag is enabled
 *  or not in the sensor.
 */
int8_t bma4_get_mag_enable(uint8_t *mag_en, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (mag_en != NULL))
    {
        rslt = bma4_read_regs(BMA4_POWER_CTRL_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *mag_en = BMA4_GET_BITS_POS_0(data, BMA4_MAG_ENABLE);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the SPI interface mode which is set for primary
 *  interface.
 */
int8_t bma4_get_spi_interface(uint8_t *spi, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (spi != NULL))
    {
        /* Read SPI mode */
        rslt = bma4_read_regs(BMA4_IF_CONFIG_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *spi = BMA4_GET_BITS_POS_0(data, BMA4_CONFIG_SPI3);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API configures the SPI interface Mode for primary interface
 */
int8_t bma4_set_spi_interface(uint8_t spi, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        if (spi <= BMA4_MAX_VALUE_SPI3)
        {
            /* Write SPI mode */
            rslt = bma4_read_regs(BMA4_IF_CONFIG_ADDR, &data, 1, dev);
            if (rslt == BMA4_OK)
            {
                data = BMA4_SET_BITS_POS_0(data, BMA4_CONFIG_SPI3, spi);
                rslt = bma4_write_regs(BMA4_IF_CONFIG_ADDR, &data, 1, dev);
            }
        }
        else
        {
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

/*!
 *  @brief This API writes the available sensor specific commands
 *  to the sensor.
 */
int8_t bma4_set_command_register(uint8_t command_reg, struct bma4_dev *dev)
{
    int8_t rslt;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Write command register */
        rslt = bma4_write_regs(BMA4_CMD_ADDR, &command_reg, 1, dev);
    }

    return rslt;
}

/*!
 * @brief This API sets the I2C device address of auxiliary sensor
 */
int8_t bma4_set_i2c_device_addr(struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0, dev_id = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Write the auxiliary I2C device address */
        rslt = bma4_read_regs(BMA4_AUX_DEV_ID_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            dev_id = BMA4_SET_BITSLICE(data, BMA4_I2C_DEVICE_ADDR, dev->aux_config.aux_dev_addr);
            rslt = bma4_write_regs(BMA4_AUX_DEV_ID_ADDR, &dev_id, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API sets the register access on MAG_IF[2], MAG_IF[3],
 *  MAG_IF[4] in the sensor. This implies that the DATA registers are
 *  not updated with Mag values automatically.
 */
int8_t bma4_set_mag_manual_enable(uint8_t mag_manual, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Write the Mag manual*/
        rslt = bma4_read_regs(BMA4_AUX_IF_CONF_ADDR, &data, 1, dev);
        dev->delay_us(BMA4_GEN_READ_WRITE_DELAY, dev->intf_ptr);

        if (rslt == BMA4_OK)
        {
            /* Set the bit of Mag manual enable */
            data = BMA4_SET_BITSLICE(data, BMA4_MAG_MANUAL_ENABLE, mag_manual);
            rslt = bma4_write_regs(BMA4_AUX_IF_CONF_ADDR, &data, 1, dev);
            if (rslt == BMA4_OK)
            {
                dev->aux_config.manual_enable = (uint8_t)mag_manual;
            }
        }
        else
        {
            /*dev->mag_manual_enable = 0;*/
            dev->aux_config.manual_enable = 0;
        }
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the Mag access is done manually or
 *  automatically in the sensor.
 *  If the Mag access is done through manual mode then Mag data registers
 *  in sensor are not updated automatically.
 */
int8_t bma4_get_mag_manual_enable(uint8_t *mag_manual, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (mag_manual != NULL))
    {
        /* Read Mag manual */
        rslt = bma4_read_regs(BMA4_AUX_IF_CONF_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *mag_manual = BMA4_GET_BITSLICE(data, BMA4_MAG_MANUAL_ENABLE);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the I2C interface configuration(if) mode
 *  for auxiliary Mag.
 */
int8_t bma4_set_aux_if_mode(uint8_t if_mode, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = bma4_read_regs(BMA4_IF_CONFIG_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            data = BMA4_SET_BITSLICE(data, BMA4_IF_CONFIG_IF_MODE, if_mode);
            rslt = bma4_write_regs(BMA4_IF_CONFIG_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API gets the address of the register of Aux Mag sensor
 *  where the data to be read.
 */
int8_t bma4_get_mag_read_addr(uint8_t *mag_read_addr, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (mag_read_addr != NULL))
    {
        rslt = bma4_read_regs(BMA4_AUX_RD_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *mag_read_addr = BMA4_GET_BITS_POS_0(data, BMA4_READ_ADDR);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the address of the register of Aux Mag sensor
 *  where the data to be read.
 */
int8_t bma4_set_mag_read_addr(uint8_t mag_read_addr, struct bma4_dev *dev)
{
    int8_t rslt;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Write the Mag read address*/
        rslt = bma4_write_regs(BMA4_AUX_RD_ADDR, &mag_read_addr, 1, dev);
    }

    return rslt;
}

/*!
 *  @brief This API gets the Aux Mag write address from the sensor.
 *  Mag write address is where the Mag data will be written.
 */
int8_t bma4_get_mag_write_addr(uint8_t *mag_write_addr, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (mag_write_addr != NULL))
    {
        rslt = bma4_read_regs(BMA4_AUX_WR_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *mag_write_addr = BMA4_GET_BITS_POS_0(data, BMA4_WRITE_ADDR);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the Aux Mag write address in the sensor.
 *  Mag write address is where the Mag data will be written.
 */
int8_t bma4_set_mag_write_addr(uint8_t mag_write_addr, struct bma4_dev *dev)
{
    int8_t rslt;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = bma4_write_regs(BMA4_AUX_WR_ADDR, &mag_write_addr, 1, dev);
    }

    return rslt;
}

/*!
 *  @brief This API reads the data from the sensor which is written to the
 *  Mag.
 */
int8_t bma4_get_mag_write_data(uint8_t *mag_write_data, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (mag_write_data != NULL))
    {
        rslt = bma4_read_regs(BMA4_AUX_WR_DATA_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *mag_write_data = BMA4_GET_BITS_POS_0(data, BMA4_WRITE_DATA);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the data in the sensor which in turn will
 *  be written to Mag.
 */
int8_t bma4_set_mag_write_data(uint8_t mag_write_data, struct bma4_dev *dev)
{
    int8_t rslt;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = bma4_write_regs(BMA4_AUX_WR_DATA_ADDR, &mag_write_data, 1, dev);
    }

    return rslt;
}

/*!
 *  @brief This API reads the x,y,z and r axis data from the auxiliary
 *  Mag BMM150/AKM9916 sensor.
 */
int8_t bma4_read_mag_xyzr(struct bma4_mag_xyzr *mag, struct bma4_dev *dev)
{
    int8_t rslt;
    uint16_t msb = 0;
    uint16_t lsb = 0;
    uint8_t data[BMA4_MAG_XYZR_DATA_LENGTH] = { 0 };

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (mag != NULL))
    {
        rslt = bma4_read_regs(BMA4_DATA_0_ADDR, data, BMA4_MAG_XYZR_DATA_LENGTH, dev);
        if (rslt == BMA4_OK)
        {
            /* Data X */
            /* X-axis LSB value shifting */
            lsb = BMA4_GET_BITSLICE(data[BMA4_MAG_X_LSB_BYTE], BMA4_DATA_MAG_X_LSB);
            msb = data[BMA4_MAG_X_MSB_BYTE];
            mag->x = (int16_t)((msb << 8) | lsb);
            mag->x = (mag->x / 0x08);

            /* Data Y */
            /* Y-axis LSB value shifting */
            lsb = BMA4_GET_BITSLICE(data[BMA4_MAG_Y_LSB_BYTE], BMA4_DATA_MAG_Y_LSB);
            msb = data[BMA4_MAG_Y_MSB_BYTE];
            mag->y = (int16_t)((msb << 8) | lsb);
            mag->y = (mag->y / 0x08);

            /* Data Z */
            /* Z-axis LSB value shifting */
            lsb = BMA4_GET_BITSLICE(data[BMA4_MAG_Z_LSB_BYTE], BMA4_DATA_MAG_Z_LSB);
            msb = data[BMA4_MAG_Z_MSB_BYTE];
            mag->z = (int16_t)((msb << 8) | lsb);
            mag->z = (mag->z / 0x02);

            /* RHall */
            /* R-axis LSB value shifting */
            lsb = BMA4_GET_BITSLICE(data[BMA4_MAG_R_LSB_BYTE], BMA4_DATA_MAG_R_LSB);
            msb = data[BMA4_MAG_R_MSB_BYTE];
            mag->r = (int16_t)((msb << 8) | lsb);
            mag->r = (mag->r / 0x04);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the burst data length (1,2,6,8 byte) of auxiliary
 *  Mag sensor.
 */
int8_t bma4_set_mag_burst(uint8_t mag_burst, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Write auxiliary burst mode length*/
        rslt = bma4_read_regs(BMA4_AUX_IF_CONF_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            data = BMA4_SET_BITS_POS_0(data, BMA4_MAG_BURST, mag_burst);
            rslt = bma4_write_regs(BMA4_AUX_IF_CONF_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the burst data length of Mag set in the sensor.
 */
int8_t bma4_get_mag_burst(uint8_t *mag_burst, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (mag_burst != NULL))
    {
        /* Write Mag burst mode length*/
        rslt = bma4_read_regs(BMA4_AUX_IF_CONF_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *mag_burst = BMA4_GET_BITS_POS_0(data, BMA4_MAG_BURST);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the FIFO data of Accel and/or Mag sensor
 *  NOTE : Dummy byte (for SPI Interface) required for FIFO data read
 *  must be given as part of data pointer in struct bma4_fifo_frame
 */
int8_t bma4_read_fifo_data(struct bma4_fifo_frame *fifo, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;
    uint8_t fifo_config_1 = 0;
    uint16_t fifo_length = 0;
    uint8_t addr = BMA4_FIFO_DATA_ADDR;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (fifo != NULL))
    {
        /* reset the fifo data structure */
        reset_fifo_data_structure(fifo);

        if (dev->intf == BMA4_SPI_INTF)
        {
            /* SPI mask added */
            addr = addr | BMA4_SPI_RD_MASK;
        }

        /* Read available fifo length */
        rslt = bma4_get_fifo_length(&fifo_length, dev);

        if (rslt == BMA4_OK)
        {
            /* Read FIFO header enable configuration */
            rslt = bma4_read_regs(BMA4_FIFO_CONFIG_1_ADDR, &fifo_config_1, 1, dev);

            if (rslt == BMA4_OK)
            {
                /* If fifo header is enabled, add sensortime overhead byte to total FIFO length */
                if ((fifo_config_1 & BMA4_FIFO_HEADER))
                {
                    /* This update will take care of dummy byte necessity based on interface selection */
                    fifo->length = (fifo_length + BMA4_SENSORTIME_OVERHEAD_BYTE + dev->dummy_byte);
                }
                /* If fifo header is enabled, exclude addition of sensortime overhead byte to total FIFO length */
                else
                {
                    /* This update will take care of dummy byte necessity based on interface selection */
                    fifo->length = (fifo_length + dev->dummy_byte);
                }
            }

            /* Read FIFO data */
            dev->intf_rslt = dev->bus_read(addr, fifo->data, (uint32_t)fifo->length, dev->intf_ptr);

            /* If interface read fails, update rslt variable with communication failure */
            if (dev->intf_rslt != BMA4_INTF_RET_SUCCESS)
            {
                rslt = BMA4_E_COM_FAIL;
            }
        }

        if (rslt == BMA4_OK)
        {
            /* read fifo frame content configuration*/
            rslt = bma4_read_regs(BMA4_FIFO_CONFIG_1_ADDR, &data, 1, dev);

            if (rslt == BMA4_OK)
            {
                /* filter fifo header enabled status */
                fifo->fifo_header_enable = data & BMA4_FIFO_HEADER;

                /* filter accel/mag data enabled status */
                fifo->fifo_data_enable = data & BMA4_FIFO_M_A_ENABLE;
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API parses and extracts the accelerometer frames from
 *  FIFO data read by the "bma4_read_fifo_data" API and stores it in the
 *  "accel_data" structure instance.
 */
int8_t bma4_extract_accel(struct bma4_accel *accel_data,
                          uint16_t *accel_length,
                          struct bma4_fifo_frame *fifo,
                          const struct bma4_dev *dev)
{
    int8_t rslt;
    uint16_t data_index = 0;
    uint16_t accel_index = 0;
    uint16_t data_read_length = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (accel_data != NULL) && (accel_length != NULL) && (fifo != NULL))
    {
        /* Check if this is the first iteration of data unpacking
         * if yes, then consider dummy byte on SPI
         */
        if (fifo->accel_byte_start_idx == 0)
        {
            /* Dummy byte included */
            fifo->accel_byte_start_idx = dev->dummy_byte;
        }

        /* Parsing the FIFO data in header-less mode */
        if (fifo->fifo_header_enable == 0)
        {
            get_accel_len_to_parse(&data_index, &data_read_length, accel_length, fifo);

            for (; data_index < data_read_length;)
            {
                unpack_acc_frm(accel_data, &data_index, &accel_index, fifo->fifo_data_enable, fifo, dev);

                /* Checks for the availability of set of FIFO data */
                check_empty_fifo(&data_index, fifo);
            }

            /* update number of accel data read*/
            *accel_length = accel_index;

            /*update the accel byte index*/
            fifo->accel_byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the FIFO data in header mode */
            extract_accel_header_mode(accel_data, accel_length, fifo, dev);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API parses and extracts the magnetometer frames from
 *  FIFO data read by the "bma4_read_fifo_data" API and stores it in the
 *  "mag_data" structure instance parameter of this API
 */
int8_t bma4_extract_mag(const struct bma4_mag *mag_data,
                        uint16_t *mag_length,
                        struct bma4_fifo_frame *fifo,
                        const struct bma4_dev *dev)
{
    int8_t rslt;
    uint16_t data_index = 0;
    uint16_t mag_index = 0;
    uint16_t data_read_length = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (mag_data != NULL) && (mag_length != NULL) && (fifo != NULL))
    {
        /* Check if this is the first iteration of data unpacking
         * if yes, then consider dummy byte on SPI
         */
        if (fifo->mag_byte_start_idx == 0)
        {
            /* Dummy byte included */
            fifo->mag_byte_start_idx = dev->dummy_byte;
        }

        /* Parsing the FIFO data in header-less mode */
        if (fifo->fifo_header_enable == 0)
        {
            get_mag_len_to_parse(&data_index, &data_read_length, mag_length, fifo);
            for (; data_index < data_read_length;)
            {
                rslt = unpack_mag_frm(mag_data, &data_index, &mag_index, fifo->fifo_data_enable, fifo, dev);

                /*Check for the availability of next
                 * two bytes of FIFO data
                 */
                check_empty_fifo(&data_index, fifo);
            }

            /* update number of Aux. sensor data read*/
            *mag_length = mag_index;

            /*update the Aux. sensor frame index*/
            fifo->mag_byte_start_idx = data_index;
        }
        else
        {
            /* Parsing the FIFO data in header mode */
            rslt = extract_mag_header_mode(mag_data, mag_length, fifo, dev);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the FIFO water mark level which is set
 *  in the sensor.
 */
int8_t bma4_get_fifo_wm(uint16_t *fifo_wm, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0, 0 };

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (fifo_wm != NULL))
    {
        /* Read the FIFO water mark level*/
        rslt = bma4_read_regs(BMA4_FIFO_WTM_0_ADDR, data, BMA4_FIFO_WM_LENGTH, dev);
        if (rslt == BMA4_OK)
        {
            *fifo_wm = (data[1] << 8) | (data[0]);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the FIFO watermark level in the sensor.
 */
int8_t bma4_set_fifo_wm(uint16_t fifo_wm, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0, 0 };

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        data[0] = BMA4_GET_LSB(fifo_wm);
        data[1] = BMA4_GET_MSB(fifo_wm);

        /* consecutive write is not possible in suspend mode hence
         * separate write is used with delay of 1 ms
         */

        /* Write the fifo watermark level*/
        rslt = bma4_write_regs(BMA4_FIFO_WTM_0_ADDR, &data[0], 1, dev);

        if (rslt == BMA4_OK)
        {
            dev->delay_us(BMA4_GEN_READ_WRITE_DELAY, dev->intf_ptr);
            rslt = bma4_write_regs((BMA4_FIFO_WTM_0_ADDR + 1), &data[1], 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the Accel FIFO data is set for filtered
 *  or unfiltered mode.
 */
int8_t bma4_get_accel_fifo_filter_data(uint8_t *accel_fifo_filter, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (accel_fifo_filter != NULL))
    {
        /* Read the Accel FIFO filter data */
        rslt = bma4_read_regs(BMA4_FIFO_DOWN_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *accel_fifo_filter = BMA4_GET_BITSLICE(data, BMA4_FIFO_FILTER_ACCEL);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the condition of Accel FIFO data either to
 *  filtered or unfiltered mode.
 */
int8_t bma4_set_accel_fifo_filter_data(uint8_t accel_fifo_filter, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        if (accel_fifo_filter <= BMA4_MAX_VALUE_FIFO_FILTER)
        {
            rslt = bma4_read_regs(BMA4_FIFO_DOWN_ADDR, &data, 1, dev);
            if (rslt == BMA4_OK)
            {
                /* Write Accel FIFO filter data */
                data = BMA4_SET_BITSLICE(data, BMA4_FIFO_FILTER_ACCEL, accel_fifo_filter);
                rslt = bma4_write_regs(BMA4_FIFO_DOWN_ADDR, &data, 1, dev);
            }
        }
        else
        {
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the down sampling rates which is configured
 *  for Accel FIFO data.
 */
int8_t bma4_get_fifo_down_accel(uint8_t *fifo_down, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (fifo_down != NULL))
    {
        /* Read the Accel FIFO down data */
        rslt = bma4_read_regs(BMA4_FIFO_DOWN_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *fifo_down = BMA4_GET_BITSLICE(data, BMA4_FIFO_DOWN_ACCEL);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the down-sampling rates for Accel FIFO.
 */
int8_t bma4_set_fifo_down_accel(uint8_t fifo_down, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Write the Accel FIFO down data */
        rslt = bma4_read_regs(BMA4_FIFO_DOWN_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            data = BMA4_SET_BITSLICE(data, BMA4_FIFO_DOWN_ACCEL, fifo_down);
            rslt = bma4_write_regs(BMA4_FIFO_DOWN_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the length of FIFO data available in the sensor
 *  in the units of bytes.
 */
int8_t bma4_get_fifo_length(uint16_t *fifo_length, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t index = 0;
    uint8_t data[BMA4_FIFO_DATA_LENGTH] = { 0, 0 };

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (fifo_length != NULL))
    {
        /* Read FIFO length*/
        rslt = bma4_read_regs(BMA4_FIFO_LENGTH_0_ADDR, data, BMA4_FIFO_DATA_LENGTH, dev);
        if (rslt == BMA4_OK)
        {
            index = BMA4_FIFO_LENGTH_MSB_BYTE;
            data[index] = BMA4_GET_BITS_POS_0(data[index], BMA4_FIFO_BYTE_COUNTER_MSB);
            *fifo_length = ((data[index] << 8) | data[index - 1]);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API aligns and compensates the Mag data of BMM150/AKM9916
 *  sensor.
 */
int8_t bma4_second_if_mag_compensate_xyz(struct bma4_mag_fifo_data mag_fifo_data,
                                         uint8_t mag_second_if,
                                         const struct bma4_mag *compensated_mag_data)
{
    int8_t rslt = BMA4_OK;

#ifdef BMM150
    int16_t mag_x = 0;
    int16_t mag_y = 0;
    int16_t mag_z = 0;
    uint16_t mag_r = 0;
#else

    /* Suppress Warnings */
    (void)(mag_second_if);
    (void)(mag_fifo_data);
#endif

    if (compensated_mag_data == NULL)
    {
        rslt = BMA4_E_NULL_PTR;
    }

#if defined(BMM150) || defined(AKM9916)
    switch (mag_second_if)
    {
#ifdef BMM150
        case BMA4_SEC_IF_BMM150:

            /* X data*/
            mag_x = (int16_t)((mag_fifo_data.mag_x_msb << 8) | (mag_fifo_data.mag_x_lsb));
            mag_x = (int16_t) (mag_x / 0x08);

            /* Y data*/
            mag_y = (int16_t)((mag_fifo_data.mag_y_msb << 8) | (mag_fifo_data.mag_y_lsb));
            mag_y = (int16_t) (mag_y / 0x08);

            /* Z data*/
            mag_z = (int16_t)((mag_fifo_data.mag_z_msb << 8) | (mag_fifo_data.mag_z_lsb));
            mag_z = (int16_t) (mag_z / 0x02);

            /* R data*/
            mag_r = (uint16_t)((mag_fifo_data.mag_r_y2_msb << 8) | (mag_fifo_data.mag_r_y2_lsb));
            mag_r = (uint16_t) (mag_r >> 2);

            /* Compensated Mag x data */
            compensated_mag_data->x = bma4_bmm150_mag_compensate_x(mag_x, mag_r);

            /* Compensated Mag y data */
            compensated_mag_data->y = bma4_bmm150_mag_compensate_y(mag_y, mag_r);

            /* Compensated Mag z data */
            compensated_mag_data->z = bma4_bmm150_mag_compensate_z(mag_z, mag_r);
            break;
#endif

#ifdef AKM9916
        case BMA4_SEC_IF_AKM09916:

            /* Compensated X data */
            compensated_mag_data->x = (int16_t)((mag_fifo_data.mag_x_msb << 8) | (mag_fifo_data.mag_x_lsb));

            /* Compensated Y data*/
            compensated_mag_data->y = (int16_t)((mag_fifo_data.mag_y_msb << 8) | (mag_fifo_data.mag_y_lsb));

            /* Compensated Z data*/
            compensated_mag_data->z = (int16_t)((mag_fifo_data.mag_z_msb << 8) | (mag_fifo_data.mag_z_lsb));
            break;
#endif
    }
#endif

    return rslt;
}

/*!
 *  @brief This API reads Mag. x,y and z axis data from either BMM150 or
 *  AKM9916 sensor
 */
int8_t bma4_read_mag_xyz(const struct bma4_mag *mag, uint8_t sensor_select, const struct bma4_dev *dev)
{
    int8_t rslt;

#if defined(AKM9916) || defined(BMM150)
    uint8_t index;
    uint16_t msb = 0;
    uint16_t lsb = 0;
    uint8_t data[BMA4_MAG_XYZ_DATA_LENGTH] = { 0 };
#else

    /* Suppress Warnings */
    (void)(sensor_select);
#endif

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((BMA4_OK != rslt) || (mag == NULL))
    {
        rslt = BMA4_E_NULL_PTR;
    }
    else
    {
#if defined(BMM150) || defined(AKM9916)
        switch (sensor_select)
        {

#ifdef BMM150
            case BMA4_SEC_IF_BMM150:
                rslt = bma4_read_regs(BMA4_DATA_0_ADDR, data, BMA4_MAG_XYZ_DATA_LENGTH, dev);
                if (rslt == BMA4_OK)
                {
                    index = BMA4_MAG_X_LSB_BYTE;

                    /*X-axis LSB value shifting*/
                    data[index] = BMA4_GET_BITSLICE(data[index], BMA4_DATA_MAG_X_LSB);

                    /* Data X */
                    msb = data[index + 1];
                    lsb = data[index];
                    mag->x = (int16_t)((msb << 8) | lsb);
                    mag->x = (mag->x / 0x08);

                    /* Data Y */
                    /*Y-axis LSB value shifting*/
                    data[index + 2] = BMA4_GET_BITSLICE(data[index + 2], BMA4_DATA_MAG_Y_LSB);
                    msb = data[index + 3];
                    lsb = data[index + 2];
                    mag->y = (int16_t)((msb << 8) | lsb);
                    mag->y = (mag->y / 0x08);

                    /* Data Z */
                    /*Z-axis LSB value shifting*/
                    data[index + 4] = BMA4_GET_BITSLICE(data[index + 4], BMA4_DATA_MAG_Z_LSB);
                    msb = data[index + 5];
                    lsb = data[index + 4];
                    mag->z = (int16_t)((msb << 8) | lsb);
                    mag->z = (mag->z / 0x02);
                }

                break;
#endif

#ifdef AKM9916
            case BMA4_SEC_IF_AKM09916:
                if (dev->aux_sensor == AKM9916_SENSOR)
                {
                    rslt = bma4_read_regs(BMA4_DATA_0_ADDR, data, BMA4_MAG_XYZ_DATA_LENGTH, dev);
                    if (rslt == BMA4_OK)
                    {
                        index = BMA4_MAG_X_LSB_BYTE;

                        /* Data X */
                        msb = data[index + 1];
                        lsb = data[index];
                        mag->x = (int16_t)((msb << 8) | lsb);

                        /* Data Y */
                        msb = data[index + 3];
                        lsb = data[index + 2];
                        mag->y = (int32_t)((msb << 8) | lsb);

                        /* Data Z */
                        msb = data[index + 5];
                        lsb = data[index + 4];
                        mag->z = (int16_t)((msb << 8) | lsb);
                    }
                }

                break;
#endif
        }
#else
        rslt = BMA4_E_OUT_OF_RANGE;
#endif
    }

    return rslt;
}

/*!
 *  @brief This API reads the auxiliary I2C interface configuration which
 *  is set in the sensor.
 */
int8_t bma4_get_if_mode(uint8_t *if_mode, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (if_mode != NULL))
    {
        /* Read auxiliary interface configuration */
        rslt = bma4_read_regs(BMA4_IF_CONFIG_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            *if_mode = BMA4_GET_BITSLICE(data, BMA4_IF_CONFIG_IF_MODE);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the auxiliary interface configuration in the
 *  sensor.
 */
int8_t bma4_set_if_mode(uint8_t if_mode, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        if (if_mode <= BMA4_MAX_IF_MODE)
        {
            /* Write the interface configuration mode */
            rslt = bma4_read_regs(BMA4_IF_CONFIG_ADDR, &data, 1, dev);
            if (rslt == BMA4_OK)
            {
                data = BMA4_SET_BITSLICE(data, BMA4_IF_CONFIG_IF_MODE, if_mode);
                rslt = bma4_write_regs(BMA4_IF_CONFIG_ADDR, &data, 1, dev);
            }
        }
        else
        {
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

/*!
 *  @brief This API reads the ASIC status from the sensor.
 *  The status information is mentioned in the below table.
 */
int8_t bma4_get_asic_status(struct bma4_asic_status *asic_status, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (asic_status != NULL))
    {
        /* Read the Mag I2C device address*/
        rslt = bma4_read_regs(BMA4_INTERNAL_ERROR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            asic_status->sleep = (data & 0x01);
            asic_status->irq_ovrn = ((data & 0x02) >> 0x01);
            asic_status->wc_event = ((data & 0x04) >> 0x02);
            asic_status->stream_transfer_active = ((data & 0x08) >> 0x03);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API enables the offset compensation for filtered and
 *  unfiltered  Accel data.
 */
int8_t bma4_set_offset_comp(uint8_t offset_en, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = bma4_read_regs(BMA4_NV_CONFIG_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            /* Write Accel FIFO filter data */
            data = BMA4_SET_BITSLICE(data, BMA4_NV_ACCEL_OFFSET, offset_en);
            rslt = bma4_write_regs(BMA4_NV_CONFIG_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API gets the status of Accel offset compensation
 */
int8_t bma4_get_offset_comp(uint8_t *offset_en, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (offset_en != NULL))
    {
        rslt = bma4_read_regs(BMA4_NV_CONFIG_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            /* Write Accel FIFO filter data */
            *offset_en = BMA4_GET_BITSLICE(data, BMA4_NV_ACCEL_OFFSET);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API checks whether the self-test functionality of the sensor
 *  is working or not.
 *  The following parameter of struct bma4_dev, should have the valid value
 *  before performing the self-test,
 *  1.  Variant and 2. Resolution
 */
int8_t bma4_perform_accel_selftest(int8_t *result, struct bma4_dev *dev)
{
    int8_t rslt;
    struct bma4_accel positive = { 0, 0, 0 };
    struct bma4_accel negative = { 0, 0, 0 };

    /*! Structure for difference of accel values in mg */
    struct bma4_selftest_delta_limit accel_data_diff_mg = { 0, 0, 0 };

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (result != NULL))
    {
        *result = BMA4_SELFTEST_FAIL;

        rslt = set_accel_selftest_config(dev);

        if (rslt == BMA4_OK)
        {
            /* Wait for 2ms after accel self-test config please refer data sheet data sheet 4.9. sensor self-test */
            dev->delay_us(BMA4_MS_TO_US(2), dev->intf_ptr);

            rslt = selftest_config(BMA4_ENABLE, dev);

            if (rslt == BMA4_OK)
            {
                /* Taking positive data */

                /* User should wait 50ms before interpreting the acceleration data.
                 * please refer data sheet 4.9. sensor self-test
                 */
                dev->delay_us(BMA4_MS_TO_US(50), dev->intf_ptr);
                rslt = bma4_read_accel_xyz(&positive, dev);

                if (rslt == BMA4_OK)
                {
                    rslt = selftest_config(BMA4_DISABLE, dev);

                    if (rslt == BMA4_OK)
                    {
                        /* User should wait 50ms before interpreting the acceleration data.
                         * please refer data sheet 4.9. sensor self-test
                         */
                        dev->delay_us(BMA4_MS_TO_US(50), dev->intf_ptr);
                        rslt = bma4_read_accel_xyz(&negative, dev);
                        if (rslt == BMA4_OK)
                        {
                            rslt = *result = get_accel_data_difference_and_validate(positive,
                                                                                    negative,
                                                                                    &accel_data_diff_mg,
                                                                                    dev);

                            if (rslt == BMA4_OK)
                            {
                                /* Triggers a soft reset */
                                rslt = bma4_soft_reset(dev);
                                dev->delay_us(BMA4_MS_TO_US(200), dev->intf_ptr);
                            }
                        }
                    }
                }
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*! @cond DOXYGEN_SUPRESS */

/* Suppressing doxygen warnings triggered for same static function names present across various sensor variant
 * directories */

/*!
 *  @brief This Internal API validates accel self-test status from positive and negative axes input
 */
static int8_t get_accel_data_difference_and_validate(struct bma4_accel positive,
                                                     struct bma4_accel negative,
                                                     struct bma4_selftest_delta_limit *accel_data_diff_mg,
                                                     const struct bma4_dev *dev)
{
    int8_t rslt;

    /*! Structure for difference of accel values in g */
    struct bma4_selftest_delta_limit accel_data_diff = { 0, 0, 0 };

    accel_data_diff.x = ABS(positive.x - negative.x);
    accel_data_diff.y = ABS(positive.y - negative.y);
    accel_data_diff.z = ABS(positive.z - negative.z);

    /*! Converting LSB of the differences of accel values to mg */
    convert_lsb_g(&accel_data_diff, accel_data_diff_mg, dev);

    /*! Validating self-test for accel values in mg */
    rslt = validate_selftest(accel_data_diff_mg, dev);

    return rslt;
}

/*! @endcond */

/*!
 *  @brief API sets the interrupt to either interrupt1 or
 *  interrupt2 pin in the sensor.
 */
int8_t bma4_map_interrupt(uint8_t int_line, uint16_t int_map, uint8_t enable, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data[3] = { 0, 0, 0 };
    uint8_t index[2] = { BMA4_INT_MAP_1_ADDR, BMA4_INT_MAP_2_ADDR };

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = bma4_read_regs(BMA4_INT_MAP_1_ADDR, data, 3, dev);
        if (rslt == BMA4_OK)
        {
            if (enable == TRUE)
            {
                /* Feature interrupt mapping */
                data[int_line] |= (uint8_t)(int_map & (0x00FF));

                /* Hardware interrupt mapping */
                data[2] |= (uint8_t)((int_map & (0xFF00)) >> (8 - (4 * int_line)));
            }
            else
            {
                /* Feature interrupt un-mapping */
                data[int_line] &= (~(uint8_t)(int_map & (0x00FF)));

                /* Hardware interrupt un-mapping */
                data[2] &= (~(uint8_t)((int_map & (0xFF00)) >> (8 - (4 * int_line))));
            }

            rslt = bma4_write_regs(index[int_line], &data[int_line], 1, dev);
            if (rslt == BMA4_OK)
            {
                rslt = bma4_write_regs(BMA4_INT_MAP_DATA_ADDR, &data[2], 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 *  @brief This API sets the interrupt mode in the sensor.
 */
int8_t bma4_set_interrupt_mode(uint8_t mode, struct bma4_dev *dev)
{
    int8_t rslt;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        if ((mode == BMA4_NON_LATCH_MODE) || (mode == BMA4_LATCH_MODE))
        {
            rslt = bma4_write_regs(BMA4_INTR_LATCH_ADDR, &mode, 1, dev);
        }
        else
        {
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

/*!
 *  @brief This API gets the interrupt mode which is set in the sensor.
 */
int8_t bma4_get_interrupt_mode(uint8_t *mode, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (mode != NULL))
    {
        rslt = bma4_read_regs(BMA4_INTR_LATCH_ADDR, &data, 1, dev);
        *mode = data;
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API sets the auxiliary Mag(BMM150 or AKM9916) output data
 *  rate and offset.
 */
int8_t bma4_set_aux_mag_config(const struct bma4_aux_mag_config *aux_mag, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (aux_mag != NULL))
    {
        if ((aux_mag->odr >= BMA4_OUTPUT_DATA_RATE_0_78HZ) && (aux_mag->odr <= BMA4_OUTPUT_DATA_RATE_1600HZ) &&
            ((aux_mag->offset & BMA4_MAG_CONFIG_OFFSET_MSK) == 0x00))
        {
            data = (uint8_t)(aux_mag->odr | ((aux_mag->offset << BMA4_MAG_CONFIG_OFFSET_POS)));
            rslt = bma4_write_regs(BMA4_AUX_CONFIG_ADDR, &data, 1, dev);
        }
        else
        {
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the auxiliary Mag(BMM150 or AKM9916) output data
 *  rate and offset.
 */
int8_t bma4_get_aux_mag_config(struct bma4_aux_mag_config *aux_mag, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (aux_mag != NULL))
    {
        rslt = bma4_read_regs(BMA4_AUX_CONFIG_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            aux_mag->odr = (data & 0x0F);
            aux_mag->offset = (data & BMA4_MAG_CONFIG_OFFSET_MSK) >> 4;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*! @brief This API sets the FIFO configuration in the sensor. */
int8_t bma4_set_fifo_config(uint8_t config, uint8_t enable, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0, 0 };
    uint8_t fifo_config_0 = config & BMA4_FIFO_CONFIG_0_MASK;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = bma4_read_regs(BMA4_FIFO_CONFIG_0_ADDR, data, BMA4_FIFO_CONFIG_LENGTH, dev);
        if (rslt == BMA4_OK)
        {
            if (fifo_config_0 > 0)
            {
                if (enable == TRUE)
                {
                    data[0] = data[0] | fifo_config_0;
                }
                else
                {
                    data[0] = data[0] & (~fifo_config_0);
                }
            }

            if (enable == TRUE)
            {
                data[1] = data[1] | (config & BMA4_FIFO_CONFIG_1_MASK);
            }
            else
            {
                data[1] = data[1] & (~(config & BMA4_FIFO_CONFIG_1_MASK));
            }

            /* Burst write is not possible in suspend mode hence
             * separate write is used with delay of 1 ms
             */
            rslt = bma4_write_regs(BMA4_FIFO_CONFIG_0_ADDR, &data[0], 1, dev);

            if (rslt == BMA4_OK)
            {
                dev->delay_us(BMA4_GEN_READ_WRITE_DELAY, dev->intf_ptr);
                rslt = bma4_write_regs(BMA4_FIFO_CONFIG_1_ADDR, &data[1], 1, dev);
            }
        }
    }

    return rslt;
}

/*! @brief This API reads the FIFO configuration from the sensor.
 */
int8_t bma4_get_fifo_config(uint8_t *fifo_config, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0, 0 };

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (fifo_config != NULL))
    {
        rslt = bma4_read_regs(BMA4_FIFO_CONFIG_0_ADDR, data, BMA4_FIFO_CONFIG_LENGTH, dev);
        if (rslt == BMA4_OK)
        {
            *fifo_config = ((uint8_t)((data[0] & BMA4_FIFO_CONFIG_0_MASK) | (data[1])));
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*! @brief This function sets the electrical behaviour of interrupt pin1 or
 *  pin2 in the sensor.
 */
int8_t bma4_set_int_pin_config(const struct bma4_int_pin_config *int_pin_config, uint8_t int_line, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t interrupt_address_array[2] = { BMA4_INT1_IO_CTRL_ADDR, BMA4_INT2_IO_CTRL_ADDR };
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (int_pin_config != NULL))
    {
        if (int_line <= 1)
        {
            data =
                ((uint8_t)((int_pin_config->edge_ctrl & BMA4_INT_EDGE_CTRL_MASK) |
                           ((int_pin_config->lvl << 1) & BMA4_INT_LEVEL_MASK) |
                           ((int_pin_config->od << 2) & BMA4_INT_OPEN_DRAIN_MASK) |
                           ((int_pin_config->output_en << 3) & BMA4_INT_OUTPUT_EN_MASK) |
                           ((int_pin_config->input_en << 4) & BMA4_INT_INPUT_EN_MASK)));
            rslt = bma4_write_regs(interrupt_address_array[int_line], &data, 1, dev);
        }
        else
        {
            rslt = BMA4_E_INT_LINE_INVALID;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*! @brief This API reads the electrical behavior of interrupt pin1 or pin2
 *  from the sensor.
 */
int8_t bma4_get_int_pin_config(struct bma4_int_pin_config *int_pin_config, uint8_t int_line, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t interrupt_address_array[2] = { BMA4_INT1_IO_CTRL_ADDR, BMA4_INT2_IO_CTRL_ADDR };
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (int_pin_config != NULL))
    {
        if (int_line <= 1)
        {
            rslt = bma4_read_regs(interrupt_address_array[int_line], &data, 1, dev);

            /* Assign interrupt configurations to the
             * structure members
             */
            if (rslt == BMA4_OK)
            {
                int_pin_config->edge_ctrl = data & BMA4_INT_EDGE_CTRL_MASK;
                int_pin_config->lvl = ((data & BMA4_INT_LEVEL_MASK) >> BMA4_INT_LEVEL_POS);
                int_pin_config->od = ((data & BMA4_INT_OPEN_DRAIN_MASK) >> BMA4_INT_OPEN_DRAIN_POS);
                int_pin_config->output_en = ((data & BMA4_INT_OUTPUT_EN_MASK) >> BMA4_INT_OUTPUT_EN_POS);
                int_pin_config->input_en = ((data & BMA4_INT_INPUT_EN_MASK) >> BMA4_INT_INPUT_EN_POS);
            }
        }
        else
        {
            rslt = BMA4_E_INT_LINE_INVALID;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API reads the Feature and Hardware interrupt status from the sensor.
 */
int8_t bma4_read_int_status(uint16_t *int_status, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data[2] = { 0 };

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (int_status != NULL))
    {
        rslt = bma4_read_regs(BMA4_INT_STAT_0_ADDR, data, 2, dev);
        if (rslt == BMA4_OK)
        {
            *int_status = data[0];
            *((uint8_t *)int_status + 1) = data[1];
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API initializes the auxiliary interface to access
 * auxiliary sensor
 */
int8_t bma4_aux_interface_init(struct bma4_dev *dev)
{
    /* Variable to return error codes */
    int8_t rslt;

    /* Check for Null pointer error */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Set the auxiliary sensor configuration */
        rslt = set_aux_interface_config(dev);
        if (rslt != BMA4_OK)
        {
            rslt = BMA4_E_AUX_CONFIG_FAIL;
        }
    }

    return rslt;
}

/*!
 * @brief This API reads the data from the auxiliary sensor
 */
int8_t bma4_aux_read(uint8_t aux_reg_addr, uint8_t *aux_data, uint16_t len, struct bma4_dev *dev)
{
    /* Variable to return error codes */
    int8_t rslt;

    /* Check for Null pointer error */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (aux_data != NULL))
    {
        /* Read the data from the data register in terms of
         * user defined length
         */
        rslt = extract_aux_data(aux_reg_addr, aux_data, len, dev);
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API writes the data into the auxiliary sensor
 */
int8_t bma4_aux_write(uint8_t aux_reg_addr, const uint8_t *aux_data, uint16_t len, struct bma4_dev *dev)
{
    int8_t rslt;

    /* Check for Null pointer error */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (aux_data != NULL))
    {
        /* Write data in terms of user defined length  */
        if (len > 0)
        {
            while (len--)
            {
                /* First set data to write */
                rslt = bma4_write_regs(BMA4_AUX_WR_DATA_ADDR, aux_data, 1, dev);
                dev->delay_us(BMA4_AUX_COM_DELAY, dev->intf_ptr);
                if (rslt == BMA4_OK)
                {
                    /* Then set address to write */
                    rslt = bma4_write_regs(BMA4_AUX_WR_ADDR, &aux_reg_addr, 1, dev);
                    dev->delay_us(BMA4_AUX_COM_DELAY, dev->intf_ptr);

                    /* Increment data array and register
                     * address until user-defined length is
                     * greater than 0
                     */
                    if ((rslt == BMA4_OK) && (len > 0))
                    {
                        aux_data++;
                        aux_reg_addr++;
                    }
                }
                else
                {
                    rslt = BMA4_E_COM_FAIL;
                }
            }
        }
        else
        {
            rslt = BMA4_E_RD_WR_LENGTH_INVALID;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API gets the re-mapped x, y and z axes from the sensor and
 * updates the values in the device structure.
 */
int8_t bma4_get_remap_axes(struct bma4_remap *remap_data,
                           uint8_t *feature_config,
                           uint8_t index,
                           uint8_t feature_len,
                           struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Null-pointer check */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Get the re-mapped axes from the sensor */
        rslt = get_remap_axes(&dev->remap, feature_config, index, feature_len, dev);
        if (rslt == BMA4_OK)
        {
            /* Store the receive re-mapped axis and sign from device structure */
            receive_remap_axis(dev->remap.x_axis, dev->remap.x_axis_sign, &remap_data->x);
            receive_remap_axis(dev->remap.y_axis, dev->remap.y_axis_sign, &remap_data->y);
            receive_remap_axis(dev->remap.z_axis, dev->remap.z_axis_sign, &remap_data->z);
        }
    }

    return rslt;
}

/*!
 * @brief This API sets the re-mapped x, y and z axes to the sensor and
 * updates the them in the device structure.
 */
int8_t bma4_set_remap_axes(const struct bma4_remap *remapped_axis,
                           uint8_t *feature_config,
                           uint8_t index,
                           uint8_t feature_len,
                           struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store all the re-mapped axes */
    uint8_t remap_axes = 0;

    /* Null-pointer check */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Check whether all the axes are re-mapped */
        remap_axes = remapped_axis->x | remapped_axis->y | remapped_axis->z;

        /* If all the axes are re-mapped */
        if ((remap_axes & BMA4_AXIS_MASK) == BMA4_AXIS_MASK)
        {
            /* Store the value of re-mapped in device structure */
            assign_remap_axis(remapped_axis->x, &dev->remap.x_axis, &dev->remap.x_axis_sign);
            assign_remap_axis(remapped_axis->y, &dev->remap.y_axis, &dev->remap.y_axis_sign);
            assign_remap_axis(remapped_axis->z, &dev->remap.z_axis, &dev->remap.z_axis_sign);

            /* Set the re-mapped axes in the sensor */
            rslt = set_remap_axes(&dev->remap, feature_config, index, feature_len, dev);
        }
        else
        {
            rslt = BMA4_E_REMAP_ERROR;
        }
    }

    return rslt;
}

/*****************************************************************************/
/*! @cond DOXYGEN_SUPRESS */

/* Suppressing doxygen warnings triggered for same static function names present across various sensor variant
 * directories */

/* Static function definition */

/*!
 *  @brief This API converts lsb value of axes to mg for self-test *
 */
static void convert_lsb_g(const struct bma4_selftest_delta_limit *accel_data_diff,
                          struct bma4_selftest_delta_limit *accel_data_diff_mg,
                          const struct bma4_dev *dev)
{
    uint32_t lsb_per_g;

    /*! Range considered for self-test is 8g */
    uint8_t range = 8;

    /*! lsb_per_g for the respective resolution and 8g range*/
    lsb_per_g = (uint32_t)(power(2, dev->resolution) / (2 * range));

    /*! accel x value in mg */
    accel_data_diff_mg->x = (accel_data_diff->x / (int32_t)lsb_per_g) * 1000;

    /*! accel y value in mg */
    accel_data_diff_mg->y = (accel_data_diff->y / (int32_t)lsb_per_g) * 1000;

    /*! accel z value in mg */
    accel_data_diff_mg->z = (accel_data_diff->z / (int32_t)lsb_per_g) * 1000;
}

/*!
 *  @brief This API writes the config stream data in memory using burst mode
 *  @note index value should be even number.
 */
static int8_t stream_transfer_write(const uint8_t *stream_data, uint16_t index, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t asic_msb = (uint8_t)((index / 2) >> 4);
    uint8_t asic_lsb = ((index / 2) & 0x0F);

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (stream_data != NULL))
    {
        rslt = bma4_write_regs(BMA4_RESERVED_REG_5B_ADDR, &asic_lsb, 1, dev);
        if (rslt == BMA4_OK)
        {
            rslt = bma4_write_regs(BMA4_RESERVED_REG_5C_ADDR, &asic_msb, 1, dev);
            if (rslt == BMA4_OK)
            {
                rslt = write_regs(BMA4_FEATURE_CONFIG_ADDR, (uint8_t *)stream_data, dev->read_write_len, dev);
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief This API enables or disables the Accel self-test feature in the
 *  sensor.
 */
static int8_t set_accel_selftest_enable(uint8_t accel_selftest_enable, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Read the self-test register */
        rslt = bma4_read_regs(BMA4_ACC_SELF_TEST_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            data = BMA4_SET_BITS_POS_0(data, BMA4_ACCEL_SELFTEST_ENABLE, accel_selftest_enable);
            rslt = bma4_write_regs(BMA4_ACC_SELF_TEST_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 *  @brief This API selects the sign of Accel self-test excitation.
 */
static int8_t set_accel_selftest_sign(uint8_t accel_selftest_sign, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        if (accel_selftest_sign <= BMA4_MAX_VALUE_SELFTEST_SIGN)
        {
            /* Read the Accel self-test sign*/
            rslt = bma4_read_regs(BMA4_ACC_SELF_TEST_ADDR, &data, 1, dev);
            if (rslt == BMA4_OK)
            {
                data = BMA4_SET_BITSLICE(data, BMA4_ACCEL_SELFTEST_SIGN, accel_selftest_sign);
                rslt = bma4_write_regs(BMA4_ACC_SELF_TEST_ADDR, &data, 1, dev);
            }
        }
        else
        {
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

/*!
 *  @brief This API sets the Accel self-test amplitude in the sensor.
 */
static int8_t set_accel_selftest_amp(uint8_t accel_selftest_amp, struct bma4_dev *dev)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        if (accel_selftest_amp <= BMA4_MAX_VALUE_SELFTEST_AMP)
        {
            /* Write  self-test amplitude*/
            rslt = bma4_read_regs(BMA4_ACC_SELF_TEST_ADDR, &data, 1, dev);
            if (rslt == BMA4_OK)
            {
                data = BMA4_SET_BITSLICE(data, BMA4_SELFTEST_AMP, accel_selftest_amp);
                rslt = bma4_write_regs(BMA4_ACC_SELF_TEST_ADDR, &data, 1, dev);
            }
        }
        else
        {
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

/*!
 *  @brief This function enables and configures the Accel which is needed
 *  for self-test operation.
 */
static int8_t set_accel_selftest_config(struct bma4_dev *dev)
{
    int8_t rslt;
    struct bma4_accel_config accel = { 0, 0, 0, 0 };

    accel.odr = BMA4_OUTPUT_DATA_RATE_1600HZ;
    accel.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    accel.perf_mode = BMA4_ENABLE;
    accel.range = BMA4_ACCEL_RANGE_8G;

    rslt = bma4_set_accel_enable(BMA4_ENABLE, dev);
    dev->delay_us(BMA4_MS_TO_US(5), dev->intf_ptr);

    if (rslt == BMA4_OK)
    {
        rslt = bma4_set_accel_config(&accel, dev);
    }

    return rslt;
}

/*!
 *  @brief This API is used to reset the FIFO related configurations
 *  in the fifo_frame structure.
 *
 */
static void reset_fifo_data_structure(struct bma4_fifo_frame *fifo)
{
    /*Prepare for next FIFO read by resetting FIFO's
     * internal data structures
     */
    fifo->accel_byte_start_idx = 0;
    fifo->mag_byte_start_idx = 0;
    fifo->sc_frame_byte_start_idx = 0;
    fifo->sensor_time = 0;
    fifo->skipped_frame_count = 0;
    fifo->accel_dropped_frame_count = 0;
    fifo->mag_dropped_frame_count = 0;
}

/*!
 *  @brief This API computes the number of bytes of accel FIFO data
 *  which is to be parsed in header-less mode
 */
static void get_accel_len_to_parse(uint16_t *start_idx,
                                   uint16_t *len,
                                   const uint16_t *acc_count,
                                   const struct bma4_fifo_frame *fifo)
{
    /*Data start index*/
    *start_idx = fifo->accel_byte_start_idx;
    if (fifo->fifo_data_enable == BMA4_FIFO_A_ENABLE)
    {
        /*Len has the number of bytes to loop for */
        *len = (uint16_t)(((*acc_count) * BMA4_FIFO_A_LENGTH));
    }
    else if (fifo->fifo_data_enable == BMA4_FIFO_M_A_ENABLE)
    {
        /*Len has the number of bytes to loop for */
        *len = (uint16_t)(((*acc_count) * BMA4_FIFO_MA_LENGTH));
    }
    else
    {
        /*Only aux. sensor or no sensor is enabled in FIFO,
         * so there will be no accel data.
         * Update the data index as complete
         */
        *start_idx = fifo->length;
    }

    if ((*len) > fifo->length)
    {
        /*Handling the case where more data is requested
         * than available
         */
        *len = fifo->length;
    }
}

/*!
 *  @brief This API checks the fifo read data as empty frame, if it
 *  is empty frame then moves the index to last byte.
 */
static void check_empty_fifo(uint16_t *data_index, const struct bma4_fifo_frame *fifo)
{
    if ((*data_index + 6) < fifo->length)
    {
        /* Check if FIFO is empty (For X, Y and Z)*/
        if (((fifo->data[(*data_index)] == BMA4_FIFO_LSB_CONFIG_CHECK) &&
             (fifo->data[(*data_index) + 1] == BMA4_FIFO_MSB_CONFIG_CHECK)) &&
            ((fifo->data[(*data_index) + 2] == BMA4_FIFO_LSB_CONFIG_CHECK) &&
             (fifo->data[(*data_index) + 3] == BMA4_FIFO_MSB_CONFIG_CHECK)) &&
            ((fifo->data[(*data_index) + 4] == BMA4_FIFO_LSB_CONFIG_CHECK) &&
             (fifo->data[(*data_index) + 5] == BMA4_FIFO_MSB_CONFIG_CHECK)))
        {
            /*Update the data index as complete*/
            *data_index = fifo->length;
        }
    }
}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in header mode.
 *
 */
static void extract_accel_header_mode(struct bma4_accel *accel_data,
                                      uint16_t *accel_length,
                                      struct bma4_fifo_frame *fifo,
                                      const struct bma4_dev *dev)
{
    uint8_t frame_header = 0;
    uint16_t data_index;
    uint16_t accel_index = 0;
    uint16_t frame_to_read = *accel_length;

    for (data_index = fifo->accel_byte_start_idx; data_index < fifo->length;)
    {
        /*Header byte is stored in the variable frame_header*/
        frame_header = fifo->data[data_index];

        /*Get the frame details from header*/
        frame_header = frame_header & BMA4_FIFO_TAG_INTR_MASK;

        /*Index is moved to next byte where the data is starting*/
        data_index++;
        switch (frame_header)
        {
            /* Accel frame */
            case BMA4_FIFO_HEAD_A:
            case BMA4_FIFO_HEAD_M_A:
                unpack_acc_frm(accel_data, &data_index, &accel_index, frame_header, fifo, dev);
                break;

            /* Aux. sensor frame */
            case BMA4_FIFO_HEAD_M:
                move_next_frame(&data_index, BMA4_FIFO_M_LENGTH, fifo);
                break;

            /* Sensor time frame */
            case BMA4_FIFO_HEAD_SENSOR_TIME:
                unpack_sensortime_frame(&data_index, fifo);
                break;

            /* Skip frame */
            case BMA4_FIFO_HEAD_SKIP_FRAME:
                unpack_skipped_frame(&data_index, fifo);
                break;

            /* Input config frame */
            case BMA4_FIFO_HEAD_INPUT_CONFIG:
                move_next_frame(&data_index, 1, fifo);
                break;

            /* Sample drop frame */
            case BMA4_FIFO_HEAD_SAMPLE_DROP:
                unpack_dropped_frame(&data_index, fifo);
                break;

            /* Over read FIFO data */
            case BMA4_FIFO_HEAD_OVER_READ_MSB:

                /* Update the data index as complete*/
                data_index = fifo->length;
                break;
            default:
                break;
        }
        if (frame_to_read == accel_index)
        {
            /*Number of frames to read completed*/
            break;
        }
    }

    /*Update number of accel data read*/
    *accel_length = accel_index;

    /*Update the accel frame index*/
    fifo->accel_byte_start_idx = data_index;
}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data in both header mode and header-less mode.
 *  It update the idx value which is used to store the index of
 *  the current data byte which is parsed.
 */
static void unpack_acc_frm(struct bma4_accel *acc,
                           uint16_t *idx,
                           uint16_t *acc_idx,
                           uint8_t frm,
                           const struct bma4_fifo_frame *fifo,
                           const struct bma4_dev *dev)
{

    switch (frm)
    {
        case BMA4_FIFO_HEAD_A:
        case BMA4_FIFO_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMA4_FIFO_A_LENGTH) > fifo->length)
            {
                /*Update the data index as complete*/
                *idx = fifo->length;
                break;
            }

            /*Unpack the data array into the structure instance "acc" */
            unpack_accel_data(&acc[*acc_idx], *idx, fifo, dev);

            /*Move the data index*/
            *idx = *idx + BMA4_FIFO_A_LENGTH;
            (*acc_idx)++;
            break;
        case BMA4_FIFO_HEAD_M_A:
        case BMA4_FIFO_M_A_ENABLE:

            /*Partial read, then skip the data*/
            if ((*idx + BMA4_FIFO_MA_LENGTH) > fifo->length)
            {
                /*Update the data index as complete*/
                *idx = fifo->length;
                break;
            }

            /*Unpack the data array into structure instance "acc"*/
            unpack_accel_data(&acc[*acc_idx], *idx + BMA4_MA_FIFO_A_X_LSB, fifo, dev);

            /*Move the data index*/
            *idx = *idx + BMA4_FIFO_MA_LENGTH;
            (*acc_idx)++;
            break;

        /* Aux. sensor frame */
        case BMA4_FIFO_HEAD_M:
        case BMA4_FIFO_M_ENABLE:
            (*idx) = (*idx) + BMA4_FIFO_M_LENGTH;
            break;
        default:
            break;
    }
}

/*!
 *  @brief This API is used to parse the accelerometer data from the
 *  FIFO data and store it in the instance of the structure bma4_accel.
 */
static void unpack_accel_data(struct bma4_accel *accel_data,
                              uint16_t data_start_index,
                              const struct bma4_fifo_frame *fifo,
                              const struct bma4_dev *dev)
{
    uint16_t data_lsb;
    uint16_t data_msb;

    /* Accel raw x data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    accel_data->x = (int16_t)((data_msb << 8) | data_lsb);

    /* Accel raw y data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    accel_data->y = (int16_t)((data_msb << 8) | data_lsb);

    /* Accel raw z data */
    data_lsb = fifo->data[data_start_index++];
    data_msb = fifo->data[data_start_index++];
    accel_data->z = (int16_t)((data_msb << 8) | data_lsb);
    if (dev->resolution == BMA4_12_BIT_RESOLUTION)
    {
        accel_data->x = (accel_data->x / 0x10);
        accel_data->y = (accel_data->y / 0x10);
        accel_data->z = (accel_data->z / 0x10);
    }
    else if (dev->resolution == BMA4_14_BIT_RESOLUTION)
    {
        accel_data->x = (accel_data->x / 0x04);
        accel_data->y = (accel_data->y / 0x04);
        accel_data->z = (accel_data->z / 0x04);
    }
}

/*!
 *  @brief This API computes the number of bytes of Mag FIFO data which is
 *  to be parsed in header-less mode
 *
 */
static void get_mag_len_to_parse(uint16_t *start_idx,
                                 uint16_t *len,
                                 const uint16_t *mag_count,
                                 const struct bma4_fifo_frame *fifo)
{
    /*Data start index*/
    *start_idx = fifo->mag_byte_start_idx;
    if (fifo->fifo_data_enable == BMA4_FIFO_M_ENABLE)
    {
        /*Len has the number of bytes to loop for */
        *len = (uint16_t)(((*mag_count) * BMA4_FIFO_M_LENGTH));
    }
    else if (fifo->fifo_data_enable == BMA4_FIFO_M_A_ENABLE)
    {
        /*Len has the number of bytes to loop for */
        *len = (uint16_t)(((*mag_count) * BMA4_FIFO_MA_LENGTH));
    }
    else
    {
        /*Only accel sensor or no sensor is enabled in FIFO,
         * so there will be no mag data.
         * Update the data index as complete
         */
        *start_idx = fifo->length;
    }

    /*Handling the case where more data is requested than available*/
    if ((*len) > fifo->length)
    {
        /*Len is equal to the FIFO length*/
        *len = fifo->length;
    }
}

/*!
 *  @brief This API is used to parse the magnetometer data from the
 *  FIFO data in header mode.
 *
 */
static int8_t extract_mag_header_mode(const struct bma4_mag *data,
                                      uint16_t *len,
                                      struct bma4_fifo_frame *fifo,
                                      const struct bma4_dev *dev)
{
    int8_t rslt = BMA4_OK;
    uint8_t frame_header = 0;
    uint16_t data_index;
    uint16_t mag_index = 0;
    uint16_t frame_to_read = *len;

    for (data_index = fifo->mag_byte_start_idx; data_index < fifo->length;)
    {
        /*Header byte is stored in the variable frame_header*/
        frame_header = fifo->data[data_index];

        /*Get the frame details from header*/
        frame_header = frame_header & BMA4_FIFO_TAG_INTR_MASK;

        /*Index is moved to next byte where the data is starting*/
        data_index++;
        switch (frame_header)
        {
            /* Aux. sensor frame */
            case BMA4_FIFO_HEAD_M:
            case BMA4_FIFO_HEAD_M_A:
                rslt = unpack_mag_frm(data, &data_index, &mag_index, frame_header, fifo, dev);
                break;

            /* Aux. sensor frame */
            case BMA4_FIFO_HEAD_A:
                move_next_frame(&data_index, BMA4_FIFO_A_LENGTH, fifo);
                break;

            /* Sensor time frame */
            case BMA4_FIFO_HEAD_SENSOR_TIME:
                unpack_sensortime_frame(&data_index, fifo);
                break;

            /* Skip frame */
            case BMA4_FIFO_HEAD_SKIP_FRAME:
                unpack_skipped_frame(&data_index, fifo);
                break;

            /* Input config frame */
            case BMA4_FIFO_HEAD_INPUT_CONFIG:
                move_next_frame(&data_index, 1, fifo);
                break;

            /* Sample drop frame */
            case BMA4_FIFO_HEAD_SAMPLE_DROP:
                unpack_dropped_frame(&data_index, fifo);
                break;
            case BMA4_FIFO_HEAD_OVER_READ_MSB:

                /*Update the data index as complete*/
                data_index = fifo->length;
                break;
            default:
                break;
        }
        if (frame_to_read == mag_index)
        {
            /*Number of frames to read completed*/
            break;
        }
    }

    /*update number of Aux. sensor data read*/
    *len = mag_index;

    /*update the Aux. sensor frame index*/
    fifo->mag_byte_start_idx = data_index;

    return rslt;
}

/*!
 *  @brief This API is used to parse the magnetometer data from the
 *  FIFO data in both header mode and header-less mode and update the
 *  data_index value which is used to store the index of the current
 *  data byte which is parsed.
 *
 */
static int8_t unpack_mag_frm(const struct bma4_mag *data,
                             uint16_t *idx,
                             uint16_t *mag_idx,
                             uint8_t frm,
                             const struct bma4_fifo_frame *fifo,
                             const struct bma4_dev *dev)
{
    int8_t rslt = BMA4_OK;

    switch (frm)
    {
        case BMA4_FIFO_HEAD_M:
        case BMA4_FIFO_M_ENABLE:

            /*partial read, then skip the data*/
            if ((*idx + BMA4_FIFO_M_LENGTH) > fifo->length)
            {
                /*update the data index as complete*/
                *idx = fifo->length;
                break;
            }

            /*unpack the data array into Aux. sensor data structure*/
            rslt = unpack_mag_data(&data[*mag_idx], *idx, fifo, dev);

            /*move the data index*/
            *idx = *idx + BMA4_FIFO_M_LENGTH;
            (*mag_idx)++;
            break;
        case BMA4_FIFO_HEAD_M_A:
        case BMA4_FIFO_M_A_ENABLE:

            /*partial read, then skip the data*/
            if ((*idx + BMA4_FIFO_MA_LENGTH) > fifo->length)
            {
                /*update the data index as complete*/
                *idx = fifo->length;
                break;
            }

            /*unpack the data array into Aux. sensor data structure*/
            rslt = unpack_mag_data(&data[*mag_idx], *idx, fifo, dev);

            /*move the data index to next frame*/
            *idx = *idx + BMA4_FIFO_MA_LENGTH;
            (*mag_idx)++;
            break;

        /* aux. sensor frame */
        case BMA4_FIFO_HEAD_A:
        case BMA4_FIFO_A_ENABLE:
            (*idx) = (*idx) + BMA4_FIFO_A_LENGTH;
            break;
        default:
            break;
    }

    return rslt;
}

/*!
 *  @brief This API is used to parse the auxiliary magnetometer data from
 *  the FIFO data and store it in the instance of the structure mag_data.
 *
 */
static int8_t unpack_mag_data(const struct bma4_mag *mag_data,
                              uint16_t start_idx,
                              const struct bma4_fifo_frame *fifo,
                              const struct bma4_dev *dev)
{
    int8_t rslt;
    struct bma4_mag_fifo_data mag_fifo_data;

    /* Aux. mag sensor raw x data */
    mag_fifo_data.mag_x_lsb = fifo->data[start_idx++];
    mag_fifo_data.mag_x_msb = fifo->data[start_idx++];

    /* Aux. mag sensor raw y data */
    mag_fifo_data.mag_y_lsb = fifo->data[start_idx++];
    mag_fifo_data.mag_y_msb = fifo->data[start_idx++];

    /* Aux. mag sensor raw z data */
    mag_fifo_data.mag_z_lsb = fifo->data[start_idx++];
    mag_fifo_data.mag_z_msb = fifo->data[start_idx++];

    /* Aux. mag sensor raw r data */
    mag_fifo_data.mag_r_y2_lsb = fifo->data[start_idx++];
    mag_fifo_data.mag_r_y2_msb = fifo->data[start_idx++];

    /*Compensated FIFO data output*/
    rslt = bma4_second_if_mag_compensate_xyz(mag_fifo_data, dev->aux_sensor, mag_data);

    return rslt;
}

/*!
 *  @brief This API is used to parse and store the sensor time from the
 *  FIFO data in the structure instance dev.
 *
 */
static void unpack_sensortime_frame(uint16_t *data_index, struct bma4_fifo_frame *fifo)
{
    uint32_t sensor_time_byte3 = 0;
    uint16_t sensor_time_byte2 = 0;
    uint8_t sensor_time_byte1 = 0;

    /*Partial read, then move the data index to last data*/
    if ((*data_index + BMA4_SENSOR_TIME_LENGTH) > fifo->length)
    {
        /*Update the data index as complete*/
        *data_index = fifo->length;
    }
    else
    {
        sensor_time_byte3 = fifo->data[(*data_index) + BMA4_SENSOR_TIME_MSB_BYTE] << 16;
        sensor_time_byte2 = fifo->data[(*data_index) + BMA4_SENSOR_TIME_XLSB_BYTE] << 8;
        sensor_time_byte1 = fifo->data[(*data_index)];

        /* Sensor time */
        fifo->sensor_time = (uint32_t)(sensor_time_byte3 | sensor_time_byte2 | sensor_time_byte1);
        *data_index = (*data_index) + BMA4_SENSOR_TIME_LENGTH;
    }
}

/*!
 *  @brief This API is used to parse and store the skipped_frame_count from
 *  the FIFO data in the structure instance dev.
 */
static void unpack_skipped_frame(uint16_t *data_index, struct bma4_fifo_frame *fifo)
{
    /*Partial read, then move the data index to last data*/
    if (*data_index >= fifo->length)
    {
        /*Update the data index as complete*/
        *data_index = fifo->length;
    }
    else
    {
        fifo->skipped_frame_count = fifo->data[*data_index];

        /*Move the data index*/
        *data_index = (*data_index) + 1;
    }
}

/*!
 *  @brief This API is used to parse and store the dropped_frame_count from
 *  the FIFO data in the structure instance dev.
 */
static void unpack_dropped_frame(uint16_t *data_index, struct bma4_fifo_frame *fifo)
{
    uint8_t dropped_frame = 0;

    /*Partial read, then move the data index to last data*/
    if (*data_index >= fifo->length)
    {
        /*Update the data index as complete*/
        *data_index = fifo->length;
    }
    else
    {
        /*Extract accel and mag dropped frame count*/
        dropped_frame = fifo->data[*data_index] & BMA4_ACCEL_AUX_FIFO_DROP;

        /*Move the data index and update the dropped frame count*/
        switch (dropped_frame)
        {
            case BMA4_ACCEL_FIFO_DROP:
                *data_index = (*data_index) + BMA4_FIFO_A_LENGTH;
                fifo->accel_dropped_frame_count = fifo->accel_dropped_frame_count + 1;
                break;
            case BMA4_AUX_FIFO_DROP:
                *data_index = (*data_index) + BMA4_FIFO_M_LENGTH;
                fifo->mag_dropped_frame_count = fifo->mag_dropped_frame_count + 1;
                break;
            case BMA4_ACCEL_AUX_FIFO_DROP:
                *data_index = (*data_index) + BMA4_FIFO_MA_LENGTH;
                fifo->accel_dropped_frame_count = fifo->accel_dropped_frame_count + 1;
                fifo->mag_dropped_frame_count = fifo->mag_dropped_frame_count + 1;
                break;
            default:
                break;
        }
    }
}

/*!
 *  @brief This API is used to move the data index ahead of the
 *  current_frame_length parameter when unnecessary FIFO data appears while
 *  extracting the user specified data.
 */
static void move_next_frame(uint16_t *data_index, uint8_t current_frame_length, const struct bma4_fifo_frame *fifo)
{
    /*Partial read, then move the data index to last data*/
    if ((*data_index + current_frame_length) > fifo->length)
    {
        /*Update the data index as complete*/
        *data_index = fifo->length;
    }
    else
    {
        /*Move the data index to next frame*/
        *data_index = *data_index + current_frame_length;
    }
}

/*!
 *  @brief This function validates the Accel self-test data and decides the
 *  result of self-test operation.
 */
static int8_t validate_selftest(const struct bma4_selftest_delta_limit *accel_data_diff, const struct bma4_dev *dev)
{
    int8_t rslt = 0;
    uint16_t x_axis_signal_diff = 0;
    uint16_t y_axis_signal_diff = 0;
    uint16_t z_axis_signal_diff = 0;

    /* Set self-test amplitude based on variant */
    switch (dev->variant)
    {
        case BMA42X_VARIANT:
            x_axis_signal_diff = BMA42X_ST_ACC_X_AXIS_SIGNAL_DIFF;
            y_axis_signal_diff = BMA42X_ST_ACC_Y_AXIS_SIGNAL_DIFF;
            z_axis_signal_diff = BMA42X_ST_ACC_Z_AXIS_SIGNAL_DIFF;
            break;

        case BMA42X_B_VARIANT:
            x_axis_signal_diff = BMA42X_B_ST_ACC_X_AXIS_SIGNAL_DIFF;
            y_axis_signal_diff = BMA42X_B_ST_ACC_Y_AXIS_SIGNAL_DIFF;
            z_axis_signal_diff = BMA42X_B_ST_ACC_Z_AXIS_SIGNAL_DIFF;
            break;

        case BMA45X_VARIANT:
            x_axis_signal_diff = BMA45X_ST_ACC_X_AXIS_SIGNAL_DIFF;
            y_axis_signal_diff = BMA45X_ST_ACC_Y_AXIS_SIGNAL_DIFF;
            z_axis_signal_diff = BMA45X_ST_ACC_Z_AXIS_SIGNAL_DIFF;
            break;

        default:
            rslt = BMA4_E_INVALID_SENSOR;
            break;
    }

    if (rslt != BMA4_E_INVALID_SENSOR)
    {
        if ((accel_data_diff->x <= x_axis_signal_diff) && (accel_data_diff->y <= y_axis_signal_diff) &&
            (accel_data_diff->z <= z_axis_signal_diff))
        {
            rslt = BMA4_SELFTEST_DIFF_X_Y_AND_Z_AXIS_FAILED;
        }
        else if ((accel_data_diff->x <= x_axis_signal_diff) && (accel_data_diff->y <= y_axis_signal_diff))
        {
            rslt = BMA4_SELFTEST_DIFF_X_AND_Y_AXIS_FAILED;
        }
        else if ((accel_data_diff->x <= x_axis_signal_diff) && (accel_data_diff->z <= z_axis_signal_diff))
        {
            rslt = BMA4_SELFTEST_DIFF_X_AND_Z_AXIS_FAILED;
        }
        else if ((accel_data_diff->y <= y_axis_signal_diff) && (accel_data_diff->z <= z_axis_signal_diff))
        {
            rslt = BMA4_SELFTEST_DIFF_Y_AND_Z_AXIS_FAILED;
        }
        else if (accel_data_diff->x <= x_axis_signal_diff)
        {
            rslt = BMA4_SELFTEST_DIFF_X_AXIS_FAILED;
        }
        else if (accel_data_diff->y <= y_axis_signal_diff)
        {
            rslt = BMA4_SELFTEST_DIFF_Y_AXIS_FAILED;
        }
        else if (accel_data_diff->z <= z_axis_signal_diff)
        {
            rslt = BMA4_SELFTEST_DIFF_Z_AXIS_FAILED;
        }
        else
        {
            rslt = BMA4_SELFTEST_PASS;
        }
    }

    return rslt;
}

/*!
 * @brief This API is used to calculate the power of 2
 */
static int32_t power(int16_t base, uint8_t resolution)
{
    uint8_t i = 1;

    /* Initialize variable to store the power of 2 value */
    int32_t value = 1;

    for (; i <= resolution; i++)
    {
        value = (int32_t)(value * base);
    }

    return value;
}

/*!
 * @brief This internal API brings up the secondary interface to access
 * auxiliary sensor *
 */
static int8_t set_aux_interface_config(struct bma4_dev *dev)
{
    /* Variable to return error codes */
    int8_t rslt;

    /* Check for null pointer error */
    rslt = null_pointer_check(dev);
    if (rslt == BMA4_OK)
    {
        /* Enable the auxiliary sensor */
        rslt = bma4_set_mag_enable(0x01, dev);
        dev->delay_us(BMA4_AUX_COM_DELAY, dev->intf_ptr);

        if (rslt == BMA4_OK)
        {
            /* Disable advance power save */
            rslt = bma4_set_advance_power_save(0x00, dev);
            dev->delay_us(BMA4_AUX_COM_DELAY, dev->intf_ptr);

            if (rslt == BMA4_OK)
            {
                /* Set the I2C device address of auxiliary device */
                rslt = bma4_set_i2c_device_addr(dev);
                dev->delay_us(BMA4_AUX_COM_DELAY, dev->intf_ptr);

                if (rslt == BMA4_OK)
                {
                    /* Set auxiliary interface to manual mode */
                    rslt = bma4_set_mag_manual_enable(dev->aux_config.manual_enable, dev);
                    dev->delay_us(BMA4_AUX_COM_DELAY, dev->intf_ptr);

                    if (rslt == BMA4_OK)
                    {
                        /* Set the number of bytes for burst read */
                        rslt = bma4_set_mag_burst(dev->aux_config.burst_read_length, dev);
                        dev->delay_us(BMA4_AUX_COM_DELAY, dev->intf_ptr);

                        if (rslt == BMA4_OK)
                        {
                            /* Switch on the the auxiliary interface mode */
                            rslt = bma4_set_if_mode(dev->aux_config.if_mode, dev);
                            dev->delay_us(BMA4_AUX_COM_DELAY, dev->intf_ptr);
                        }
                    }
                }
            }
        }
    }

    return rslt;
}

/*!
 * @brief This internal API reads the data from the auxiliary sensor
 * depending on burst length configured
 */
static int8_t extract_aux_data(uint8_t aux_reg_addr, uint8_t *aux_data, uint16_t len, struct bma4_dev *dev)
{
    /* Variable to return error codes */
    int8_t rslt;

    /* Pointer variable to read data from the register */
    uint8_t data[15] = { 0 };

    /* Variable to define length counts */
    uint8_t len_count = 0;

    /* Variable to define burst read length */
    uint8_t burst_len = 0;

    /* Variable to define read length */
    uint8_t read_length = 0;

    /* Variable to define the number of burst reads */
    uint8_t burst_count;

    /* Variable to define address of the data register*/
    uint8_t aux_read_addr = BMA4_DATA_0_ADDR;

    /* Extract burst read length in a variable */
    rslt = map_read_len(&burst_len, dev);
    if ((rslt == BMA4_OK) && (aux_data != NULL))
    {
        for (burst_count = 0; burst_count < len; burst_count += burst_len)
        {
            /* Set the address whose data is to be read */
            rslt = bma4_set_mag_read_addr(aux_reg_addr, dev);
            dev->delay_us(BMA4_AUX_COM_DELAY, dev->intf_ptr);
            if (rslt == BMA4_OK)
            {
                /* If user defined length is valid */
                if (len > 0)
                {
                    /* Read the data from the data register */
                    rslt = bma4_read_regs(aux_read_addr, data, (uint8_t)burst_len, dev);
                    dev->delay_us(BMA4_AUX_COM_DELAY, dev->intf_ptr);
                    if (rslt == BMA4_OK)
                    {
                        /* If defined user length or remaining
                         * length after a burst read  is less than
                         * burst length
                         */
                        if ((len - burst_count) < burst_len)
                        {
                            /* Read length is equal to burst
                             * length or remaining length
                             */
                            read_length = (uint8_t)(len - burst_count);
                        }
                        else
                        {
                            /* Read length is equal to burst
                             * length
                             */
                            read_length = burst_len;
                        }

                        /* Copy the read data in terms of given
                         * read length
                         */
                        for (len_count = 0; len_count < read_length; len_count++)
                        {
                            aux_data[burst_count + len_count] = data[len_count];
                        }

                        /* Increment the register address by
                         * burst read length
                         */
                        aux_reg_addr += burst_len;
                    }
                    else
                    {
                        rslt = BMA4_E_RD_WR_LENGTH_INVALID;
                    }
                }
                else
                {
                    rslt = BMA4_E_COM_FAIL;
                }
            }
            else
            {
                rslt = BMA4_E_COM_FAIL;
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API maps the actual burst read length with user
 * length set.
 */
static int8_t map_read_len(uint8_t *len, const struct bma4_dev *dev)
{
    /* Variable to return error codes */
    int8_t rslt = BMA4_OK;

    switch (dev->aux_config.burst_read_length)
    {
        case BMA4_AUX_READ_LEN_0:
            *len = 1;
            break;
        case BMA4_AUX_READ_LEN_1:
            *len = 2;
            break;
        case BMA4_AUX_READ_LEN_2:
            *len = 6;
            break;
        case BMA4_AUX_READ_LEN_3:
            *len = 8;
            break;
        default:
            rslt = BMA4_E_OUT_OF_RANGE;
            break;
    }

    return rslt;
}

/*!
 * @brief This internal API checks null pointer error
 */
static int8_t null_pointer_check(const struct bma4_dev *dev)
{
    int8_t rslt = BMA4_OK;

    if ((dev == NULL) || (dev->bus_read == NULL) || (dev->bus_write == NULL) || (dev->delay_us == NULL))
    {
        rslt = BMA4_E_NULL_PTR;
    }
    else
    {
        rslt = BMA4_OK;
    }

    return rslt;
}

/*! @endcond */

/*!
 *  @brief This API does soft reset
 */
int8_t bma4_soft_reset(struct bma4_dev *dev)
{
    int8_t rslt;

    /* Variable to read the dummy byte */
    uint8_t dummy_read = 0;

    /* Variable contains soft reset command */
    uint8_t command_reg = BMA4_SOFT_RESET;

    /* Check the dev structure as NULL */
    rslt = null_pointer_check(dev);

    /* Check the bma4 structure as NULL */
    if (rslt == BMA4_OK)
    {
        /* Write command register */
        rslt = bma4_write_regs(BMA4_CMD_ADDR, &command_reg, 1, dev);

        if (rslt == BMA4_OK)
        {
            if (dev->intf == BMA4_SPI_INTF)
            {
                /* Dummy read to bring interface to SPI */
                rslt = bma4_read_regs(BMA4_CHIP_ID_ADDR, &dummy_read, 1, dev);
            }
        }
    }

    return rslt;
}

/*!
 * @brief This API performs Fast Offset Compensation for accelerometer.
 */
int8_t bma4_perform_accel_foc(const struct bma4_accel_foc_g_value *accel_g_value, struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Structure to define the accelerometer configurations */
    struct bma4_accel_config acc_cfg = { 0, 0, 0, 0 };

    /* Variable to store status of advance power save */
    uint8_t aps = 0;

    /* Variable to store status of accelerometer enable */
    uint8_t acc_en = 0;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (accel_g_value != NULL))
    {
        /* Check for input validity */
        if (((ABS(accel_g_value->x) + ABS(accel_g_value->y) + ABS(accel_g_value->z)) == 1) &&
            ((accel_g_value->sign == 1) || (accel_g_value->sign == 0)))
        {
            /* Save accelerometer configurations, accelerometer
             * enable status and advance power save status
             */
            rslt = save_accel_foc_config(&acc_cfg, &aps, &acc_en, dev);

            /* Set configurations for FOC */
            if (rslt == BMA4_OK)
            {
                rslt = set_accel_foc_config(dev);
            }

            /* Perform accelerometer FOC */
            if (rslt == BMA4_OK)
            {
                rslt = perform_accel_foc(accel_g_value, &acc_cfg, dev);
            }

            /* Restore the saved configurations */
            if (rslt == BMA4_OK)
            {
                rslt = restore_accel_foc_config(&acc_cfg, aps, acc_en, dev);
            }
        }
        else
        {
            rslt = BMA4_E_OUT_OF_RANGE;
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API saves the configurations before performing FOC.
 */
static int8_t save_accel_foc_config(struct bma4_accel_config *acc_cfg,
                                    uint8_t *aps,
                                    uint8_t *acc_en,
                                    struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to get the status from PWR_CTRL register */
    uint8_t pwr_ctrl_data = 0;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (acc_cfg != NULL) && (aps != NULL) && (acc_en != NULL))
    {
        /* Get accelerometer configurations to be saved */
        rslt = bma4_get_accel_config(acc_cfg, dev);
        if (rslt == BMA4_OK)
        {
            /* Get accelerometer enable status to be saved */
            rslt = bma4_read_regs(BMA4_POWER_CTRL_ADDR, &pwr_ctrl_data, 1, dev);
            if (rslt == BMA4_OK)
            {
                *acc_en = BMA4_GET_BITSLICE(pwr_ctrl_data, BMA4_ACCEL_ENABLE);
            }

            /* Get advance power save mode to be saved */
            if (rslt == BMA4_OK)
            {
                rslt = bma4_get_advance_power_save(aps, dev);
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API sets configurations for performing accelerometer FOC.
 */
static int8_t set_accel_foc_config(struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to set the accelerometer configuration value */
    uint8_t acc_conf_data = BMA4_FOC_ACC_CONF_VAL;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Disabling offset compensation */
        rslt = set_bma4_accel_offset_comp(BMA4_DISABLE, dev);
        if (rslt == BMA4_OK)
        {
            /* Set accelerometer configurations to 50Hz, continuous mode, CIC mode */
            rslt = bma4_write_regs(BMA4_ACCEL_CONFIG_ADDR, &acc_conf_data, 1, dev);
            if (rslt == BMA4_OK)
            {
                /* Set accelerometer to normal mode by enabling it */
                rslt = bma4_set_accel_enable(BMA4_ENABLE, dev);
                if (rslt == BMA4_OK)
                {
                    /* Disable advance power save mode */
                    rslt = bma4_set_advance_power_save(BMA4_DISABLE, dev);
                }
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API enables/disables the offset compensation for
 * filtered and un-filtered accelerometer data.
 */
static int8_t set_bma4_accel_offset_comp(uint8_t offset_en, struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to store data */
    uint8_t data = 0;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Enable/Disable offset compensation */
        rslt = bma4_read_regs(BMA4_NV_CONFIG_ADDR, &data, 1, dev);
        if (rslt == BMA4_OK)
        {
            data = BMA4_SET_BITSLICE(data, BMA4_NV_ACCEL_OFFSET, offset_en);
            rslt = bma4_write_regs(BMA4_NV_CONFIG_ADDR, &data, 1, dev);
        }
    }

    return rslt;
}

/*!
 * @brief This internal API performs Fast Offset Compensation for accelerometer.
 */
static int8_t perform_accel_foc(const struct bma4_accel_foc_g_value *accel_g_value,
                                const struct bma4_accel_config *acc_cfg,
                                struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define count */
    uint8_t loop;

    /* Variable to store status read from the status register */
    uint8_t reg_status = 0;

    /* Array of structure to store accelerometer data */
    struct bma4_accel accel_value[128] = { { 0 } };

    /* Structure to store accelerometer data temporarily */
    struct bma4_foc_temp_value temp = { 0, 0, 0 };

    /* Structure to store the average of accelerometer data */
    struct bma4_accel accel_avg = { 0, 0, 0 };

    /* Variable to define LSB per g value */
    uint16_t lsb_per_g = 0;

    /* Variable to define range */
    uint8_t range = 0;

    /* Variable to set limit for FOC sample */
    uint8_t limit = 128;

    /* Structure to store accelerometer data deviation from ideal value */
    struct bma4_offset_delta delta = { 0, 0, 0 };

    /* Structure to store accelerometer offset values */
    struct bma4_accel_offset offset = { 0, 0, 0 };

    /* Variable tries max 5 times for interrupt then generates timeout */
    uint8_t try_cnt;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (accel_g_value != NULL) && (acc_cfg != NULL))
    {
        for (loop = 0; loop < limit; loop++)
        {
            try_cnt = 5;
            while (try_cnt && (!(reg_status & BMA4_STAT_DATA_RDY_ACCEL_MSK)))
            {
                /* 20ms delay for 50Hz ODR */
                dev->delay_us(BMA4_MS_TO_US(20), dev->intf_ptr);
                rslt = bma4_read_regs(BMA4_STATUS_ADDR, &reg_status, 1, dev);
                try_cnt--;
            }

            if ((rslt == BMA4_OK) && (reg_status & BMA4_STAT_DATA_RDY_ACCEL_MSK))
            {
                rslt = bma4_read_accel_xyz(&accel_value[loop], dev);
            }

            if (rslt == BMA4_OK)
            {
                rslt = bma4_read_accel_xyz(&accel_value[loop], dev);
            }

            if (rslt == BMA4_OK)
            {
                /* Store the data in a temporary structure */
                temp.x = temp.x + (int32_t)accel_value[loop].x;
                temp.y = temp.y + (int32_t)accel_value[loop].y;
                temp.z = temp.z + (int32_t)accel_value[loop].z;
            }
            else
            {
                break;
            }
        }

        if (rslt == BMA4_OK)
        {
            /* Take average of x, y and z data for lesser noise */
            accel_avg.x = (int16_t)(temp.x / 128);
            accel_avg.y = (int16_t)(temp.y / 128);
            accel_avg.z = (int16_t)(temp.z / 128);

            /* Get the exact range value */
            map_accel_range(acc_cfg->range, &range);

            /* Get the smallest possible measurable acceleration level given the range and
             * resolution */
            lsb_per_g = (uint16_t)(power(2, dev->resolution) / (2 * range));

            /* Compensate acceleration data against gravity */
            comp_for_gravity(lsb_per_g, accel_g_value, &accel_avg, &delta);

            /* Scale according to offset register resolution */
            scale_bma4_accel_offset(range, &delta, &offset, dev->resolution);

            /* Invert the accelerometer offset data */
            invert_bma4_accel_offset(&offset);

            /* Write offset data in the offset compensation register */
            rslt = write_bma4_accel_offset(&offset, dev);

            /* Enable offset compensation */
            if (rslt == BMA4_OK)
            {
                rslt = set_bma4_accel_offset_comp(BMA4_ENABLE, dev);
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API converts the accelerometer range value into
 * corresponding integer value.
 */
static void map_accel_range(uint8_t range_in, uint8_t *range_out)
{
    switch (range_in)
    {
        case BMA4_ACCEL_RANGE_2G:
            *range_out = 2;
            break;
        case BMA4_ACCEL_RANGE_4G:
            *range_out = 4;
            break;
        case BMA4_ACCEL_RANGE_8G:
            *range_out = 8;
            break;
        case BMA4_ACCEL_RANGE_16G:
            *range_out = 16;
            break;
        default:

            /* By default RANGE 4G is set */
            *range_out = 4;
            break;
    }
}

/*!
 * @brief This internal API compensate the accelerometer data against gravity.
 */
static void comp_for_gravity(uint16_t lsb_per_g,
                             const struct bma4_accel_foc_g_value *g_val,
                             const struct bma4_accel *data,
                             struct bma4_offset_delta *comp_data)
{
    /* Array to store the accelerometer values in LSB */
    int16_t accel_value_lsb[3] = { 0 };

    /* Convert g-value to LSB */
    accel_value_lsb[BMA4_X_AXIS] = (int16_t)(lsb_per_g * g_val->x);
    accel_value_lsb[BMA4_Y_AXIS] = (int16_t)(lsb_per_g * g_val->y);
    accel_value_lsb[BMA4_Z_AXIS] = (int16_t)(lsb_per_g * g_val->z);

    /* Get the compensated values for X, Y and Z axis */
    comp_data->x = (data->x - accel_value_lsb[BMA4_X_AXIS]);
    comp_data->y = (data->y - accel_value_lsb[BMA4_Y_AXIS]);
    comp_data->z = (data->z - accel_value_lsb[BMA4_Z_AXIS]);
}

/*!
 * @brief This internal API scales the compensated accelerometer data according
 * to the offset register resolution.
 */
static void scale_bma4_accel_offset(uint8_t range,
                                    const struct bma4_offset_delta *comp_data,
                                    struct bma4_accel_offset *data,
                                    uint8_t resolution)
{
    /* Variable to store the position of bit having 3.9mg resolution */
    int8_t bit_pos_3_9mg;

    /* Variable to store the position previous of bit having 3.9mg resolution */
    int8_t bit_pos_3_9mg_prev_bit = 0;

    /* Variable to store the round-off value */
    uint8_t round_off = 0;

    /* Find the bit position of 3.9mg */
    bit_pos_3_9mg = get_bit_pos_3_9mg(range, resolution);

    if (bit_pos_3_9mg < 0)
    {
        bit_pos_3_9mg = 0;
    }

    /* Round off, consider if the next bit is high */
    if (bit_pos_3_9mg > 0)
    {
        bit_pos_3_9mg_prev_bit = bit_pos_3_9mg - 1;
        round_off = (uint8_t)(power(2, ((uint8_t) bit_pos_3_9mg_prev_bit)));
    }

    /* Scale according to offset register resolution */
    data->x = (uint8_t)((comp_data->x + round_off) / power(2, ((uint8_t) bit_pos_3_9mg)));
    data->y = (uint8_t)((comp_data->y + round_off) / power(2, ((uint8_t) bit_pos_3_9mg)));
    data->z = (uint8_t)((comp_data->z + round_off) / power(2, ((uint8_t) bit_pos_3_9mg)));
}

/*!
 * @brief This internal API inverts the accelerometer offset data.
 */
static void invert_bma4_accel_offset(struct bma4_accel_offset *offset_data)
{
    /* Get the offset data */
    offset_data->x = (uint8_t)((offset_data->x) * (-1));
    offset_data->y = (uint8_t)((offset_data->y) * (-1));
    offset_data->z = (uint8_t)((offset_data->z) * (-1));
}

/*!
 * @brief This internal API writes the offset data in the offset compensation
 * register.
 */
static int8_t write_bma4_accel_offset(const struct bma4_accel_offset *offset, struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Array to store the offset data */
    uint8_t data_array[3] = { 0 };

    data_array[0] = offset->x;
    data_array[1] = offset->y;
    data_array[2] = offset->z;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        /* Offset values are written in the offset register */
        rslt = bma4_write_regs(BMA4_OFFSET_0_ADDR, data_array, 3, dev);
    }

    return rslt;
}

/*!
 * @brief This internal API finds the bit position of 3.9mg according to given
 * range and resolution.
 */
static int8_t get_bit_pos_3_9mg(uint8_t range, uint8_t resolution)
{
    /* Variable to store the bit position of 3.9mg resolution */
    int8_t bit_pos_3_9mg;

    /* Variable to shift the bits according to the resolution  */
    uint32_t divisor = 1;

    /* Scaling factor to get the bit position of 3.9 mg resolution */
    int16_t scale_factor = -1;

    /* Variable to store temporary value */
    uint16_t temp;

    /* Shift left by the times of resolution */
    divisor = divisor << (resolution - 1);

    /* Get the bit position to be shifted */
    temp = (uint16_t)(divisor / (range * 128));

    /* Get the scaling factor until bit position is shifted to last bit */
    while (temp != 1)
    {
        scale_factor++;
        temp = temp >> 1;
    }

    /* Scaling factor is the bit position of 3.9 mg resolution */
    bit_pos_3_9mg = (int8_t) scale_factor;

    return bit_pos_3_9mg;
}

/*!
 * @brief This internal API restores the configurations saved before performing
 * accelerometer FOC.
 */
static int8_t restore_accel_foc_config(const struct bma4_accel_config *acc_cfg,
                                       uint8_t aps,
                                       uint8_t acc_en,
                                       struct bma4_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to get the status from PWR_CTRL register */
    uint8_t pwr_ctrl_data = 0;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if ((rslt == BMA4_OK) && (acc_cfg != NULL))
    {
        /* Restore the saved accelerometer configurations */
        rslt = bma4_set_accel_config(acc_cfg, dev);
        if (rslt == BMA4_OK)
        {
            /* Restore the saved accelerometer enable status */
            rslt = bma4_read_regs(BMA4_POWER_CTRL_ADDR, &pwr_ctrl_data, 1, dev);
            if (rslt == BMA4_OK)
            {
                pwr_ctrl_data = BMA4_SET_BITSLICE(pwr_ctrl_data, BMA4_ACCEL_ENABLE, acc_en);
                rslt = bma4_write_regs(BMA4_POWER_CTRL_ADDR, &pwr_ctrl_data, 1, dev);

                /* Restore the saved advance power save */
                if (rslt == BMA4_OK)
                {
                    rslt = bma4_set_advance_power_save(aps, dev);
                }
            }
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API is to store remapped axis and sign values
 * in device structure
 */
static void assign_remap_axis(uint8_t remap_axis, uint8_t *axis, uint8_t *sign)
{
    /* Variable to store the re-mapped axis value */
    uint8_t axis_val = remap_axis & BMA4_AXIS_MASK;

    switch (axis_val)
    {
        case BMA4_X:

            /* If mapped to x-axis */
            (*axis) = BMA4_MAP_X_AXIS;
            break;
        case BMA4_Y:

            /* If mapped to y-axis */
            (*axis) = BMA4_MAP_Y_AXIS;
            break;
        case BMA4_Z:

            /* If mapped to z-axis */
            (*axis) = BMA4_MAP_Z_AXIS;
            break;
        default:
            break;
    }

    /* Store the re-mapped axis sign in the device structure */
    if (remap_axis & BMA4_AXIS_SIGN)
    {
        /* If axis mapped to negative sign */
        (*sign) = BMA4_MAP_NEGATIVE;
    }
    else
    {
        /* If axis mapped to positive sign */
        (*sign) = BMA4_MAP_POSITIVE;
    }
}

/*!
 * @brief This internal API is to receive remapped axis and sign values
 * in device structure and to local structure
 */
static void receive_remap_axis(uint8_t remap_axis, uint8_t remap_sign, uint8_t *axis)
{
    /* Get the re-mapped axis value from device structure */
    switch (remap_axis)
    {
        case BMA4_MAP_X_AXIS:

            /* If mapped to x-axis */
            (*axis) = BMA4_X;
            break;
        case BMA4_MAP_Y_AXIS:

            /* If mapped to y-axis */
            (*axis) = BMA4_Y;
            break;
        case BMA4_MAP_Z_AXIS:

            /* If mapped to z-axis */
            (*axis) = BMA4_Z;
            break;
        default:
            break;
    }

    /* Get the re-mapped axis sign from device structure */
    if (remap_sign)
    {
        /* If axis is mapped to negative sign */
        (*axis) |= BMA4_AXIS_SIGN;
    }
}

/*!
 * @brief This API performs x, y and z-axis re-mapping in the sensor.
 */
static int8_t set_remap_axes(const struct bma4_axes_remap *remap_data,
                             uint8_t *feature_config,
                             uint8_t index,
                             uint8_t feature_len,
                             struct bma4_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    /* Variable to define x-axis to be re-mapped */
    uint8_t x_axis;

    /* Variable to define y-axis to be re-mapped */
    uint8_t y_axis;

    /* Variable to define z-axis to be re-mapped */
    uint8_t z_axis;

    /* Variable to define x-axis sign to be re-mapped */
    uint8_t x_axis_sign;

    /* Variable to define y-axis sign to be re-mapped */
    uint8_t y_axis_sign;

    /* Variable to define z-axis sign to be re-mapped */
    uint8_t z_axis_sign;

    if (remap_data != NULL)
    {
        /* Read the configuration file */
        rslt = bma4_read_regs(BMA4_FEATURE_CONFIG_ADDR, feature_config, feature_len, dev);

        if (rslt == BMA4_OK)
        {
            /* Get x-axis to be re-mapped */
            x_axis = remap_data->x_axis & BMA4_X_AXIS_MSK;

            /* Get x-axis sign to be re-mapped */
            x_axis_sign = (remap_data->x_axis_sign << BMA4_X_AXIS_SIGN_POS) & BMA4_X_AXIS_SIGN_MSK;

            /* Get y-axis to be re-mapped */
            y_axis = (remap_data->y_axis << BMA4_Y_AXIS_POS) & BMA4_Y_AXIS_MSK;

            /* Get y-axis sign to be re-mapped */
            y_axis_sign = (remap_data->y_axis_sign << BMA4_Y_AXIS_SIGN_POS) & BMA4_Y_AXIS_SIGN_MSK;

            /* Get z-axis to be re-mapped */
            z_axis = (remap_data->z_axis << BMA4_Z_AXIS_POS) & BMA4_Z_AXIS_MSK;

            /* Get z-axis sign to be re-mapped */
            z_axis_sign = remap_data->z_axis_sign & BMA4_Z_AXIS_SIGN_MSK;

            /* Set the first byte for axis re-mapping */
            feature_config[index] = x_axis | x_axis_sign | y_axis | y_axis_sign | z_axis;

            /* Set the second byte for axis re-mapping */
            feature_config[index + 1] = z_axis_sign;

            /* Set the re-mapped axes */
            rslt = bma4_write_regs(BMA4_FEATURE_CONFIG_ADDR, feature_config, feature_len, dev);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This API reads the x, y and z axis re-mapped data from the sensor.
 */
static int8_t get_remap_axes(struct bma4_axes_remap *remap_data,
                             uint8_t *feature_config,
                             uint8_t index,
                             uint8_t feature_len,
                             struct bma4_dev *dev)
{
    /* Variable to hold execution status */
    int8_t rslt;

    if (remap_data != NULL)
    {
        /* Read the configuration file */
        rslt = bma4_read_regs(BMA4_FEATURE_CONFIG_ADDR, feature_config, feature_len, dev);

        if (rslt == BMA4_OK)
        {
            /* Get re-mapped x-axis */
            remap_data->x_axis = BMA4_GET_BITS_POS_0(feature_config[index], BMA4_X_AXIS);

            /* Get re-mapped x-axis sign */
            remap_data->x_axis_sign = BMA4_GET_BITSLICE(feature_config[index], BMA4_X_AXIS_SIGN);

            /* Get re-mapped y-axis */
            remap_data->y_axis = BMA4_GET_BITSLICE(feature_config[index], BMA4_Y_AXIS);

            /* Get re-mapped y-axis sign */
            remap_data->y_axis_sign = BMA4_GET_BITSLICE(feature_config[index], BMA4_Y_AXIS_SIGN);

            /* Get re-mapped z-axis */
            remap_data->z_axis = BMA4_GET_BITSLICE(feature_config[index], BMA4_Z_AXIS);

            /* Get re-mapped z-axis sign */
            remap_data->z_axis_sign = BMA4_GET_BITS_POS_0(feature_config[index + 1], BMA4_Z_AXIS_SIGN);
        }
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}

/*!
 * @brief This internal API gets the re-mapped accelerometer/gyroscope data.
 */
static void get_remapped_data(struct bma4_accel *data, const struct bma4_dev *dev)
{
    /* Array to defined the re-mapped sensor data */
    int16_t remap_data[3] = { 0 };
    int16_t pos_multiplier = INT16_C(1);
    int16_t neg_multiplier = INT16_C(-1);

    /* Fill the array with the un-mapped sensor data */
    remap_data[0] = data->x;
    remap_data[1] = data->y;
    remap_data[2] = data->z;

    /* Get the re-mapped x axis data */
    if (dev->remap.x_axis_sign == BMA4_MAP_POSITIVE)
    {
        data->x = (int16_t)(remap_data[dev->remap.x_axis] * pos_multiplier);
    }
    else
    {
        data->x = (int16_t)(remap_data[dev->remap.x_axis] * neg_multiplier);
    }

    /* Get the re-mapped y axis data */
    if (dev->remap.y_axis_sign == BMA4_MAP_POSITIVE)
    {
        data->y = (int16_t)(remap_data[dev->remap.y_axis] * pos_multiplier);
    }
    else
    {
        data->y = (int16_t)(remap_data[dev->remap.y_axis] * neg_multiplier);
    }

    /* Get the re-mapped z axis data */
    if (dev->remap.z_axis_sign == BMA4_MAP_POSITIVE)
    {
        data->z = (int16_t)(remap_data[dev->remap.z_axis] * pos_multiplier);
    }
    else
    {
        data->z = (int16_t)(remap_data[dev->remap.z_axis] * neg_multiplier);
    }
}

/*!
 *  @brief This API performs the steps needed for self-test operation
 *  before reading the Accel self-test data.
 */
static int8_t selftest_config(uint8_t sign, struct bma4_dev *dev)
{
    int8_t rslt;

    /* NULL pointer check */
    rslt = null_pointer_check(dev);

    if (rslt == BMA4_OK)
    {
        rslt = set_accel_selftest_enable(BMA4_ENABLE, dev);

        if (rslt == BMA4_OK)
        {
            rslt = set_accel_selftest_sign(sign, dev);

            if (rslt == BMA4_OK)
            {
                /* Set self-test amplitude based on variant */
                switch (dev->variant)
                {
                    case BMA42X_VARIANT:

                        /* Set self-test amplitude to high for BMA42x */
                        rslt = set_accel_selftest_amp(BMA4_ENABLE, dev);
                        break;

                    case BMA42X_B_VARIANT:

                        /* Set self-test amplitude to low for BMA42x_B */
                        rslt = set_accel_selftest_amp(BMA4_DISABLE, dev);
                        break;

                    case BMA45X_VARIANT:

                        /* Set self-test amplitude to low for BMA45x */
                        rslt = set_accel_selftest_amp(BMA4_DISABLE, dev);
                        break;

                    default:
                        rslt = BMA4_E_INVALID_SENSOR;
                        break;
                }
            }
        }
    }

    return rslt;
}

/**
* Copyright (c) 2025 Bosch Sensortec GmbH. All rights reserved.
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
* @file       bhi360_api_entry.c
* @date       2025-03-28
* @version    v2.2.0
*
*/
#include "bhi360.h"
#include "bhi360_defs.h"
#include "bhi360_activity_param.h"
#include "bhi360_bsec_param.h"
#include "bhi360_bsx_algo_param.h"
#include "bhi360_head_orientation_param.h"
#include "bhi360_multi_tap_param.h"
#include "bhi360_phy_sensor_ctrl_param.h"
#include "bhi360_system_param.h"
#include "bhi360_virtual_sensor_conf_param.h"
#include "bhi360_virtual_sensor_info_param.h"
#include "bhi360_event_data.h"
#include "bhi360_parse.h"

typedef struct
{
    const char *sensor_api_name;
    void *implementation_ptr;
} SensorAPIEntry;

#include "bhi360_api_entry.h"

const SensorAPIEntry bhi360_sensor_api_entry[] = {
    /*lint -e611 Warning 611: Cast to general void pointer for later use */
    { "bhy_init", (void *)bhi360_init }, { "bhy_get_chip_id", (void *)bhi360_get_chip_id },
    { "bhy_get_and_process_fifo", (void *)bhi360_get_and_process_fifo },
    { "bhy_register_fifo_parse_callback", (void *)bhi360_register_fifo_parse_callback },
    { "bhy_set_host_intf_ctrl", (void *)bhi360_set_host_intf_ctrl },
    { "bhy_get_host_intf_ctrl", (void *)bhi360_get_host_intf_ctrl },
    { "bhy_activity_param_set_hearable_config", (void *)bhi360_activity_param_set_hearable_config },
    { "bhy_activity_param_get_hearable_config", (void *)bhi360_activity_param_get_hearable_config },
    { "bhy_activity_param_set_wearable_config", (void *)bhi360_activity_param_set_wearable_config },
    { "bhy_activity_param_get_wearable_config", (void *)bhi360_activity_param_get_wearable_config },
    { "bhy_bsec_param_set_algo_state", (void *)bhi360_bsec_param_set_algo_state },
    { "bhy_bsec_param_get_algo_state", (void *)bhi360_bsec_param_get_algo_state },
    { "bhy_bsec_param_set_temp_offset", (void *)bhi360_bsec_param_set_temp_offset },
    { "bhy_bsec_param_get_temp_offset", (void *)bhi360_bsec_param_get_temp_offset },
    { "bhy_bsec_param_set_sample_rate", (void *)bhi360_bsec_param_set_sample_rate },
    { "bhy_bsec_param_get_sample_rate", (void *)bhi360_bsec_param_get_sample_rate },
    { "bhy_bsx_algo_param_get_bsx_states", (void *)bhi360_bsx_algo_param_get_bsx_states },
    { "bhy_bsx_algo_param_set_bsx_states", (void *)bhi360_bsx_algo_param_set_bsx_states },
    { "bhy_bsx_algo_param_get_bsx_version", (void *)bhi360_bsx_algo_param_get_bsx_version },
    { "bhy_event_data_parse_quaternion", (void *)bhi360_event_data_parse_quaternion },
    { "bhy_event_data_parse_orientation", (void *)bhi360_event_data_parse_orientation },
    { "bhy_event_data_parse_xyz", (void *)bhi360_event_data_parse_xyz },
    { "bhy_event_data_head_orientation_quat_parsing", (void *)bhi360_event_data_head_orientation_quat_parsing },
    { "bhy_event_data_head_orientation_eul_parsing", (void *)bhi360_event_data_head_orientation_eul_parsing },
    { "bhy_event_data_multi_tap_parsing", (void *)bhi360_event_data_multi_tap_parsing },
    { "bhy_event_data_wrist_gesture_detect_parsing", (void *)bhi360_event_data_wrist_gesture_detect_parsing },
    { "bhy_event_data_parse_air_quality", (void *)bhi360_event_data_parse_air_quality },
    { "bhy_head_orientation_param_trigger_hmc_calibration",
      (void *)bhi360_head_orientation_param_trigger_hmc_calibration },
    { "bhy_head_orientation_param_set_hmc_configuration", (void *)bhi360_head_orientation_param_set_hmc_configuration },
    { "bhy_head_orientation_param_get_hmc_configuration", (void *)bhi360_head_orientation_param_get_hmc_configuration },
    { "bhy_head_orientation_param_set_default_hmc_cfg", (void *)bhi360_head_orientation_param_set_default_hmc_cfg },
    { "bhy_head_orientation_param_get_hmc_version", (void *)bhi360_head_orientation_param_get_hmc_version },
    { "bhy_head_orientation_param_set_hmc_quat_cal_cor_cfg",
      (void *)bhi360_head_orientation_param_set_hmc_quat_cal_cor_cfg },
    { "bhy_head_orientation_param_get_hmc_quat_cal_cor_cfg",
      (void *)bhi360_head_orientation_param_get_hmc_quat_cal_cor_cfg },
    { "bhy_head_orientation_param_set_hmc_mode_vector_x", (void *)bhi360_head_orientation_param_set_hmc_mode_vector_x },
    { "bhy_head_orientation_param_get_hmc_mode_vector_x", (void *)bhi360_head_orientation_param_get_hmc_mode_vector_x },
    { "bhy_head_orientation_param_set_quat_init_head_corr",
      (void *)bhi360_head_orientation_param_set_quat_init_head_corr },
    { "bhy_head_orientation_param_get_quat_init_head_corr",
      (void *)bhi360_head_orientation_param_get_quat_init_head_corr },
    { "bhy_head_orientation_param_get_ho_version", (void *)bhi360_head_orientation_param_get_ho_version },
    { "bhy_head_orientation_param_set_eul_init_head_corr",
      (void *)bhi360_head_orientation_param_set_eul_init_head_corr },
    { "bhy_head_orientation_param_get_eul_init_head_corr",
      (void *)bhi360_head_orientation_param_get_eul_init_head_corr },
    { "bhy_logbin_start_meta", (void *)bhi360_logbin_start_meta },
    { "bhy_logbin_add_meta", (void *)bhi360_logbin_add_meta },
    { "bhy_logbin_end_meta", (void *)bhi360_logbin_end_meta },
    { "bhy_logbin_add_data", (void *)bhi360_logbin_add_data },
    { "bhy_multi_tap_param_set_config", (void *)bhi360_multi_tap_param_set_config },
    { "bhy_multi_tap_param_get_config", (void *)bhi360_multi_tap_param_get_config },
    { "bhy_multi_tap_param_detector_set_config", (void *)bhi360_multi_tap_param_detector_set_config },
    { "bhy_multi_tap_param_detector_get_config", (void *)bhi360_multi_tap_param_detector_get_config },
    { "bhy_phy_sensor_ctrl_param_accel_set_foc_calibration",
      (void *)bhi360_phy_sensor_ctrl_param_accel_set_foc_calibration },
    { "bhy_phy_sensor_ctrl_param_accel_get_foc_calibration",
      (void *)bhi360_phy_sensor_ctrl_param_accel_get_foc_calibration },
    { "bhy_phy_sensor_ctrl_param_accel_set_power_mode", (void *)bhi360_phy_sensor_ctrl_param_accel_set_power_mode },
    { "bhy_phy_sensor_ctrl_param_accel_get_power_mode", (void *)bhi360_phy_sensor_ctrl_param_accel_get_power_mode },
    { "bhy_phy_sensor_ctrl_param_accel_set_axis_remapping",
      (void *)bhi360_phy_sensor_ctrl_param_accel_set_axis_remapping },
    { "bhy_phy_sensor_ctrl_param_accel_get_axis_remapping",
      (void *)bhi360_phy_sensor_ctrl_param_accel_get_axis_remapping },
    { "bhy_phy_sensor_ctrl_param_accel_trigger_nvm_writing",
      (void *)bhi360_phy_sensor_ctrl_param_accel_trigger_nvm_writing },
    { "bhy_phy_sensor_ctrl_param_accel_get_nvm_status", (void *)bhi360_phy_sensor_ctrl_param_accel_get_nvm_status },
    { "bhy_phy_sensor_ctrl_param_gyro_set_foc_calibration",
      (void *)bhi360_phy_sensor_ctrl_param_gyro_set_foc_calibration },
    { "bhy_phy_sensor_ctrl_param_gyro_get_foc_calibration",
      (void *)bhi360_phy_sensor_ctrl_param_gyro_get_foc_calibration },
    { "bhy_phy_sensor_ctrl_param_gyro_set_ois_config", (void *)bhi360_phy_sensor_ctrl_param_gyro_set_ois_config },
    { "bhy_phy_sensor_ctrl_param_gyro_get_ois_config", (void *)bhi360_phy_sensor_ctrl_param_gyro_get_ois_config },
    { "bhy_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg",
      (void *)bhi360_phy_sensor_ctrl_param_gyro_set_fast_startup_cfg },
    { "bhy_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg",
      (void *)bhi360_phy_sensor_ctrl_param_gyro_get_fast_startup_cfg },
    { "bhy_phy_sensor_ctrl_param_gyro_start_comp_retrim", (void *)bhi360_phy_sensor_ctrl_param_gyro_start_comp_retrim },
    { "bhy_phy_sensor_ctrl_param_gyro_get_crt_status", (void *)bhi360_phy_sensor_ctrl_param_gyro_get_crt_status },
    { "bhy_phy_sensor_ctrl_param_gyro_set_power_mode", (void *)bhi360_phy_sensor_ctrl_param_gyro_set_power_mode },
    { "bhy_phy_sensor_ctrl_param_gyro_get_power_mode", (void *)bhi360_phy_sensor_ctrl_param_gyro_get_power_mode },
    { "bhy_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg", (void *)bhi360_phy_sensor_ctrl_param_gyro_set_auto_trim_cfg },
    { "bhy_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg", (void *)bhi360_phy_sensor_ctrl_param_gyro_get_auto_trim_cfg },
    { "bhy_phy_sensor_ctrl_param_gyro_trigger_nvm_writing",
      (void *)bhi360_phy_sensor_ctrl_param_gyro_trigger_nvm_writing },
    { "bhy_phy_sensor_ctrl_param_gyro_get_nvm_status", (void *)bhi360_phy_sensor_ctrl_param_gyro_get_nvm_status },
    { "bhy_phy_sensor_ctrl_param_magnet_set_power_mode", (void *)bhi360_phy_sensor_ctrl_param_magnet_set_power_mode },
    { "bhy_phy_sensor_ctrl_param_magnet_get_power_mode", (void *)bhi360_phy_sensor_ctrl_param_magnet_get_power_mode },
    { "bhy_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg",
      (void *)bhi360_phy_sensor_ctrl_param_set_wrist_wear_wakeup_cfg },
    { "bhy_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg",
      (void *)bhi360_phy_sensor_ctrl_param_get_wrist_wear_wakeup_cfg },
    { "bhy_phy_sensor_ctrl_param_set_any_motion_config", (void *)bhi360_phy_sensor_ctrl_param_set_any_motion_config },
    { "bhy_phy_sensor_ctrl_param_get_any_motion_config", (void *)bhi360_phy_sensor_ctrl_param_get_any_motion_config },
    { "bhy_phy_sensor_ctrl_param_set_no_motion_config", (void *)bhi360_phy_sensor_ctrl_param_set_no_motion_config },
    { "bhy_phy_sensor_ctrl_param_get_no_motion_config", (void *)bhi360_phy_sensor_ctrl_param_get_no_motion_config },
    { "bhy_phy_sensor_ctrl_param_set_wrist_gesture_cfg", (void *)bhi360_phy_sensor_ctrl_param_set_wrist_gesture_cfg },
    { "bhy_phy_sensor_ctrl_param_get_wrist_gesture_cfg", (void *)bhi360_phy_sensor_ctrl_param_get_wrist_gesture_cfg },
    { "bhy_phy_sensor_ctrl_param_baro_set_press_type_1_cfg",
      (void *)bhi360_phy_sensor_ctrl_param_baro_set_press_type_1_cfg },
    { "bhy_phy_sensor_ctrl_param_baro_get_press_type_1_cfg",
      (void *)bhi360_phy_sensor_ctrl_param_baro_get_press_type_1_cfg },
    { "bhy_phy_sensor_ctrl_param_baro_set_press_type_2_cfg",
      (void *)bhi360_phy_sensor_ctrl_param_baro_set_press_type_2_cfg },
    { "bhy_phy_sensor_ctrl_param_baro_get_press_type_2_cfg",
      (void *)bhi360_phy_sensor_ctrl_param_baro_get_press_type_2_cfg },
    { "bhy_phy_sensor_ctrl_param_set_step_counter_config",
      (void *)bhi360_phy_sensor_ctrl_param_set_step_counter_config },
    { "bhy_phy_sensor_ctrl_param_get_step_counter_config",
      (void *)bhi360_phy_sensor_ctrl_param_get_step_counter_config },
    { "bhy_system_param_set_meta_event_control", (void *)bhi360_system_param_set_meta_event_control },
    { "bhy_system_param_get_meta_event_control", (void *)bhi360_system_param_get_meta_event_control },
    { "bhy_system_param_set_wakeup_fifo_control", (void *)bhi360_system_param_set_wakeup_fifo_control },
    { "bhy_system_param_set_nonwakeup_fifo_control", (void *)bhi360_system_param_set_nonwakeup_fifo_control },
    { "bhy_system_param_get_fifo_control", (void *)bhi360_system_param_get_fifo_control },
    { "bhy_system_param_get_firmware_version", (void *)bhi360_system_param_get_firmware_version },
    { "bhy_system_param_get_timestamps", (void *)bhi360_system_param_get_timestamps },
    { "bhy_system_param_get_virtual_sensor_present", (void *)bhi360_system_param_get_virtual_sensor_present },
    { "bhy_system_param_get_physical_sensor_present", (void *)bhi360_system_param_get_physical_sensor_present },
    { "bhy_system_param_get_physical_sensor_info", (void *)bhi360_system_param_get_physical_sensor_info },
    { "bhy_system_param_set_physical_sensor_info", (void *)bhi360_system_param_set_physical_sensor_info },
    { "bhy_virtual_sensor_conf_param_set_cfg", (void *)bhi360_virtual_sensor_conf_param_set_cfg },
    { "bhy_virtual_sensor_conf_param_get_cfg", (void *)bhi360_virtual_sensor_conf_param_get_cfg },
    { "bhy_virtual_sensor_info_param_get_info", (void *)bhi360_virtual_sensor_info_param_get_info },
    { "bhy_get_regs", (void *)bhi360_get_regs }, { "bhy_set_regs", (void *)bhi360_set_regs },
    { "bhy_get_product_id", (void *)bhi360_get_product_id }, { "bhy_get_revision_id", (void *)bhi360_get_revision_id },
    { "bhy_get_rom_version", (void *)bhi360_get_rom_version },
    { "bhy_get_kernel_version", (void *)bhi360_get_kernel_version },
    { "bhy_get_user_version", (void *)bhi360_get_user_version },
    { "bhy_get_boot_status", (void *)bhi360_get_boot_status },
    { "bhy_get_host_status", (void *)bhi360_get_host_status },
    { "bhy_get_feature_status", (void *)bhi360_get_feature_status },
    { "bhy_set_virt_sensor_range", (void *)bhi360_set_virt_sensor_range },
    { "bhy_flush_fifo", (void *)bhi360_flush_fifo },
    { "bhy_set_fifo_format_ctrl", (void *)bhi360_set_fifo_format_ctrl },
    { "bhy_upload_firmware_to_ram", (void *)bhi360_upload_firmware_to_ram },
    { "bhy_upload_firmware_to_ram_partly", (void *)bhi360_upload_firmware_to_ram_partly },
    { "bhy_boot_from_ram", (void *)bhi360_boot_from_ram },
    { "bhy_set_host_interrupt_ctrl", (void *)bhi360_set_host_interrupt_ctrl },
    { "bhy_get_host_interrupt_ctrl", (void *)bhi360_get_host_interrupt_ctrl },
    { "bhy_get_interrupt_status", (void *)bhi360_get_interrupt_status },
    { "bhy_set_timestamp_event_req", (void *)bhi360_set_timestamp_event_req },
    { "bhy_get_hw_timestamp_ns", (void *)bhi360_get_hw_timestamp_ns },
    { "bhy_set_host_ctrl", (void *)bhi360_set_host_ctrl }, { "bhy_get_host_ctrl", (void *)bhi360_get_host_ctrl },
    { "bhy_soft_reset", (void *)bhi360_soft_reset }, { "bhy_perform_self_test", (void *)bhi360_perform_self_test },
    { "bhy_perform_foc", (void *)bhi360_perform_foc },
    { "bhy_get_orientation_matrix", (void *)bhi360_get_orientation_matrix },
    { "bhy_get_post_mortem_data", (void *)bhi360_get_post_mortem_data },
    { "bhy_deregister_fifo_parse_callback", (void *)bhi360_deregister_fifo_parse_callback },
    { "bhy_update_virtual_sensor_list", (void *)bhi360_update_virtual_sensor_list },
    { "bhy_get_sensor_info", (void *)bhi360_get_sensor_info }, { "bhy_set_parameter", (void *)bhi360_set_parameter },
    { "bhy_get_parameter", (void *)bhi360_get_parameter }, { "bhy_get_error_value", (void *)bhi360_get_error_value },
    { "bhy_soft_passthrough_transfer", (void *)bhi360_soft_passthrough_transfer },
    { "bhy_is_sensor_available", (void *)bhi360_is_sensor_available },
    { "bhy_is_physical_sensor_available", (void *)bhi360_is_physical_sensor_available },
    { "bhy_get_variant_id", (void *)bhi360_get_variant_id }, { "bhy_inject_data", (void *)bhi360_inject_data },
    { "bhy_set_data_injection_mode", (void *)bhi360_set_data_injection_mode },
    { "bhy_clear_fifo", (void *)bhi360_clear_fifo }, { "bhy_read_status", (void *)bhi360_read_status },
    { "bhy_parse_get_sensor_details", (void *)bhi360_parse_get_sensor_details },
    { "bhy_parse_add_sensor_details", (void *)bhi360_parse_add_sensor_details },
    { "bhy_parse_meta_event", (void *)bhi360_parse_meta_event },
    { "bhy_parse_3axis_s16", (void *)bhi360_parse_3axis_s16 }, { "bhy_parse_euler", (void *)bhi360_parse_euler },
    { "bhy_parse_quaternion", (void *)bhi360_parse_quaternion },
    { "bhy_parse_s16_as_float", (void *)bhi360_parse_s16_as_float },
    { "bhy_parse_scalar_u32", (void *)bhi360_parse_scalar_u32 },
    { "bhy_parse_scalar_event", (void *)bhi360_parse_scalar_event },
    { "bhy_parse_activity", (void *)bhi360_parse_activity },
    { "bhy_parse_u24_as_float", (void *)bhi360_parse_u24_as_float },
    { "bhy_parse_scalar_u8", (void *)bhi360_parse_scalar_u8 }, { "bhy_parse_generic", (void *)bhi360_parse_generic },
    { "bhy_parse_device_ori", (void *)bhi360_parse_device_ori },
    { "bhy_parse_debug_message", (void *)bhi360_parse_debug_message },
    { "bhy_parse_multitap", (void *)bhi360_parse_multitap },
    { "bhy_parse_wrist_gesture_detect", (void *)bhi360_parse_wrist_gesture_detect },
    { "bhy_parse_air_quality", (void *)bhi360_parse_air_quality }, { "bhy_parse_hmc", (void *)bhi360_parse_hmc },
    { "bhy_parse_oc", (void *)bhi360_parse_oc }, { "bhy_parse_ec", (void *)bhi360_parse_ec },
    { "bhy_set_downsampling_flag", (void *)bhi360_set_downsampling_flag },
    { "bhy_get_downsampling_flag", (void *)bhi360_get_downsampling_flag },
    { "bhy_set_downsampling_odr", (void *)bhi360_set_downsampling_odr },
    { "bhy_parse_temperature_celsius", (void *)bhi360_parse_temperature_celsius },
    { "bhy_parse_humidity", (void *)bhi360_parse_humidity }, { "bhy_parse_pressure", (void *)bhi360_parse_pressure },
    { "bhy_parse_altitude", (void *)bhi360_parse_altitude }, { "end of list", NULL }

    /*lint +e611 */
};
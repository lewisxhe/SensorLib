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
 * @file      BoschFirmware.h
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2025-01-27
 */
#pragma once



// BHI260 Firmwares
#if defined(BOSCH_APP30_SHUTTLE_BHI260_FW)
    #include "bosch/firmware/bhi260/bosch_app30_shuttle_bhi260.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_AUX_BMM150FW)
    #include "bosch/firmware/bhi260/bosch_app30_shuttle_bhi260_aux_bmm150.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_BME68X)
    #include "bosch/firmware/bhi260/bosch_app30_shuttle_bhi260_bme68x.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_BMP390)
    #include "bosch/firmware/bhi260/bosch_app30_shuttle_bhi260_bmp390.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_TURBO)
    #include "bosch/firmware/bhi260/bosch_app30_shuttle_bhi260_turbo.h"
#elif defined(BOSCH_BHI260_AUX_BEM280)
    #include "bosch/firmware/bhi260/bosch_bhi260_aux_bem280.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_BEM280)
    #include "bosch/firmware/bhi260/bosch_bhi260_aux_bmm150_bem280.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_BEM280_GPIO)
    #include "bosch/firmware/bhi260/bosch_bhi260_aux_bmm150_bem280_gpio.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_GPIO)
    #include "bosch/firmware/bhi260/bosch_bhi260_aux_bmm150_gpio.h"
#elif defined(BOSCH_BHI260_GPIO)
    #include "bosch/firmware/bhi260/bosch_bhi260_gpio.h"
#elif defined(BOSCH_BHI260_KLIO)
    #include "bosch/firmware/bhi260/bosch_bhi260_klio.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_AUX_BMM150_FLASH)
    #include "bosch/firmware/bhi260/bosch_app30_shuttle_bhi260_aux_bmm150_flash.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_BME68X_FLASH)
    #include "bosch/firmware/bhi260/bosch_app30_shuttle_bhi260_bme68x_flash.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_BMP390_FLASH)
    #include "bosch/firmware/bhi260/bosch_app30_shuttle_bhi260_bmp390_flash.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_FLASH)
    #include "bosch/firmware/bhi260/bosch_app30_shuttle_bhi260_flash.h"
#elif defined(BOSCH_APP30_SHUTTLE_BHI260_TURBO_FLASH)
    #include "bosch/firmware/bhi260/bosch_app30_shuttle_bhi260_turbo_flash.h"
#elif defined(BOSCH_BHI260_AUX_BEM280_FLASH)
    #include "bosch/firmware/bhi260/bosch_bhi260_aux_bem280_flash.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_BEM280_FLASH)
    #include "bosch/firmware/bhi260/bosch_bhi260_aux_bmm150_bem280_flash.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_BEM280_GPIO_FLASH)
    #include "bosch/firmware/bhi260/bosch_bhi260_aux_bmm150_bem280_gpio_flash.h"
#elif defined(BOSCH_BHI260_AUX_BMM150_GPIO_FLASH)
    #include "bosch/firmware/bhi260/bosch_bhi260_aux_bmm150_gpio_flash.h"
#elif defined(BOSCH_BHI260_GPIO_FLASH)
    #include "bosch/firmware/bhi260/bosch_bhi260_gpio_flash.h"
#elif defined(BOSCH_BHI260_KLIO_FLASH)
    #include "bosch/firmware/bhi260/bosch_bhi260_klio_flash.h"
#elif defined(BOSCH_BHI260_KLIO_TURBO_FLASH)
    #include "bosch/firmware/bhi260/bosch_bhi260_klio_turbo_flash.h"

// BHI360 Firmwares
#elif defined(BOSCH_DATA_INJECT_BHI360)
    #include "bosch/firmware/bhi360/bosch_data_inject_bhi360.h"
#elif defined(BOSCH_DATA_INJECT_BHI360_BMM150)
    #include "bosch/firmware/bhi360/bosch_data_inject_bhi360_bmm150.h"
#elif defined(BOSCH_DATA_INJECT_BHI360_BMM150_HEAD_ORIENTATION)
    #include "bosch/firmware/bhi360/bosch_data_inject_bhi360_bmm150_head_orientation.h"
#elif defined(BOSCH_DATA_INJECT_BHI360_BMM350)
    #include "bosch/firmware/bhi360/bosch_data_inject_bhi360_bmm350.h"
#elif defined(BOSCH_DATA_INJECT_BHI360_BMM350_HEAD_ORIENTATION)
    #include "bosch/firmware/bhi360/bosch_data_inject_bhi360_bmm350_head_orientation.h"
#elif defined(BOSCH_DATA_INJECT_BHI360_ENV)
    #include "bosch/firmware/bhi360/bosch_data_inject_bhi360_env.h"
#elif defined(BOSCH_DATA_INJECT_BHI360_HEAD_ORIENTATION)
    #include "bosch/firmware/bhi360/bosch_data_inject_bhi360_head_orientation.h"
#elif defined(BOSCH_DATA_INJECT_BHI360_MOTION_AI)
    #include "bosch/firmware/bhi360/bosch_data_inject_bhi360_motion_ai.h"
#elif defined(BOSCH_SHUTTLE3_BHI360)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_AUX_BMM150)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_aux_bmm150.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_BMM150)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_bmm150.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_BMM150_BMP580_BME688)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_bmm150_bmp580_bme688.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_BMM150_HEAD_ORIENTATION)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_bmm150_head_orientation.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_BMM350C)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_bmm350c.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_BMM350C_BME688_IAQ)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_bmm350c_bme688_iaq.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_BMM350C_BMP580)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_bmm350c_bmp580.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_BMM350C_BMP580_BME688)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_bmm350c_bmp580_bme688.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_BMM350C_HEAD_ORIENTATION)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_bmm350c_head_orientation.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_BMM350C_POLL)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_bmm350c_poll.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_BMM350C_TURBO)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_bmm350c_turbo.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_BMP580_TEST_EXAMPLE)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_bmp580_test_example.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_HW_ACTIVITY)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_hw_activity.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_HW_ACTIVITY_TURBO)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_hw_activity_turbo.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_IMU_HEAD_ORIENTATION)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_imu_head_orientation.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_POLLING_STEP_COUNTER)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_polling_step_counter.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_TEMP)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_temp.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_TEST_DATA_SOURCE)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_test_data_source.h"
#elif defined(BOSCH_SHUTTLE3_BHI360_TURBO)
    #include "bosch/firmware/bhi360/bosch_shuttle3_bhi360_turbo.h"
#else
    #warning "None of the defined conditions were met, so no firmware will be included".
#endif

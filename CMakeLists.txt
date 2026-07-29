# SensorLib - ESP-IDF Component CMakeLists.txt
# Commonly used I2C & SPI sensor drivers for ESP-IDF / Arduino

function(sensorlib_excluded out_var)
    set(result FALSE)
    foreach(option ${ARGN})
        if(CONFIG_SENSORLIB_EXCLUDE_${option})
            set(result TRUE)
        endif()
    endforeach()
    set(${out_var} ${result} PARENT_SCOPE)
endfunction()

function(sensorlib_add_exclude_definition definitions_var option)
    if(CONFIG_SENSORLIB_EXCLUDE_${option})
        list(APPEND ${definitions_var} SENSORLIB_EXCLUDE_${option}=1)
        set(${definitions_var} ${${definitions_var}} PARENT_SCOPE)
    endif()
endfunction()

set(sensorlib_exclude_options
    ALL
    IMU
    ACCELEROMETER
    MAGNETOMETER
    LIGHT_SENSOR
    RTC
    TOUCH
    HAPTIC
    PMIC
    GAUGE
    IO_EXPANDER
    ACTUATOR
    FINGER_NAVIGATION
    WIRE_HELPER
    QMI8658
    BHI
    BHI260
    BHI260_KLIO
    BHI360
    BMA4XX
    BMA422
    BMA423
    BMA456H
    BMA4XX_EXTRA
    BMM150
    QMC5883L
    QMC5883P
    QMC6309
    QMC6310
    CM32181
    LTR553
    PCF8563
    PCF85063
    TOUCH_CST226
    TOUCH_CST816
    TOUCH_CST92XX
    TOUCH_CST3530
    TOUCH_CSTXXX
    TOUCH_CHSC5816
    TOUCH_FT6X36
    TOUCH_GT911
    TOUCH_GT9895
    TOUCH_HI8561
    HAPTIC_DRV2605
    HAPTIC_AW86224
    PMIC_SY6970
    PMIC_BQ25896
    PMIC_AXP192
    PMIC_AXP202
    PMIC_AXP2101
    PMIC_AXP517
    GAUGE_AXP2602
    GAUGE_BQ27220
    IO_EXPANDER_XL9555
    IO_EXPANDER_PCA9570
    IO_EXPANDER_SPI
    AW9364
    PAWA350
)

set(sensorlib_compile_definitions)
foreach(option ${sensorlib_exclude_options})
    sensorlib_add_exclude_definition(sensorlib_compile_definitions ${option})
endforeach()

set(src_dirs
    src
    src/platform
)

set(exclude_srcs)

sensorlib_excluded(wire_helper_excluded ALL WIRE_HELPER)
if(wire_helper_excluded)
    list(APPEND exclude_srcs src/SensorWireHelper.cpp)
endif()

sensorlib_excluded(qmi8658_excluded ALL IMU QMI8658)
if(NOT qmi8658_excluded)
    list(APPEND src_dirs src/sensor/imu/qmi8658)
endif()

sensorlib_excluded(bhi260_excluded ALL IMU BHI BHI260)
sensorlib_excluded(bhi260_klio_excluded ALL IMU BHI BHI260 BHI260_KLIO)
sensorlib_excluded(bhi360_excluded ALL IMU BHI BHI360)

if(NOT bhi260_excluded)
    list(APPEND src_dirs src/sensor/imu/bhi260)
    if(bhi260_klio_excluded)
        list(APPEND exclude_srcs src/sensor/imu/bhi260/SensorBHI260AP_Klio.cpp)
    endif()
endif()

if(NOT bhi260_excluded OR NOT bhi360_excluded)
    list(APPEND src_dirs
        src/bosch
        src/bosch/bhi260x
    )
    if(bhi260_klio_excluded)
        list(APPEND exclude_srcs src/bosch/bhi260x/bhy2_klio.c)
    endif()
endif()

if(NOT bhi360_excluded)
    list(APPEND src_dirs
        src/sensor/imu/bhi360
        src/bosch/bhi36x
    )
endif()

sensorlib_excluded(bma422_excluded ALL ACCELEROMETER BMA4XX BMA422)
sensorlib_excluded(bma423_excluded ALL ACCELEROMETER BMA4XX BMA423)
sensorlib_excluded(bma456h_excluded ALL ACCELEROMETER BMA4XX BMA456H)
sensorlib_excluded(bma4xx_extra_excluded ALL ACCELEROMETER BMA4XX BMA4XX_EXTRA)

if(NOT bma422_excluded OR NOT bma423_excluded OR NOT bma456h_excluded OR NOT bma4xx_extra_excluded)
    list(APPEND src_dirs src/bosch/bma4xx)
    if(bma422_excluded)
        list(APPEND exclude_srcs src/bosch/bma4xx/bma422_an.c)
    endif()
    if(bma423_excluded)
        list(APPEND exclude_srcs src/bosch/bma4xx/bma423.c)
    endif()
    if(bma456h_excluded)
        list(APPEND exclude_srcs src/bosch/bma4xx/bma456h.c)
    endif()
    if(bma4xx_extra_excluded)
        list(APPEND exclude_srcs
            src/bosch/bma4xx/bma456_an.c
            src/bosch/bma4xx/bma456mm.c
            src/bosch/bma4xx/bma456_tablet.c
            src/bosch/bma4xx/bma456w.c
        )
    endif()
endif()

sensorlib_excluded(bmm150_excluded ALL MAGNETOMETER BMM150)
if(NOT bmm150_excluded)
    list(APPEND src_dirs
        src/bosch/BMM150
        src/sensor/magnetometer/bmm150
    )
endif()

sensorlib_excluded(pawa350_excluded ALL FINGER_NAVIGATION PAWA350)
if(NOT pawa350_excluded)
    list(APPEND src_dirs src/sensor/pixart/pawA350)
endif()

sensorlib_excluded(haptic_drv2605_excluded ALL HAPTIC HAPTIC_DRV2605)
sensorlib_excluded(haptic_aw86224_excluded ALL HAPTIC HAPTIC_AW86224)
if(NOT haptic_drv2605_excluded OR NOT haptic_aw86224_excluded)
    list(APPEND src_dirs src/haptic)
    if(haptic_drv2605_excluded)
        list(APPEND exclude_srcs src/haptic/HapticDriver_DRV2605.cpp)
    endif()
    if(haptic_aw86224_excluded)
        list(APPEND exclude_srcs src/haptic/HapticDriver_AW86224.cpp)
    endif()
endif()

sensorlib_excluded(pmic_sy6970_excluded ALL PMIC PMIC_SY6970)
if(NOT pmic_sy6970_excluded)
    list(APPEND src_dirs src/pmic/silergy/sy6970)
endif()

sensorlib_excluded(pmic_bq25896_excluded ALL PMIC PMIC_BQ25896)
if(NOT pmic_bq25896_excluded)
    list(APPEND src_dirs src/pmic/ti/bq25896)
endif()

sensorlib_excluded(pmic_axp192_excluded ALL PMIC PMIC_AXP192)
if(NOT pmic_axp192_excluded)
    list(APPEND src_dirs src/pmic/xpowers/axp192)
endif()

sensorlib_excluded(pmic_axp202_excluded ALL PMIC PMIC_AXP202)
if(NOT pmic_axp202_excluded)
    list(APPEND src_dirs src/pmic/xpowers/axp202)
endif()

sensorlib_excluded(pmic_axp2101_excluded ALL PMIC PMIC_AXP2101)
if(NOT pmic_axp2101_excluded)
    list(APPEND src_dirs src/pmic/xpowers/axp2101)
endif()

sensorlib_excluded(pmic_axp517_excluded ALL PMIC PMIC_AXP517)
if(NOT pmic_axp517_excluded)
    list(APPEND src_dirs src/pmic/xpowers/axp517)
endif()

sensorlib_excluded(rtc_excluded ALL RTC)
if(NOT rtc_excluded)
    list(APPEND src_dirs src/time)
endif()

sensorlib_excluded(touch_cst226_excluded ALL TOUCH TOUCH_CST226)
sensorlib_excluded(touch_cst816_excluded ALL TOUCH TOUCH_CST816)
sensorlib_excluded(touch_cst92xx_excluded ALL TOUCH TOUCH_CST92XX)
sensorlib_excluded(touch_cst3530_excluded ALL TOUCH TOUCH_CST3530)
sensorlib_excluded(touch_cstxxx_excluded ALL TOUCH TOUCH_CSTXXX)
sensorlib_excluded(touch_chsc5816_excluded ALL TOUCH TOUCH_CHSC5816)
sensorlib_excluded(touch_ft6x36_excluded ALL TOUCH TOUCH_FT6X36)
sensorlib_excluded(touch_gt911_excluded ALL TOUCH TOUCH_GT911)
sensorlib_excluded(touch_gt9895_excluded ALL TOUCH TOUCH_GT9895)
sensorlib_excluded(touch_hi8561_excluded ALL TOUCH TOUCH_HI8561)

if(NOT touch_cst226_excluded OR NOT touch_cst816_excluded OR
   NOT touch_cst92xx_excluded OR NOT touch_cst3530_excluded OR
   NOT touch_cstxxx_excluded OR NOT touch_chsc5816_excluded OR
   NOT touch_ft6x36_excluded OR NOT touch_gt911_excluded OR
   NOT touch_gt9895_excluded OR NOT touch_hi8561_excluded)
    list(APPEND src_dirs src/touch)
    if(touch_cst226_excluded)
        list(APPEND exclude_srcs src/touch/TouchDrvCST226.cpp)
    endif()
    if(touch_cst816_excluded)
        list(APPEND exclude_srcs src/touch/TouchDrvCST816.cpp)
    endif()
    if(touch_cst92xx_excluded)
        list(APPEND exclude_srcs src/touch/TouchDrvCST92xx.cpp)
    endif()
    if(touch_cst3530_excluded)
        list(APPEND exclude_srcs src/touch/TouchDrvCST3530.cpp)
    endif()
    if(touch_cstxxx_excluded)
        list(APPEND exclude_srcs src/touch/TouchDrvCSTXXX.cpp)
    endif()
    if(touch_chsc5816_excluded)
        list(APPEND exclude_srcs src/touch/TouchDrvCHSC5816.cpp)
    endif()
    if(touch_ft6x36_excluded)
        list(APPEND exclude_srcs src/touch/TouchDrvFT6X36.cpp)
    endif()
    if(touch_gt911_excluded)
        list(APPEND exclude_srcs src/touch/TouchDrvGT911.cpp)
    endif()
    if(touch_gt9895_excluded)
        list(APPEND exclude_srcs src/touch/TouchDrvGT9895.cpp)
    endif()
    if(touch_hi8561_excluded)
        list(APPEND exclude_srcs src/touch/TouchDrvHI8561.cpp)
    endif()
endif()

set(include_dirs
    src
    src/actuator
    src/bosch
    src/bosch/BMM150
    src/bosch/bma4xx
    src/bosch/bhi260x
    src/bosch/bhi36x
    src/bosch/firmware/bhi260
    src/bosch/firmware/bhi360
    src/expander
    src/gauge
    src/gauge/ti
    src/gauge/xpowers
    src/haptic
    src/platform
    src/platform/arduino
    src/platform/comm
    src/platform/espidf
    src/pmic
    src/pmic/silergy
    src/pmic/silergy/sy6970
    src/pmic/ti
    src/pmic/ti/bq25896
    src/pmic/xpowers
    src/pmic/xpowers/axp192
    src/pmic/xpowers/axp1xx
    src/pmic/xpowers/axp202
    src/pmic/xpowers/axp2101
    src/pmic/xpowers/axp517
    src/sensor
    src/sensor/accelerometer/bma
    src/sensor/imu
    src/sensor/imu/bhi260
    src/sensor/imu/bhi360
    src/sensor/imu/qmi8658
    src/sensor/light_sensor
    src/sensor/magnetometer/bmm150
    src/sensor/magnetometer/qmc
    src/sensor/pixart/pawA350
    src/time
    src/time/pcf85063
    src/time/pcf8563
    src/touch
)

set(requires
    esp_timer
    esp_driver_gpio
    esp_driver_i2c
    esp_driver_spi
    driver
)

idf_component_register(
    SRC_DIRS      ${src_dirs}
    EXCLUDE_SRCS  ${exclude_srcs}
    INCLUDE_DIRS  ${include_dirs}
    REQUIRES      ${requires}
)

if(sensorlib_compile_definitions)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC ${sensorlib_compile_definitions})
endif()

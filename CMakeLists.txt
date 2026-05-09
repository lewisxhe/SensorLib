# SensorLib - ESP-IDF Component CMakeLists.txt
# Commonly used I2C & SPI sensor drivers for ESP-IDF / Arduino

# ──────────────────────────────────────────────────────────────
# Source directories (contains .c / .cpp files)
# ──────────────────────────────────────────────────────────────
set(src_dirs
    # Core utilities
    src
    # Bosch sensor HAL & vendor libs
    src/bosch
    src/bosch/BMM150
    src/bosch/bma4xx
    src/bosch/bhi260x
    src/bosch/bhi36x
    # Haptic drivers (AW86224, DRV2605)
    src/haptic
    # Platform abstraction (I2C/SPI/HW impl)
    src/platform
    # PMIC - Silergy
    src/pmic/silergy/sy6970
    # PMIC - TI
    src/pmic/ti/bq25896
    # PMIC - X-Powers
    src/pmic/xpowers/axp192
    src/pmic/xpowers/axp202
    src/pmic/xpowers/axp2101
    src/pmic/xpowers/axp517
    # Sensors - IMU
    src/sensor/imu/bhi260
    src/sensor/imu/bhi360
    src/sensor/imu/qmi8658
    # Sensors - Magnetometer
    src/sensor/magnetometer/bmm150
    # Sensors - Pixart proximity
    src/sensor/pixart/pawA350
    # RTC
    src/time
    # Touch controllers
    src/touch
)

# ──────────────────────────────────────────────────────────────
# Include directories (header-only dirs + leaf source dirs)
# ──────────────────────────────────────────────────────────────
set(include_dirs
    # Root (SensorLib.h, SensorWireHelper, etc.)
    src
    # Actuator base classes
    src/actuator
    # Bosch sensor headers
    src/bosch
    src/bosch/BMM150
    src/bosch/bma4xx
    src/bosch/bhi260x
    src/bosch/bhi36x
    # Bosch firmware blobs (header-wrapped .fw data)
    src/bosch/firmware/bhi260
    src/bosch/firmware/bhi360
    # GPIO expander (XL9555, PCA9570, etc.)
    src/expander
    # Fuel gauge
    src/gauge
    src/gauge/ti
    src/gauge/xpowers
    # Haptic drivers
    src/haptic
    # Platform abstraction layers
    src/platform
    src/platform/arduino
    src/platform/comm
    src/platform/espidf
    # PMIC common & sub-families
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
    # Sensors
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
    # RTC
    src/time
    src/time/pcf85063
    src/time/pcf8563
    # Touch controllers
    src/touch
)

# ──────────────────────────────────────────────────────────────
# Component dependencies
#
# ESP-IDF 5.x+ provides modular driver components:
#   esp_driver_i2c  -> driver/i2c_master.h  (new API)
#   esp_driver_gpio -> driver/gpio.h
#   esp_driver_spi  -> driver/spi_master.h
#
# The legacy 'driver' component is a compatibility shim that
# re-exports old-API headers (driver/i2c.h). It is kept here
# because the code uses CONFIG_SENSORLIB_ESP_IDF_OLD_API to
# conditionally include driver/i2c.h for ESP-IDF < 5.0 compat.
#
# esp_timer -> provides esp_timer.h used in platform/espidf HW layer
# ──────────────────────────────────────────────────────────────
set(requires
    esp_timer
    esp_driver_gpio
    esp_driver_i2c
    esp_driver_spi
    driver
)

idf_component_register(
    SRC_DIRS      ${src_dirs}
    INCLUDE_DIRS  ${include_dirs}
    REQUIRES      ${requires}
)

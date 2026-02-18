set(src_dirs ./src
             ./src/touch
             ./src/platform
             ./src/bosch
             ./src/bosch/BMM150
            ./src/bosch/bma4xx
            ./src/bosch/bhi260x
            ./src/bosch/bhi36x
             )
set(include_dirs ./src
                ./src/REG
                ./src/touch
                ./src/platform
                ./src/bosch
                ./src/bosch/firmware/bhi260
                ./src/bosch/firmware/bhi360
                ./src/bosch/BMM150
                ./src/bosch/bma4xx
                ./src/bosch/bhi260x
                ./src/bosch/bhi36x
                ./src/sensor
                )

idf_component_register(SRC_DIRS ${src_dirs} 
                        INCLUDE_DIRS ${include_dirs}
                        REQUIRES esp_timer 
                        esp_driver_gpio 
                        esp_driver_i2c  
                        esp_driver_spi 
                        driver)

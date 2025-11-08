idf_component_register(SRCS "ssd1680.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_driver_spi
                    REQUIRES esp_driver_gpio
                )

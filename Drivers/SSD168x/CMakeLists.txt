idf_component_register(
    SRC_DIRS "Source"
    INCLUDE_DIRS "Source" "Source/include"
    REQUIRES Tactility driver esp_driver_spi esp_driver_gpio
)

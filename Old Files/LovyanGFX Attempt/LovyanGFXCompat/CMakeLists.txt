idf_component_register(
    SRCS 
        "Source/LovyanGFXDisplay.cpp"
    INCLUDE_DIRS "Source"
    REQUIRES lvgl esp_lcd Tactility LovyanGFX
)

idf_component_register(
    SRC_DIRS "Source"
    INCLUDE_DIRS "Source"
    REQUIRES Tactility driver EspLcdCompat esp_lcd_ssd1685 esp_lcd lvgl esp_timer
)

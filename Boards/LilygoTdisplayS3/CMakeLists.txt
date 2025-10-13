file(GLOB_RECURSE SOURCE_FILES Source/*.c*)

idf_component_register(
    SRCS ${SOURCE_FILES}
    INCLUDE_DIRS "Source"
    REQUIRES Tactility lvgl esp_lcd esp_lvgl_port driver PwmBacklight lvgl espressif__esp_lcd_panel_io_additions
)

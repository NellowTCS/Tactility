#include "board_2432s024_touch.h"

#include <esp_lcd_touch_cst816s.h>
#include <esp_err.h>
#include <driver/i2c.h>
#include <esp_check.h>

#define CST816_I2C_PORT (0)

const char* TAG = "cst816";

static esp_err_t prv_init_io(esp_lcd_panel_io_handle_t* io_handle) {
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_33,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = GPIO_NUM_32,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 400000
    };

    ESP_RETURN_ON_ERROR(
        i2c_param_config(CST816_I2C_PORT, &i2c_conf),
        TAG,
        "i2c config failed"
    );

    ESP_RETURN_ON_ERROR(
        i2c_driver_install(CST816_I2C_PORT, i2c_conf.mode, 0, 0, 0),
        TAG,
        "i2c driver install failed"
    );

    const esp_lcd_panel_io_i2c_config_t touch_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();

    ESP_RETURN_ON_ERROR(
        esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)CST816_I2C_PORT, &touch_io_config, io_handle),
        TAG,
        "esp_lcd_panel creation failed"
    );

    return ESP_OK;
}

static esp_err_t prv_create_touch(esp_lcd_panel_io_handle_t io_handle, esp_lcd_touch_handle_t* touch_handle) {
    esp_lcd_touch_config_t config = {
        .x_max = 240,
        .y_max = 320,
        .rst_gpio_num = GPIO_NUM_25,
        .int_gpio_num = GPIO_NUM_21,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .interrupt_callback = NULL,
    };

    return esp_lcd_touch_new_i2c_cst816s(io_handle, &config, touch_handle);
}

nb_touch_driver_t board_2432s024_create_touch_driver() {
    nb_touch_driver_t driver = {
        .name = "cst816s_2432s024",
        .init_io = prv_init_io,
        .create_touch = &prv_create_touch
    };
    return driver;
}

#include "Ssd1681Display.h"

#include <Tactility/Log.h>
#include <esp_lcd_panel_commands.h>
#include <esp_lcd_panel_dev.h>
#include <esp_lcd_panel_ssd1681.h>
#include <esp_lvgl_port.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "ssd1681_display"

bool Ssd1681Display::createIoHandle(esp_lcd_panel_io_handle_t& ioHandle) {
    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = configuration->csPin,
        .dc_gpio_num = configuration->dcPin,
        .spi_mode = 0,
        .pclk_hz = 10 * 1000 * 1000,
        .trans_queue_depth = 10,
        .on_color_trans_done = nullptr,
        .user_ctx = nullptr,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = {
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0
        }
    };

    if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)configuration->spiHost, &io_config, &ioHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create SPI IO handle");
        return false;
    }

    return true;
}

bool Ssd1681Display::createPanelHandle(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t& panelHandle) {
    if (configuration->resetPin != GPIO_NUM_NC) {
        gpio_config_t reset_gpio_config = {
            .pin_bit_mask = 1ULL << configuration->resetPin,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&reset_gpio_config);
        
        gpio_set_level(configuration->resetPin, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(configuration->resetPin, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = GPIO_NUM_NC,
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
        .bits_per_pixel = 1,
        .flags = {
            .reset_active_high = 0,
        },
        .vendor_config = &vendorConfig,
    };

    if (esp_lcd_new_panel_ssd1681(ioHandle, &panel_config, &panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create SSD1681 panel");
        return false;
    }

    if (esp_lcd_panel_reset(panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Panel reset failed");
        return false;
    }

    if (esp_lcd_panel_init(panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Panel init failed");
        return false;
    }

    TT_LOG_I(TAG, "SSD1681 e-paper display initialized successfully");

    return true;
}

lvgl_port_display_cfg_t Ssd1681Display::getLvglPortDisplayConfig(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t panelHandle) {
    bool swap_xy = (configuration->rotation == 1 || configuration->rotation == 3);
    bool mirror_x = (configuration->rotation == 2 || configuration->rotation == 3);
    bool mirror_y = (configuration->rotation == 1 || configuration->rotation == 2);

    uint32_t logical_width = swap_xy ? configuration->height : configuration->width;
    uint32_t logical_height = swap_xy ? configuration->width : configuration->height;

    TT_LOG_I(TAG, "LVGL config: physical=%dx%d logical=%dx%d rotation=%d buffer_size=%lu pixels",
             configuration->width, configuration->height, logical_width, logical_height,
             configuration->rotation, configuration->bufferSize);
    TT_LOG_I(TAG, "LVGL rotation: swap_xy=%d, mirror_x=%d, mirror_y=%d",
             swap_xy, mirror_x, mirror_y);

    return {
        .io_handle = ioHandle,
        .panel_handle = panelHandle,
        .control_handle = nullptr,
        .buffer_size = configuration->bufferSize > 0 ? configuration->bufferSize : (logical_width * logical_height),
        .double_buffer = false,
        .trans_size = 0,
        .hres = logical_width,
        .vres = logical_height,
        .monochrome = true,
        .rotation = {
            .swap_xy = swap_xy,
            .mirror_x = mirror_x,
            .mirror_y = mirror_y,
        },
        .color_format = LV_COLOR_FORMAT_RGB565,
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
            .sw_rotate = false,
            .swap_bytes = false,
            .full_refresh = false,
            .direct_mode = false
        }
    };
}

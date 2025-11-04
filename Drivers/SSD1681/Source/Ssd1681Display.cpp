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

// SSD1681 specific dimensions for GDEY029T71H
#define SSD1681_WIDTH  168
#define SSD1681_HEIGHT 384
#define SSD1681_SOURCE_SHIFT 8  // S8..S175 connected to display sources

bool Ssd1681Display::createIoHandle(esp_lcd_panel_io_handle_t& ioHandle) {
    // SPI bus configuration for e-paper
    const esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = configuration->csPin,
        .dc_gpio_num = configuration->dcPin,
        .spi_mode = 0,
        .pclk_hz = 10 * 1000 * 1000,  // 10MHz - safe for e-paper
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
    // Hardware reset
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

    // Configure BUSY pin with interrupt for refresh detection
    if (configuration->busyPin != GPIO_NUM_NC) {
        gpio_config_t busy_gpio_config = {
            .pin_bit_mask = 1ULL << configuration->busyPin,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_NEGEDGE,  // Trigger when BUSY goes LOW (refresh done)
        };
        gpio_config(&busy_gpio_config);
        gpio_intr_disable(configuration->busyPin);  // Will be enabled before refresh
    }

    // Create ESP-IDF panel with custom config for GDEY029T71H
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = GPIO_NUM_NC,  // Already handled above
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
        .bits_per_pixel = 1,  // Monochrome e-paper
        .flags = {
            .reset_active_high = 0,
        },
        .vendor_config = nullptr,
    };

    // SSD1681-specific configuration
    esp_lcd_ssd1681_config_t ssd1681_config = {
        .busy_gpio_num = configuration->busyPin,
        .non_copy_mode = false,  // Let driver manage buffer copies
    };
    panel_config.vendor_config = &ssd1681_config;

    if (esp_lcd_new_panel_ssd1681(ioHandle, &panel_config, &panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create SSD1681 panel");
        return false;
    }

    // Initialize the panel
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
    // Determine rotation settings
    bool swap_xy = (configuration->rotation == 1 || configuration->rotation == 3);
    bool mirror_x = (configuration->rotation == 2 || configuration->rotation == 3);
    bool mirror_y = (configuration->rotation == 1 || configuration->rotation == 2);

    // Logical dimensions after rotation
    uint32_t logical_width = swap_xy ? configuration->height : configuration->width;
    uint32_t logical_height = swap_xy ? configuration->width : configuration->height;

    TT_LOG_I(TAG, "LVGL config: physical=%dx%d logical=%dx%d rotation=%d (swap_xy=%d, mirror_x=%d, mirror_y=%d)",
             configuration->width, configuration->height, logical_width, logical_height,
             configuration->rotation, swap_xy, mirror_x, mirror_y);

    return {
        .io_handle = ioHandle,
        .panel_handle = panelHandle,
        .control_handle = nullptr,
        .buffer_size = configuration->bufferSize > 0 ? configuration->bufferSize : (logical_width * logical_height / 10),
        .double_buffer = false,  // E-paper doesn't need double buffering
        .trans_size = 0,
        .hres = logical_width,
        .vres = logical_height,
        .monochrome = true,  // ESP-LVGL-port handles RGB565 → 1-bit conversion!
        .rotation = {
            .swap_xy = swap_xy,
            .mirror_x = mirror_x,
            .mirror_y = mirror_y,
        },
        .color_format = LV_COLOR_FORMAT_RGB565,  // Use RGB565, monochrome flag converts it!
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,  // Use PSRAM for rotation buffer
            .sw_rotate = true,  // Let esp_lvgl_port handle rotation
            .swap_bytes = false,
            .full_refresh = false,  // E-paper supports partial refresh
            .direct_mode = false
        }
    };
}

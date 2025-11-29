#include "Ssd1685Display.h"

#include <Tactility/Log.h>
#include <esp_lcd_panel_commands.h>
#include <esp_lcd_panel_dev.h>
#include <esp_lvgl_port.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>
#include <algorithm>

// Bayer 4x4 dithering for monochrome conversion
static inline bool bayer4x4Dither(uint16_t rgb565_pixel, int x, int y) {
    // Extract RGB565 components
    uint8_t r = ((rgb565_pixel >> 11) & 0x1F) << 3; // 5 bits -> 8 bits
    uint8_t g = ((rgb565_pixel >> 5) & 0x3F) << 2;  // 6 bits -> 8 bits
    uint8_t b = (rgb565_pixel & 0x1F) << 3;         // 5 bits -> 8 bits
    
    // Calculate brightness (weighted average)
    uint8_t brightness = (r * 77 + g * 151 + b * 28) >> 8;

    static const uint8_t bayer4[4][4] = {
        { 0,  8,  2, 10},
        {12,  4, 14,  6},
        { 3, 11,  1,  9},
        {15,  7, 13,  5}
    };

    uint8_t thresh = (uint8_t)(bayer4[y & 3][x & 3] * 16 + 8);
    return brightness > thresh; // true = white, false = black
}

bool Ssd1685Display::createIoHandle(esp_lcd_panel_io_handle_t& ioHandle) {
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

bool Ssd1685Display::createPanelHandle(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t& panelHandle) {
    // Handle reset pin manually if provided
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

    // Allocate and configure the vendor-specific config
    vendorConfig = new esp_lcd_ssd1685_config_t();
    vendorConfig->busy_gpio = configuration->busyPin;
    vendorConfig->refresh_mode = SSD1685_REFRESH_FULL;
    vendorConfig->swap_axes = false;
    vendorConfig->mirror_x = false;
    vendorConfig->mirror_y = false;

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = GPIO_NUM_NC, // We handle reset manually above
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
        .bits_per_pixel = 1,
        .flags = {
            .reset_active_high = 0,
        },
        .vendor_config = vendorConfig,
    };

    if (esp_lcd_new_panel_ssd1685(ioHandle, &panel_config, vendorConfig, &panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create SSD1685 panel");
        delete vendorConfig;
        vendorConfig = nullptr;
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

    // Initial clear - fill with white
    TT_LOG_I(TAG, "Issuing initial full-screen clear (white) for SSD1685 panel");
    const size_t clear_size = configuration->width * configuration->height / 8;
    uint8_t *white_buffer = (uint8_t *)heap_caps_malloc(clear_size, MALLOC_CAP_DMA);
    if (white_buffer) {
        memset(white_buffer, 0xFF, clear_size); // 0xFF = white in monochrome
        
        esp_err_t r = esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, configuration->width, configuration->height, white_buffer);
        if (r != ESP_OK) {
            TT_LOG_W(TAG, "Initial draw_bitmap returned %d", r);
        }
        
        // Wait for BUSY to finish
        TT_LOG_I(TAG, "Waiting for BUSY to clear after initial refresh...");
        int busy_pin = configuration->busyPin;
        if (busy_pin != GPIO_NUM_NC) {
            int timeout = 0;
            while (gpio_get_level((gpio_num_t)busy_pin) && timeout < 60) {
                vTaskDelay(pdMS_TO_TICKS(50));
                timeout++;
            }
            if (timeout >= 60) {
                TT_LOG_W(TAG, "BUSY timeout waiting for initial clear");
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
        TT_LOG_I(TAG, "Initial refresh finished");
        heap_caps_free(white_buffer);
    } else {
        TT_LOG_W(TAG, "Unable to allocate white buffer for initial clear");
    }

    TT_LOG_I(TAG, "SSD1685 e-paper display initialized successfully");

    return true;
}

lvgl_port_display_cfg_t Ssd1685Display::getLvglPortDisplayConfig(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t panelHandle) {
    bool swap_xy = (configuration->rotation == 1 || configuration->rotation == 3);
    bool mirror_x = (configuration->rotation == 2 || configuration->rotation == 3);
    bool mirror_y = (configuration->rotation == 1 || configuration->rotation == 2);

    uint32_t logical_width = swap_xy ? configuration->height : configuration->width;
    uint32_t logical_height = swap_xy ? configuration->width : configuration->height;

    // For e-paper: use full-screen buffer in PSRAM
    uint32_t buffer_size = logical_width * logical_height;

    TT_LOG_I(TAG, "LVGL config: physical=%dx%d logical=%dx%d rotation=%d buffer_size=%lu pixels",
             configuration->width, configuration->height, logical_width, logical_height,
             configuration->rotation, buffer_size);
    TT_LOG_I(TAG, "LVGL rotation: swap_xy=%d, mirror_x=%d, mirror_y=%d",
             swap_xy, mirror_x, mirror_y);

    // Set up custom flush callback that handles RGB565->monochrome conversion
    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = ioHandle,
        .panel_handle = panelHandle,
        .control_handle = nullptr,
        .buffer_size = buffer_size,
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
            .full_refresh = true,
            .direct_mode = false
        }
    };

    return disp_cfg;
}

bool Ssd1685Display::startLvgl() {
    // Call base class to create the LVGL display
    if (!EspLcdDisplay::startLvgl()) {
        return false;
    }

    // Now override the flush callback with our custom one
    lv_display_t* lvglDisplay = getLvglDisplay();
    if (lvglDisplay) {
        lv_display_set_flush_cb(lvglDisplay, customFlushCallback);
        lv_display_set_user_data(lvglDisplay, this);
        TT_LOG_I(TAG, "Set custom flush callback for monochrome conversion");
    }

    return true;
}

void Ssd1685Display::customFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<Ssd1685Display*>(lv_display_get_user_data(disp));
    if (!self) {
        lv_display_flush_ready(disp);
        return;
    }

    int32_t hor_res = lv_display_get_horizontal_resolution(disp);
    int32_t ver_res = lv_display_get_vertical_resolution(disp);

    TT_LOG_I(TAG, "Flush callback: logical=%dx%d physical=%dx%d", 
             hor_res, ver_res, self->configuration->width, self->configuration->height);

    // Convert RGB565 buffer to 1-bit monochrome with dithering
    const size_t mono_size = (hor_res * ver_res + 7) / 8;
    uint8_t* mono_buffer = (uint8_t*)heap_caps_malloc(mono_size, MALLOC_CAP_DMA);
    if (!mono_buffer) {
        TT_LOG_E(TAG, "Failed to allocate monochrome buffer");
        lv_display_flush_ready(disp);
        return;
    }

    memset(mono_buffer, 0xFF, mono_size); // Start with all white

    // Convert with dithering
    uint16_t* rgb_pixels = (uint16_t*)px_map;
    for (int32_t y = 0; y < ver_res; y++) {
        for (int32_t x = 0; x < hor_res; x++) {
            uint16_t pixel = rgb_pixels[y * hor_res + x];
            bool is_white = bayer4x4Dither(pixel, x, y);

            uint32_t pixel_idx = y * hor_res + x;
            uint32_t byte_idx = pixel_idx / 8;
            uint8_t bit_idx = 7 - (pixel_idx % 8);

            if (!is_white) {
                mono_buffer[byte_idx] &= ~(1 << bit_idx);
            }
        }
    }

    // Send to display
    esp_err_t err = esp_lcd_panel_draw_bitmap(
        self->panelHandle,
        0, 0,
        self->configuration->width,
        self->configuration->height,
        mono_buffer
    );

    if (err != ESP_OK) {
        TT_LOG_E(TAG, "draw_bitmap failed: %d", err);
    }

    heap_caps_free(mono_buffer);

    // Wait for busy if pin is configured
    if (self->configuration->busyPin != GPIO_NUM_NC) {
        int timeout = 0;
        while (gpio_get_level((gpio_num_t)self->configuration->busyPin) && timeout < 100) {
            vTaskDelay(pdMS_TO_TICKS(50));
            timeout++;
        }
        if (timeout >= 100) {
            TT_LOG_W(TAG, "BUSY timeout after flush");
        }
    } else {
        // Conservative delay if no busy pin
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    lv_display_flush_ready(disp);
}

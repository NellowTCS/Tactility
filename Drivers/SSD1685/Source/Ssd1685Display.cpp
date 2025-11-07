#include "Ssd1685Display.h"

#include <Tactility/Log.h>
#include <esp_lcd_panel_commands.h>
#include <esp_lcd_panel_dev.h>
#include <esp_lcd_panel_ssd1685.h>
#include <esp_lvgl_port.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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

    if (esp_lcd_new_panel_ssd1685(ioHandle, &panel_config, &panelHandle) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create SSD1685 panel");
        return false;
    }

    if (esp_lcd_panel_set_gap(panelHandle, configuration->gapX, configuration->gapY) != ESP_OK) {
        TT_LOG_E(TAG, "Failed to apply panel gap (%d,%d)", configuration->gapX, configuration->gapY);
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

    // Force an initial full-screen white clear and immediate refresh.
    // GoodDisplay panels require a full clear after init to show anything.
    TT_LOG_I(TAG, "Issuing initial full-screen clear (white) for SSD1685 panel");
    const size_t clear_size = configuration->width * configuration->height / 8;
    uint8_t *white_buffer = (uint8_t *)heap_caps_malloc(clear_size, MALLOC_CAP_DMA);
    if (white_buffer) {
        memset(white_buffer, 0xFF, clear_size); // 0xFF = white in this panel format
        // ensure next write goes to black VRAM (we write white as "no black")
        epaper_panel_set_bitmap_color(panelHandle, SSD1685_EPAPER_BITMAP_BLACK);
        // write to VRAM (this will not necessarily refresh yet)
        esp_err_t r = esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, configuration->width, configuration->height, white_buffer);
        if (r != ESP_OK) {
            TT_LOG_W(TAG, "Initial draw_bitmap returned %d", r);
        }
        
        // Also clear the red VRAM to prevent ghosting
        memset(white_buffer, 0x00, clear_size); // 0x00 = no red
        epaper_panel_set_bitmap_color(panelHandle, SSD1685_EPAPER_BITMAP_RED);
        r = esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, configuration->width, configuration->height, white_buffer);
        if (r != ESP_OK) {
            TT_LOG_W(TAG, "Initial red VRAM clear returned %d", r);
        }
        
        // Force immediate refresh
        r = epaper_panel_refresh_screen(panelHandle);
        if (r != ESP_OK) {
            TT_LOG_W(TAG, "epaper_panel_refresh_screen returned %d", r);
        } else {
            // wait for BUSY to finish
            TT_LOG_I(TAG, "Waiting for BUSY to clear after initial refresh...");
            int busy_pin = configuration->busyPin;
            if (busy_pin != GPIO_NUM_NC) {
                // BUSY is high while busy, wait until it goes low
                while (gpio_get_level((gpio_num_t)busy_pin)) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
            } else {
                // fallback small delay
                vTaskDelay(pdMS_TO_TICKS(3000));
            }
            TT_LOG_I(TAG, "Initial refresh finished");
        }
        heap_caps_free(white_buffer);
    } else {
        TT_LOG_W(TAG, "Unable to allocate white buffer for initial clear; skipping initial clear");
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

    // Monochrome displays need full-screen buffer, allocate in PSRAM
    uint32_t buffer_size = logical_width * logical_height;

    TT_LOG_I(TAG, "LVGL config: physical=%dx%d logical=%dx%d rotation=%d buffer_size=%lu pixels",
             configuration->width, configuration->height, logical_width, logical_height,
             configuration->rotation, buffer_size);
    TT_LOG_I(TAG, "LVGL rotation: swap_xy=%d, mirror_x=%d, mirror_y=%d",
             swap_xy, mirror_x, mirror_y);

    return {
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
            .full_refresh = false,
            .direct_mode = false
        }
    };
}

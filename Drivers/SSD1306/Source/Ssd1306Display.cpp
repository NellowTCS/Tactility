#include "Ssd1306Display.h"

#include <Tactility/Log.h>

#include <esp_lcd_panel_commands.h>
#include <esp_lcd_panel_dev.h>
#include <esp_lcd_panel_ssd1306.h>
#include <esp_lvgl_port.h>
#include <esp_lcd_panel_ops.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <memory>

constexpr auto TAG = "SSD1306";

bool Ssd1306Display::createIoHandle(esp_lcd_panel_io_handle_t& outHandle) {
    TT_LOG_I(TAG, "Creating I2C IO handle");
    TT_LOG_I(TAG, "  I2C Port: %d", configuration->port);
    TT_LOG_I(TAG, "  Device Address: 0x%02X", configuration->deviceAddress);

    // Small delay to ensure Vext/power is stable (some Heltec boards need a bit more time).
    TT_LOG_I(TAG, "Delay before creating I2C IO to allow OLED power to stabilize");
    vTaskDelay(pdMS_TO_TICKS(200));

    // Use standard panel IO config
    esp_lcd_panel_io_i2c_config_t panel_io_config = {
        .dev_addr = configuration->deviceAddress,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .flags = {
            .dc_low_on_data = false,
            .disable_control_phase = false,
        },
    };

    TT_LOG_I(TAG, "Calling esp_lcd_new_panel_io_i2c()...");
    esp_err_t ret = esp_lcd_new_panel_io_i2c(
        (esp_lcd_i2c_bus_handle_t)configuration->port, 
        &panel_io_config, 
        &outHandle
    );
    
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create I2C panel IO. Error code: 0x%X (%s)", ret, esp_err_to_name(ret));
        return false;
    }

    TT_LOG_I(TAG, "I2C panel IO created successfully. Handle: %p", outHandle);
    return true;
}

bool Ssd1306Display::createPanelHandle(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t& panelHandle) {
    TT_LOG_I(TAG, "Creating SSD1306 panel handle");
    TT_LOG_I(TAG, "  IO Handle: %p", ioHandle);
    TT_LOG_I(TAG, "  Reset Pin: %d", configuration->resetPin);
    TT_LOG_I(TAG, "  Resolution: %ux%u", configuration->horizontalResolution, configuration->verticalResolution);
    TT_LOG_I(TAG, "  Invert Color: %s", configuration->invertColor ? "true" : "false");

    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = static_cast<uint8_t>(configuration->verticalResolution)
    };

    TT_LOG_I(TAG, "SSD1306 config height: %u", ssd1306_config.height);

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = configuration->resetPin,
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
        .bits_per_pixel = 1,
        .flags = {
            .reset_active_high = false
        },
        .vendor_config = &ssd1306_config
    };

    TT_LOG_I(TAG, "Calling esp_lcd_new_panel_ssd1306()...");
    esp_err_t ret = esp_lcd_new_panel_ssd1306(ioHandle, &panel_config, &panelHandle);
    
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create SSD1306 panel. Error code: 0x%X (%s)", ret, esp_err_to_name(ret));
        return false;
    }

    TT_LOG_I(TAG, "SSD1306 panel created. Handle: %p", panelHandle);

    TT_LOG_I(TAG, "Calling esp_lcd_panel_reset()...");
    ret = esp_lcd_panel_reset(panelHandle);
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to reset panel. Error code: 0x%X (%s)", ret, esp_err_to_name(ret));
        // try a small extra delay and a second reset attempt
        vTaskDelay(pdMS_TO_TICKS(50));
        ret = esp_lcd_panel_reset(panelHandle);
        if (ret != ESP_OK) {
            TT_LOG_E(TAG, "Second panel reset attempt failed: 0x%X (%s)", ret, esp_err_to_name(ret));
            return false;
        }
    }
    TT_LOG_I(TAG, "Panel reset successful");

    TT_LOG_I(TAG, "Calling esp_lcd_panel_init()...");
    ret = esp_lcd_panel_init(panelHandle);
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to init panel. Error code: 0x%X (%s)", ret, esp_err_to_name(ret));
        return false;
    }
    TT_LOG_I(TAG, "Panel init successful");

    TT_LOG_I(TAG, "Calling esp_lcd_panel_invert_color(invert=%s)...", configuration->invertColor ? "true" : "false");
    ret = esp_lcd_panel_invert_color(panelHandle, configuration->invertColor);
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to set panel invert color. Error code: 0x%X (%s)", ret, esp_err_to_name(ret));
        return false;
    }
    TT_LOG_I(TAG, "Invert color set successfully");

    TT_LOG_I(TAG, "Calling esp_lcd_panel_disp_on_off(true)...");
    ret = esp_lcd_panel_disp_on_off(panelHandle, true);
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to turn display on. Error code: 0x%X (%s)", ret, esp_err_to_name(ret));
        return false;
    }
    TT_LOG_I(TAG, "Display turned on successfully");

    // Small settle delay for the panel after display-on command
    vTaskDelay(pdMS_TO_TICKS(100));

    // Diagnostic: try a small set of candidate column offsets and record the first one that returns ESP_OK
    // This helps with boards that map visible columns with an offset (e.g. +32 used by some Heltec variants).
    TT_LOG_I(TAG, "=== Attempting column offset detection (heuristic) ===");
    const int candidates[] = { 0, 32, 2, 4 };
    const size_t candidate_count = sizeof(candidates) / sizeof(candidates[0]);

    const size_t bytes = (configuration->horizontalResolution * configuration->verticalResolution) / 8;
    uint8_t* full = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
    if (!full) {
        TT_LOG_W(TAG, "OOM allocating diagnostic buffer (%u bytes)", (unsigned)bytes);
    } else {
        memset(full, 0xFF, bytes); // all pixels on

        int chosen = 0; // default 0
        for (size_t i = 0; i < candidate_count; ++i) {
            int off = candidates[i];
            // compute end x using offset. keep it within reasonable bounds (esp_lcd may clip)
            int x_end = off + (int)configuration->horizontalResolution - 1;
            TT_LOG_I(TAG, "Trying column offset %d (draw to x=%d..%d)", off, off, x_end);
            esp_err_t draw_ret = esp_lcd_panel_draw_bitmap(panelHandle, off, 0, x_end, (int)configuration->verticalResolution - 1, full);
            if (draw_ret == ESP_OK) {
                TT_LOG_I(TAG, "Draw returned ESP_OK for offset %d", off);
                // Heuristic: accept this offset as the selected one and stop trying further offsets
                chosen = off;
                break;
            } else {
                TT_LOG_W(TAG, "Draw failed for offset %d: 0x%X (%s)", off, draw_ret, esp_err_to_name(draw_ret));
            }
            vTaskDelay(pdMS_TO_TICKS(120));
        }

        // store detected offset in configuration
        configuration->columnOffset = chosen;

        // Apply the offset using esp_lcd_panel_set_gap so the panel driver will add it automatically
        esp_err_t gap_ret = esp_lcd_panel_set_gap(panelHandle, configuration->columnOffset, 0);
        if (gap_ret == ESP_OK) {
            TT_LOG_I(TAG, "Set panel gap (column offset) to %d via esp_lcd_panel_set_gap()", configuration->columnOffset);
        } else {
            TT_LOG_W(TAG, "esp_lcd_panel_set_gap() failed: 0x%X (%s). Continuing with stored columnOffset for higher layers.", gap_ret, esp_err_to_name(gap_ret));
            // If set_gap fails, higher layers can still apply columnOffset if needed.
            setDriverColumnOffset(configuration->columnOffset);
        }

        TT_LOG_I(TAG, "Selected column offset: %d", configuration->columnOffset);

        heap_caps_free(full);
    }

    // Test I2C communication with the original small test
    TT_LOG_I(TAG, "=== Testing I2C data flow (original small test) ===");
    uint8_t test_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    TT_LOG_I(TAG, "Attempting to draw test bitmap (should turn pixels on)...");
    esp_err_t test_ret = esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, 8, 1, test_data);

    if (test_ret != ESP_OK) {
        TT_LOG_E(TAG, "Test draw_bitmap failed. Error code: 0x%X (%s)", test_ret, esp_err_to_name(test_ret));
        TT_LOG_W(TAG, "This means I2C communication might not be working!");
    } else {
        TT_LOG_I(TAG, "Test draw_bitmap succeeded! I2C communication appears to be working.");
        TT_LOG_I(TAG, "If I see pixels on the display now, rendering is the issue.");
        TT_LOG_I(TAG, "If I see nothing, the issue is with LVGL rendering pipeline or panel column offsets.");
    }

    return true;
}

lvgl_port_display_cfg_t Ssd1306Display::getLvglPortDisplayConfig(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t panelHandle) {
    TT_LOG_I(TAG, "=== Creating LVGL port display config ===");
    TT_LOG_I(TAG, "  Buffer size: %u pixels (%u bytes at 1-bit)", 
        configuration->bufferSize,
        configuration->bufferSize / 8);
    TT_LOG_I(TAG, "  Resolution: %ux%u", configuration->horizontalResolution, configuration->verticalResolution);
    TT_LOG_I(TAG, "  Monochrome: true, Color format: I1");
    TT_LOG_I(TAG, "  IO Handle: %p", ioHandle);
    TT_LOG_I(TAG, "  Panel Handle: %p", panelHandle);

    // esp_lcd framework handles flushing via esp_lcd_panel_draw_bitmap()
    // LVGL will call lvgl_port_add_disp() which registers the panel with the display
    lvgl_port_display_cfg_t config = {
        .io_handle = ioHandle,
        .panel_handle = panelHandle,
        .control_handle = nullptr,
        .buffer_size = configuration->bufferSize,
        .double_buffer = false,
        .trans_size = 0,
        .hres = configuration->horizontalResolution,
        .vres = configuration->verticalResolution,
        .monochrome = true,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .color_format = LV_COLOR_FORMAT_I1,
        .flags = {
            .buff_dma = false,
            .buff_spiram = false,
            .sw_rotate = false,
            .swap_bytes = false,
            .full_refresh = true,
            .direct_mode = false  // Let esp_lcd handle the drawing
        }
    };

    TT_LOG_I(TAG, "LVGL config created. Will be passed to lvgl_port_add_disp()");
    
    return config;
}

#include "Ssd1306Display.h"

#include <Tactility/Log.h>

#include <esp_lcd_panel_commands.h>
#include <esp_lcd_panel_dev.h>
#include <esp_lcd_panel_ssd1306.h>
#include <esp_lvgl_port.h>
#include <esp_lcd_panel_ops.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

constexpr auto TAG = "SSD1306";

bool Ssd1306Display::createIoHandle(esp_lcd_panel_io_handle_t& outHandle) {
    TT_LOG_I(TAG, "Creating I2C IO handle");
    TT_LOG_I(TAG, "  I2C Port: %d", configuration->port);
    TT_LOG_I(TAG, "  Device Address: 0x%02X", configuration->deviceAddress);

    // Small delay to ensure Vext/power is stable (some Heltec boards need a bit more time).
    vTaskDelay(pdMS_TO_TICKS(200));

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
        return false;
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

    // Small settle delay
    vTaskDelay(pdMS_TO_TICKS(100));

    // Test I2C communication with multiple patterns
    TT_LOG_I(TAG, "=== Testing I2C data flow with multiple patterns ===");
    
    // Pattern 1: All ones (pixels ON) - full screen
    TT_LOG_I(TAG, "Test 1: Sending 0xFF pattern (all pixels should be ON)...");
    uint8_t pattern_all_on[128] = {};
    memset(pattern_all_on, 0xFF, sizeof(pattern_all_on));
    esp_err_t ret1 = esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, 127, 7, pattern_all_on);
    TT_LOG_I(TAG, "  Result: 0x%X", ret1);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Pattern 2: All zeros (pixels OFF)
    TT_LOG_I(TAG, "Test 2: Sending 0x00 pattern (all pixels should be OFF)...");
    uint8_t pattern_all_off[128] = {};
    memset(pattern_all_off, 0x00, sizeof(pattern_all_off));
    esp_err_t ret2 = esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, 127, 7, pattern_all_off);
    TT_LOG_I(TAG, "  Result: 0x%X", ret2);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Pattern 3: Alternating (checkerboard)
    TT_LOG_I(TAG, "Test 3: Sending alternating pattern (checkerboard)...");
    uint8_t pattern_checker[128] = {};
    for(int i = 0; i < 128; i++) {
        pattern_checker[i] = (i % 2) ? 0xAA : 0x55;  // 10101010 / 01010101
    }
    esp_err_t ret3 = esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, 127, 7, pattern_checker);
    TT_LOG_I(TAG, "  Result: 0x%X", ret3);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Pattern 4: First half ON, second half OFF
    TT_LOG_I(TAG, "Test 4: Sending split pattern (left ON, right OFF)...");
    uint8_t pattern_split[128] = {};
    memset(pattern_split, 0xFF, 64);
    memset(pattern_split + 64, 0x00, 64);
    esp_err_t ret4 = esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, 127, 7, pattern_split);
    TT_LOG_I(TAG, "  Result: 0x%X", ret4);
    
    if (ret1 != ESP_OK || ret2 != ESP_OK || ret3 != ESP_OK || ret4 != ESP_OK) {
        TT_LOG_W(TAG, "One or more test draws failed");
    } else {
        TT_LOG_I(TAG, "All test patterns sent successfully via I2C");
        TT_LOG_I(TAG, "If you see ANY changes on the display, I2C is working");
        TT_LOG_I(TAG, "If nothing changes, the display may need different initialization");
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
            .direct_mode = false
        }
    };

    TT_LOG_I(TAG, "LVGL config ready. Will be passed to lvgl_port_add_disp()");
    
    return config;
}
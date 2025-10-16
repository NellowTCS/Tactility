#include "Ssd1306Display.h"

#include <Tactility/Log.h>

#include <esp_lcd_panel_commands.h>
#include <esp_lcd_panel_dev.h>
#include <esp_lcd_panel_ssd1306.h>
#include <esp_lvgl_port.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <memory>

constexpr auto TAG = "SSD1306";

bool Ssd1306Display::createIoHandle(esp_lcd_panel_io_handle_t& outHandle) {
    TT_LOG_I(TAG, "Creating I2C IO handle");
    TT_LOG_I(TAG, "  I2C Port: %d", configuration->port);
    TT_LOG_I(TAG, "  Device Address: 0x%02X", configuration->deviceAddress);

    // Small delay to ensure Vext/power is stable
    TT_LOG_I(TAG, "Delay before creating I2C IO to allow OLED power to stabilize");
    vTaskDelay(pdMS_TO_TICKS(200));

    // Try a set of candidate panel_io configurations to work around possible bus framing differences.
    // Some displays / boards expect different control phase layouts (dc_bit_offset) or disabled control phase.
    const struct CandidateCfg {
        uint8_t control_phase_bytes;
        uint8_t dc_bit_offset;
        bool disable_control_phase;
        const char* desc;
    } candidates[] = {
        { 1, 6, false, "original dc_bit_offset=6, control phase enabled (original default)" },
        { 1, 0, false, "dc_bit_offset=0, control phase enabled" },
        { 1, 0, true,  "dc_bit_offset=0, disable control phase (data-only writes)" }
    };

    for (size_t i = 0; i < sizeof(candidates)/sizeof(candidates[0]); ++i) {
        TT_LOG_I(TAG, "Attempt %u: trying panel IO config: %s", (unsigned)i+1, candidates[i].desc);

        esp_lcd_panel_io_i2c_config_t panel_io_config = {
            .dev_addr = configuration->deviceAddress,
            .control_phase_bytes = candidates[i].control_phase_bytes,
            .dc_bit_offset = candidates[i].dc_bit_offset,
            .flags = {
                .dc_low_on_data = false,
                .disable_control_phase = candidates[i].disable_control_phase,
            },
        };

        TT_LOG_I(TAG, "Calling esp_lcd_new_panel_io_i2c()...");
        esp_err_t ret = esp_lcd_new_panel_io_i2c(
            (esp_lcd_i2c_bus_handle_t)configuration->port,
            &panel_io_config,
            &outHandle
        );

        if (ret == ESP_OK) {
            TT_LOG_I(TAG, "I2C panel IO created successfully. Handle: %p (candidate %u)", outHandle, (unsigned)i+1);
            return true;
        } else {
            TT_LOG_W(TAG, "esp_lcd_new_panel_io_i2c attempt %u failed: 0x%X (%s)", (unsigned)i+1, ret, esp_err_to_name(ret));
            // continue and try next candidate
        }
    }

    TT_LOG_E(TAG, "Failed to create I2C panel IO with all candidate configurations");
    return false;
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

    // Perform a larger diagnostic test: full-screen fill so I can visually confirm the panel works.
    TT_LOG_I(TAG, "=== Performing full-screen diagnostic draw ===");
    const size_t bytes = (configuration->horizontalResolution * configuration->verticalResolution) / 8;
    uint8_t *full = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
    if (!full) {
        TT_LOG_W(TAG, "OOM allocating full-screen test buffer (%u bytes)", (unsigned)bytes);
    } else {
        // Fill all bytes -> all pixels on
        memset(full, 0xFF, bytes);
        TT_LOG_I(TAG, "Attempting full-screen draw (%ux%u -> %u bytes)...", configuration->horizontalResolution, configuration->verticalResolution, (unsigned)bytes);
        esp_err_t draw_ret = esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, configuration->horizontalResolution, configuration->verticalResolution, full);
        if (draw_ret != ESP_OK) {
            TT_LOG_W(TAG, "Full-screen draw failed: 0x%X (%s)", draw_ret, esp_err_to_name(draw_ret));
        } else {
            TT_LOG_I(TAG, "Full-screen draw returned ESP_OK. If the panel remains blank, the problem is likely LVGL rendering or column offset/sh1106 mismatch.");
        }
        heap_caps_free(full);
    }

    // Do a small invert test (flip invert on/off)
    TT_LOG_I(TAG, "Flipping invert mode briefly to help visual test...");
    ret = esp_lcd_panel_invert_color(panelHandle, !configuration->invertColor);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(250));
        // Restore original invert
        esp_lcd_panel_invert_color(panelHandle, configuration->invertColor);
        TT_LOG_I(TAG, "Invert flip completed");
    } else {
        TT_LOG_W(TAG, "Failed to flip invert mode: 0x%X (%s)", ret, esp_err_to_name(ret));
    }

    // Small checker test (top-left 32x16) to ensure smaller rectangles draw ok
    TT_LOG_I(TAG, "Drawing a small test rectangle (checker) at top-left");
    const int tx = 0, ty = 0, tw = 32, th = 16;
    const size_t tbytes = (tw * th) / 8;
    uint8_t *tbuf = (uint8_t*)heap_caps_malloc(tbytes, MALLOC_CAP_8BIT);
    if (tbuf) {
        // Produce vertical stripes pattern mapped into one-bit pages: easiest is to set columns to 0xFF for visible stripes
        memset(tbuf, 0x00, tbytes);
        memset(tbuf, 0xFF, tbytes);
        esp_err_t rect_ret = esp_lcd_panel_draw_bitmap(panelHandle, tx, ty, tw, th, tbuf);
        if (rect_ret != ESP_OK) {
            TT_LOG_W(TAG, "Small rectangle draw failed: 0x%X (%s)", rect_ret, esp_err_to_name(rect_ret));
        } else {
            TT_LOG_I(TAG, "Small rectangle draw returned ESP_OK");
        }
        heap_caps_free(tbuf);
    } else {
        TT_LOG_W(TAG, "OOM allocating small test buffer (%u bytes)", (unsigned)tbytes);
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
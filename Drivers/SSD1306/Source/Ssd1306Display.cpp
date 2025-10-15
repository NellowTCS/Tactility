#include "Ssd1306Display.h"

#include <Tactility/Log.h>

#include <esp_lcd_panel_commands.h>
#include <esp_lcd_panel_dev.h>
#include <esp_lcd_panel_ssd1306.h>
#include <esp_lvgl_port.h>

constexpr auto TAG = "SSD1306";

bool Ssd1306Display::createIoHandle(esp_lcd_panel_io_handle_t& outHandle) {
    TT_LOG_I(TAG, "Creating I2C IO handle");
    TT_LOG_I(TAG, "  I2C Port: %d", configuration->port);
    TT_LOG_I(TAG, "  Device Address: 0x%02X", configuration->deviceAddress);

    const esp_lcd_panel_io_i2c_config_t panel_io_config = {
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

    // === DEBUG STEP 2: Test I2C communication ===
    TT_LOG_I(TAG, "=== POST-INIT DEBUG (STEP 2) ===");
    TT_LOG_I(TAG, "Panel handle: %p", panelHandle);
    TT_LOG_I(TAG, "IO handle: %p", ioHandle);

    // Try to manually write test data to see if I2C is alive
    uint8_t test_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t test_ret = esp_lcd_panel_draw_bitmap(panelHandle, 0, 0, 8, 1, test_data);
    TT_LOG_I(TAG, "Test draw_bitmap result: 0x%X (%s)", test_ret, esp_err_to_name(test_ret));
    TT_LOG_I(TAG, "=== END DEBUG ===");

    return true;
}

lvgl_port_display_cfg_t Ssd1306Display::getLvglPortDisplayConfig(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t panelHandle) {
    TT_LOG_I(TAG, "Creating LVGL port display config");
    TT_LOG_I(TAG, "  Buffer size: %u pixels", configuration->bufferSize);
    TT_LOG_I(TAG, "  Resolution: %ux%u", configuration->horizontalResolution, configuration->verticalResolution);
    TT_LOG_I(TAG, "  Monochrome: true");
    TT_LOG_I(TAG, "  Color format: LV_COLOR_FORMAT_I1");

    return lvgl_port_display_cfg_t {
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
}

bool Ssd1306Display::startLvgl() {
    TT_LOG_I(TAG, "Starting LVGL for SSD1306 display");

    // === DEBUG STEP 1: Check LVGL display pointer ===
    TT_LOG_I(TAG, "=== PRE-LVGL DEBUG (STEP 1) ===");
    TT_LOG_I(TAG, "LVGL display pointer (before register): %p", lvglDisplay);
    
    if (lvglDisplay) {
        TT_LOG_I(TAG, "LVGL display resolution: %ux%u", 
            lv_display_get_horizontal_resolution(lvglDisplay), 
            lv_display_get_vertical_resolution(lvglDisplay));
    } else {
        TT_LOG_W(TAG, "WARNING: LVGL display is NULL before registration!");
    }
    TT_LOG_I(TAG, "=== END DEBUG ===");

    return true;
}

lv_display_t* Ssd1306Display::getLvglDisplay() const {
    return lvglDisplay;
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto display = std::make_shared<Ssd1306Display>(
        std::make_unique<Ssd1306Display::Configuration>(
            HELTEC_LCD_I2C_PORT,
            HELTEC_LCD_I2C_ADDRESS,
            HELTEC_LCD_PIN_RST,
            HELTEC_LCD_HORIZONTAL_RESOLUTION,
            HELTEC_LCD_VERTICAL_RESOLUTION,
            nullptr, // no touch
            false    // don't invert
        )
    );

    return display;
}
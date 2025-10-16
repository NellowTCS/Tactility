#include "Ssd1306Display.h"

#include <Tactility/Log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>

constexpr auto TAG = "SSD1306_LVGL";

// I2C control bytes
#define OLED_CONTROL_BYTE_CMD_SINGLE  0x80
#define OLED_CONTROL_BYTE_CMD_STREAM  0x00
#define OLED_CONTROL_BYTE_DATA_STREAM 0x40

// Commands
#define OLED_CMD_SET_CONTRAST              0x81
#define OLED_CMD_DISPLAY_NORMAL            0xA6
#define OLED_CMD_DISPLAY_INVERTED          0xA7
#define OLED_CMD_DISPLAY_OFF               0xAE
#define OLED_CMD_DISPLAY_ON                0xAF
#define OLED_CMD_SET_MEMORY_ADDR_MODE      0x20
#define OLED_CMD_SET_COLUMN_RANGE          0x21
#define OLED_CMD_SET_PAGE_RANGE            0x22
#define OLED_CMD_SET_SEGMENT_REMAP         0xA1
#define OLED_CMD_SET_COM_SCAN_MODE_REMAP   0xC8
#define OLED_CMD_SET_DISPLAY_CLK_DIV       0xD5
#define OLED_CMD_SET_PRECHARGE             0xD9
#define OLED_CMD_SET_VCOMH_DESELCT         0xDB
#define OLED_CMD_SET_CHARGE_PUMP           0x8D
#define OLED_CMD_SET_MUX_RATIO             0xA8
#define OLED_CMD_SET_COM_PIN_MAP           0xDA
#define OLED_CMD_SET_DISPLAY_OFFSET        0xD3
#define OLED_CMD_SET_DISPLAY_START_LINE    0x40

// Helper macros
#define BIT_SET(a, b) ((a) |= (1U << (b)))
#define BIT_CLEAR(a, b) ((a) &= ~(1U << (b)))

// Private helper to send I2C data
static esp_err_t ssd1306_i2c_write(i2c_port_t port, uint8_t addr, uint8_t control_byte, 
                                   const uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, control_byte, true);
    
    if (data != nullptr && len > 0) {
        i2c_master_write(cmd, (uint8_t *)data, len, true);
    }
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

bool Ssd1306Display::createIoHandle(esp_lcd_panel_io_handle_t& outHandle) {
    TT_LOG_I(TAG, "SSD1306 I2C setup (not using esp_lcd panel IO)");
    // We're handling I2C manually, so we don't need an esp_lcd IO handle
    // Return a dummy non-null value so the base class doesn't complain
    outHandle = (esp_lcd_panel_io_handle_t)0x1;
    return true;
}

bool Ssd1306Display::createPanelHandle(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t& panelHandle) {
    TT_LOG_I(TAG, "Initializing SSD1306 display");

    // Hardware reset
    if (configuration->resetPin != GPIO_NUM_NC) {
        gpio_config_t rst_cfg = {
            .pin_bit_mask = 1ULL << configuration->resetPin,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&rst_cfg);
        
        gpio_set_level(configuration->resetPin, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(configuration->resetPin, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Init sequence - all commands at once
    uint8_t init_cmds[] = {
        OLED_CMD_SET_CHARGE_PUMP,
        0x14,
        OLED_CMD_SET_SEGMENT_REMAP,
        OLED_CMD_SET_COM_SCAN_MODE_REMAP,
        OLED_CMD_SET_CONTRAST,
        0xFF,
        configuration->invertColor ? OLED_CMD_DISPLAY_INVERTED : OLED_CMD_DISPLAY_NORMAL,
        OLED_CMD_DISPLAY_ON
    };

    esp_err_t ret = ssd1306_i2c_write(configuration->port, configuration->deviceAddress,
                                       OLED_CONTROL_BYTE_CMD_STREAM, init_cmds, sizeof(init_cmds));
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to init SSD1306: 0x%X", ret);
        return false;
    }

    TT_LOG_I(TAG, "SSD1306 initialized successfully");
    
    // Return a dummy non-null handle
    panelHandle = (esp_lcd_panel_handle_t)0x1;
    return true;
}

lvgl_port_display_cfg_t Ssd1306Display::getLvglPortDisplayConfig(esp_lcd_panel_io_handle_t ioHandle, 
                                                                   esp_lcd_panel_handle_t panelHandle) {
    TT_LOG_I(TAG, "Creating LVGL display config");
    TT_LOG_I(TAG, "  Resolution: %ux%u", configuration->horizontalResolution, configuration->verticalResolution);
    TT_LOG_I(TAG, "  Buffer: %u pixels", configuration->bufferSize);

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

// Override the LVGL port display registration to use our custom flush
bool Ssd1306Display::startLvgl() {
    TT_LOG_I(TAG, "Starting LVGL with custom SSD1306 driver");

    if (!EspLcdDisplay::startLvgl()) {
        TT_LOG_E(TAG, "Failed to start LVGL display");
        return false;
    }

    // Store reference for flush callback
    g_ssd1306_instance = this;

    TT_LOG_I(TAG, "LVGL SSD1306 display started successfully");
    return true;
}

// Static instance for callback
Ssd1306Display* Ssd1306Display::g_ssd1306_instance = nullptr;

// LVGL flush callback called whenever LVGL needs to update the display
void Ssd1306Display::lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    if (g_ssd1306_instance == nullptr) {
        return;
    }

    auto cfg = g_ssd1306_instance->configuration;
    
    // Convert pixel coordinates to SSD1306 pages
    // SSD1306 uses 8-pixel vertical pages
    uint8_t row1 = area->y1 >> 3;  // Divide by 8
    uint8_t row2 = area->y2 >> 3;

    // Send column and page range commands
    uint8_t range_cmd[] = {
        OLED_CMD_SET_MEMORY_ADDR_MODE,
        0x00,  // Horizontal addressing mode
        OLED_CMD_SET_COLUMN_RANGE,
        (uint8_t)area->x1,
        (uint8_t)area->x2,
        OLED_CMD_SET_PAGE_RANGE,
        row1,
        row2,
    };

    esp_err_t ret = ssd1306_i2c_write(cfg->port, cfg->deviceAddress,
                                       OLED_CONTROL_BYTE_CMD_STREAM, range_cmd, sizeof(range_cmd));
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to set column/page range: 0x%X", ret);
        return;
    }

    // Send pixel data
    uint16_t data_len = cfg->horizontalResolution * (1 + row2 - row1);
    ret = ssd1306_i2c_write(cfg->port, cfg->deviceAddress,
                            OLED_CONTROL_BYTE_DATA_STREAM, px_map, data_len);
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to send pixel data: 0x%X", ret);
        return;
    }

    lv_display_flush_ready(disp);
}

lv_display_t* Ssd1306Display::getLvglDisplay() const {
    // This is handled by the base class EspLcdDisplay
    return nullptr;  // Will be set by base class
}
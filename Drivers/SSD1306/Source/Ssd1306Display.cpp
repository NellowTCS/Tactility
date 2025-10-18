#include "Ssd1306Display.h"

#include <Tactility/Log.h>
#include <esp_lcd_panel_commands.h>
#include <esp_lcd_panel_dev.h>
#include <esp_lcd_panel_ssd1306.h>
#include <esp_lvgl_port.h>
#include <esp_lcd_panel_ops.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

constexpr auto TAG = "SSD1306";

// SSD1306 Commands
#define SSD1306_CMD_DISPLAY_OFF           0xAE
#define SSD1306_CMD_DISPLAY_ON            0xAF
#define SSD1306_CMD_SET_CLOCK_DIV         0xD5
#define SSD1306_CMD_SET_MUX_RATIO         0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET    0xD3
#define SSD1306_CMD_SET_START_LINE        0x40
#define SSD1306_CMD_CHARGE_PUMP           0x8D
#define SSD1306_CMD_MEM_ADDR_MODE         0x20
#define SSD1306_CMD_SEG_REMAP             0xA1
#define SSD1306_CMD_COM_SCAN_DEC          0xC8
#define SSD1306_CMD_COM_PINS              0xDA
#define SSD1306_CMD_SET_CONTRAST          0x81
#define SSD1306_CMD_SET_PRECHARGE         0xD9
#define SSD1306_CMD_SET_VCOMH             0xDB
#define SSD1306_CMD_NORMAL_DISPLAY        0xA6
#define SSD1306_CMD_COLUMN_ADDR           0x21
#define SSD1306_CMD_PAGE_ADDR             0x22

// I2C control bytes
#define I2C_CONTROL_BYTE_CMD_SINGLE       0x80
#define I2C_CONTROL_BYTE_DATA_STREAM      0x40

static esp_err_t ssd1306_i2c_send_cmd(i2c_port_t port, uint8_t addr, uint8_t cmd) {
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    if (!handle) return ESP_ERR_NO_MEM;
    
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(handle, I2C_CONTROL_BYTE_CMD_SINGLE, true);
    i2c_master_write_byte(handle, cmd, true);
    i2c_master_stop(handle);
    esp_err_t ret = i2c_master_cmd_begin(port, handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(handle);
    return ret;
}

static esp_err_t ssd1306_send_init_sequence(i2c_port_t port, uint8_t addr, uint8_t height) {
    TT_LOG_I(TAG, "Sending SSD1306 init sequence for %d height...", height);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_DISPLAY_OFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_CLOCK_DIV);
    ssd1306_i2c_send_cmd(port, addr, 0xF0);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_MUX_RATIO);
    ssd1306_i2c_send_cmd(port, addr, height - 1);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_DISPLAY_OFFSET);
    ssd1306_i2c_send_cmd(port, addr, 0x00);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_START_LINE);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_CHARGE_PUMP);
    ssd1306_i2c_send_cmd(port, addr, 0x14);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_MEM_ADDR_MODE);
    ssd1306_i2c_send_cmd(port, addr, 0x00);  // Horizontal mode
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SEG_REMAP | 0x01);
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_COM_SCAN_DEC);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_COM_PINS);
    ssd1306_i2c_send_cmd(port, addr, height == 64 ? 0x12 : 0x02);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_CONTRAST);
    ssd1306_i2c_send_cmd(port, addr, 0xCF);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_PRECHARGE);
    ssd1306_i2c_send_cmd(port, addr, 0xF1);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_VCOMH);
    ssd1306_i2c_send_cmd(port, addr, 0x40);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_NORMAL_DISPLAY);
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_DISPLAY_ON);

    TT_LOG_I(TAG, "Init sequence complete");
    return ESP_OK;
}

// write a simple checkerboard directly to the panel (full-screen, 8 pages x 128 bytes)
static void ssd1306_write_checkerboard(i2c_port_t port, uint8_t addr, uint8_t height, uint8_t width) {
    const uint8_t bytes_per_page = width / 8;
    for (uint8_t page = 0; page < (height / 8); ++page) {
        // set page and full column addr in single sequence (commands separated here for clarity)
        ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_PAGE_ADDR);
        ssd1306_i2c_send_cmd(port, addr, page);
        ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_COLUMN_ADDR);
        ssd1306_i2c_send_cmd(port, addr, 0);
        ssd1306_i2c_send_cmd(port, addr, width - 1);

        // build page buffer: alternate 0xFF/0x00 per column block to create visible bands
        uint8_t buf[128];
        for (int i = 0; i < bytes_per_page; ++i) {
            // simple pattern: every other byte is 0xFF to produce vertical stripes
            buf[i] = ( (i + page) & 1 ) ? 0xFF : 0x00;
        }

        // send data
        i2c_cmd_handle_t h = i2c_cmd_link_create();
        if (!h) return;
        i2c_master_start(h);
        i2c_master_write_byte(h, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(h, I2C_CONTROL_BYTE_DATA_STREAM, true);
        i2c_master_write(h, buf, bytes_per_page, true);
        i2c_master_stop(h);
        esp_err_t ret = i2c_master_cmd_begin(port, h, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(h);
        if (ret != ESP_OK) {
            TT_LOG_E(TAG, "Checkerboard write failed on page %u: %s", page, esp_err_to_name(ret));
            return;
        }
    }
}

// Custom LVGL flush callback
static void ssd1306_flush_callback(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    Ssd1306Display *display = (Ssd1306Display *)lv_display_get_user_data(disp);
    if (display) {
        display->flushDirect(area, px_map);
    }
    lv_display_flush_ready(disp);
}

bool Ssd1306Display::createIoHandle(esp_lcd_panel_io_handle_t& outHandle) {
    TT_LOG_I(TAG, "Creating I2C IO handle");

    vTaskDelay(pdMS_TO_TICKS(200));

    const esp_lcd_panel_io_i2c_config_t panel_io_config = {
        .dev_addr = configuration->deviceAddress,
        .control_phase_bytes = 1,
        .dc_bit_offset = 6,
        .flags = {
            .dc_low_on_data = false,
            .disable_control_phase = false,
        },
    };

    esp_err_t ret = esp_lcd_new_panel_io_i2c(
        (esp_lcd_i2c_bus_handle_t)configuration->port, 
        &panel_io_config, 
        &outHandle
    );
    
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create I2C panel IO. Error code: 0x%X (%s)", ret, esp_err_to_name(ret));
        return false;
    }

    TT_LOG_I(TAG, "I2C panel IO created successfully");
    return true;
}

bool Ssd1306Display::createPanelHandle(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t& panelHandle) {
    TT_LOG_I(TAG, "Creating SSD1306 panel handle");

    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = static_cast<uint8_t>(configuration->verticalResolution)
    };

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = configuration->resetPin,
        .color_space = ESP_LCD_COLOR_SPACE_MONOCHROME,
        .bits_per_pixel = 1,
        .flags = {
            .reset_active_high = false
        },
        .vendor_config = &ssd1306_config
    };

    esp_err_t ret = esp_lcd_new_panel_ssd1306(ioHandle, &panel_config, &panelHandle);
    
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to create SSD1306 panel. Error code: 0x%X (%s)", ret, esp_err_to_name(ret));
        return false;
    }

    TT_LOG_I(TAG, "SSD1306 panel created");

    // Hardware reset manually
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
    
    // Send our custom init sequence via I2C
    ssd1306_send_init_sequence(configuration->port, configuration->deviceAddress, 
                               configuration->verticalResolution);
    

    // Quick test: draw a checkerboard to validate panel/I2C independent of LVGL
    TT_LOG_I(TAG, "SSD1306: performing checkerboard hardware test");
    ssd1306_write_checkerboard(configuration->port, configuration->deviceAddress,
                               configuration->verticalResolution, configuration->horizontalResolution);
    vTaskDelay(pdMS_TO_TICKS(500)); // visible pause

    TT_LOG_I(TAG, "Panel initialization complete");
    return true;
}

lvgl_port_display_cfg_t Ssd1306Display::getLvglPortDisplayConfig(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t panelHandle) {
    TT_LOG_I(TAG, "LVGL config: %ux%u buffer=%u", 
        configuration->horizontalResolution, 
        configuration->verticalResolution,
        configuration->bufferSize);

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

    vTaskDelay(pdMS_TO_TICKS(100));

    lv_display_t *disp = lv_display_get_default();
    if (disp != nullptr) {
        lv_display_set_render_mode(disp, LV_DISPLAY_RENDER_MODE_FULL);
        lv_display_set_user_data(disp, this);
        lv_display_set_flush_cb(disp, ssd1306_flush_callback);
        TT_LOG_I(TAG, "Custom flush callback registered");
    }

    TT_LOG_I(TAG, "LVGL config ready");
    return config;
}

static void log_pxmap_snippet(const char *tag, const uint8_t *buf, int len) {
    // Print up to len bytes, in hex, 16 bytes per line
    const int perline = 16;
    int printed = 0;
    while (printed < len) {
        int chunk = (len - printed) > perline ? perline : (len - printed);
        char line[perline * 3 + 1];
        int pos = 0;
        for (int i = 0; i < chunk; ++i) {
            pos += snprintf(&line[pos], sizeof(line) - pos, "%02X ", buf[printed + i]);
        }
        TT_LOG_I(TAG, "%s: %s", tag, line);
        printed += chunk;
    }
}

void Ssd1306Display::flushDirect(const lv_area_t *area, uint8_t *px_map) {
    if (!area || !px_map) return;
    
    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2 + 1; // convert to half-open range (exclusive)
    
    uint8_t page_start = y1 / 8;
    uint8_t page_end = (y2 + 7) / 8; // exclusive end page index
    
    const uint16_t bytes_per_page = configuration->horizontalResolution / 8;
    const uint16_t pages_in_area = (page_end > page_start) ? (page_end - page_start) : 0;
    const uint32_t expected_bytes = pages_in_area * bytes_per_page;

    // Debug dump: area + basic counts
    if (configuration->debugDumpPxMap) {
        TT_LOG_I(TAG, "flushDirect: area x1=%d y1=%d x2=%d y2=%d", area->x1, area->y1, area->x2, area->y2);
        TT_LOG_I(TAG, "flushDirect: page_start=%u page_end=%u pages=%u bytes_per_page=%u expected_bytes=%u",
                 page_start, page_end, (unsigned)pages_in_area, (unsigned)bytes_per_page, (unsigned)expected_bytes);

        // Print a small snippet of the incoming px_map (first up to 64 bytes or expected_bytes)
        int dump_len = (int)expected_bytes;
        if (dump_len > 64) dump_len = 64;
        if (dump_len > 0) {
            log_pxmap_snippet("px_map_first_bytes", px_map, dump_len);
        } else {
            TT_LOG_I(TAG, "px_map: expected_bytes == 0, nothing to dump");
        }
    }

    for (uint8_t page = page_start; page < page_end; page++) {
        // Set page address (single-byte commands)
        ssd1306_i2c_send_cmd(configuration->port, configuration->deviceAddress, SSD1306_CMD_PAGE_ADDR);
        ssd1306_i2c_send_cmd(configuration->port, configuration->deviceAddress, page);
        
        // Set column address to full range (0..127)
        ssd1306_i2c_send_cmd(configuration->port, configuration->deviceAddress, SSD1306_CMD_COLUMN_ADDR);
        ssd1306_i2c_send_cmd(configuration->port, configuration->deviceAddress, 0);
        ssd1306_i2c_send_cmd(configuration->port, configuration->deviceAddress, configuration->horizontalResolution - 1);
        
        // Calculate data offset for this page INSIDE px_map
        // px_map is expected to contain pages in order starting from page_start,
        // so offset is (page - page_start) * bytes_per_page
        uint16_t offset = (page - page_start) * bytes_per_page;

        // Extra debug per-page (first 16 bytes of this page)
        if (configuration->debugDumpPxMap) {
            int snippet_len = bytes_per_page < 16 ? bytes_per_page : 16;
            log_pxmap_snippet(("page_snippet_" + std::to_string(page)).c_str(), px_map + offset, snippet_len);
        }
        
        // Write data bytes for this page (always full-width rows per page)
        i2c_cmd_handle_t handle = i2c_cmd_link_create();
        if (!handle) {
            TT_LOG_E(TAG, "Failed to create I2C handle");
            continue;
        }
        
        i2c_master_start(handle);
        i2c_master_write_byte(handle, (configuration->deviceAddress << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(handle, I2C_CONTROL_BYTE_DATA_STREAM, true);
        i2c_master_write(handle, px_map + offset, bytes_per_page, true);
        i2c_master_stop(handle);
        
        esp_err_t ret = i2c_master_cmd_begin(configuration->port, handle, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(handle);
        
        if (ret != ESP_OK) {
            TT_LOG_E(TAG, "I2C transfer failed on page %d: %s", page, esp_err_to_name(ret));
        }
    }
}
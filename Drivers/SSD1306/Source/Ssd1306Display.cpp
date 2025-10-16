#include "Ssd1306Display.h"

#include <Tactility/Log.h>
#include <esp_lvgl_port.h>
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
#define I2C_CONTROL_BYTE_CMD_STREAM       0x00
#define I2C_CONTROL_BYTE_DATA_STREAM      0x40

static esp_err_t ssd1306_i2c_send_cmd(i2c_port_t port, uint8_t addr, uint8_t cmd) {
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(handle, I2C_CONTROL_BYTE_CMD_SINGLE, true);
    i2c_master_write_byte(handle, cmd, true);
    i2c_master_stop(handle);
    esp_err_t ret = i2c_master_cmd_begin(port, handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(handle);
    return ret;
}

static esp_err_t ssd1306_i2c_send_data(i2c_port_t port, uint8_t addr, const uint8_t *data, size_t len) {
    i2c_cmd_handle_t handle = i2c_cmd_link_create();
    i2c_master_start(handle);
    i2c_master_write_byte(handle, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(handle, I2C_CONTROL_BYTE_DATA_STREAM, true);
    i2c_master_write(handle, (uint8_t *)data, len, true);
    i2c_master_stop(handle);
    esp_err_t ret = i2c_master_cmd_begin(port, handle, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(handle);
    return ret;
}

static esp_err_t ssd1306_i2c_init(i2c_port_t port, uint8_t addr) {
    TT_LOG_I(TAG, "Sending SSD1306 init sequence...");
    
    // Based on Heltec's SSD1306Wire library
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_DISPLAY_OFF);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_CLOCK_DIV);
    ssd1306_i2c_send_cmd(port, addr, 0xF0);  // High clock speed
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_MUX_RATIO);
    ssd1306_i2c_send_cmd(port, addr, 0x3F);  // 64 MUX (0x3F for 64px height)
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_DISPLAY_OFFSET);
    ssd1306_i2c_send_cmd(port, addr, 0x00);
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_START_LINE);  // 0x40
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_CHARGE_PUMP);
    ssd1306_i2c_send_cmd(port, addr, 0x14);  // Enable charge pump
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_MEM_ADDR_MODE);
    ssd1306_i2c_send_cmd(port, addr, 0x00);  // Horizontal addressing mode
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SEG_REMAP | 0x01);  // Remap columns
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_COM_SCAN_DEC);      // COM scan direction
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_COM_PINS);
    ssd1306_i2c_send_cmd(port, addr, 0x12);  // COM pins config
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_CONTRAST);
    ssd1306_i2c_send_cmd(port, addr, 0xCF);  // Contrast
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_PRECHARGE);
    ssd1306_i2c_send_cmd(port, addr, 0xF1);  // Precharge period
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_SET_VCOMH);
    ssd1306_i2c_send_cmd(port, addr, 0x40);  // VCOMH deselect level
    
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_NORMAL_DISPLAY);
    ssd1306_i2c_send_cmd(port, addr, SSD1306_CMD_DISPLAY_ON);
    
    TT_LOG_I(TAG, "SSD1306 init complete");
    return ESP_OK;
}

bool Ssd1306Display::createIoHandle(esp_lcd_panel_io_handle_t& outHandle) {
    TT_LOG_I(TAG, "Creating custom I2C IO (bypassing esp_lcd)");
    
    // Delay for power stabilization
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Initialize the display with custom I2C commands
    esp_err_t ret = ssd1306_i2c_init(configuration->port, configuration->deviceAddress);
    if (ret != ESP_OK) {
        TT_LOG_E(TAG, "Failed to init SSD1306: 0x%X", ret);
        return false;
    }
    
    // Return dummy handle (we're handling I2C manually)
    outHandle = (esp_lcd_panel_io_handle_t)0xDEADBEEF;
    return true;
}

bool Ssd1306Display::createPanelHandle(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t& panelHandle) {
    TT_LOG_I(TAG, "Creating panel handle (custom I2C mode, no esp_lcd panel)");
    panelHandle = (esp_lcd_panel_handle_t)0xCAFEBABE;
    return true;
}

lvgl_port_display_cfg_t Ssd1306Display::getLvglPortDisplayConfig(esp_lcd_panel_io_handle_t ioHandle, esp_lcd_panel_handle_t panelHandle) {
    TT_LOG_I(TAG, "=== Creating LVGL port display config ===");
    TT_LOG_I(TAG, "  Buffer size: %u pixels (%u bytes at 1-bit)", 
        configuration->bufferSize,
        configuration->bufferSize / 8);
    TT_LOG_I(TAG, "  Resolution: %ux%u", configuration->horizontalResolution, configuration->verticalResolution);

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

    return config;
}
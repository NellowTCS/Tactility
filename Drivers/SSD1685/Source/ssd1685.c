/**
 * @file ssd1685.c
 * SSD1685 e-paper controller driver
 */
 
#include "ssd1685.h"
#include <string.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "SSD1685";

/* Partial update LUT - fast refresh waveform */
static const uint8_t ssd1685_lut_partial[] = {
    0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
    0x00, 0x00, 0x00, 0x22, 0x17, 0x41, 0xB0, 0x32, 0x36,
};

static inline void ssd1685_command_mode(ssd1685_handle_t *handle);
static inline void ssd1685_data_mode(ssd1685_handle_t *handle);
static inline void ssd1685_waitbusy(ssd1685_handle_t *handle, int wait_ms);
static inline void ssd1685_hw_reset(ssd1685_handle_t *handle);
static inline void ssd1685_write_cmd(ssd1685_handle_t *handle, uint8_t cmd, uint8_t *data, size_t len);
static inline void ssd1685_send_cmd(ssd1685_handle_t *handle, uint8_t cmd);
static inline void ssd1685_send_data(ssd1685_handle_t *handle, uint8_t *data, uint16_t length);
static inline void ssd1685_set_window(ssd1685_handle_t *handle, uint16_t sx, uint16_t ex, uint16_t ys, uint16_t ye);
static inline void ssd1685_set_cursor(ssd1685_handle_t *handle, uint16_t sx, uint16_t ys);
static void ssd1685_update_display_full(ssd1685_handle_t *handle);
static void ssd1685_update_display_partial(ssd1685_handle_t *handle);

void ssd1685_init_io(ssd1685_handle_t *handle)
{
    if (handle->dc_pin >= 0) {
        gpio_set_direction(handle->dc_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(handle->dc_pin, 1);
    }

    if (handle->busy_pin >= 0) {
        gpio_set_direction(handle->busy_pin, GPIO_MODE_INPUT);
    }

    if (handle->rst_pin >= 0) {
        gpio_set_direction(handle->rst_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(handle->rst_pin, 1);
    }

    handle->last_update_mode = SSD1685_UPDATE_MODE_INIT;
    handle->partial_mode_active = false;
    handle->partial_refresh_count = 0;

    ESP_LOGI(TAG, "IO pins initialized");
}

void ssd1685_deinit_io(ssd1685_handle_t *handle)
{
    if (handle->dc_pin >= 0) {
        gpio_set_direction(handle->dc_pin, GPIO_MODE_INPUT);
    }
    if (handle->busy_pin >= 0) {
        gpio_set_direction(handle->busy_pin, GPIO_MODE_INPUT);
    }
    if (handle->rst_pin >= 0) {
        gpio_set_direction(handle->rst_pin, GPIO_MODE_INPUT);
    }
}

static inline void ssd1685_command_mode(ssd1685_handle_t *handle)
{
    if (handle->dc_pin >= 0) {
        gpio_set_level(handle->dc_pin, 0);
    }
}

static inline void ssd1685_data_mode(ssd1685_handle_t *handle)
{
    if (handle->dc_pin >= 0) {
        gpio_set_level(handle->dc_pin, 1);
    }
}

static inline void ssd1685_waitbusy(ssd1685_handle_t *handle, int wait_ms)
{
    if (handle->busy_pin < 0) {
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    for (int i = 0; i < (wait_ms * 10); i++) {
        if (gpio_get_level(handle->busy_pin) != SSD1685_BUSY_LEVEL) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGE(TAG, "busy exceeded %dms", wait_ms);
}

static inline void ssd1685_hw_reset(ssd1685_handle_t *handle)
{
    if (handle->rst_pin < 0) {
        return;
    }

    gpio_set_level(handle->rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(SSD1685_RESET_DELAY));
    gpio_set_level(handle->rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(SSD1685_RESET_DELAY));
}

static inline void ssd1685_write_cmd(ssd1685_handle_t *handle, uint8_t cmd, uint8_t *data, size_t len)
{
    ssd1685_command_mode(handle);
    
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_transmit(handle->spi_handle, &t);

    if (data != NULL && len > 0) {
        ssd1685_data_mode(handle);
        t.length = len * 8;
        t.tx_buffer = data;
        spi_device_transmit(handle->spi_handle, &t);
    }
}

static inline void ssd1685_send_cmd(ssd1685_handle_t *handle, uint8_t cmd)
{
    ssd1685_write_cmd(handle, cmd, NULL, 0);
}

static inline void ssd1685_send_data(ssd1685_handle_t *handle, uint8_t *data, uint16_t length)
{
    ssd1685_data_mode(handle);
    
    spi_transaction_t t = {
        .length = length * 8,
        .tx_buffer = data,
    };
    spi_device_transmit(handle->spi_handle, &t);
}

static inline void ssd1685_set_window(ssd1685_handle_t *handle, uint16_t sx, uint16_t ex, uint16_t ys, uint16_t ye)
{
    uint8_t tmp[4] = {0};

    // Set RAM X address (horizontal)
    tmp[0] = (sx + SSD1685_SOURCE_SHIFT) / 8;
    tmp[1] = (ex + SSD1685_SOURCE_SHIFT) / 8;
    ssd1685_write_cmd(handle, SSD1685_CMD_SET_RAM_X_ADDR_START_END, tmp, 2);

    // Set RAM Y address (vertical)
    tmp[0] = ys & 0xFF;
    tmp[1] = (ys >> 8) & 0xFF;
    tmp[2] = ye & 0xFF;
    tmp[3] = (ye >> 8) & 0xFF;
    ssd1685_write_cmd(handle, SSD1685_CMD_SET_RAM_Y_ADDR_START_END, tmp, 4);
}

static inline void ssd1685_set_cursor(ssd1685_handle_t *handle, uint16_t sx, uint16_t ys)
{
    uint8_t tmp[2] = {0};

    tmp[0] = (sx + SSD1685_SOURCE_SHIFT) / 8;
    ssd1685_write_cmd(handle, SSD1685_CMD_SET_RAM_X_ADDR_COUNTER, tmp, 1);

    tmp[0] = ys & 0xFF;
    tmp[1] = (ys >> 8) & 0xFF;
    ssd1685_write_cmd(handle, SSD1685_CMD_SET_RAM_Y_ADDR_COUNTER, tmp, 2);
}

void ssd1685_init(ssd1685_handle_t *handle)
{
    uint8_t tmp[3] = {0};

    ESP_LOGI(TAG, "Starting full initialization");

    ssd1685_hw_reset(handle);
    ssd1685_waitbusy(handle, SSD1685_WAIT);

    // Software reset
    ssd1685_send_cmd(handle, SSD1685_CMD_SW_RESET);
    ssd1685_waitbusy(handle, SSD1685_WAIT);

    // Driver output control
    tmp[0] = (EPD_PANEL_HEIGHT - 1) & 0xFF;
    tmp[1] = ((EPD_PANEL_HEIGHT - 1) >> 8) & 0xFF;
    tmp[2] = 0x00;
    ssd1685_write_cmd(handle, SSD1685_CMD_DRIVER_OUTPUT_CTRL, tmp, 3);

    // Data entry mode
    uint8_t scan_mode = SSD1685_DATA_ENTRY_XIYIY;
    ssd1685_write_cmd(handle, SSD1685_CMD_DATA_ENTRY_MODE, &scan_mode, 1);

    // Set window to full screen
    ssd1685_set_window(handle, 0, EPD_PANEL_WIDTH - 1, 0, EPD_PANEL_HEIGHT - 1);

    // Border waveform
    uint8_t border = SSD1685_BORDER_WAVEFORM_INIT;
    ssd1685_write_cmd(handle, SSD1685_CMD_BORDER_WAVEFORM_CTRL, &border, 1);

    // Display update control
    tmp[0] = 0x00;
    tmp[1] = 0x80;
    ssd1685_write_cmd(handle, SSD1685_CMD_DISPLAY_UPDATE_CTRL1, tmp, 2);

    // Temperature sensor
    tmp[0] = 0x80;
    ssd1685_write_cmd(handle, SSD1685_CMD_TEMPERATURE_SENSOR, tmp, 1);

    // Set cursor to start
    ssd1685_set_cursor(handle, 0, 0);
    ssd1685_waitbusy(handle, SSD1685_WAIT);

    handle->last_update_mode = SSD1685_UPDATE_MODE_FULL;
    handle->partial_mode_active = false;
    handle->partial_refresh_count = 0;

    ESP_LOGI(TAG, "Full initialization complete");
}

void ssd1685_init_partial(ssd1685_handle_t *handle)
{
    uint8_t tmp[3] = {0};

    ESP_LOGI(TAG, "Initializing partial update mode");

    // No full reset - just reconfigure
    ssd1685_waitbusy(handle, SSD1685_WAIT);

    // Load partial update LUT
    ssd1685_write_cmd(handle, SSD1685_CMD_WRITE_LUT, (uint8_t*)ssd1685_lut_partial, sizeof(ssd1685_lut_partial));

    // Write temperature for fast update
    tmp[0] = 0x5A; // 90 in decimal, represents ~22°C
    tmp[1] = 0x00;
    ssd1685_write_cmd(handle, SSD1685_CMD_WRITE_TEMP_REGISTER, tmp, 2);

    // Border waveform for partial
    uint8_t border = SSD1685_BORDER_WAVEFORM_PARTIAL;
    ssd1685_write_cmd(handle, SSD1685_CMD_BORDER_WAVEFORM_CTRL, &border, 1);

    // Display update control for partial
    tmp[0] = 0x00;
    tmp[1] = 0x00;
    ssd1685_write_cmd(handle, SSD1685_CMD_DISPLAY_UPDATE_CTRL1, tmp, 2);

    handle->partial_mode_active = true;
    handle->last_update_mode = SSD1685_UPDATE_MODE_PARTIAL;

    ESP_LOGI(TAG, "Partial update mode initialized");
}

void ssd1685_set_partial_window(ssd1685_handle_t *handle, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    // Ensure coordinates are on byte boundaries
    x &= 0xFFF8;  // Align to 8-pixel boundary
    w = (w + 7) & 0xFFF8;  // Round up to 8-pixel boundary

    uint16_t xe = x + w - 1;
    uint16_t ye = y + h - 1;

    // Clamp to screen bounds
    if (xe >= EPD_PANEL_WIDTH) xe = EPD_PANEL_WIDTH - 1;
    if (ye >= EPD_PANEL_HEIGHT) ye = EPD_PANEL_HEIGHT - 1;

    ssd1685_set_window(handle, x, xe, y, ye);
    ssd1685_set_cursor(handle, x, y);
}

static void ssd1685_update_display_full(ssd1685_handle_t *handle)
{
    uint8_t tmp;

    ESP_LOGD(TAG, "Executing full display update");

    // Full update sequence
    tmp = 0xF7;
    ssd1685_write_cmd(handle, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &tmp, 1);
    ssd1685_send_cmd(handle, SSD1685_CMD_MASTER_ACTIVATION);
    ssd1685_waitbusy(handle, 2000);  // Full refresh takes ~2 seconds

    handle->partial_refresh_count = 0;
}

static void ssd1685_update_display_partial(ssd1685_handle_t *handle)
{
    uint8_t tmp;

    ESP_LOGD(TAG, "Executing partial display update");

    // Partial update sequence
    tmp = 0xDF;  // Fast partial update
    ssd1685_write_cmd(handle, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &tmp, 1);
    ssd1685_send_cmd(handle, SSD1685_CMD_MASTER_ACTIVATION);
    ssd1685_waitbusy(handle, 400);  // Partial refresh takes ~400ms

    handle->partial_refresh_count++;
}

void ssd1685_flush_full(ssd1685_handle_t *handle, const uint8_t *buffer)
{
    size_t linelen = EPD_PANEL_WIDTH / 8;
    const uint8_t *data = buffer;

    ESP_LOGI(TAG, "Flushing FULL refresh");

    // Reinitialize for full update if coming from partial mode
    if (handle->partial_mode_active) {
        ssd1685_init(handle);
    }

    // Write to black/white RAM
    ssd1685_set_window(handle, 0, EPD_PANEL_WIDTH - 1, 0, EPD_PANEL_HEIGHT - 1);
    ssd1685_set_cursor(handle, 0, 0);
    ssd1685_send_cmd(handle, SSD1685_CMD_WRITE_BLACK_VRAM);
    
    for (size_t row = 0; row < EPD_PANEL_HEIGHT; row++) {
        ssd1685_send_data(handle, (uint8_t *)data, linelen);
        data += linelen;
    }

    // Also write to red RAM (unused, but set to same as BW for consistency)
    data = buffer;
    ssd1685_set_cursor(handle, 0, 0);
    ssd1685_send_cmd(handle, SSD1685_CMD_WRITE_RED_VRAM);
    
    for (size_t row = 0; row < EPD_PANEL_HEIGHT; row++) {
        ssd1685_send_data(handle, (uint8_t *)data, linelen);
        data += linelen;
    }

    ssd1685_update_display_full(handle);
}

void ssd1685_flush_partial(ssd1685_handle_t *handle, const uint8_t *buffer)
{
    size_t linelen = EPD_PANEL_WIDTH / 8;
    const uint8_t *data = buffer;

    ESP_LOGI(TAG, "Flushing PARTIAL refresh (count: %d)", handle->partial_refresh_count);

    // Force full refresh every 5 partial updates to prevent ghosting
    if (handle->partial_refresh_count >= 5) {
        ESP_LOGI(TAG, "Forcing full refresh after 5 partial updates");
        ssd1685_flush_full(handle, buffer);
        return;
    }

    // Initialize partial mode if needed
    if (!handle->partial_mode_active) {
        ssd1685_init_partial(handle);
    }

    // Write to black/white RAM
    ssd1685_set_window(handle, 0, EPD_PANEL_WIDTH - 1, 0, EPD_PANEL_HEIGHT - 1);
    ssd1685_set_cursor(handle, 0, 0);
    ssd1685_send_cmd(handle, SSD1685_CMD_WRITE_BLACK_VRAM);
    
    for (size_t row = 0; row < EPD_PANEL_HEIGHT; row++) {
        ssd1685_send_data(handle, (uint8_t *)data, linelen);
        data += linelen;
    }

    ssd1685_update_display_partial(handle);
}

void ssd1685_deep_sleep(ssd1685_handle_t *handle)
{
    uint8_t data[] = {0x01};

    ssd1685_waitbusy(handle, SSD1685_WAIT);
    ssd1685_write_cmd(handle, SSD1685_CMD_DEEP_SLEEP_MODE, data, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    handle->partial_mode_active = false;
    handle->last_update_mode = SSD1685_UPDATE_MODE_INIT;
}

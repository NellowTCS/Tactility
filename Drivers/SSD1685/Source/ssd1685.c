/**
 * @file ssd1685.c
 * SSD1685 e-paper controller driver for GDEY029T71H panel
 * Ported to ESP-IDF v5 + LVGL v9
 * Based on SSD1680 driver by Aram Vartanyan
 */
 
#include "ssd1685.h"
#include <string.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "SSD1685";

#define BIT_SET(a, b)    ((a) |= (1U << (b)))
#define BIT_CLEAR(a, b)  ((a) &= ~(1U << (b)))

static uint8_t ssd1685_scan_mode = SSD1685_DATA_ENTRY_XIYIY;
static uint8_t partial_counter = 0;
static uint8_t ssd1685_border_init[] = {SSD1685_BORDER_WAVEFORM_INIT};
static uint8_t ssd1685_border_part[] = {SSD1685_BORDER_WAVEFORM_PARTIAL};

static inline void ssd1685_set_window(ssd1685_handle_t *handle, uint16_t sx, uint16_t ex, uint16_t ys, uint16_t ye);
static inline void ssd1685_set_cursor(ssd1685_handle_t *handle, uint16_t sx, uint16_t ys);
static void ssd1685_update_display(ssd1685_handle_t *handle, bool isPartial);
static inline void ssd1685_waitbusy(ssd1685_handle_t *handle, int wait_ms);
static inline void ssd1685_hw_reset(ssd1685_handle_t *handle);
static inline void ssd1685_write_cmd(ssd1685_handle_t *handle, uint8_t cmd, uint8_t *data, size_t len);
static inline void ssd1685_send_cmd(ssd1685_handle_t *handle, uint8_t cmd);
static inline void ssd1685_send_data(ssd1685_handle_t *handle, uint8_t *data, uint16_t length);

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

    tmp[0] = (sx + SSD1685_SOURCE_SHIFT) / 8;
    tmp[1] = (ex + SSD1685_SOURCE_SHIFT) / 8;

    ssd1685_write_cmd(handle, SSD1685_CMD_SET_RAM_X_ADDR_START_END, tmp, 2);

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

static void ssd1685_update_display(ssd1685_handle_t *handle, bool isPartial)
{
    uint8_t tmp = isPartial ? 0xDF : 0xF7;

    ssd1685_write_cmd(handle, SSD1685_CMD_DISPLAY_UPDATE_CTRL2, &tmp, 1);
    ssd1685_write_cmd(handle, SSD1685_CMD_MASTER_ACTIVATION, NULL, 0);
    ssd1685_waitbusy(handle, SSD1685_WAIT);
}

void ssd1685_init(ssd1685_handle_t *handle)
{
    uint8_t tmp[3] = {0};

    ssd1685_waitbusy(handle, SSD1685_WAIT);

    ssd1685_write_cmd(handle, SSD1685_CMD_SW_RESET, NULL, 0);
    ssd1685_waitbusy(handle, SSD1685_WAIT);

    tmp[0] = (EPD_PANEL_HEIGHT - 1) & 0xFF;
    tmp[1] = ((EPD_PANEL_HEIGHT - 1) >> 8) & 0xFF;
    tmp[2] = 0x00;
    ssd1685_write_cmd(handle, SSD1685_CMD_DRIVER_OUTPUT_CTRL, tmp, 3);

    ssd1685_write_cmd(handle, SSD1685_CMD_DATA_ENTRY_MODE, &ssd1685_scan_mode, 1);

    ssd1685_set_window(handle, 0, EPD_PANEL_WIDTH - 1, 0, EPD_PANEL_HEIGHT - 1);

    ssd1685_write_cmd(handle, SSD1685_CMD_BORDER_WAVEFORM_CTRL, ssd1685_border_init, 1);

    tmp[0] = 0x00;
    tmp[1] = 0x80;
    ssd1685_write_cmd(handle, SSD1685_CMD_DISPLAY_UPDATE_CTRL1, tmp, 2);

    tmp[0] = 0x80;
    ssd1685_write_cmd(handle, SSD1685_CMD_TEMPERATURE_SENSOR, tmp, 1);

    ssd1685_set_cursor(handle, 0, EPD_PANEL_HEIGHT - 1);
    ssd1685_waitbusy(handle, SSD1685_WAIT);

    ESP_LOGI(TAG, "Display initialized");
}

void ssd1685_flush_buffer(ssd1685_handle_t *handle, const uint8_t *buffer, bool partial)
{
    size_t linelen = EPD_PANEL_WIDTH / 8;
    const uint8_t *data = buffer;

    if (!partial) {
        ESP_LOGD(TAG, "Flushing FULL refresh");

        ssd1685_send_cmd(handle, SSD1685_CMD_WRITE_BLACK_VRAM);
        for (size_t row = 0; row < EPD_PANEL_HEIGHT; row++) {
            ssd1685_send_data(handle, (uint8_t *)data, linelen);
            data += linelen;
        }

        data = buffer;

        ssd1685_send_cmd(handle, SSD1685_CMD_WRITE_RED_VRAM);
        for (size_t row = 0; row < EPD_PANEL_HEIGHT; row++) {
            ssd1685_send_data(handle, (uint8_t *)data, linelen);
            data += linelen;
        }

        ssd1685_update_display(handle, false);
        partial_counter = 5;
    } else {
        ESP_LOGD(TAG, "Flushing PARTIAL refresh");

        ssd1685_hw_reset(handle);
        ssd1685_write_cmd(handle, SSD1685_CMD_BORDER_WAVEFORM_CTRL, ssd1685_border_part, 1);

        ssd1685_send_cmd(handle, SSD1685_CMD_WRITE_BLACK_VRAM);
        data = buffer;
        for (size_t row = 0; row < EPD_PANEL_HEIGHT; row++) {
            ssd1685_send_data(handle, (uint8_t *)data, linelen);
            data += linelen;
        }

        ssd1685_update_display(handle, true);
        partial_counter--;
    }
}

void ssd1685_deep_sleep(ssd1685_handle_t *handle)
{
    uint8_t data[] = {0x01};

    ssd1685_waitbusy(handle, SSD1685_WAIT);
    ssd1685_write_cmd(handle, SSD1685_CMD_DEEP_SLEEP_MODE, data, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
}

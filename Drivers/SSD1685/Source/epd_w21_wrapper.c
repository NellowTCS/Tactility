#include "epd_w21_wrapper.h"
#include <string.h>
#include <esp_log.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "EPD_W21";

// Helper functions
static inline void epd_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static inline int epd_is_busy(epd_w21_handle_t *handle) {
    return gpio_get_level(handle->busy_pin);
}

static void epd_wait_busy(epd_w21_handle_t *handle) {
    ESP_LOGD(TAG, "Waiting for busy...");
    uint32_t timeout = 400; // 400ms for partial, more for full
    while (epd_is_busy(handle) && timeout > 0) {
        epd_delay(10);
        timeout -= 10;
    }
    if (timeout == 0) {
        ESP_LOGW(TAG, "Busy timeout");
    }
}

static void epd_reset(epd_w21_handle_t *handle) {
    gpio_set_level(handle->rst_pin, 0);
    epd_delay(10);
    gpio_set_level(handle->rst_pin, 1);
    epd_delay(10);
}

static void epd_write_cmd(epd_w21_handle_t *handle, uint8_t cmd) {
    gpio_set_level(handle->cs_pin, 0);
    gpio_set_level(handle->dc_pin, 0);
    
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_transmit(handle->spi_handle, &t);
    
    gpio_set_level(handle->cs_pin, 1);
}

static void epd_write_data(epd_w21_handle_t *handle, uint8_t data) {
    gpio_set_level(handle->cs_pin, 0);
    gpio_set_level(handle->dc_pin, 1);
    
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
    };
    spi_device_transmit(handle->spi_handle, &t);
    
    gpio_set_level(handle->cs_pin, 1);
}

static void epd_write_data_buf(epd_w21_handle_t *handle, const uint8_t *data, size_t len) {
    gpio_set_level(handle->cs_pin, 0);
    gpio_set_level(handle->dc_pin, 1);
    
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
    };
    spi_device_transmit(handle->spi_handle, &t);
    
    gpio_set_level(handle->cs_pin, 1);
}

void epd_w21_init_io(epd_w21_handle_t *handle) {
    gpio_set_direction(handle->dc_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(handle->rst_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(handle->cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(handle->busy_pin, GPIO_MODE_INPUT);
    
    gpio_set_level(handle->dc_pin, 0);
    gpio_set_level(handle->rst_pin, 1);
    gpio_set_level(handle->cs_pin, 1);
    
    ESP_LOGI(TAG, "IO initialized");
}

void epd_w21_deinit_io(epd_w21_handle_t *handle) {
    gpio_set_direction(handle->dc_pin, GPIO_MODE_INPUT);
    gpio_set_direction(handle->rst_pin, GPIO_MODE_INPUT);
    gpio_set_direction(handle->cs_pin, GPIO_MODE_INPUT);
    gpio_set_direction(handle->busy_pin, GPIO_MODE_INPUT);
}

void epd_w21_init(epd_w21_handle_t *handle) {
    ESP_LOGI(TAG, "Initializing display");
    
    epd_reset(handle);
    epd_wait_busy(handle);
    
    // SW Reset
    epd_write_cmd(handle, 0x12);
    epd_wait_busy(handle);
    
    // Border waveform
    epd_write_cmd(handle, 0x3C);
    epd_write_data(handle, 0x01);
    
    // Driver output control
    epd_write_cmd(handle, 0x01);
    epd_write_data(handle, (EPD_HEIGHT - 1) % 256);
    epd_write_data(handle, (EPD_HEIGHT - 1) / 256);
    epd_write_data(handle, 0x00);
    
    // Data entry mode
    epd_write_cmd(handle, 0x11);
    epd_write_data(handle, 0x01);
    
    // Set RAM X address
    epd_write_cmd(handle, 0x44);
    epd_write_data(handle, 0x00);
    epd_write_data(handle, EPD_WIDTH / 8 - 1);
    
    // Set RAM Y address
    epd_write_cmd(handle, 0x45);
    epd_write_data(handle, (EPD_HEIGHT - 1) % 256);
    epd_write_data(handle, (EPD_HEIGHT - 1) / 256);
    epd_write_data(handle, 0x00);
    epd_write_data(handle, 0x00);
    
    // Border waveform
    epd_write_cmd(handle, 0x3C);
    epd_write_data(handle, 0x05);
    
    // Temperature sensor
    epd_write_cmd(handle, 0x18);
    epd_write_data(handle, 0x80);
    
    // Set RAM X address counter
    epd_write_cmd(handle, 0x4E);
    epd_write_data(handle, 0x00);
    
    // Set RAM Y address counter
    epd_write_cmd(handle, 0x4F);
    epd_write_data(handle, (EPD_HEIGHT - 1) % 256);
    epd_write_data(handle, (EPD_HEIGHT - 1) / 256);
    
    epd_wait_busy(handle);
    
    ESP_LOGI(TAG, "Display initialized");
}

void epd_w21_display_full(epd_w21_handle_t *handle, const uint8_t *data) {
    ESP_LOGI(TAG, "Full refresh");
    
    // Write to black/white RAM
    epd_write_cmd(handle, 0x24);
    for (uint32_t i = 0; i < EPD_ARRAY; i++) {
        epd_write_data(handle, data[i]);
    }
    
    // Write to red RAM (same data for now)
    epd_write_cmd(handle, 0x26);
    for (uint32_t i = 0; i < EPD_ARRAY; i++) {
        epd_write_data(handle, 0x00);
    }
    
    // Update display
    epd_write_cmd(handle, 0x22);
    epd_write_data(handle, 0xF4);
    epd_write_cmd(handle, 0x20);
    
    epd_wait_busy(handle);
    epd_delay(100); // Extra delay for full refresh
}

void epd_w21_display_partial(epd_w21_handle_t *handle, const uint8_t *data) {
    ESP_LOGI(TAG, "Partial refresh");
    
    // Border waveform for partial
    epd_write_cmd(handle, 0x3C);
    epd_write_data(handle, 0xC0);
    
    // Write to black/white RAM
    epd_write_cmd(handle, 0x24);
    for (uint32_t i = 0; i < EPD_ARRAY; i++) {
        epd_write_data(handle, data[i]);
    }
    
    // Partial update
    epd_write_cmd(handle, 0x22);
    epd_write_data(handle, 0xDF);
    epd_write_cmd(handle, 0x20);
    
    epd_wait_busy(handle);
}

void epd_w21_clear(epd_w21_handle_t *handle) {
    ESP_LOGI(TAG, "Clearing screen");
    
    // Write white to RAM
    epd_write_cmd(handle, 0x24);
    for (uint32_t i = 0; i < EPD_ARRAY; i++) {
        epd_write_data(handle, 0xFF);
    }
    
    epd_write_cmd(handle, 0x26);
    for (uint32_t i = 0; i < EPD_ARRAY; i++) {
        epd_write_data(handle, 0x00);
    }
    
    // Update
    epd_write_cmd(handle, 0x22);
    epd_write_data(handle, 0xF4);
    epd_write_cmd(handle, 0x20);
    
    epd_wait_busy(handle);
}

void epd_w21_sleep(epd_w21_handle_t *handle) {
    epd_write_cmd(handle, 0x10);
    epd_write_data(handle, 0x01);
    epd_delay(100);
}
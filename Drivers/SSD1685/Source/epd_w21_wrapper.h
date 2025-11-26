#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EPD_WIDTH   168
#define EPD_HEIGHT  384
#define EPD_ARRAY   (EPD_WIDTH * EPD_HEIGHT / 8)

typedef struct {
    spi_device_handle_t spi_handle;
    gpio_num_t dc_pin;
    gpio_num_t rst_pin;
    gpio_num_t busy_pin;
    gpio_num_t cs_pin;
} epd_w21_handle_t;

// Initialize pins
void epd_w21_init_io(epd_w21_handle_t *handle);
void epd_w21_deinit_io(epd_w21_handle_t *handle);

// Initialize display for full refresh
void epd_w21_init(epd_w21_handle_t *handle);

// Display full screen
void epd_w21_display_full(epd_w21_handle_t *handle, const uint8_t *data);

// Display partial update
void epd_w21_display_partial(epd_w21_handle_t *handle, const uint8_t *data);

// Deep sleep
void epd_w21_sleep(epd_w21_handle_t *handle);

// Clear screen
void epd_w21_clear(epd_w21_handle_t *handle);

#ifdef __cplusplus
}
#endif
#pragma once

#include "driver/i2c.h"
#include "driver/gpio.h"

// Main bus, used by GT911 touch hardware and keyboard
#define TDECK_I2C_BUS_HANDLE (0)

// SPI
#define TDECK_SPI_HOST SPI2_HOST
#define TDECK_SPI_PIN_SCLK GPIO_NUM_40
#define TDECK_SPI_PIN_MOSI GPIO_NUM_41
#define TDECK_SPI_PIN_MISO GPIO_NUM_38
#define TDECK_SPI_TRANSFER_SIZE_LIMIT (TDECK_LCD_HORIZONTAL_RESOLUTION * TDECK_LCD_SPI_TRANSFER_HEIGHT * (TDECK_LCD_BITS_PER_PIXEL / 8))

// Power on
#define TDECK_POWERON_GPIO GPIO_NUM_10
#define TDECK_POWERON_DELAY 2000 // milliseconds - see bootstrap.c for explanation

// Display
#define TDECK_LCD_SPI_HOST SPI2_HOST
#define TDECK_LCD_PIN_CS GPIO_NUM_12
#define TDECK_LCD_PIN_DC GPIO_NUM_11 // RS
#define TDECK_LCD_PIN_BACKLIGHT GPIO_NUM_42
#define TDECK_LCD_SPI_FREQUENCY 40000000
#define TDECK_LCD_HORIZONTAL_RESOLUTION 320
#define TDECK_LCD_VERTICAL_RESOLUTION 240
#define TDECK_LCD_BITS_PER_PIXEL 16
#define TDECK_LCD_DRAW_BUFFER_HEIGHT (TDECK_LCD_VERTICAL_RESOLUTION / 10)
#define TDECK_LCD_SPI_TRANSFER_HEIGHT (TDECK_LCD_VERTICAL_RESOLUTION / 10)

// LVGL
// The minimum task stack seems to be about 3500, but that crashes the wifi app in some scenarios
#define TDECK_LVGL_TASK_STACK_DEPTH 4000

// Dipslay backlight (PWM)
#define TDECK_LCD_BACKLIGHT_LEDC_TIMER LEDC_TIMER_0
#define TDECK_LCD_BACKLIGHT_LEDC_MODE LEDC_LOW_SPEED_MODE
#define TDECK_LCD_BACKLIGHT_LEDC_OUTPUT_IO TDECK_LCD_PIN_BACKLIGHT
#define TDECK_LCD_BACKLIGHT_LEDC_CHANNEL LEDC_CHANNEL_0
#define TDECK_LCD_BACKLIGHT_LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define TDECK_LCD_BACKLIGHT_LEDC_DUTY (191)
#define TDECK_LCD_BACKLIGHT_LEDC_FREQUENCY (1000)

// Touch (GT911)
#define TDECK_TOUCH_I2C_BUS_HANDLE TDECK_I2C_BUS_HANDLE
#define TDECK_TOUCH_X_MAX 240
#define TDECK_TOUCH_Y_MAX 320
#define TDECK_TOUCH_PIN_INT GPIO_NUM_16

// SD Card
#define TDECK_SDCARD_SPI_HOST SPI2_HOST
#define TDECK_SDCARD_PIN_CS GPIO_NUM_39
#define TDECK_SDCARD_SPI_FREQUENCY 800000U
#define TDECK_SDCARD_FORMAT_ON_MOUNT_FAILED false
#define TDECK_SDCARD_MAX_OPEN_FILES 4
#define TDECK_SDCARD_ALLOC_UNIT_SIZE (16 * 1024)
#define TDECK_SDCARD_STATUS_CHECK_ENABLED false

// Keyboard
#define TDECK_KEYBOARD_I2C_BUS_HANDLE TDECK_I2C_BUS_HANDLE
#define TDECK_KEYBOARD_SLAVE_ADDRESS 0x55

// Lora (optional)
#define TDECK_RADIO_PIN_CS GPIO_NUM_9

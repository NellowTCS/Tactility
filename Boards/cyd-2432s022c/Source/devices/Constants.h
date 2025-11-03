#pragma once

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"

// Display (ST7789 with 8-bit parallel interface)
#define DISPLAY_HORIZONTAL_RESOLUTION 240
#define DISPLAY_VERTICAL_RESOLUTION 320
#define DISPLAY_DRAW_BUFFER_HEIGHT (DISPLAY_VERTICAL_RESOLUTION / 10)
#define DISPLAY_DRAW_BUFFER_SIZE (DISPLAY_HORIZONTAL_RESOLUTION * DISPLAY_DRAW_BUFFER_HEIGHT)
#define DISPLAY_PCLK_HZ 12000000
#define DISPLAY_BUS_WIDTH 8

// GPIO pins for ST7789 8-bit parallel interface
#define DISPLAY_I80_D0 GPIO_NUM_15
#define DISPLAY_I80_D1 GPIO_NUM_13
#define DISPLAY_I80_D2 GPIO_NUM_12
#define DISPLAY_I80_D3 GPIO_NUM_14
#define DISPLAY_I80_D4 GPIO_NUM_27
#define DISPLAY_I80_D5 GPIO_NUM_25
#define DISPLAY_I80_D6 GPIO_NUM_33
#define DISPLAY_I80_D7 GPIO_NUM_32

#define DISPLAY_WR GPIO_NUM_4
#define DISPLAY_RD GPIO_NUM_2    // Not used in i80 write-only mode, but kept for reference
#define DISPLAY_DC GPIO_NUM_16
#define DISPLAY_CS GPIO_NUM_17
#define DISPLAY_RST GPIO_NUM_NC  // No reset pin defined, will skip hardware reset if NC
#define DISPLAY_BL GPIO_NUM_0
#define DISPLAY_BACKLIGHT_ON_LEVEL 1  // Assuming active-high, adjust if needed

// I2C for CST820 touch controller
#define TOUCH_PORT I2C_NUM_0
#define TOUCH_SDA GPIO_NUM_21
#define TOUCH_SCL GPIO_NUM_22
#define TOUCH_ADDRESS 0x15
#define TOUCH_SPEED 400000

// SPI for SD card
#define SDCARD_HOST SPI3_HOST
#define SDCARD_CS GPIO_NUM_5
#define SDCARD_MOSI GPIO_NUM_23
#define SDCARD_MISO GPIO_NUM_19
#define SDCARD_SCLK GPIO_NUM_18
#define SDCARD_MAX_TRANSFER_SIZE 8192

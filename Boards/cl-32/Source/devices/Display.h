#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <memory>

// E-Paper Display pins (matching CL-32 hardware)
constexpr auto EPD_SPI_HOST = SPI2_HOST;
constexpr auto EPD_PIN_CS = GPIO_NUM_6;
constexpr auto EPD_PIN_DC = GPIO_NUM_13;
constexpr auto EPD_PIN_RST = GPIO_NUM_12;
constexpr auto EPD_PIN_BUSY = GPIO_NUM_14;

// Display specifications
constexpr auto EPD_WIDTH = 168;
constexpr auto EPD_HEIGHT = 384;

// No backlight for e-paper
constexpr auto LCD_PIN_BACKLIGHT = GPIO_NUM_NC;

// SPI transfer limit for e-paper (smaller buffers)
constexpr auto LCD_SPI_TRANSFER_SIZE_LIMIT = 4096;

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay();

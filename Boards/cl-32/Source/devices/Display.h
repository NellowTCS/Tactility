#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <memory>

// Pin definitions for cl-32 board (updated to match CL-32 Arduino code: CL32_epd_cs=6, CL32_dc=13, CL32_rst=12, CL32_bsy=14)
#define EPD_CS GPIO_NUM_6   // Chip select
#define EPD_DC GPIO_NUM_13  // Data/command
#define EPD_RST GPIO_NUM_12 // Reset
#define EPD_BUSY GPIO_NUM_14 // Busy signal

// SPI transfer size limit for e-paper (small buffer)
#define LCD_SPI_TRANSFER_SIZE_LIMIT 1024

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay();
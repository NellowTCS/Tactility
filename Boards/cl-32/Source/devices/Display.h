#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <memory>

// Pin definitions for cl-32 board
#define EPD_CS GPIO_NUM_6   // Chip select
#define EPD_DC GPIO_NUM_13  // Data/command
#define EPD_RST GPIO_NUM_12 // Reset
#define EPD_BUSY GPIO_NUM_14 // Busy signal

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay();
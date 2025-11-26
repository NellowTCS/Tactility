#pragma once

#include "Tactility/hal/sdcard/SdCardDevice.h"
#include <driver/gpio.h>
#include <driver/spi_common.h>

using tt::hal::sdcard::SdCardDevice;

constexpr auto SDCARD_SPI_HOST = SPI2_HOST;
constexpr auto SDCARD_PIN_CS = GPIO_NUM_7;

std::shared_ptr<SdCardDevice> createSdCard();
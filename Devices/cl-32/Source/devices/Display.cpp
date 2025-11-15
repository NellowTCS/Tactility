#include "Display.h"
#include "Ssd1685Display.h"
#include <esp_log.h>

static const char* TAG = "Display";

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay(spi_device_handle_t spiHandle) {
    if (!spiHandle) {
        ESP_LOGE(TAG, "SPI handle is null");
        return nullptr;
    }

    auto config = Ssd1685Display::Configuration {
        .width = EPD_WIDTH,
        .height = EPD_HEIGHT,
        .dcPin = EPD_PIN_DC,
        .rstPin = EPD_PIN_RST,
        .busyPin = EPD_PIN_BUSY,
        .spiHandle = spiHandle,
    };

    return std::make_shared<Ssd1685Display>(config);
}
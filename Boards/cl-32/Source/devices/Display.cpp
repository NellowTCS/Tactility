#include "Display.h"
#include "Ssd1685Display.h"

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto config = Ssd1685Display::Configuration {
        .width = EPD_WIDTH,
        .height = EPD_HEIGHT,
        .dcPin = EPD_PIN_DC,
        .rstPin = EPD_PIN_RST,
        .busyPin = EPD_PIN_BUSY,
        .spiHost = EPD_SPI_HOST,
    };

    return std::make_shared<Ssd1685Display>(config);
}
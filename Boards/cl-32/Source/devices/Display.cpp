#include "Display.h"
#include "GxEPD2Display.h"

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto config = GxEPD2Display::Configuration {
        .width = EPD_WIDTH,
        .height = EPD_HEIGHT,
        .csPin = EPD_PIN_CS,
        .dcPin = EPD_PIN_DC,
        .rstPin = EPD_PIN_RST,
        .busyPin = EPD_PIN_BUSY,
        .spiHost = EPD_SPI_HOST,
        .rotation = 0  // 0=portrait, 1=landscape 90°CW, 2=portrait 180°, 3=landscape 270°CW
    };

    return std::make_shared<GxEPD2Display>(config);
}
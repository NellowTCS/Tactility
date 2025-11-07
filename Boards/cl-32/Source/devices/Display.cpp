#include "Display.h"
#include <Ssd1685Display.h>

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {[]
    auto config = std::make_unique<Ssd1685Display::Configuration>(
        EPD_SPI_HOST,   // spiHost
        EPD_PIN_CS,     // csPin
        EPD_PIN_DC,     // dcPin
        EPD_PIN_RST,    // resetPin
        EPD_PIN_BUSY,   // busyPin
        EPD_WIDTH,      // width (168)
        EPD_HEIGHT,     // height (384)
        1,              // rotation: 0=portrait, 1=landscape 90°CW, 2=180°, 3=270°CW
        nullptr         // touch device
    );

    // SSD1685 panels route source lines with an 8-pixel offset; compensate via panel gap
    config->gapX = 8;
    config->gapY = 0;

    return std::make_shared<Ssd1685Display>(std::move(config));
}
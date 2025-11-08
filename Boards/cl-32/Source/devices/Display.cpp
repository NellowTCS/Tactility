#include "Display.h"
#include "Ssd168xDisplay.h"
#include <memory>

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay()
{
    auto config = std::make_unique<Ssd168xDisplay::Configuration>(
        Ssd168xDisplay::Configuration{
            .controller = SSD1685,
            .width = EPD_WIDTH,     // 168
            .height = EPD_HEIGHT,   // 384
            .csPin = EPD_PIN_CS,
            .dcPin = EPD_PIN_DC,
            .resetPin = EPD_PIN_RST,
            .busyPin = EPD_PIN_BUSY,
            .spiHost = EPD_SPI_HOST,
            .rotation = 1  // 0=portrait, 1=landscape 90°CW
        }
    );

    return std::make_shared<Ssd168xDisplay>(std::move(config));
}

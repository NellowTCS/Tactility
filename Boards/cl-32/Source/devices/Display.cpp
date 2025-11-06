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
        .rotation = 1  // 0=portrait, 1=landscape 90°CW, 2=portrait 180°, 3=landscape 270°CW
    };

    auto disp = std::make_shared<GxEPD2Display>(config);
    // Compensate panel driver SOURCE_SHIFT (quick test). Remove if not needed.
    disp->setGap(-GxEPD2_290_GDEY029T71H::SOURCE_SHIFT, 0);
    return disp;
}
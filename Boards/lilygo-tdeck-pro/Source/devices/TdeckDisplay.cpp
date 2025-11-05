#include "TdeckDisplay.h"
#include "TdeckConstants.h"

#include <Tactility/Log.h>

#define TAG "EpaperDisplay"

bool TdeckDisplay::start() {
    TT_LOG_I(TAG, "Starting");

    GxEPD2Display::Configuration config = {
        .width = 240,
        .height = 320,
        .csPin = (gpio_num_t)BOARD_EPD_CS,
        .dcPin = (gpio_num_t)BOARD_EPD_DC,
        .rstPin = GPIO_NUM_NC,  // No RST pin connected
        .busyPin = (gpio_num_t)BOARD_EPD_BUSY,
        .spiHost = BOARD_EPD_SPI_HOST,
        .rotation = 0  // Portrait
    };

    gxDisplay = std::make_unique<GxEPD2Display>(config);

    if (!gxDisplay->start()) {
        TT_LOG_E(TAG, "Failed to start GxEPD2 display");
        return false;
    }

    if (!gxDisplay->startLvgl()) {
        TT_LOG_E(TAG, "Failed to start LVGL");
        gxDisplay->stop();
        gxDisplay.reset();
        return false;
    }

    TT_LOG_I(TAG, "Finished");
    return true;
}

bool TdeckDisplay::stop() {
    if (gxDisplay) {
        gxDisplay->stopLvgl();
        gxDisplay->stop();
        gxDisplay.reset();
    }
    return true;
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    return std::make_shared<TdeckDisplay>();
}

#include "YellowDisplay.h"
#include "YellowDisplayConstants.h"
#include "SoftXpt2046Touch.h"
#include <Ili934xDisplay.h>
#include <PwmBacklight.h>
#include "esp_log.h"

static const char* TAG = "YellowDisplay";

static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    ESP_LOGI(TAG, "Creating software SPI touch");
    auto config = std::make_unique<SoftXpt2046Touch::Configuration>(
        CYD_DISPLAY_HORIZONTAL_RESOLUTION,  // xMax = 240
        CYD_DISPLAY_VERTICAL_RESOLUTION,    // yMax = 320
        false,  // swapXy
        false,  // mirrorX
        false,  // mirrorY
        240,    // xMinRaw (was 200, flipped from Y)
        3800,   // xMaxRaw (was 3700, flipped from Y)
        200,    // yMinRaw (was 240, flipped from X)
        3700    // yMaxRaw (was 3800, flipped from X)
    );
    return std::make_shared<SoftXpt2046Touch>(std::move(config));
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto touch = createTouch();
    auto configuration = std::make_unique<Ili934xDisplay::Configuration>(
        CYD_DISPLAY_SPI_HOST,
        CYD_DISPLAY_PIN_CS,
        CYD_DISPLAY_PIN_DC,
        CYD_DISPLAY_HORIZONTAL_RESOLUTION,
        CYD_DISPLAY_VERTICAL_RESOLUTION,
        touch
    );
    configuration->mirrorX = true;
    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;
    return std::make_shared<Ili934xDisplay>(std::move(configuration));
}

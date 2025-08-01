#include "YellowDisplay.h"
#include "YellowDisplayConstants.h"
#include "XPT2046-Bitbang.h"
#include <Ili934xDisplay.h>
#include <PwmBacklight.h>
#include <Tactility/hal/touch/TouchDevice.h>
#include <esp_log.h>
#include <string>

static const char* TAG = "YellowDisplay";

// Global to hold reference (only needed if calling stop() later)
static std::unique_ptr<XPT2046_Bitbang> touch;

static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    ESP_LOGI(TAG, "Creating bitbang SPI touch");

    // Create bitbang config object
    auto config = std::make_unique<XPT2046_Bitbang::Configuration>(
        CYD_TOUCH_MOSI_PIN,
        CYD_TOUCH_MISO_PIN,
        CYD_TOUCH_SCK_PIN,
        CYD_TOUCH_CS_PIN,
        CYD_DISPLAY_HORIZONTAL_RESOLUTION,
        CYD_DISPLAY_VERTICAL_RESOLUTION,
        false,  // swapXY
        false,  // mirrorX
        false   // mirrorY
    );

    // Allocate the driver
    touch = std::make_unique<XPT2046_Bitbang>(std::move(config));

    // Start the driver and load calibration from NVS
    if (!touch->start(nullptr)) {
        ESP_LOGE(TAG, "Touch driver start failed");
    }

    return std::shared_ptr<tt::hal::touch::TouchDevice>(touch.get(), [](tt::hal::touch::TouchDevice*) {
        // No delete needed; `touch` is managed above
    });
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto touch_device = createTouch();
    auto configuration = std::make_unique<Ili934xDisplay::Configuration>(
        CYD_DISPLAY_SPI_HOST,
        CYD_DISPLAY_PIN_CS,
        CYD_DISPLAY_PIN_DC,
        CYD_DISPLAY_HORIZONTAL_RESOLUTION,
        CYD_DISPLAY_VERTICAL_RESOLUTION,
        touch_device
    );
    configuration->mirrorX = true;
    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;
    configuration->rgbElementOrder = LCD_RGB_ELEMENT_ORDER_BGR;
    return std::make_shared<Ili934xDisplay>(std::move(configuration));
}

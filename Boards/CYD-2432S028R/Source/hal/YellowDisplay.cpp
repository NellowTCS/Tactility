#include "YellowDisplay.h"
#include "YellowDisplayConstants.h"
#include "SoftXpt2046Touch.h"
#include <Ili934xDisplay.h>
#include <PwmBacklight.h>
#include "esp_log.h"
#include "nvs_flash.h"

static const char* TAG = "YellowDisplay";

static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    ESP_LOGI(TAG, "Creating software SPI touch");
    // Default calibration (XPT2046 ADC range, per esp_lcd_touch_xpt2046)
    uint16_t xMinRaw = 300, xMaxRaw = 3800, yMinRaw = 300, yMaxRaw = 3800;

    // Load from NVS
    nvs_handle_t nvs;
    if (nvs_open("touch_cal", NVS_READONLY, &nvs) == ESP_OK) {
        uint16_t cal[4];
        size_t size = sizeof(cal);
        if (nvs_get_blob(nvs, "cal_data", cal, &size) == ESP_OK && size == sizeof(cal)) {
            xMinRaw = cal[0];
            xMaxRaw = cal[1];
            yMinRaw = cal[2];
            yMaxRaw = cal[3];
            ESP_LOGI(TAG, "Loaded NVS calibration: xMinRaw=%u, xMaxRaw=%u, yMinRaw=%u, yMaxRaw=%u",
                     xMinRaw, xMaxRaw, yMinRaw, yMaxRaw);
        } else {
            ESP_LOGW(TAG, "No valid NVS calibration, using defaults: xMinRaw=%u, xMaxRaw=%u, yMinRaw=%u, yMaxRaw=%u",
                     xMinRaw, xMaxRaw, yMinRaw, yMaxRaw);
        }
        nvs_close(nvs);
    } else {
        ESP_LOGW(TAG, "NVS open failed, using default calibration");
    }

    auto config = std::make_unique<SoftXpt2046Touch::Configuration>(
        CYD_DISPLAY_HORIZONTAL_RESOLUTION,  // xMax = 240
        CYD_DISPLAY_VERTICAL_RESOLUTION,   // yMax = 320
        false,  // swapXy
        false,  // mirrorX
        false,  // mirrorY
        xMinRaw,  // xMinRaw
        xMaxRaw,  // xMaxRaw
        yMinRaw,  // yMinRaw
        yMaxRaw   // yMaxRaw
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
    configuration->mirrorX = true;  // Keep for display
    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;
    return std::make_shared<Ili934xDisplay>(std::move(configuration));
}

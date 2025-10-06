// Touch.cpp
#include "Touch.h"
#include "Constants.h"
#include <Cst816Touch.h>
#include "Tactility/settings/DisplaySettings.h"
#include <esp_log.h>
#include <memory>

#define TAG "Touch"

std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    ESP_LOGI(TAG, "Creating Touch");

    // Use the new DisplaySettings API
    auto settings = tt::settings::display::loadOrGetDefault();
    lv_display_rotation_t rotation = tt::settings::display::toLvglDisplayRotation(settings.orientation);

    bool swapXY = (rotation == LV_DISPLAY_ROTATION_90 || rotation == LV_DISPLAY_ROTATION_270);
    bool mirrorX = (rotation == LV_DISPLAY_ROTATION_270);
    bool mirrorY = (rotation == LV_DISPLAY_ROTATION_90);

    auto configuration = std::make_unique<Cst816sTouch::Configuration>(
        CYD_2432S022C_TOUCH_I2C_PORT,
        CYD_2432S022C_LCD_HORIZONTAL_RESOLUTION,
        CYD_2432S022C_LCD_VERTICAL_RESOLUTION,
        swapXY,
        mirrorX,
        mirrorY
    );

    auto touch = std::make_shared<Cst816sTouch>(std::move(configuration));
    if (!touch) {
        ESP_LOGE(TAG, "Failed to create Cst816sTouch");
        return nullptr;
    }

    ESP_LOGI(TAG, "Touch created: %p", touch.get());
    return touch;
}

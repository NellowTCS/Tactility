#include "Display.h"
#include "Touch.h"
#include "CustomEspDisplay.h"
#include <memory>
#include <Tactility/Log.h>
#include <Tactility/Lock.h>
#include <Tactility/MutexLock.h>

#define TAG "Display"

// Create esp_lcd ST7789 display with touch integration
std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    TT_LOG_I(TAG, "Creating ST7789 display");

    auto touch = createTouch();
    if (!touch) {
        TT_LOG_W(TAG, "Proceeding without touch");
    }

    // Create display lock using concrete MutexLock implementation
    auto lock = std::make_shared<tt::MutexLock>();

    // Create and initialize the display
    auto display = std::make_shared<CustomEspDisplay>(lock);

    // Add touch device if available
    if (touch) {
        display->setTouchDevice(touch);
    }

    // Start the display
    if (!display->start()) {
        TT_LOG_E(TAG, "Failed to start display");
        return nullptr;
    }

    TT_LOG_I(TAG, "Touch Device: %s", touch ? "Enabled" : "Disabled");
    if (touch) {
        TT_LOG_I(TAG, "  Touch Type: %s", touch->getName().c_str());
    }

    TT_LOG_I(TAG, "ST7789 display created successfully");
    return display;
}
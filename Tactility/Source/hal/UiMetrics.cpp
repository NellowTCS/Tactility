#include <Tactility/hal/UiMetrics.h>

#include <Tactility/Log.h>
#include <lvgl.h>
#include <algorithm>

namespace tt::hal {

constexpr auto* TAG = "UiMetrics";

enum class ScreenClass {
    Tiny,      // < 100px min dimension
    Small,     // < 200px min dimension
    Large,     // < 400px min dimension
    ExtraLarge // >= 400px min dimension
};

static ScreenClass getScreenClass(int minDimension) {
    if (minDimension < 100) {
        return ScreenClass::Tiny;
    } else if (minDimension < 200) {
        return ScreenClass::Small;
    } else if (minDimension < 400) {
        return ScreenClass::Large;
    } else {
        return ScreenClass::ExtraLarge;
    }
}

static const char* getScreenClassName(ScreenClass screenClass) {
    switch (screenClass) {
        case ScreenClass::Tiny:       return "Tiny";
        case ScreenClass::Small:      return "Small";
        case ScreenClass::Large:      return "Large";
        case ScreenClass::ExtraLarge: return "ExtraLarge";
        default:                      return "Unknown";
    }
}

UiMetrics UiMetrics::calculate(int screenWidth, int screenHeight) {
    UiMetrics metrics;
    
    int minDim = std::min(screenWidth, screenHeight);
    int maxDim = std::max(screenWidth, screenHeight);
    auto screenClass = getScreenClass(minDim);
    
    switch (screenClass) {
        case ScreenClass::Tiny:
            // For extremely small displays (T-Dongle 80x160, Heltec 64x128)
            // Very compact UI to fit basic functionality
            metrics.toolbarHeight = 18;
            metrics.toolbarFont = &lv_font_montserrat_14;
            metrics.toolbarTitlePadding = 2;
            metrics.toolbarButtonInset = 10;
            
            metrics.buttonPadding = 1;
            metrics.buttonRadius = 2;
            
            metrics.objectPadding = 1;
            metrics.objectGap = 2;
            metrics.objectRadius = 2;
            metrics.objectBorderWidth = 1;
            
            metrics.listPadding = 1;
            metrics.listButtonVertPadding = 1;
            
            metrics.switchWidth = 20;
            metrics.switchHeight = 12;
            metrics.dropdownHeight = 80;
            metrics.textareaPadding = 1;
            
            metrics.launcherButtonSize = 32;
            metrics.generalVerticalPadding = 0;
            metrics.localeSettingsOffset = -1;
            metrics.systemInfoPadding = 1;
            metrics.wifiManageScrollbarWidth = 1;
            break;
            
        case ScreenClass::Small:
            // For small displays (Cardputer 240x135, StickC 135x240)
            // Compact UI - original "Smallest" scale
            metrics.toolbarHeight = 22;
            metrics.toolbarFont = &lv_font_montserrat_14;
            metrics.toolbarTitlePadding = 4;
            metrics.toolbarButtonInset = 8;
            
            metrics.buttonPadding = 2;
            metrics.buttonRadius = 3;
            
            metrics.objectPadding = 2;
            metrics.objectGap = 4;
            metrics.objectRadius = 3;
            metrics.objectBorderWidth = 1;
            
            metrics.listPadding = 2;
            metrics.listButtonVertPadding = 2;
            
            metrics.switchWidth = 25;
            metrics.switchHeight = 15;
            metrics.dropdownHeight = 100;
            metrics.textareaPadding = 2;
            
            metrics.launcherButtonSize = 40;
            metrics.generalVerticalPadding = 0;
            metrics.localeSettingsOffset = -2;
            metrics.systemInfoPadding = 2;
            metrics.wifiManageScrollbarWidth = 2;
            break;
            
        case ScreenClass::Large:
            // For 240x320 up to ~400px displays - matches old Default scale
            // Use -1 to indicate "use LVGL theme defaults, don't override"
            metrics.toolbarHeight = 40;
            metrics.toolbarFont = &lv_font_montserrat_18;
            metrics.toolbarTitlePadding = 8;
            metrics.toolbarButtonInset = 6;
            
            metrics.buttonPadding = -1;  // Use theme default
            metrics.buttonRadius = -1;   // Use theme default
            
            metrics.objectPadding = -1;  // Use theme default
            metrics.objectGap = -1;      // Use theme default
            metrics.objectRadius = -1;   // Use theme default
            metrics.objectBorderWidth = -1; // Use theme default
            
            metrics.listPadding = -1;    // Use theme default
            metrics.listButtonVertPadding = -1; // Use theme default
            
            metrics.switchWidth = -1;    // Use theme default
            metrics.switchHeight = -1;   // Use theme default
            metrics.dropdownHeight = -1; // Use theme default
            metrics.textareaPadding = -1; // Use theme default
            
            metrics.launcherButtonSize = 64;
            metrics.generalVerticalPadding = 4;
            metrics.localeSettingsOffset = -10;
            metrics.systemInfoPadding = 12;
            metrics.wifiManageScrollbarWidth = 8;
            break;
            
        case ScreenClass::ExtraLarge:
            // For very large displays (800x480, large panels)
            // Larger UI with much larger elements
            metrics.toolbarHeight = 48;
            metrics.toolbarFont = &lv_font_montserrat_18;
            metrics.toolbarTitlePadding = 10;
            metrics.toolbarButtonInset = 8;
            
            metrics.buttonPadding = 8;
            metrics.buttonRadius = 6;
            
            metrics.objectPadding = 8;
            metrics.objectGap = 10;
            metrics.objectRadius = 6;
            metrics.objectBorderWidth = 2;
            
            metrics.listPadding = 6;
            metrics.listButtonVertPadding = 6;
            
            metrics.switchWidth = 40;
            metrics.switchHeight = 24;
            metrics.dropdownHeight = 250;
            metrics.textareaPadding = 8;
            
            metrics.launcherButtonSize = 80;
            metrics.generalVerticalPadding = 6;
            metrics.localeSettingsOffset = -12;
            metrics.systemInfoPadding = 14;
            metrics.wifiManageScrollbarWidth = 10;
            break;
    }
    
    TT_LOG_I(TAG, "Screen: %dx%d (min=%d, max=%d) -> %s",
        screenWidth, screenHeight, minDim, maxDim, getScreenClassName(screenClass));
    TT_LOG_I(TAG, "  Toolbar: h=%d, font=%s, btn=%d",
        metrics.toolbarHeight,
        metrics.toolbarFont == &lv_font_montserrat_14 ? "14" : "18",
        metrics.launcherButtonSize);
    
    return metrics;
}

UiMetrics UiMetrics::calculateFromDisplay() {
    auto* display = lv_display_get_default();
    
    if (display == nullptr) {
        // No display initialized yet - return safe defaults for medium screens
        TT_LOG_W(TAG, "No display found, using default 240x320 metrics");
        return calculate(240, 320);
    }
    
    int width = static_cast<int>(lv_display_get_physical_horizontal_resolution(display));
    int height = static_cast<int>(lv_display_get_physical_vertical_resolution(display));
    
    return calculate(width, height);
}

} // namespace tt::hal

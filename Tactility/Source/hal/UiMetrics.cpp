#include <Tactility/hal/UiMetrics.h>
#include <Tactility/hal/display/DisplayDevice.h>

#include <Tactility/Log.h>
#include <lvgl.h>
#include <algorithm>
#include <cmath>

namespace tt::hal {

constexpr auto* TAG = "UiMetrics";

enum class ScreenClass {
    Tiny,      // Very small physical displays or < 100px
    Small,     // Small displays or < 150px  
    Medium,    // Medium displays or < 200px
    Large,     // Standard displays or < 400px
    ExtraLarge // Large displays or >= 400px
};

static ScreenClass getScreenClass(int minDimension, int maxDimension, float diagonalInches) {
    // If diagonal size is provided, calculate DPI and use that for classification
    if (diagonalInches > 0.0f) {
        // Calculate diagonal resolution in pixels using Pythagorean theorem
        float diagonalPixels = std::sqrt(minDimension * minDimension + maxDimension * maxDimension);
        float dpi = diagonalPixels / diagonalInches;
        
        // Classify based on physical size, not just resolution
        // High DPI tiny screens need smaller UI despite higher resolution
        if (diagonalInches < 2.0f) {
            // Very small physical displays (T-Dongle, Heltec, Waveshare 1.47")
            return ScreenClass::Tiny;
        } else if (diagonalInches < 3.0f) {
            // Small displays (2-3")
            return ScreenClass::Small;
        } else if (diagonalInches < 4.0f) {
            // Medium displays (3-4")
            return ScreenClass::Medium;
        } else if (diagonalInches < 6.5f) {
            // Standard displays (4-6.5")
            return ScreenClass::Large;
        } else {
            // Large displays (> 6.5")
            return ScreenClass::ExtraLarge;
        }
    }
    
    // Fallback to resolution-based classification if no diagonal info
    if (minDimension < 100) {
        return ScreenClass::Tiny;
    } else if (minDimension < 150) {
        return ScreenClass::Small;
    } else if (minDimension < 200) {
        return ScreenClass::Medium;
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
        case ScreenClass::Medium:     return "Medium";
        case ScreenClass::Large:      return "Large";
        case ScreenClass::ExtraLarge: return "ExtraLarge";
        default:                      return "Unknown";
    }
}

UiMetrics UiMetrics::calculate(int screenWidth, int screenHeight, float diagonalInches) {
    UiMetrics metrics;
    
    int minDim = std::min(screenWidth, screenHeight);
    int maxDim = std::max(screenWidth, screenHeight);
    auto screenClass = getScreenClass(minDim, maxDim, diagonalInches);
    
    // Calculate DPI info for logging
    if (diagonalInches > 0.0f) {
        float diagonalPixels = std::sqrt(minDim * minDim + maxDim * maxDim);
        float dpi = diagonalPixels / diagonalInches;
        TT_LOG_I(TAG, "Screen: %dx%d, %.2f\" diagonal, %.0f DPI -> %s",
            screenWidth, screenHeight, diagonalInches, dpi, getScreenClassName(screenClass));
    } else {
        TT_LOG_I(TAG, "Screen: %dx%d (min=%d, max=%d) -> %s",
            screenWidth, screenHeight, minDim, maxDim, getScreenClassName(screenClass));
    }
    
    switch (screenClass) {
        case ScreenClass::Tiny:
            // For extremely small displays (T-Dongle 80x160, Heltec 64x128)
            // Very compact UI to fit basic functionality
            metrics.toolbarHeight = 18;
            // Use a smaller font for very tiny displays to keep UI readable
            metrics.toolbarFont = &lv_font_montserrat_12;
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
            // For small displays (Cardputer 135px, StickC 135px)
            // Very compact UI - original "Smallest" scale
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
            
        case ScreenClass::Medium:
            // For medium displays
            // Balanced between Small and Large
            metrics.toolbarHeight = 28;
            metrics.toolbarFont = &lv_font_montserrat_14;
            metrics.toolbarTitlePadding = 6;
            metrics.toolbarButtonInset = 7;
            
            metrics.buttonPadding = 4;
            metrics.buttonRadius = 4;
            
            metrics.objectPadding = 4;
            metrics.objectGap = 6;
            metrics.objectRadius = 4;
            metrics.objectBorderWidth = 1;
            
            metrics.listPadding = 3;
            metrics.listButtonVertPadding = 3;
            
            metrics.switchWidth = 30;
            metrics.switchHeight = 18;
            metrics.dropdownHeight = 120;
            metrics.textareaPadding = 4;
            
            metrics.launcherButtonSize = 48;
            metrics.generalVerticalPadding = 3;
            metrics.localeSettingsOffset = -6;
            metrics.systemInfoPadding = 6;
            metrics.wifiManageScrollbarWidth = 5;
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
    
    TT_LOG_I(TAG, "  Toolbar: h=%d, font=%s, launcher=%d",
        metrics.toolbarHeight,
        (metrics.toolbarFont == &lv_font_montserrat_12) ? "12" :
        ((metrics.toolbarFont == &lv_font_montserrat_14) ? "14" : "18"),
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
    
    // Safety check for valid resolution values
    if (width <= 0 || height <= 0 || width > 10000 || height > 10000) {
        TT_LOG_W(TAG, "Invalid display resolution %dx%d, using defaults", width, height);
        return calculate(240, 320);
    }
    
    // Try to get physical size from display driver if available
    float diagonalInches = 0.0f;
    auto* user_data = lv_display_get_user_data(display);
    if (user_data != nullptr) {
        // Attempt to cast to DisplayDevice and get physical size
        auto* display_device = static_cast<tt::hal::display::DisplayDevice*>(user_data);
        if (display_device != nullptr) {
            diagonalInches = display_device->getPhysicalDiagonalInches();
        }
    }
    
    return calculate(width, height, diagonalInches);
}

} // namespace tt::hal

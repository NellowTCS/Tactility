#pragma once

#include <cstdint>

// Forward declare LVGL font type to avoid pulling in all of lvgl.h
struct _lv_font_t;

namespace tt::hal {

class UiMetrics {
public:
    // Toolbar metrics
    int toolbarHeight;           ///< Height of the top toolbar in pixels
    const _lv_font_t* toolbarFont; ///< Font to use for toolbar text
    int toolbarTitlePadding;     ///< Left padding for toolbar title text
    int toolbarButtonInset;      ///< Inset from toolbar edge for buttons
    
    // Widget metrics - buttons
    int buttonPadding;           ///< Internal padding for button widgets
    int buttonRadius;            ///< Corner radius for button widgets
    
    // Widget metrics - objects (containers)
    int objectPadding;           ///< Internal padding for container objects
    int objectGap;               ///< Gap between flex items in containers
    int objectRadius;            ///< Corner radius for container objects
    int objectBorderWidth;       ///< Border width for container objects
    
    // Widget metrics - lists
    int listPadding;             ///< Padding for list widgets
    int listButtonVertPadding;   ///< Vertical padding for list item buttons
    
    // Widget metrics - other
    int switchWidth;             ///< Width of switch widgets in pixels
    int switchHeight;            ///< Height of switch widgets in pixels
    int dropdownHeight;          ///< Maximum height for dropdown lists
    int textareaPadding;         ///< Internal padding for textarea widgets
    
    // Application-specific metrics
    int launcherButtonSize;      ///< Size of launcher app buttons
    int generalVerticalPadding;  ///< General vertical padding for settings etc
    int localeSettingsOffset;    ///< Horizontal offset for timezone label
    int systemInfoPadding;       ///< Bottom padding for system info labels
    int wifiManageScrollbarWidth; ///< Vertical padding for wifi manage items
    
    /**
     * Calculate UI metrics based on screen dimensions.
     * Uses the minimum of width/height to determine scaling class:
     * - Tiny: < 100px (T-Dongle, Heltec)
     * - Small: < 200px (Cardputer, StickC) - matches old "Smallest"
     * - Large: < 400px (240x320, 320x240, etc) - matches old "Default"
     * - ExtraLarge: >= 400px (800x480, large panels) - larger UI elements
     * 
     * @param screenWidth Physical horizontal resolution in pixels
     * @param screenHeight Physical vertical resolution in pixels
     * @return Calculated metrics instance optimized for this screen size
     */
    static UiMetrics calculate(int screenWidth, int screenHeight);
    
    /**
     * Calculate metrics from the default LVGL display.
     * Automatically detects screen resolution from LVGL.
     * Safe to call even if no display is initialized (returns safe defaults).
     * 
     * @return Calculated metrics based on current display, or defaults if no display
     */
    static UiMetrics calculateFromDisplay();

private:
    UiMetrics() = default;
};

} // namespace tt::hal

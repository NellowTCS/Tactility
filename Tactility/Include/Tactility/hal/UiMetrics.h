#pragma once

#include <cstdint>

// Forward declare LVGL font type to avoid pulling in all of lvgl.h
struct _lv_font_t;

namespace tt::hal {

class UiMetrics {
public:
    // Toolbar metrics
    int toolbarHeight = 0;           ///< Height of the top toolbar in pixels
    const _lv_font_t* toolbarFont = nullptr; ///< Font to use for toolbar text
    int toolbarTitlePadding = 0;     ///< Left padding for toolbar title text
    int toolbarButtonInset = 0;      ///< Inset from toolbar edge for buttons
    
    // Widget metrics - buttons
    int buttonPadding = 0;           ///< Internal padding for button widgets
    int buttonRadius = 0;            ///< Corner radius for button widgets
    
    // Widget metrics - objects (containers)
    int objectPadding = 0;           ///< Internal padding for container objects
    int objectGap = 0;               ///< Gap between flex items in containers
    int objectRadius = 0;            ///< Corner radius for container objects
    int objectBorderWidth = 0;       ///< Border width for container objects
    
    // Widget metrics - lists
    int listPadding = 0;             ///< Padding for list widgets
    int listButtonVertPadding = 0;   ///< Vertical padding for list item buttons
    
    // Widget metrics - other
    int switchWidth = 0;             ///< Width of switch widgets in pixels
    int switchHeight = 0;            ///< Height of switch widgets in pixels
    int dropdownHeight = 0;          ///< Maximum height for dropdown lists
    int textareaPadding = 0;         ///< Internal padding for textarea widgets
    
    // Application-specific metrics
    int launcherButtonSize = 0;      ///< Size of launcher app buttons
    int generalVerticalPadding = 0;  ///< General vertical padding for settings etc
    int localeSettingsOffset = 0;    ///< Horizontal offset for timezone label
    int systemInfoPadding = 0;       ///< Bottom padding for system info labels
    int wifiManageScrollbarWidth = 0; ///< Vertical padding for wifi manage items
    
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

    /**
     * Default constructor - initializes all fields to zero.
     * Use calculate() or calculateFromDisplay() to get proper values.
     */
    UiMetrics() = default;
};

} // namespace tt::hal

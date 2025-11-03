#include "Constants.h"
#include "Display.h"
#include "Touch.h"

#include <Tactility/Log.h>
#include <St7789i8080Display.h>
#include <PwmBacklight.h>

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    // Create configuration
    auto config = St7789i8080Display::Configuration(
        DISPLAY_CS,   // CS
        DISPLAY_DC,   // DC
        DISPLAY_WR,   // WR
        DISPLAY_RD,   // RD
        { DISPLAY_I80_D0, DISPLAY_I80_D1, DISPLAY_I80_D2, DISPLAY_I80_D3,
          DISPLAY_I80_D4, DISPLAY_I80_D5, DISPLAY_I80_D6, DISPLAY_I80_D7 }, // D0..D7
        DISPLAY_RST,   // RST
        DISPLAY_BL     // BL
    );
    
    // Set resolution explicitly
    config.horizontalResolution = DISPLAY_HORIZONTAL_RESOLUTION;
    config.verticalResolution = DISPLAY_VERTICAL_RESOLUTION;
    config.backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;

    // Add touch device
    config.touch = createTouch();
    
    if (!config.touch) {
        TT_LOG_E("Display", "Failed to create touch device");
        return nullptr;
    }
    
    auto display = std::make_shared<St7789i8080Display>(config);
    return display;
}
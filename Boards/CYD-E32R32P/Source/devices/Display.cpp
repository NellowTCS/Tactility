#include "Display.h"

#include <Xpt2046SoftSpi.h>
#include <St7789Display.h>
#include <PwmBacklight.h>
#include <Tactility/hal/touch/TouchDevice.h>

// Create the XPT2046 touch device using software SPI pins shared with the display
static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    auto config = std::make_unique<Xpt2046SoftSpi::Configuration>(
        CYD_TOUCH_MOSI_PIN,
        CYD_TOUCH_MISO_PIN,
        CYD_TOUCH_SCK_PIN,
        CYD_TOUCH_CS_PIN,
        CYD_DISPLAY_HORIZONTAL_RESOLUTION,
        CYD_DISPLAY_VERTICAL_RESOLUTION,
        false, // swap_xy
        false, // invert_x
        false  // invert_y
    );

    return std::make_shared<Xpt2046SoftSpi>(std::move(config));
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto configuration = std::make_unique<St7789Display::Configuration>(
        CYD_DISPLAY_SPI_HOST,
        CYD_DISPLAY_PIN_CS,
        CYD_DISPLAY_PIN_DC,
        CYD_DISPLAY_HORIZONTAL_RESOLUTION,
        CYD_DISPLAY_VERTICAL_RESOLUTION,
        createTouch(),
        false, // swapXY
        false, // mirrorX
        false, // mirrorY
        false, // invertColor
        0,     // bufferSize: default (1/10 of screen)
        0,     // gapX
        0      // gapY
    );

    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;
    return std::make_shared<St7789Display>(std::move(configuration));
}
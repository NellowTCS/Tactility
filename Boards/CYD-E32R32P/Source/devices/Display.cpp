#include "Display.h"

#include <Xpt2046Touch.h>
#include <St7789Display.h>
#include <PwmBacklight.h>
#include <Tactility/hal/touch/TouchDevice.h>

// Create the XPT2046 touch device (hardware/esp_lcd driver)
static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    auto config = std::make_unique<Xpt2046Touch::Configuration>(
        CYD_DISPLAY_SPI_HOST,                                // spi device / bus (SPI2_HOST)
        CYD_TOUCH_CS_PIN,                                    // touch CS (IO33)
        (uint16_t)CYD_DISPLAY_HORIZONTAL_RESOLUTION,         // x max
        (uint16_t)CYD_DISPLAY_VERTICAL_RESOLUTION,           // y max
        false,                                               // swapXy
        true,                                               // mirrorX
        true                                                // mirrorY
    );

    return std::make_shared<Xpt2046Touch>(std::move(config));
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    // Use the ST7789 display driver (ST7789P3 panel)
    auto configuration = std::make_unique<St7789Display::Configuration>(
        CYD_DISPLAY_SPI_HOST,                    // spi host/device (SPI2_HOST)
        CYD_DISPLAY_PIN_CS,                      // CS pin (IO15)
        CYD_DISPLAY_PIN_DC,                      // DC pin (IO2)
        CYD_DISPLAY_HORIZONTAL_RESOLUTION,       // horizontal resolution
        CYD_DISPLAY_VERTICAL_RESOLUTION,         // vertical resolution
        createTouch(),                           // touch device
        false,                                   // swapXY
        false,                                   // mirrorX
        false,                                   // mirrorY
        false,                                   // invertColor
        0,                                       // bufferSize (0 -> default 1/10 screen)
        0,                                       // gapX
        0                                        // gapY
        LCD_RGB_ELEMENT_ORDER_BGR                // colorOrder
    );

    // Backlight control via PWM driver
    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;
    return std::make_shared<St7789Display>(std::move(configuration));
}
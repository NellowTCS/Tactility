#include "Display.h"

#include <Xpt2046SoftSpi.h>
#include <Ili934xDisplay.h>
#include <PwmBacklight.h>
#include <Tactility/hal/touch/TouchDevice.h>

/**
 * @brief Create a touch device instance backed by an XPT2046 soft-SPI driver.
 *
 * Constructs and returns a shared_ptr to an Xpt2046SoftSpi touch device configured
 * with the board-specific SPI pins and the display's horizontal and vertical resolution.
 *
 * @return std::shared_ptr<tt::hal::touch::TouchDevice> Shared pointer to the configured touch device.
 */
static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    auto config = std::make_unique<Xpt2046SoftSpi::Configuration>(
        CYD_TOUCH_MOSI_PIN,
        CYD_TOUCH_MISO_PIN,
        CYD_TOUCH_SCK_PIN,
        CYD_TOUCH_CS_PIN,
        CYD_DISPLAY_HORIZONTAL_RESOLUTION,
        CYD_DISPLAY_VERTICAL_RESOLUTION,
        false,
        true,
        false
    );

    return std::make_shared<Xpt2046SoftSpi>(std::move(config));
}

/**
 * @brief Create and return a configured display device for the CYD board.
 *
 * Constructs an Ili934xDisplay configured with the CYD SPI host, CS/DC pins,
 * horizontal/vertical resolution, a software SPI XPT2046 touch device, and
 * the PWM backlight duty function. The display is set to use BGR element order.
 *
 * @return std::shared_ptr<tt::hal::display::DisplayDevice> Shared pointer to the initialized display device.
 */
std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto configuration = std::make_unique<Ili934xDisplay::Configuration>(
        CYD_DISPLAY_SPI_HOST,
        CYD_DISPLAY_PIN_CS,
        CYD_DISPLAY_PIN_DC,
        CYD_DISPLAY_HORIZONTAL_RESOLUTION,
        CYD_DISPLAY_VERTICAL_RESOLUTION,
        createTouch(),
        false,
        true,
        false,
        false,
        0,
        LCD_RGB_ELEMENT_ORDER_BGR
    );
    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;
    return std::make_shared<Ili934xDisplay>(std::move(configuration));
}

#include "YellowDisplay.h"
#include "Cst816Touch.h"
#include "YellowDisplayConstants.h"
#include "sdkconfig.h"

#include <St7789Display.h>
#include <PwmBacklight.h>

static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    // Note: GPIO 25 for reset and GPIO 21 for interrupt?
    auto configuration = std::make_unique<Cst816sTouch::Configuration>(
        I2C_NUM_0,
        240,
        320
    );

    return std::make_shared<Cst816sTouch>(std::move(configuration));
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {

    auto touch = createTouch();

    // Use only the parameters not already set in SDK config
    auto configuration = std::make_unique<St7789Display::Configuration>(
        CONFIG_LCD_I80_BUS_SUPPORT,              // Should be passed if relevant
        CONFIG_LCD_I80_BUS_WIDTH,                // Bus width
        CONFIG_LCD_I80_BUS_CONFIG_CLK_SRC,       // Clock source
        CONFIG_LCD_I80_BUS_CONFIG_DC,            // Data/Command pin
        CONFIG_LCD_I80_BUS_CONFIG_WR,            // Write pin
        CONFIG_LCD_I80_BUS_CONFIG_CS_GPIO_NUM,   // Chip Select GPIO (ensure this is correct)
        CONFIG_LCD_I80_BUS_CONFIG_BUS_WIDTH,     // Bus width (likely same as `CONFIG_LCD_I80_BUS_WIDTH`)
        touch                                    // Touch device (created separately)
    );

    configuration->mirrorX = true;
    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;

    return std::make_shared<St7789Display>(std::move(configuration));
}


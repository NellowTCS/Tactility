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

    // Ensure parameters match the constructor
    auto configuration = std::make_unique<St7789Display::Configuration>(
        static_cast<esp_lcd_spi_bus_handle_t>(CONFIG_LCD_I80_BUS_SUPPORT),  // Convert if necessary
        static_cast<gpio_num_t>(CONFIG_LCD_I80_BUS_CONFIG_DC),              // Ensure correct type
        static_cast<gpio_num_t>(CONFIG_LCD_I80_BUS_CONFIG_WR),              // Ensure correct type
        CONFIG_LCD_I80_BUS_WIDTH,                                           // Assuming this is unsigned int
        CONFIG_LCD_I80_BUS_CONFIG_CLK_SRC,                                   // Assuming this is unsigned int
        touch,                                                              // Shared pointer for touch
        true, false, false, false, 0                                        // Additional bools and uint32_t
    );

    configuration->mirrorX = true;
    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;

    return std::make_shared<St7789Display>(std::move(configuration));
}


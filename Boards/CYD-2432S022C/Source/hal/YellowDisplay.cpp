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

    // Replace Ili934xDisplay with St7789Display for I80 parallel interface
    auto configuration = std::make_unique<St7789Display::Configuration>(
        CONFIG_LCD_I80_BUS_SUPPORT,
        CONFIG_LCD_I80_BUS_WIDTH,
        CONFIG_LCD_I80_BUS_CONFIG_CLK_SRC,
        CONFIG_LCD_I80_BUS_CONFIG_DC,
        CONFIG_LCD_I80_BUS_CONFIG_WR,
        CONFIG_LCD_I80_BUS_CONFIG_DATA_GPIO_D8, // Adjust these GPIOs as necessary
        CONFIG_LCD_I80_BUS_CONFIG_DATA_GPIO_D9,
        CONFIG_LCD_I80_BUS_CONFIG_DATA_GPIO_D10,
        CONFIG_LCD_I80_BUS_CONFIG_DATA_GPIO_D11,
        CONFIG_LCD_I80_BUS_CONFIG_DATA_GPIO_D12,
        CONFIG_LCD_I80_BUS_CONFIG_DATA_GPIO_D13,
        CONFIG_LCD_I80_BUS_CONFIG_DATA_GPIO_D14,
        CONFIG_LCD_I80_BUS_CONFIG_DATA_GPIO_D15,
        CONFIG_LCD_I80_BUS_CONFIG_BUS_WIDTH,
        touch
    );

    configuration->mirrorX = true;
    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;

    return std::make_shared<St7789Display>(std::move(configuration));
}

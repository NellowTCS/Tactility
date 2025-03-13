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
        LCD_I80_BUS_SUPPORT,
        LCD_I80_BUS_WIDTH,
        LCD_I80_BUS_CONFIG_CLK_SRC,
        LCD_I80_BUS_CONFIG_DC,
        LCD_I80_BUS_CONFIG_WR,
        LCD_I80_BUS_CONFIG_DATA_GPIO_D8, // Adjust these GPIOs as necessary
        LCD_I80_BUS_CONFIG_DATA_GPIO_D9,
        LCD_I80_BUS_CONFIG_DATA_GPIO_D10,
        LCD_I80_BUS_CONFIG_DATA_GPIO_D11,
        LCD_I80_BUS_CONFIG_DATA_GPIO_D12,
        LCD_I80_BUS_CONFIG_DATA_GPIO_D13,
        LCD_I80_BUS_CONFIG_DATA_GPIO_D14,
        LCD_I80_BUS_CONFIG_DATA_GPIO_D15,
        LCD_I80_BUS_CONFIG_BUS_WIDTH,
        touch
    );

    configuration->mirrorX = true;
    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;

    return std::make_shared<St7789Display>(std::move(configuration));
}

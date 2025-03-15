#include "YellowDisplay.h"
#include "Cst816Touch.h"
#include "YellowDisplayConstants.h"
#include "sdkconfig.h"
#include <St7789Display.h>
#include <PwmBacklight.h>

static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    auto configuration = std::make_unique<Cst816sTouch::Configuration>(
        I2C_NUM_0,
        240,
        320
    );
    return std::make_shared<Cst816sTouch>(std::move(configuration));
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto touch = createTouch();

    // I80 config for ST7789
    auto configuration = std::make_unique<St7789Display::Configuration>(
        1,                  // I80 bus ID (arbitrary, matches CONFIG_LCD_I80_BUS_SUPPORT)
        GPIO_NUM_16,        // DC
        GPIO_NUM_4,         // WR
        8,                  // Bus width (8-bit parallel)
        LCD_CLK_SRC_PLL160M,// Clock source
        touch,
        true, false, false, false, 0  // Mirror/rotation flags
    );

    // Set data pins explicitly if required by constructor
    configuration->dataPins = {GPIO_NUM_15, GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_14,
                              GPIO_NUM_27, GPIO_NUM_25, GPIO_NUM_33, GPIO_NUM_32};
    configuration->csPin = GPIO_NUM_17;  // CS
    configuration->mirrorX = true;
    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;

    return std::make_shared<St7789Display>(std::move(configuration));
}

#include "Display.h"
#include "Axs15231bDisplay.h"
#include "Axs15231bTouch.h"

#include <PwmBacklight.h>

#define JC3248W535C_LCD_DRAW_BUFFER_SIZE (320 * (480 / 10))

static std::shared_ptr<tt::hal::touch::TouchDevice> createTouch() {
    auto configuration = std::make_unique<Axs15231bTouch::Configuration>(
        I2C_NUM_0,
        320,
        480,
        false,
        false,
        false,
        GPIO_NUM_NC,
        GPIO_NUM_3
    );

    return std::make_shared<Axs15231bTouch>(std::move(configuration));
}

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    auto touch = createTouch();

    auto configuration = std::make_unique<Axs15231bDisplay::Configuration>(
        SPI2_HOST,
        GPIO_NUM_45,
        GPIO_NUM_NC,
        GPIO_NUM_NC,
        320,
        480,
        touch,
        false,
        true,
        true,
        false,
        JC3248W535C_LCD_DRAW_BUFFER_SIZE
    );

    configuration->backlightDutyFunction = driver::pwmbacklight::setBacklightDuty;

    auto display = std::make_shared<Axs15231bDisplay>(std::move(configuration));
    return std::reinterpret_pointer_cast<tt::hal::display::DisplayDevice>(display);
}
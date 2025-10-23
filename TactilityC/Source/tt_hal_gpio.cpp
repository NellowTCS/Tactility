#include "tt_hal_gpio.h"
#include <Tactility/hal/gpio/Gpio.h>
#include <map>

extern "C" {

using namespace tt::hal;

// Storage for C callback wrappers
static std::map<GpioPin, std::pair<GpioInterruptHandler, void*>> cHandlers;

bool tt_hal_gpio_configure(GpioPin pin, GpioMode mode, bool pullUp, bool pullDown) {
    return gpio::configure(pin, static_cast<gpio::Mode>(mode), pullUp, pullDown);
}

bool tt_hal_gpio_configure_with_pin_bitmask(uint64_t pinBitMask, GpioMode mode, bool pullUp, bool pullDown) {
    return gpio::configureWithPinBitmask(pinBitMask, static_cast<gpio::Mode>(mode), pullUp, pullDown);
}

bool tt_hal_gpio_set_mode(GpioPin pin, GpioMode mode) {
    return gpio::setMode(pin, static_cast<gpio::Mode>(mode));
}

bool tt_hal_gpio_get_level(GpioPin pin) {
    return gpio::getLevel(pin);
}

bool tt_hal_gpio_set_level(GpioPin pin, bool level) {
    return gpio::setLevel(pin, level);
}

int tt_hal_gpio_get_pin_count() {
    return gpio::getPinCount();
}

bool tt_hal_gpio_install_interrupt_service() {
    return gpio::installInterruptService();
}

bool tt_hal_gpio_attach_interrupt(GpioPin pin, GpioInterruptMode mode, GpioInterruptHandler handler, void* user_data) {
    // Store the C handler and user data
    cHandlers[pin] = {handler, user_data};
    
    // Create a C++ lambda that calls the C handler
    auto cppHandler = [pin]() {
        auto it = cHandlers.find(pin);
        if (it != cHandlers.end() && it->second.first) {
            it->second.first(it->second.second);
        }
    };
    
    return gpio::attachInterrupt(pin, static_cast<gpio::InterruptMode>(mode), cppHandler);
}

bool tt_hal_gpio_detach_interrupt(GpioPin pin) {
    cHandlers.erase(pin);
    return gpio::detachInterrupt(pin);
}

bool tt_hal_gpio_enable_interrupt(GpioPin pin) {
    return gpio::enableInterrupt(pin);
}

bool tt_hal_gpio_disable_interrupt(GpioPin pin) {
    return gpio::disableInterrupt(pin);
}

}
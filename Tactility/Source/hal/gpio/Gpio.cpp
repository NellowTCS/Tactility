#include <Tactility/hal/gpio/Gpio.h>

#ifdef ESP_PLATFORM
#include <driver/gpio.h>
#include <map>
#include <esp_log.h>
#endif

namespace tt::hal::gpio {

#ifdef ESP_PLATFORM

constexpr gpio_num_t toEspPin(Pin pin) { return static_cast<gpio_num_t>(pin); }

constexpr gpio_mode_t toEspGpioMode(Mode mode) {
    switch (mode) {
        case Mode::Input:
            return GPIO_MODE_INPUT;
        case Mode::Output:
            return GPIO_MODE_OUTPUT;
        case Mode::OutputOpenDrain:
            return GPIO_MODE_OUTPUT_OD;
        case Mode::InputOutput:
            return GPIO_MODE_INPUT_OUTPUT;
        case Mode::InputOutputOpenDrain:
            return GPIO_MODE_INPUT_OUTPUT_OD;
        case Mode::Disable:
        default:
            return GPIO_MODE_DISABLE;
    }
}

constexpr gpio_int_type_t toEspInterruptMode(InterruptMode mode) {
    switch (mode) {
        case InterruptMode::RisingEdge:
            return GPIO_INTR_POSEDGE;
        case InterruptMode::FallingEdge:
            return GPIO_INTR_NEGEDGE;
        case InterruptMode::AnyEdge:
            return GPIO_INTR_ANYEDGE;
        case InterruptMode::LowLevel:
            return GPIO_INTR_LOW_LEVEL;
        case InterruptMode::HighLevel:
            return GPIO_INTR_HIGH_LEVEL;
        case InterruptMode::Disable:
        default:
            return GPIO_INTR_DISABLE;
    }
}

// Storage for interrupt handlers (protected by ESP-IDF's ISR context)
static std::map<Pin, InterruptHandler> interruptHandlers;
static bool isrServiceInstalled = false;

// Trampoline function that ESP-IDF calls, which then calls our C++ handler
static void IRAM_ATTR gpioIsrTrampoline(void* arg) {
    Pin pin = reinterpret_cast<Pin>(arg);
    auto it = interruptHandlers.find(pin);
    if (it != interruptHandlers.end() && it->second) {
        it->second();
    }
}

#endif

bool getLevel(Pin pin) {
#ifdef ESP_PLATFORM
    return gpio_get_level(toEspPin(pin)) == 1;
#else
    return false;
#endif
}

bool setLevel(Pin pin, bool level) {
#ifdef ESP_PLATFORM
    return gpio_set_level(toEspPin(pin), level) == ESP_OK;
#else
    return true;
#endif
}

int getPinCount() {
#ifdef ESP_PLATFORM
    return GPIO_NUM_MAX;
#else
    return 16;
#endif
}

bool configureWithPinBitmask(uint64_t pinBitMask, Mode mode, bool pullUp, bool pullDown) {
#ifdef ESP_PLATFORM
    gpio_config_t sd_gpio_config = {
        .pin_bit_mask = pinBitMask,
        .mode = toEspGpioMode(mode),
        .pull_up_en = pullUp ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = pullDown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&sd_gpio_config) == ESP_OK;
#else
    return true;
#endif
}

bool configure(Pin pin, Mode mode, bool pullUp, bool pullDown) {
#ifdef ESP_PLATFORM
    return configureWithPinBitmask(BIT64(toEspPin(pin)), mode, pullUp, pullDown);
#else
    return true;
#endif
}

bool setMode(Pin pin, Mode mode) {
#ifdef ESP_PLATFORM
    return gpio_set_direction(toEspPin(pin), toEspGpioMode(mode)) == ESP_OK;
#endif
    return true;
}

bool installInterruptService() {
#ifdef ESP_PLATFORM
    if (isrServiceInstalled) {
        return true;
    }
    esp_err_t err = gpio_install_isr_service(0);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means it's already installed (which is fine)
        isrServiceInstalled = true;
        return true;
    }
    return false;
#else
    return true;
#endif
}

bool attachInterrupt(Pin pin, InterruptMode mode, InterruptHandler handler) {
#ifdef ESP_PLATFORM
    if (!isrServiceInstalled) {
        if (!installInterruptService()) {
            ESP_LOGE("GPIO", "Failed to install ISR service");
            return false;
        }
    }

    // Store the handler before configuring hardware
    interruptHandlers[pin] = std::move(handler);

    // Configure interrupt type
    esp_err_t err = gpio_set_intr_type(toEspPin(pin), toEspInterruptMode(mode));
    if (err != ESP_OK) {
        ESP_LOGE("GPIO", "Failed to set interrupt type for pin %d: %d", pin, err);
        interruptHandlers.erase(pin);
        return false;
    }

    // Add ISR handler with pin as argument
    err = gpio_isr_handler_add(toEspPin(pin), gpioIsrTrampoline, reinterpret_cast<void*>(pin));
    if (err != ESP_OK) {
        ESP_LOGE("GPIO", "Failed to add ISR handler for pin %d: %d", pin, err);
        interruptHandlers.erase(pin);
        gpio_set_intr_type(toEspPin(pin), GPIO_INTR_DISABLE);
        return false;
    }

    ESP_LOGI("GPIO", "Attached interrupt to pin %d, mode %d", pin, static_cast<int>(mode));
    return true;
#else
    return true;
#endif
}

bool detachInterrupt(Pin pin) {
#ifdef ESP_PLATFORM
    // Disable interrupt first
    gpio_set_intr_type(toEspPin(pin), GPIO_INTR_DISABLE);
    
    // Remove ISR handler
    gpio_isr_handler_remove(toEspPin(pin));
    
    // Remove from our map
    interruptHandlers.erase(pin);
    
    return true;
#else
    return true;
#endif
}

bool enableInterrupt(Pin pin) {
#ifdef ESP_PLATFORM
    return gpio_intr_enable(toEspPin(pin)) == ESP_OK;
#else
    return true;
#endif
}

bool disableInterrupt(Pin pin) {
#ifdef ESP_PLATFORM
    return gpio_intr_disable(toEspPin(pin)) == ESP_OK;
#else
    return true;
#endif
}

}

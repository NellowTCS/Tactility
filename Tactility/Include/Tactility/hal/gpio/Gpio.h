#pragma once

#include <cstdint>
#include <functional>

namespace tt::hal::gpio {

typedef unsigned int Pin;
constexpr Pin NO_PIN = -1;

/** @warning The order must match GpioMode from tt_hal_gpio.h */
enum class Mode {
    Disable = 0,
    Input,
    Output,
    OutputOpenDrain,
    InputOutput,
    InputOutputOpenDrain
};

/** Interrupt trigger modes */
enum class InterruptMode {
    Disable = 0,
    RisingEdge,
    FallingEdge,
    AnyEdge,
    LowLevel,
    HighLevel
};

/** Interrupt handler callback type */
using InterruptHandler = std::function<void()>;

/** Configure a single pin */
bool configure(Pin pin, Mode mode, bool pullUp, bool pullDown);

/** Configure a set of pins defined by their bit index */
bool configureWithPinBitmask(uint64_t pinBitMask, Mode mode, bool pullUp, bool pullDown);

bool setMode(Pin pin, Mode mode);

bool getLevel(Pin pin);

bool setLevel(Pin pin, bool level);

int getPinCount();

/** Install the GPIO interrupt service (must be called before attaching interrupts) */
bool installInterruptService();

/** Attach an interrupt handler to a pin
 * @param pin The GPIO pin
 * @param mode The interrupt trigger mode
 * @param handler The callback function to call when interrupt fires
 * @return true on success
 */
bool attachInterrupt(Pin pin, InterruptMode mode, InterruptHandler handler);

/** Detach interrupt handler from a pin
 * @param pin The GPIO pin
 * @return true on success
 */
bool detachInterrupt(Pin pin);

/** Enable interrupts on a pin (after being disabled)
 * @param pin The GPIO pin
 * @return true on success
 */
bool enableInterrupt(Pin pin);

/** Disable interrupts on a pin temporarily (without detaching)
 * @param pin The GPIO pin
 * @return true on success
 */
bool disableInterrupt(Pin pin);

}
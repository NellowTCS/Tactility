#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Logical GPIO pin identifier used by the HAL. Typically maps to the SoC GPIO number. */
typedef unsigned int GpioPin;
/** Value indicating that no GPIO pin is used/applicable. */
#define GPIO_NO_PIN -1

/** GPIO pin mode used by the HAL.
 * @warning The order must match tt::hal::gpio::Mode
 */
enum GpioMode {
    /** Pin is disabled (high-impedance). */
    GpioModeDisable = 0,
    /** Pin configured as input only. */
    GpioModeInput,
    /** Pin configured as push-pull output only. */
    GpioModeOutput,
    /** Pin configured as open-drain output only. */
    GpioModeOutputOpenDrain,
    /** Pin configured for both input and output (push-pull). */
    GpioModeInputOutput,
    /** Pin configured for both input and output (open-drain). */
    GpioModeInputOutputOpenDrain
};

/** GPIO interrupt modes.
 * @warning The order must match tt::hal::gpio::InterruptMode
 */
enum GpioInterruptMode {
    /** Interrupt disabled. */
    GpioInterruptModeDisable = 0,
    /** Trigger on rising edge (low to high). */
    GpioInterruptModeRisingEdge,
    /** Trigger on falling edge (high to low). */
    GpioInterruptModeFallingEdge,
    /** Trigger on any edge (rising or falling). */
    GpioInterruptModeAnyEdge,
    /** Trigger when pin is low. */
    GpioInterruptModeLowLevel,
    /** Trigger when pin is high. */
    GpioInterruptModeHighLevel
};

/** GPIO interrupt handler callback type */
typedef void (*GpioInterruptHandler)(void* user_data);

/** Configure a single GPIO pin.
 * @param[in] pin      GPIO number to configure.
 * @param[in] mode     Desired I/O mode for the pin.
 * @param[in] pullUp   Enable internal pull-up if true.
 * @param[in] pullDown Enable internal pull-down if true.
 * @return true on success, false if the pin is invalid or configuration failed.
 */
bool tt_hal_gpio_configure(GpioPin pin, GpioMode mode, bool pullUp, bool pullDown);

/** Configure a set of GPIO pins in one call.
 * The bit index of pin N is (1ULL << N).
 * @param[in] pinBitMask Bit mask of pins to configure.
 * @param[in] mode       Desired I/O mode for the pins.
 * @param[in] pullUp     Enable internal pull-up on the selected pins if true.
 * @param[in] pullDown   Enable internal pull-down on the selected pins if true.
 * @return true on success, false if any pin is invalid or configuration failed.
 */
bool tt_hal_gpio_configure_with_pin_bitmask(uint64_t pinBitMask, GpioMode mode, bool pullUp, bool pullDown);

/** Set the input/output mode for the specified pin.
 * @param[in] pin  The pin to configure.
 * @param[in] mode The mode to set.
 * @return true on success, false if the pin is invalid or mode not supported.
 */
bool tt_hal_gpio_set_mode(GpioPin pin, GpioMode mode);

/** Read the current logic level of a pin.
 * The pin should be configured for input or input/output.
 * @param[in] pin The pin to read.
 * @return true if the level is high, false if low. If the pin is invalid, the
 *         behavior is implementation-defined and may return false.
 */
bool tt_hal_gpio_get_level(GpioPin pin);

/** Drive the output level of a pin.
 * The pin should be configured for output or input/output.
 * @param[in] pin   The pin to drive.
 * @param[in] level Output level to set (true = high, false = low).
 * @return true on success, false if the pin is invalid or not configured as output.
 */
bool tt_hal_gpio_set_level(GpioPin pin, bool level);

/** Get the number of GPIO pins available on this platform.
 * @return The count of valid GPIO pins.
 */
int tt_hal_gpio_get_pin_count();

/** Install the GPIO interrupt service.
 * Must be called before attaching any interrupt handlers.
 * @return true on success, false if installation failed.
 */
bool tt_hal_gpio_install_interrupt_service();

/** Attach an interrupt handler to a GPIO pin.
 * @param[in] pin       The GPIO pin to attach the interrupt to.
 * @param[in] mode      The interrupt trigger mode.
 * @param[in] handler   The callback function to invoke when the interrupt fires.
 * @param[in] user_data User data pointer passed to the handler.
 * @return true on success, false if attachment failed.
 */
bool tt_hal_gpio_attach_interrupt(GpioPin pin, GpioInterruptMode mode, GpioInterruptHandler handler, void* user_data);

/** Detach an interrupt handler from a GPIO pin.
 * @param[in] pin The GPIO pin to detach from.
 * @return true on success, false if detachment failed.
 */
bool tt_hal_gpio_detach_interrupt(GpioPin pin);

/** Enable interrupts on a GPIO pin.
 * @param[in] pin The GPIO pin.
 * @return true on success, false if enable failed.
 */
bool tt_hal_gpio_enable_interrupt(GpioPin pin);

/** Disable interrupts on a GPIO pin.
 * @param[in] pin The GPIO pin.
 * @return true on success, false if disable failed.
 */
bool tt_hal_gpio_disable_interrupt(GpioPin pin);

#ifdef __cplusplus
}
#endif

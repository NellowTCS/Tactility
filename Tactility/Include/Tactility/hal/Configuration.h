#pragma once

#include <Tactility/hal/sdcard/SdCardDevice.h>
#include <Tactility/hal/spi/Spi.h>
#include <Tactility/hal/uart/Uart.h>
#include <Tactility/hal/UiMetrics.h>

#include "i2c/I2c.h"

namespace tt::hal {

typedef bool (*InitBoot)();

typedef std::vector<std::shared_ptr<Device>> DeviceVector;

typedef std::shared_ptr<Device> (*CreateDevice)();

enum class LvglInit {
    Default,
    None
};

/** Affects LVGL widget style
 * @deprecated Use UiMetrics instead - this enum is kept for backwards compatibility only.
 * The system now automatically calculates UI scaling based on screen resolution.
 */
enum class UiScale {
    /** Ideal for very small non-touch screen devices (e.g. Waveshare S3 LCD 1.3") */
    Smallest,
    /** Nothing was changed in the LVGL UI/UX */
    Default
};

struct Configuration {
    /**
     * Called before I2C/SPI/etc is initialized.
     * Used for powering on the peripherals manually.
     */
    const InitBoot _Nullable initBoot = nullptr;

    /** Init behaviour: default (esp_lvgl_port for ESP32, nothing for PC) or None (nothing on any platform). Only used in Tactility, not in TactilityHeadless. */
    const LvglInit lvglInit = LvglInit::Default;

    /** @deprecated Use uiMetrics instead. This field is ignored and kept only for backwards compatibility. */
    const UiScale uiScale = UiScale::Default;
    
    /** Dynamic UI scaling metrics - automatically calculated from screen resolution at runtime.
     * This replaces the old binary uiScale enum with a continuous scaling system.
     * Do not set this manually - it will be calculated automatically after display initialization.
     * Marked mutable so it can be calculated even when Configuration is const.
     */
    mutable UiMetrics uiMetrics;

    std::function<DeviceVector()> createDevices = [] { return std::vector<std::shared_ptr<Device>>(); };

    /** A list of I2C interface configurations */
    const std::vector<i2c::Configuration> i2c = {};

    /** A list of SPI interface configurations */
    const std::vector<spi::Configuration> spi = {};

    /** A list of UART interface configurations */
    const std::vector<uart::Configuration> uart = {};
};

} // namespace

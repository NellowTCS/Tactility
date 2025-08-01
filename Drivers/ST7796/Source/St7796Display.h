#pragma once

#include "Tactility/hal/display/DisplayDevice.h"

#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_types.h>
#include <functional>
#include <lvgl.h>

class St7796Display final : public tt::hal::display::DisplayDevice {

public:

    class Configuration {

    public:

        Configuration(
            esp_lcd_spi_bus_handle_t spi_bus_handle,
            gpio_num_t csPin,
            gpio_num_t dcPin,
            unsigned int horizontalResolution,
            unsigned int verticalResolution,
            std::shared_ptr<tt::hal::touch::TouchDevice> touch,
            bool swapXY = false,
            bool mirrorX = false,
            bool mirrorY = false,
            bool invertColor = false,
            unsigned int gapX = 0,
            unsigned int gapY = 0,
            uint32_t bufferSize = 0 // Size in pixel count. 0 means default, which is 1/10 of the screen size
        ) : spiBusHandle(spi_bus_handle),
            csPin(csPin),
            dcPin(dcPin),
            horizontalResolution(horizontalResolution),
            verticalResolution(verticalResolution),
            swapXY(swapXY),
            mirrorX(mirrorX),
            mirrorY(mirrorY),
            invertColor(invertColor),
            gapX(gapX),
            gapY(gapY),
            bufferSize(bufferSize),
            touch(std::move(touch)) {}

        esp_lcd_spi_bus_handle_t spiBusHandle;
        gpio_num_t csPin;
        gpio_num_t dcPin;
        gpio_num_t resetPin = GPIO_NUM_NC;
        unsigned int pixelClockFrequency = 80'000'000; // Hertz
        size_t transactionQueueDepth = 2;
        unsigned int horizontalResolution;
        unsigned int verticalResolution;
        bool swapXY = false;
        bool mirrorX = false;
        bool mirrorY = false;
        bool invertColor = false;
        unsigned int gapX = 0;
        unsigned int gapY = 0;
        uint32_t bufferSize = 0; // Size in pixel count. 0 means default, which is 1/10 of the screen size
        std::shared_ptr<tt::hal::touch::TouchDevice> touch;
        std::function<void(uint8_t)> _Nullable backlightDutyFunction = nullptr;
    };

private:

    std::unique_ptr<Configuration> configuration;
    esp_lcd_panel_io_handle_t ioHandle = nullptr;
    esp_lcd_panel_handle_t panelHandle = nullptr;
    lv_display_t* displayHandle = nullptr;

public:

    explicit St7796Display(std::unique_ptr<Configuration> inConfiguration) : configuration(std::move(inConfiguration)) {
        assert(configuration != nullptr);
    }

    std::string getName() const final { return "ST7796"; }
    std::string getDescription() const final { return "ST7796 display"; }

    bool start() final;

    bool stop() final;

    std::shared_ptr<tt::hal::touch::TouchDevice> _Nullable createTouch() final { return configuration->touch; }

    void setBacklightDuty(uint8_t backlightDuty) final {
        if (configuration->backlightDutyFunction != nullptr) {
            configuration->backlightDutyFunction(backlightDuty);
        }
    }

    void setGammaCurve(uint8_t index) final;
    uint8_t getGammaCurveCount() const final { return 4; };

    bool supportsBacklightDuty() const final { return configuration->backlightDutyFunction != nullptr; }

    lv_display_t* _Nullable getLvglDisplay() const final { return displayHandle; }
};

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay();

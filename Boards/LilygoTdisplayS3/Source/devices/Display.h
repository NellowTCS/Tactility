#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_types.h>
#include <lvgl.h>

#include <memory>

class I8080St7789Display : public tt::hal::display::DisplayDevice {
public:
    struct Configuration {
        gpio_num_t csPin;
        gpio_num_t dcPin;
        gpio_num_t wrPin;
        gpio_num_t rdPin;
        std::array<gpio_num_t, 8> dataPins;
        gpio_num_t resetPin;
        gpio_num_t backlightPin;
        unsigned int pixelClockFrequency = 10 * 1000 * 1000;
        size_t transactionQueueDepth = 10;
        size_t bufferSize = 170 * 320;

        Configuration(gpio_num_t cs, gpio_num_t dc, gpio_num_t wr, gpio_num_t rd, std::array<gpio_num_t, 8> data,
                      gpio_num_t rst, gpio_num_t bl)
            : csPin(cs), dcPin(dc), wrPin(wr), rdPin(rd), dataPins(data), resetPin(rst), backlightPin(bl) {}
    };

private:
    std::unique_ptr<Configuration> configuration;
    esp_lcd_i80_bus_handle_t i80BusHandle = nullptr;
    esp_lcd_panel_io_handle_t ioHandle = nullptr;
    esp_lcd_panel_handle_t panelHandle = nullptr;

public:
    explicit I8080St7789Display(std::unique_ptr<Configuration> config) : configuration(std::move(config)) {}

    bool initialize();

    std::string getName() const override { return "I8080 ST7789"; }
    std::string getDescription() const override { return "I8080-based ST7789 display"; }
};

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay();
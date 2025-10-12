#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_types.h>
#include <lvgl.h>
#include <array>
#include <cstdint>

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
    Configuration configuration;
    esp_lcd_i80_bus_handle_t i80BusHandle = nullptr;
    esp_lcd_panel_io_handle_t ioHandle = nullptr;
    esp_lcd_panel_handle_t panelHandle = nullptr;

public:
    explicit I8080St7789Display(const Configuration& config) : configuration(config) {}

    bool initialize();

    std::string getName() const override { return "I8080 ST7789"; }
    std::string getDescription() const override { return "I8080-based ST7789 display"; }

    bool start() override { return true; }
    bool stop() override { return true; }
    std::shared_ptr<touch::TouchDevice> getTouchDevice() override { return nullptr; }
    bool supportsLvgl() const override { return true; }
    bool startLvgl() override { return true; }
    bool stopLvgl() override { return true; }
    lv_display_t* getLvglDisplay() const override { return nullptr; }
    bool supportsDisplayDriver() const override { return false; }
    std::shared_ptr<DisplayDriver> getDisplayDriver() override { return nullptr; }
};

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay();
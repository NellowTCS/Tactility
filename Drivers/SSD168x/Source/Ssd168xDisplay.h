#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <memory>
#include <lvgl.h>

#include "ssd1680.h"

class Ssd168xDisplay : public tt::hal::display::DisplayDevice {
public:
    struct Configuration {
        ssd1680_controller_t controller;  // SSD1680 or SSD1685
        uint16_t width;
        uint16_t height;
        gpio_num_t csPin;
        gpio_num_t dcPin;
        gpio_num_t resetPin;
        gpio_num_t busyPin;
        spi_host_device_t spiHost;
        uint8_t rotation;  // 0=portrait, 1=90° CW, 2=180°, 3=270° CW
    };

    explicit Ssd168xDisplay(std::unique_ptr<Configuration> inConfiguration);
    ~Ssd168xDisplay() override;

    // DisplayDevice interface
    std::string getName() const override;
    std::string getDescription() const override;
    bool start() override;
    bool stop() override;

    // Touch support (none for e-paper)
    std::shared_ptr<tt::hal::touch::TouchDevice> _Nullable getTouchDevice() override;

    // LVGL support
    bool supportsLvgl() const override;
    bool startLvgl() override;
    bool stopLvgl() override;
    lv_display_t* _Nullable getLvglDisplay() const override;

    // DisplayDriver (not implemented - e-paper needs LVGL for buffering)
    bool supportsDisplayDriver() const override;
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable getDisplayDriver() override;

private:
    struct Configuration configuration;
    ssd1680_handle_t ssd1680_handle;
    lv_display_t* lvglDisplay;
    uint8_t* frameBuffer;
    SemaphoreHandle_t framebufferMutex;
    
    static void lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
};

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay();

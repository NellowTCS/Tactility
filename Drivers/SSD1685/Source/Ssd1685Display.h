#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <memory>
#include <lvgl.h>

// Forward declare C struct
typedef struct epd_w21_handle_t epd_w21_handle_t;

class Ssd1685Display : public tt::hal::display::DisplayDevice {
public:
    struct Configuration {
        uint16_t width;
        uint16_t height;
        gpio_num_t dcPin;
        gpio_num_t rstPin;
        gpio_num_t busyPin;
        spi_device_handle_t spiHandle;
    };

    explicit Ssd1685Display(const Configuration& config);
    ~Ssd1685Display() override;

    std::string getName() const override;
    std::string getDescription() const override;
    bool start() override;
    bool stop() override;

    std::shared_ptr<tt::hal::touch::TouchDevice> _Nullable getTouchDevice() override;

    bool supportsLvgl() const override;
    bool startLvgl() override;
    bool stopLvgl() override;
    lv_display_t* _Nullable getLvglDisplay() const override;

    bool supportsDisplayDriver() const override;
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable getDisplayDriver() override;

    uint16_t getWidth() const;
    uint16_t getHeight() const;

private:
    Configuration _config;
    epd_w21_handle_t _epd_handle;
    lv_display_t* _lvglDisplay;
    lv_color_t* _drawBuf;
    uint8_t* _monoBuffer;
    bool _initialized;
    uint32_t _refreshCount;

    static void lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
};
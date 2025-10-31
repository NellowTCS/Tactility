#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <memory>
#include <lvgl.h>

class GxEPD2_290_GDEY029T71H; // Forward declaration

class GxEPD2Display : public tt::hal::display::DisplayDevice {
public:
    struct Configuration {
        uint16_t width;
        uint16_t height;
        gpio_num_t csPin;
        gpio_num_t dcPin;
        gpio_num_t rstPin;
        gpio_num_t busyPin;
        spi_host_device_t spiHost;
        uint8_t rotation = 0; // 0=none, 1=90°CCW, 2=90°CW
    };

    explicit GxEPD2Display(const Configuration& config);
    ~GxEPD2Display() override;

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

    // DisplayDriver (not implemented)
    bool supportsDisplayDriver() const override;
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable getDisplayDriver() override;

    // Minimal public helpers for tests / external use (safe, small API)
    uint16_t getWidth() const;
    uint16_t getHeight() const;

    // Write a packed 1bpp buffer to the panel via the driver (safe wrapper).
    // The buffer must be packed MSB-first per row, rows padded to byte boundary.
    void writeRawImage(const uint8_t* bitmap, int16_t x, int16_t y, int16_t w, int16_t h, bool invert = false, bool mirror_y = false);

    // Refresh the display; partial = true -> partial refresh, false -> full refresh
    void refreshDisplay(bool partial) ;

private:
    Configuration _config;
    std::unique_ptr<GxEPD2_290_GDEY029T71H> _display;
    lv_display_t* _lvglDisplay;
    lv_color_t* _drawBuf1;
    lv_color_t* _drawBuf2;

    static constexpr size_t DRAW_BUF_LINES = 10;

    static void lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
};

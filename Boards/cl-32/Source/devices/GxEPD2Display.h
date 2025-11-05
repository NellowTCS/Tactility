#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <memory>
#include <lvgl.h>
#include <esp_timer.h>

class GxEPD2_290_GDEY029T71H; // Forward declaration

class GxEPD2Display : public tt::hal::display::DisplayDevice {
public:
    struct Configuration {
        uint16_t width;          // Physical panel width
        uint16_t height;         // Physical panel height
        gpio_num_t csPin;
        gpio_num_t dcPin;
        gpio_num_t rstPin;
        gpio_num_t busyPin;
        spi_host_device_t spiHost;
        uint8_t rotation;        // 0=portrait, 1=90° CW, 2=180°, 3=270° CW
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

    // DisplayDriver (not implemented - e-paper needs LVGL for buffering)
    bool supportsDisplayDriver() const override;
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable getDisplayDriver() override;

    // Public API for tests/external use
    uint16_t getWidth() const;
    uint16_t getHeight() const;
    void writeRawImage(const uint8_t* bitmap, int16_t x, int16_t y, int16_t w, int16_t h, 
                       bool invert = false, bool mirror_y = false);
    void refreshDisplay(bool partial);

private:
    Configuration _config;
    std::unique_ptr<GxEPD2_290_GDEY029T71H> _display;
    lv_display_t* _lvglDisplay;
    lv_color_t* _drawBuf1;
    lv_color_t* _drawBuf2;

    // Number of logical rows per LVGL draw buffer. Adjust based on available RAM.
    // The startLvgl() now allocates buffers using LVGL logical width (swapped for 90/270).
    static constexpr size_t DRAW_BUF_LINES = 50;

    // Refresh debounce timer - we schedule a single partial refresh shortly after
    // LVGL writes complete to batch multiple small writes into one panel refresh.
    esp_timer_handle_t _refreshTimer;
    bool _refreshScheduled;

    static void lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
    
    // Convert RGB565 pixel to monochrome (true=white, false=black)
    static inline bool rgb565ToMono(lv_color_t pixel);

    // Timer callback used for debouncing partial refreshes
    static void refreshTimerCb(void* arg);

    // Helper to create/start the refresh timer; defined in the .cpp file.
    static void ensureTimerCreated(GxEPD2Display* self);
};

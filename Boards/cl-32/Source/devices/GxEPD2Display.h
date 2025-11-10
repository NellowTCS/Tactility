#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <memory>
#include <lvgl.h>
#include <esp_timer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <cstdint>

#include <Adafruit_GFX.h>
#include "GxEPD2/GxEPD2_BW.h"

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

    // Offsets to apply to all physical coordinates (useful to shift panels)
    void setGap(int16_t gapX, int16_t gapY) { _gapX = gapX; _gapY = gapY; }
    void getGap(int16_t& gapX, int16_t& gapY) const { gapX = _gapX; gapY = _gapY; }

private:
    Configuration _config;
    std::unique_ptr<GxEPD2_290_GDEY029T71H> _epd2_native; // for direct use
    std::unique_ptr<GxEPD2_BW<GxEPD2_290_GDEY029T71H, 8>> _epd2_bw; // For Adafruit GFX integration
    lv_display_t* _lvglDisplay;
    lv_color_t* _drawBuf1;
    lv_color_t* _drawBuf2;

    // Persistent framebuffer for e-paper (stores complete display state)
    uint8_t* _frameBuffer;
    SemaphoreHandle_t _framebufferMutex;

    // Number of logical rows per LVGL draw buffer. Adjust based on available RAM.
    static constexpr size_t DRAW_BUF_LINES = 60;

    // Display worker queue item (enqueued by lvglFlushCallback)
    struct QueueItem {
        uint8_t* buf;   // MALLOC_CAP_DMA allocated buffer (1-bit packed EPD format)
        uint16_t x;
        uint16_t y;
        uint16_t w;
        uint16_t h;
    };

    // Worker queue and task
    QueueHandle_t _queue;
    TaskHandle_t _workerTaskHandle;
    static constexpr size_t QUEUE_LENGTH = 32;

    // Mutex to protect direct access to _display (and to avoid races when tests write directly)
    SemaphoreHandle_t _spiMutex;

    // Refresh debouncing/batching is handled by the worker (it groups writes and then refreshes)
    bool _workerRunning;

    // Global pixel offsets (applied to physical coords)
    int16_t _gapX;
    int16_t _gapY;

    // 8x8 dither matrix
    static constexpr uint8_t dither8[8][8] = {
        {  0, 48, 12, 60,  3, 51, 15, 63},
        { 32, 16, 44, 28, 35, 19, 47, 31},
        {  8, 56,  4, 52, 11, 59,  7, 55},
        { 40, 24, 36, 20, 43, 27, 39, 23},
        {  2, 50, 14, 62,  1, 49, 13, 61},
        { 34, 18, 46, 30, 33, 17, 45, 29},
        { 10, 58,  6, 54,  9, 57,  5, 53},
        { 42, 26, 38, 22, 41, 25, 37, 21}
    };

    // LVGL flush callback
    static void lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
    // Enhanced dithered grayscale conversion
    static bool dither8x8ToMono(lv_color_t pixel, int x, int y);

    // Worker task entry
    static void displayWorkerTask(void* arg);

    // Helper to create/destroy queue/task
    bool createWorker();
    void destroyWorker();
};

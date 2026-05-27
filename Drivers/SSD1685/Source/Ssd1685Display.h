#pragma once

/*
 * Ssd1685Display.h
 *
 * Tactility HAL driver header for SSD1685 / SSD168x e-paper panels.
 */

#include <Tactility/hal/display/DisplayDevice.h>
#include <Tactility/hal/touch/TouchDevice.h>

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_ssd1685.h>
#include <driver/spi_common.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <lvgl.h>

#include <atomic>
#include <memory>
#include <string>

class Ssd1685Display final : public tt::hal::display::DisplayDevice {

public:

    struct Configuration {

        Configuration(
            spi_host_device_t spiHost,
            gpio_num_t        csPin,
            gpio_num_t        dcPin,
            gpio_num_t        resetPin,
            gpio_num_t        busyPin,
            uint16_t          width,
            uint16_t          height,
            uint8_t           rotation = 0,
            std::shared_ptr<tt::hal::touch::TouchDevice> touch = nullptr
        ) :
            spiHost(spiHost), csPin(csPin), dcPin(dcPin),
            resetPin(resetPin), busyPin(busyPin),
            width(width), height(height),
            rotation(rotation), touch(std::move(touch))
        {}

        spi_host_device_t spiHost;
        gpio_num_t csPin, dcPin, resetPin, busyPin;
        uint16_t   width, height;

        /** 0=portrait  1=landscape CW  2=portrait180  3=landscape CCW */
        uint8_t rotation = 0;

        /** Source-line offset; many SSD168x panels need gapX=8 */
        int gapX = 0, gapY = 0;

        /** SPI clock Hz; 4 MHz is safe for all SSD168x panels */
        uint32_t spiClockHz = 4'000'000;

        /** BUSY-pin timeout ms; full refresh can take ~5 s */
        uint32_t busyTimeoutMs = 10'000;

        /**
         * Refresh mode:
         *   SSD1685_REFRESH_FULL    – ghost-free, ~3 s
         *   SSD1685_REFRESH_PARTIAL – fast, ~0.3 s, may ghost
         *   SSD1685_REFRESH_FAST    – fastest, most ghosting
         */
        ssd1685_refresh_mode_t refreshMode = SSD1685_REFRESH_FULL;

        /** Optional custom LUT; nullptr = panel OTP waveform */
        const uint8_t* customLut     = nullptr;
        size_t         customLutSize = 0;

        /** Priority of the "epd_refresh" FreeRTOS task. Default: 5. */
        UBaseType_t refreshTaskPriority = 5;

        std::shared_ptr<tt::hal::touch::TouchDevice> touch;
    };

private:

    std::unique_ptr<Configuration>  config;

    esp_lcd_panel_io_handle_t       ioHandle    = nullptr;
    esp_lcd_panel_handle_t          panelHandle = nullptr;
    lv_display_t*                   lvglDisplay = nullptr;

    uint8_t*  drawBuf1    = nullptr;   // LVGL render buffer A
    uint8_t*  drawBuf2    = nullptr;   // LVGL render buffer B
    uint8_t*  pendingBuf  = nullptr;   // refresh task's working copy
    size_t    drawBufSize = 0;

    bool started = false;

    // Semaphore pair for flush_cb ↔ refresh task ↔ wait_cb handshake:
    //   semReady: flush_cb gives → refresh task takes  (frame ready to send)
    //   semDone:  refresh task gives → wait_cb takes   (refresh complete)
    SemaphoreHandle_t  semReady = nullptr;
    SemaphoreHandle_t  semDone  = nullptr;
    TaskHandle_t       refreshTask = nullptr;
    std::atomic<bool>  stopRefreshTask { false };

    uint16_t  lvglWidth()  const;
    uint16_t  lvglHeight() const;
    esp_err_t applyRotation();

    static void refreshTaskFunc(void* arg);

    // flush_cb: called by lv_timer_handler. Must not block.
    static void flushCallback(lv_display_t* disp,
                               const lv_area_t* area,
                               uint8_t* pixelMap);

    // wait_cb: called by LVGL after flush_cb returns, outside the render lock.
    // This is where we block until the refresh task completes.
    static void flushWaitCallback(lv_display_t* disp);

public:

    explicit Ssd1685Display(std::unique_ptr<Configuration> cfg);
    ~Ssd1685Display() override;

    std::string getName()        const override { return "SSD1685"; }
    std::string getDescription() const override { return "SSD168x e-paper display driver"; }

    bool start()  override;
    bool stop()   override;

    bool supportsPowerControl()  const override { return false; }
    bool isPoweredOn()           const override { return started; }
    void setPowerOn(bool)              override {}

    std::shared_ptr<tt::hal::touch::TouchDevice> getTouchDevice() override {
        return config->touch;
    }

    bool          supportsLvgl()   const override { return true; }
    bool          startLvgl()            override;
    bool          stopLvgl()             override;
    lv_display_t* getLvglDisplay() const override { return lvglDisplay; }

    bool supportsDisplayDriver()   const override { return false; }
    std::shared_ptr<tt::hal::display::DisplayDriver> getDisplayDriver() override {
        return nullptr;
    }

    esp_err_t refresh(ssd1685_refresh_mode_t mode = SSD1685_REFRESH_FULL);
    esp_err_t clearScreen(uint8_t colorByte = 0xFF);
    esp_err_t sleep();
    esp_lcd_panel_handle_t getPanelHandle() const { return panelHandle; }
};
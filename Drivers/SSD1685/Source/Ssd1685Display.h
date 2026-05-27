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

#include <memory>
#include <string>
#include <atomic>

class Ssd1685Display final : public tt::hal::display::DisplayDevice {

public:

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    struct Configuration {

        Configuration(
            spi_host_device_t spiHost,
            gpio_num_t        csPin,
            gpio_num_t        dcPin,
            gpio_num_t        resetPin,
            gpio_num_t        busyPin,
            uint16_t          width,
            uint16_t          height,
            uint8_t           rotation  = 0,
            std::shared_ptr<tt::hal::touch::TouchDevice> touch = nullptr
        ) :
            spiHost(spiHost),
            csPin(csPin),
            dcPin(dcPin),
            resetPin(resetPin),
            busyPin(busyPin),
            width(width),
            height(height),
            rotation(rotation),
            touch(std::move(touch))
        {}

        spi_host_device_t spiHost;
        gpio_num_t        csPin;
        gpio_num_t        dcPin;
        gpio_num_t        resetPin;
        gpio_num_t        busyPin;
        uint16_t          width;
        uint16_t           height;

        /**
         * Display orientation:
         *   0 = portrait  (native)
         *   1 = landscape 90° CW
         *   2 = portrait 180°
         *   3 = landscape 90° CCW
         */
        uint8_t rotation = 0;

        /** Source-line offset compensation (many SSD168x panels need gapX=8) */
        int gapX = 0;
        int gapY = 0;

        /** SPI clock (Hz). 4 MHz is safe for all SSD168x panels. */
        uint32_t spiClockHz = 4'000'000;

        /** Max wait for BUSY to deassert. Full refresh can take ~5 s. */
        uint32_t busyTimeoutMs = 10'000;

        /**
         * Refresh mode used by the background refresh task.
         *   SSD1685_REFRESH_FULL    – ghost-free, ~3 s
         *   SSD1685_REFRESH_PARTIAL – fast, ~0.3 s, may ghost
         *   SSD1685_REFRESH_FAST    – fastest, most ghosting
         */
        ssd1685_refresh_mode_t refreshMode = SSD1685_REFRESH_FULL;

        /** Optional custom LUT. nullptr = use panel OTP waveform. */
        const uint8_t* customLut     = nullptr;
        size_t         customLutSize = 0;

        /** FreeRTOS priority for the background EPD refresh task.
         *  Must be high enough to run promptly but below configMAX_PRIORITIES-1
         *  so it doesn't starve the watchdog.  Default: 5. */
        UBaseType_t refreshTaskPriority = 5;

        std::shared_ptr<tt::hal::touch::TouchDevice> touch;
    };

private:

    std::unique_ptr<Configuration>  config;

    esp_lcd_panel_io_handle_t       ioHandle    = nullptr;
    esp_lcd_panel_handle_t          panelHandle = nullptr;
    lv_display_t*                   lvglDisplay = nullptr;

    uint8_t*  drawBuf1    = nullptr;
    uint8_t*  drawBuf2    = nullptr;
    size_t    drawBufSize = 0;

    bool started = false;

    // -----------------------------------------------------------------------
    // Background refresh task state
    // -----------------------------------------------------------------------

    /** Semaphore posted by flushCallback to trigger a refresh */
    SemaphoreHandle_t  refreshSem  = nullptr;
    /** Handle for the background "epd_refresh" task */
    TaskHandle_t       refreshTask = nullptr;
    /** Set true to ask the refresh task to exit */
    std::atomic<bool>  stopRefreshTask { false };

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    uint16_t lvglWidth()  const;
    uint16_t lvglHeight() const;
    esp_err_t applyRotation();

    /** Entry point for the background refresh FreeRTOS task */
    static void refreshTaskFunc(void* arg);

    /** LVGL flush callback – posts semaphore and returns immediately */
    static void flushCallback(lv_display_t* disp,
                               const lv_area_t* area,
                               uint8_t* pixelMap);

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

    // Extended EPD ops
    esp_err_t refresh(ssd1685_refresh_mode_t mode = SSD1685_REFRESH_FULL);
    esp_err_t clearScreen(uint8_t colorByte = 0xFF);
    esp_err_t sleep();
    esp_lcd_panel_handle_t getPanelHandle() const { return panelHandle; }
};

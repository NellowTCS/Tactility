#pragma once

#include <Tactility/hal/display/DisplayDevice.h>
#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <memory>
#include <lvgl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

extern "C" {
    #include "ssd1685.h"
}

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
    ssd1685_handle_t _ssd1685_handle;
    lv_display_t* _lvglDisplay;
    lv_color_t* _drawBuf;
    lv_color_t* _previousBuf;
    uint8_t* _monoBuffer;
    SemaphoreHandle_t _spiMutex;
    SemaphoreHandle_t _updateSemaphore;
    SemaphoreHandle_t _updateCompleteSemaphore;
    TaskHandle_t _displayTaskHandle;
    bool _initialized;
    bool _shouldStop;
    bool _fullRefreshNeeded;
    volatile bool _hasNewFrame;

    static void lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map);
    static void displayUpdateTask(void* pvParameter);
    void performUpdate();
};

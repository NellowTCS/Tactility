#include "Ssd1685Display.h"
#include "ssd1685.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <Tactility/Log.h>
#include <driver/gpio.h>

static const char* TAG = "Ssd1685Display";

Ssd1685Display::Ssd1685Display(const Configuration& config)
    : _config(config)
    , _lvglDisplay(nullptr)
    , _spiMutex(nullptr)
    , _initialized(false)
{
}

Ssd1685Display::~Ssd1685Display() {
    stopLvgl();
    stop();
    if (_spiMutex) {
        vSemaphoreDelete(_spiMutex);
        _spiMutex = nullptr;
    }
}

std::string Ssd1685Display::getName() const {
    return "SSD1685";
}

std::string Ssd1685Display::getDescription() const {
    return "E-paper display GDEY029T71H with SSD1685 controller";
}

bool Ssd1685Display::start() {
    ESP_LOGI(TAG, "Starting e-paper display...");

    _spiMutex = xSemaphoreCreateMutex();
    if (!_spiMutex) {
        ESP_LOGE(TAG, "Failed to create SPI mutex");
        return false;
    }

    if (_config.rstPin >= 0) {
        gpio_set_direction(_config.rstPin, GPIO_MODE_OUTPUT);
        gpio_set_level(_config.rstPin, 1);
    }

    if (_config.dcPin >= 0) {
        gpio_set_direction(_config.dcPin, GPIO_MODE_OUTPUT);
        gpio_set_level(_config.dcPin, 1);
    }

    if (_config.busyPin >= 0) {
        gpio_set_direction(_config.busyPin, GPIO_MODE_INPUT);
    }

    _initialized = true;
    ESP_LOGI(TAG, "E-paper display GPIO initialized");
    return true;
}

bool Ssd1685Display::stop() {
    if (_config.rstPin >= 0) {
        gpio_set_direction(_config.rstPin, GPIO_MODE_INPUT);
    }
    if (_config.dcPin >= 0) {
        gpio_set_direction(_config.dcPin, GPIO_MODE_INPUT);
    }
    if (_config.busyPin >= 0) {
        gpio_set_direction(_config.busyPin, GPIO_MODE_INPUT);
    }
    _initialized = false;
    return true;
}

std::shared_ptr<tt::hal::touch::TouchDevice> Ssd1685Display::getTouchDevice() {
    return nullptr;
}

bool Ssd1685Display::supportsLvgl() const {
    return true;
}

bool Ssd1685Display::startLvgl() {
    if (_lvglDisplay != nullptr) {
        ESP_LOGW(TAG, "LVGL already started");
        return false;
    }

    if (!_initialized) {
        ESP_LOGE(TAG, "Display not started, cannot start LVGL");
        return false;
    }

    ESP_LOGI(TAG, "Starting LVGL: %ux%u", _config.width, _config.height);

    _lvglDisplay = lv_display_create(_config.width, _config.height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        return false;
    }

    lv_display_set_rotation(_lvglDisplay, LV_DISPLAY_ROTATION_0);

    const size_t bufSize = (size_t)_config.width * (size_t)_config.height;
    lv_color_t* drawBuf = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!drawBuf) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffer (%zu bytes)", bufSize * sizeof(lv_color_t));
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "Allocated %zu bytes for full-screen buffer (%u×%u = %zu pixels)",
             bufSize * sizeof(lv_color_t), _config.width, _config.height, bufSize);

    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(_lvglDisplay, drawBuf, nullptr,
                           bufSize * sizeof(lv_color_t),
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    ssd1685_init();

    ESP_LOGI(TAG, "LVGL started successfully");
    return true;
}

bool Ssd1685Display::stopLvgl() {
    if (_lvglDisplay) {
        ESP_LOGI(TAG, "Stopping LVGL...");
        lv_color_t* buf = (lv_color_t*)lv_display_get_buf_1(_lvglDisplay);
        if (buf) {
            heap_caps_free(buf);
        }
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
    }
    return true;
}

lv_display_t* Ssd1685Display::getLvglDisplay() const {
    return _lvglDisplay;
}

bool Ssd1685Display::supportsDisplayDriver() const {
    return false;
}

std::shared_ptr<tt::hal::display::DisplayDriver> Ssd1685Display::getDisplayDriver() {
    return nullptr;
}

uint16_t Ssd1685Display::getWidth() const {
    return _config.width;
}

uint16_t Ssd1685Display::getHeight() const {
    return _config.height;
}

void Ssd1685Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<Ssd1685Display*>(lv_display_get_user_data(disp));
    if (!self) {
        lv_display_flush_ready(disp);
        return;
    }

    if (self->_spiMutex) {
        xSemaphoreTake(self->_spiMutex, portMAX_DELAY);
    }

    ssd1685_flush(nullptr, area, (lv_color_t*)px_map);

    if (self->_spiMutex) {
        xSemaphoreGive(self->_spiMutex);
    }

    lv_display_flush_ready(disp);
}

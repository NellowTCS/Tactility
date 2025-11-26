#include "Ssd1685Display.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <Tactility/Log.h>
#include <cstring>

extern "C" {
    #include "epd_w21_wrapper.h"
}

static const char* TAG = "EpdDisplay";

Ssd1685Display::Ssd1685Display(const Configuration& config)
    : _config(config)
    , _epd_handle({
        .spi_handle = config.spiHandle,
        .dc_pin = config.dcPin,
        .rst_pin = config.rstPin,
        .busy_pin = config.busyPin,
        .cs_pin = GPIO_NUM_6  // CS pin for e-paper
    })
    , _lvglDisplay(nullptr)
    , _drawBuf(nullptr)
    , _monoBuffer(nullptr)
    , _initialized(false)
    , _refreshCount(0)
{
}

Ssd1685Display::~Ssd1685Display() {
    stopLvgl();
    stop();
}

std::string Ssd1685Display::getName() const {
    return "EPD W21";
}

std::string Ssd1685Display::getDescription() const {
    return "E-paper display with working Arduino-based driver";
}

bool Ssd1685Display::start() {
    ESP_LOGI(TAG, "Starting e-paper display...");
    
    epd_w21_init_io(&_epd_handle);
    _initialized = true;
    
    ESP_LOGI(TAG, "E-paper display started");
    return true;
}

bool Ssd1685Display::stop() {
    if (_initialized) {
        epd_w21_deinit_io(&_epd_handle);
        _initialized = false;
    }
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
        ESP_LOGE(TAG, "Display not started");
        return false;
    }

    ESP_LOGI(TAG, "Starting LVGL: %ux%u", _config.width, _config.height);

    _lvglDisplay = lv_display_create(_config.width, _config.height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        return false;
    }

    lv_display_set_rotation(_lvglDisplay, LV_DISPLAY_ROTATION_0);

    const size_t pixelCount = (size_t)_config.width * (size_t)_config.height;
    const size_t bufSize = pixelCount * sizeof(lv_color_t);
    const size_t monoSize = pixelCount / 8;
    
    // Allocate frame buffer in PSRAM
    _drawBuf = (lv_color_t*)heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
    if (!_drawBuf) {
        ESP_LOGE(TAG, "Failed to allocate draw buffer");
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    // Allocate monochrome buffer in DMA memory
    _monoBuffer = (uint8_t*)heap_caps_malloc(monoSize, MALLOC_CAP_DMA);
    if (!_monoBuffer) {
        ESP_LOGE(TAG, "Failed to allocate mono buffer");
        heap_caps_free(_drawBuf);
        _drawBuf = nullptr;
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "Allocated buffers: RGB=%zu, mono=%zu", bufSize, monoSize);

    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(_lvglDisplay, _drawBuf, nullptr, bufSize, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    // Initialize display hardware
    epd_w21_init(&_epd_handle);
    epd_w21_clear(&_epd_handle);
    
    _refreshCount = 0;

    ESP_LOGI(TAG, "LVGL started successfully");
    return true;
}

bool Ssd1685Display::stopLvgl() {
    if (_lvglDisplay) {
        ESP_LOGI(TAG, "Stopping LVGL...");
        
        if (_monoBuffer) {
            heap_caps_free(_monoBuffer);
            _monoBuffer = nullptr;
        }
        
        if (_drawBuf) {
            heap_caps_free(_drawBuf);
            _drawBuf = nullptr;
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

    ESP_LOGI(TAG, "Flush callback - refresh #%d", self->_refreshCount);

    // Convert RGB565 to monochrome
    const size_t pixelCount = (size_t)self->_config.width * (size_t)self->_config.height;
    const lv_color_t* src = (const lv_color_t*)px_map;
    
    memset(self->_monoBuffer, 0, pixelCount / 8);
    
    for (size_t i = 0; i < pixelCount; i++) {
        // Calculate luminance
        uint8_t r = (src[i].red * 255) / 31;
        uint8_t g = (src[i].green * 255) / 63;
        uint8_t b = (src[i].blue * 255) / 31;
        uint16_t luma = (r * 299 + g * 587 + b * 114) / 1000;
        
        // White = 1, Black = 0
        if (luma > 127) {
            size_t byteIdx = i / 8;
            uint8_t bitIdx = 7 - (i % 8);
            self->_monoBuffer[byteIdx] |= (1 << bitIdx);
        }
    }

    // Use partial refresh after first full refresh, but do full every 5 updates
    bool usePartial = (self->_refreshCount > 0) && (self->_refreshCount % 5 != 0);
    
    if (usePartial) {
        epd_w21_display_partial(&self->_epd_handle, self->_monoBuffer);
    } else {
        epd_w21_display_full(&self->_epd_handle, self->_monoBuffer);
    }
    
    // Sleep to save power
    epd_w21_sleep(&self->_epd_handle);
    
    self->_refreshCount++;

    lv_display_flush_ready(disp);
}
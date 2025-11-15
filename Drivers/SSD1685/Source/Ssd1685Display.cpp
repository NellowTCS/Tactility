#include "Ssd1685Display.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <Tactility/Log.h>
#include <cstring>

static const char* TAG = "Ssd1685Display";

Ssd1685Display::Ssd1685Display(const Configuration& config)
    : _config(config)
    , _ssd1685_handle({
        .spi_handle = config.spiHandle,
        .dc_pin = config.dcPin,
        .rst_pin = config.rstPin,
        .busy_pin = config.busyPin,
        .last_update_mode = SSD1685_UPDATE_MODE_INIT,
        .partial_mode_active = false,
        .partial_refresh_count = 0
    })
    , _lvglDisplay(nullptr)
    , _drawBuf(nullptr)
    , _previousBuf(nullptr)
    , _monoBuffer(nullptr)
    , _spiMutex(nullptr)
    , _updateSemaphore(nullptr)
    , _updateCompleteSemaphore(nullptr)
    , _displayTaskHandle(nullptr)
    , _initialized(false)
    , _shouldStop(false)
    , _fullRefreshNeeded(true)
    , _hasNewFrame(false)
{
}

Ssd1685Display::~Ssd1685Display() {
    stopLvgl();
    stop();
    if (_spiMutex) {
        vSemaphoreDelete(_spiMutex);
        _spiMutex = nullptr;
    }
    if (_updateSemaphore) {
        vSemaphoreDelete(_updateSemaphore);
        _updateSemaphore = nullptr;
    }
    if (_updateCompleteSemaphore) {
        vSemaphoreDelete(_updateCompleteSemaphore);
        _updateCompleteSemaphore = nullptr;
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

    _updateSemaphore = xSemaphoreCreateBinary();
    if (!_updateSemaphore) {
        ESP_LOGE(TAG, "Failed to create update semaphore");
        return false;
    }

    _updateCompleteSemaphore = xSemaphoreCreateBinary();
    if (!_updateCompleteSemaphore) {
        ESP_LOGE(TAG, "Failed to create update complete semaphore");
        return false;
    }

    ssd1685_init_io(&_ssd1685_handle);
    _initialized = true;

    ESP_LOGI(TAG, "E-paper display started");
    return true;
}

bool Ssd1685Display::stop() {
    if (_initialized) {
        _shouldStop = true;
        if (_displayTaskHandle) {
            vTaskDelete(_displayTaskHandle);
            _displayTaskHandle = nullptr;
        }
        ssd1685_deinit_io(&_ssd1685_handle);
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

    const size_t pixelCount = (size_t)_config.width * (size_t)_config.height;
    const size_t bufSize = pixelCount * sizeof(lv_color_t);
    const size_t monoSize = pixelCount / 8;
    
    // Allocate current frame buffer
    _drawBuf = (lv_color_t*)heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
    if (!_drawBuf) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffer (%zu bytes)", bufSize);
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    // Allocate previous frame buffer for differential updates
    _previousBuf = (lv_color_t*)heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
    if (!_previousBuf) {
        ESP_LOGE(TAG, "Failed to allocate previous frame buffer (%zu bytes)", bufSize);
        heap_caps_free(_drawBuf);
        _drawBuf = nullptr;
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    // Allocate monochrome buffer for e-paper
    _monoBuffer = (uint8_t*)heap_caps_malloc(monoSize, MALLOC_CAP_DMA);
    if (!_monoBuffer) {
        ESP_LOGE(TAG, "Failed to allocate mono buffer (%zu bytes)", monoSize);
        heap_caps_free(_previousBuf);
        _previousBuf = nullptr;
        heap_caps_free(_drawBuf);
        _drawBuf = nullptr;
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    // Initialize previous buffer to white
    memset(_previousBuf, 0xFF, bufSize);

    ESP_LOGI(TAG, "Allocated buffers: draw=%zu, prev=%zu, mono=%zu", bufSize, bufSize, monoSize);

    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(_lvglDisplay, _drawBuf, nullptr, bufSize, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    // Create display update task
    _shouldStop = false;
    BaseType_t ret = xTaskCreatePinnedToCore(
        displayUpdateTask,
        "EPDUpdate",
        8192,
        this,
        5,  // Lower priority than LVGL
        &_displayTaskHandle,
        1   // Pin to core 1 (LVGL is on core 0)
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display update task");
        heap_caps_free(_monoBuffer);
        _monoBuffer = nullptr;
        heap_caps_free(_previousBuf);
        _previousBuf = nullptr;
        heap_caps_free(_drawBuf);
        _drawBuf = nullptr;
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    // Wait for task to start
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Do initial full refresh
    ssd1685_init(&_ssd1685_handle);
    _fullRefreshNeeded = true;

    ESP_LOGI(TAG, "LVGL started successfully");
    return true;
}

bool Ssd1685Display::stopLvgl() {
    if (_lvglDisplay) {
        ESP_LOGI(TAG, "Stopping LVGL...");
        
        _shouldStop = true;
        
        if (_displayTaskHandle) {
            // Signal the task to wake up and exit
            xSemaphoreGive(_updateSemaphore);
            vTaskDelay(pdMS_TO_TICKS(100));
            vTaskDelete(_displayTaskHandle);
            _displayTaskHandle = nullptr;
        }
        
        if (_monoBuffer) {
            heap_caps_free(_monoBuffer);
            _monoBuffer = nullptr;
        }
        
        if (_previousBuf) {
            heap_caps_free(_previousBuf);
            _previousBuf = nullptr;
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

    // Skip if already processing a frame
    if (self->_hasNewFrame) {
        ESP_LOGD(TAG, "Skipping frame - update already pending");
        lv_display_flush_ready(disp);
        return;
    }

    // Mark that we have a new frame
    self->_hasNewFrame = true;

    // Signal the update task (non-blocking)
    xSemaphoreGive(self->_updateSemaphore);

    // Wait for update to complete (with timeout to prevent deadlock)
    if (xSemaphoreTake(self->_updateCompleteSemaphore, pdMS_TO_TICKS(10000)) != pdPASS) {
        ESP_LOGW(TAG, "Display update timeout!");
    }

    self->_hasNewFrame = false;
    lv_display_flush_ready(disp);
}

void Ssd1685Display::displayUpdateTask(void* pvParameter) {
    auto* self = static_cast<Ssd1685Display*>(pvParameter);
    
    ESP_LOGI(TAG, "Display update task started");

    while (!self->_shouldStop) {
        // Wait for a frame to process
        if (xSemaphoreTake(self->_updateSemaphore, pdMS_TO_TICKS(1000)) == pdPASS) {
            if (self->_shouldStop) break;

            if (xSemaphoreTake(self->_spiMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
                self->performUpdate();
                xSemaphoreGive(self->_spiMutex);
            } else {
                ESP_LOGE(TAG, "Failed to acquire SPI mutex");
            }

            // Signal completion
            xSemaphoreGive(self->_updateCompleteSemaphore);
        }
    }

    ESP_LOGI(TAG, "Display update task stopped");
    vTaskDelete(NULL);
}

void Ssd1685Display::performUpdate() {
    const size_t pixelCount = (size_t)_config.width * (size_t)_config.height;
    
    // Convert RGB565 to monochrome
    const lv_color_t* src = _drawBuf;
    memset(_monoBuffer, 0, pixelCount / 8);
    
    for (size_t i = 0; i < pixelCount; i++) {
        // Calculate luminance from RGB565
        uint8_t r = (src[i].red * 255) / 31;
        uint8_t g = (src[i].green * 255) / 63;
        uint8_t b = (src[i].blue * 255) / 31;
        uint16_t luma = (r * 299 + g * 587 + b * 114) / 1000;
        
        // White pixels = 1, Black pixels = 0
        if (luma > 127) {
            size_t byteIdx = i / 8;
            uint8_t bitIdx = 7 - (i % 8);
            _monoBuffer[byteIdx] |= (1 << bitIdx);
        }
    }

    // Decide refresh mode
    bool usePartial = !_fullRefreshNeeded;
    
    if (usePartial && _previousBuf) {
        size_t changedPixels = 0;
        
        // Sample every 10th pixel
        for (size_t i = 0; i < pixelCount; i += 10) {
            if (memcmp(&src[i], &_previousBuf[i], sizeof(lv_color_t)) != 0) {
                changedPixels++;
                if (changedPixels > 100) break;
            }
        }
        
        changedPixels *= 10;
        
        // If >20% changed, force full refresh
        if (changedPixels > pixelCount / 5) {
            ESP_LOGI(TAG, "Large change (~%zu px), forcing full refresh", changedPixels);
            usePartial = false;
        }
    }

    // Perform update
    if (usePartial) {
        ESP_LOGI(TAG, "Performing partial refresh");
        ssd1685_flush_partial(&_ssd1685_handle, _monoBuffer);
    } else {
        ESP_LOGI(TAG, "Performing full refresh");
        ssd1685_flush_full(&_ssd1685_handle, _monoBuffer);
        _fullRefreshNeeded = false;
    }
    
    // Update previous buffer
    if (_previousBuf) {
        memcpy(_previousBuf, _drawBuf, pixelCount * sizeof(lv_color_t));
    }
    
    // Sleep mode
    ssd1685_deep_sleep(&_ssd1685_handle);
}

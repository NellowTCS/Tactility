#include "Ssd1685Display.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
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
    , _spiMutex(nullptr)
    , _flushQueue(nullptr)
    , _displayTaskHandle(nullptr)
    , _initialized(false)
    , _shouldStop(false)
    , _fullRefreshNeeded(true)
{
}

Ssd1685Display::~Ssd1685Display() {
    stopLvgl();
    stop();
    if (_spiMutex) {
        vSemaphoreDelete(_spiMutex);
        _spiMutex = nullptr;
    }
    if (_flushQueue) {
        vQueueDelete(_flushQueue);
        _flushQueue = nullptr;
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

    const size_t bufSize = (size_t)_config.width * (size_t)_config.height;
    
    // Allocate current frame buffer
    _drawBuf = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!_drawBuf) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffer (%zu bytes)", bufSize * sizeof(lv_color_t));
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    // Allocate previous frame buffer for differential updates
    _previousBuf = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!_previousBuf) {
        ESP_LOGE(TAG, "Failed to allocate previous frame buffer (%zu bytes)", bufSize * sizeof(lv_color_t));
        heap_caps_free(_drawBuf);
        _drawBuf = nullptr;
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    // Initialize previous buffer to white
    memset(_previousBuf, 0xFF, bufSize * sizeof(lv_color_t));

    ESP_LOGI(TAG, "Allocated %zu bytes each for current and previous buffers", bufSize * sizeof(lv_color_t));

    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(_lvglDisplay, _drawBuf, nullptr,
                           bufSize * sizeof(lv_color_t),
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    _flushQueue = xQueueCreate(1, sizeof(FlushRequest));
    if (!_flushQueue) {
        ESP_LOGE(TAG, "Failed to create flush queue");
        heap_caps_free(_previousBuf);
        _previousBuf = nullptr;
        heap_caps_free(_drawBuf);
        _drawBuf = nullptr;
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    _shouldStop = false;
    BaseType_t ret = xTaskCreate(
        displayUpdateTask,
        "DisplayUpdate",
        8192,  // Increased stack size
        this,
        configMAX_PRIORITIES - 2,
        &_displayTaskHandle
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display update task");
        vQueueDelete(_flushQueue);
        _flushQueue = nullptr;
        heap_caps_free(_previousBuf);
        _previousBuf = nullptr;
        heap_caps_free(_drawBuf);
        _drawBuf = nullptr;
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    // Wait for task to be ready
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
            vTaskDelete(_displayTaskHandle);
            _displayTaskHandle = nullptr;
        }
        
        if (_flushQueue) {
            vQueueDelete(_flushQueue);
            _flushQueue = nullptr;
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
    if (!self || !self->_flushQueue) {
        lv_display_flush_ready(disp);
        return;
    }

    FlushRequest req;
    req.area = *area;
    req.px_map = px_map;
    req.forceFullRefresh = self->_fullRefreshNeeded;

    if (xQueueSend(self->_flushQueue, &req, pdMS_TO_TICKS(100)) != pdPASS) {
        ESP_LOGW(TAG, "Flush queue full, dropping frame");
    }

    lv_display_flush_ready(disp);
}

void Ssd1685Display::displayUpdateTask(void* pvParameter) {
    auto* self = static_cast<Ssd1685Display*>(pvParameter);
    FlushRequest req;
    
    const size_t pixelCount = (size_t)self->_config.width * (size_t)self->_config.height;
    const size_t bufferSize = pixelCount / 8;  // 1 bit per pixel
    
    // Allocate monochrome buffer
    uint8_t* monoBuffer = (uint8_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_DMA);
    if (!monoBuffer) {
        ESP_LOGE(TAG, "Failed to allocate mono buffer");
        vTaskDelete(NULL);
        return;
    }

    while (!self->_shouldStop) {
        if (xQueueReceive(self->_flushQueue, &req, pdMS_TO_TICKS(100)) == pdPASS) {
            if (xSemaphoreTake(self->_spiMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
                
                // Convert RGB565 to monochrome (1 bit per pixel)
                const lv_color_t* src = (const lv_color_t*)req.px_map;
                memset(monoBuffer, 0, bufferSize);
                
                for (size_t i = 0; i < pixelCount; i++) {
                    // Calculate luminance from RGB565
                    uint8_t r = (src[i].red * 255) / 31;
                    uint8_t g = (src[i].green * 255) / 63;
                    uint8_t b = (src[i].blue * 255) / 31;
                    uint16_t luma = (r * 299 + g * 587 + b * 114) / 1000;
                    
                    // White pixels = 1, Black pixels = 0 (inverted for e-paper)
                    if (luma > 127) {
                        size_t byteIdx = i / 8;
                        uint8_t bitIdx = 7 - (i % 8);
                        monoBuffer[byteIdx] |= (1 << bitIdx);
                    }
                }

                // Decide if we need full or partial refresh
                bool usePartial = !req.forceFullRefresh;
                
                // Check if changes are significant enough for partial update
                if (usePartial && self->_previousBuf) {
                    size_t changedPixels = 0;
                    const lv_color_t* current = (const lv_color_t*)req.px_map;
                    
                    for (size_t i = 0; i < pixelCount && changedPixels < 1000; i++) {
                        if (memcmp(&current[i], &self->_previousBuf[i], sizeof(lv_color_t)) != 0) {
                            changedPixels++;
                        }
                    }
                    
                    // If more than 20% changed, do full refresh
                    if (changedPixels > pixelCount / 5) {
                        ESP_LOGI(TAG, "Large change detected (%zu pixels), forcing full refresh", changedPixels);
                        usePartial = false;
                    }
                }

                // Perform the update
                if (usePartial) {
                    ssd1685_flush_partial(&self->_ssd1685_handle, monoBuffer);
                } else {
                    ssd1685_flush_full(&self->_ssd1685_handle, monoBuffer);
                    self->_fullRefreshNeeded = false;
                }
                
                esp_task_wdt_reset();
                
                // Update previous buffer
                if (self->_previousBuf) {
                    memcpy(self->_previousBuf, req.px_map, pixelCount * sizeof(lv_color_t));
                }
                
                // Enter deep sleep to save power
                ssd1685_deep_sleep(&self->_ssd1685_handle);
                esp_task_wdt_reset();

                xSemaphoreGive(self->_spiMutex);
            } else {
                ESP_LOGE(TAG, "Failed to acquire SPI mutex for display update");
            }
        }
        esp_task_wdt_reset();
    }

    heap_caps_free(monoBuffer);
    vTaskDelete(NULL);
}

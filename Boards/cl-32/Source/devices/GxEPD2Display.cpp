#include "GxEPD2Display.h"
#include "GxEPD2/GxEPD2_290_GDEY029T71H.h"
#include "DisplayTester.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <cstring>
#include <cstdio>
#include <algorithm>

static const char* TAG = "GxEPD2Display";

GxEPD2Display::GxEPD2Display(const Configuration& config)
    : _config(config)
    , _epd2_native(nullptr)
    , _epd2_bw(nullptr)
    , _lvglDisplay(nullptr)
    , _drawBuf1(nullptr)
    , _drawBuf2(nullptr)
    , _frameBuffer(nullptr)
    , _framebufferMutex(nullptr)
    , _queue(nullptr)
    , _workerTaskHandle(nullptr)
    , _spiMutex(nullptr)
    , _workerRunning(false)
    , _gapX(0)
    , _gapY(0)
{}

GxEPD2Display::~GxEPD2Display() {
    stopLvgl();
    stop();
    destroyWorker();
    if (_spiMutex) {
        vSemaphoreDelete(_spiMutex);
        _spiMutex = nullptr;
    }
    if (_framebufferMutex) {
        vSemaphoreDelete(_framebufferMutex);
        _framebufferMutex = nullptr;
    }
}

std::string GxEPD2Display::getName() const {
    return "GxEPD2";
}

std::string GxEPD2Display::getDescription() const {
    return "E-paper display GDEY029T71H with GxEPD2_BW";
}

bool GxEPD2Display::start() {
    ESP_LOGI(TAG, "Starting e-paper display...");

    _spiMutex = xSemaphoreCreateMutex();
    if (!_spiMutex) {
        ESP_LOGE(TAG, "Failed to create SPI mutex");
        return false;
    }

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.mode = 0;
    dev_cfg.clock_speed_hz = 10000000;
    dev_cfg.spics_io_num = _config.csPin;
    dev_cfg.queue_size = 7;
    dev_cfg.pre_cb = nullptr;
    dev_cfg.post_cb = nullptr;

    spi_device_handle_t spi_device;
    esp_err_t ret = spi_bus_add_device(_config.spiHost, &dev_cfg, &spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        vSemaphoreDelete(_spiMutex);
        _spiMutex = nullptr;
        return false;
    }

    _epd2_native = std::make_unique<GxEPD2_290_GDEY029T71H>(
        _config.csPin, _config.dcPin, _config.rstPin, _config.busyPin);
    _epd2_native->selectSPI(_config.spiHost, dev_cfg);
    _epd2_native->init(115200);

    _epd2_bw = std::make_unique<GxEPD2_BW<GxEPD2_290_GDEY029T71H, GxEPD2_290_GDEY029T71H::HEIGHT>>(*_epd2_native);
    _epd2_bw->init(115200);
    _epd2_bw->setRotation(_config.rotation);

    ESP_LOGI(TAG, "E-paper display started successfully");
    return true;
}

bool GxEPD2Display::stop() {
    if (_epd2_bw) {
        _epd2_bw->powerOff();
        _epd2_bw.reset();
    }
    if (_epd2_native) {
        _epd2_native->hibernate();
        _epd2_native.reset();
    }
    return true;
}

std::shared_ptr<tt::hal::touch::TouchDevice> GxEPD2Display::getTouchDevice() {
    return nullptr;
}

bool GxEPD2Display::supportsLvgl() const {
    return true;
}

bool GxEPD2Display::createWorker() {
    if (_queue) return true;
    _queue = xQueueCreate(QUEUE_LENGTH, sizeof(QueueItem));
    if (!_queue) {
        ESP_LOGE(TAG, "Failed to create display queue");
        return false;
    }
    _workerRunning = true;
    BaseType_t r = xTaskCreate(
        displayWorkerTask,
        "epd_worker",
        8192,
        this,
        tskIDLE_PRIORITY + 2,
        &_workerTaskHandle
    );
    if (r != pdPASS) {
        ESP_LOGE(TAG, "Failed to create worker task");
        vQueueDelete(_queue);
        _queue = nullptr;
        _workerTaskHandle = nullptr;
        _workerRunning = false;
        return false;
    }
    ESP_LOGI(TAG, "Display worker created");
    return true;
}

void GxEPD2Display::destroyWorker() {
    if (!_queue) return;
    QueueItem term;
    term.buf = nullptr;
    term.x = term.y = term.w = term.h = 0;
    if (xQueueSend(_queue, &term, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to send termination message to display worker");
    }
    if (_workerTaskHandle) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (eTaskGetState(_workerTaskHandle) != eDeleted) {
            vTaskDelete(_workerTaskHandle);
        }
        _workerTaskHandle = nullptr;
    }
    QueueItem item;
    while (xQueueReceive(_queue, &item, 0) == pdTRUE) {
        if (item.buf) {
            heap_caps_free(item.buf);
        }
    }
    vQueueDelete(_queue);
    _queue = nullptr;
    _workerRunning = false;
    ESP_LOGI(TAG, "Display worker destroyed");
}

bool GxEPD2Display::startLvgl() {
    if (_lvglDisplay != nullptr) {
        ESP_LOGW(TAG, "LVGL already started");
        return false;
    }
    if (!_epd2_bw) {
        ESP_LOGE(TAG, "Display not started, cannot start LVGL");
        return false;
    }

    uint16_t lvgl_width = _config.width;
    uint16_t lvgl_height = _config.height;

    if (_config.rotation == 1 || _config.rotation == 3) {
        lvgl_width = _config.height;
        lvgl_height = _config.width;
    }

    ESP_LOGI(TAG, "Starting LVGL: physical=%ux%u rotation=%d -> LVGL dimensions=%ux%u",
             _config.width, _config.height, _config.rotation, lvgl_width, lvgl_height);

    _lvglDisplay = lv_display_create(lvgl_width, lvgl_height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        return false;
    }

    lv_display_set_rotation(_lvglDisplay, LV_DISPLAY_ROTATION_0);

    const size_t bufSize = (size_t)lvgl_width * (size_t)lvgl_height;
    _drawBuf1 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!_drawBuf1) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffer (%zu bytes)", bufSize * sizeof(lv_color_t));
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "Allocated %zu bytes for full-screen buffer (%d×%d = %zu pixels)", 
             bufSize * sizeof(lv_color_t), (int)lvgl_width, (int)lvgl_height, bufSize);

    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(_lvglDisplay, _drawBuf1, nullptr,
                           bufSize * sizeof(lv_color_t),
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    if (!createWorker()) {
        ESP_LOGW(TAG, "Failed to create display worker");
    }

    ESP_LOGI(TAG, "LVGL started successfully");

    return true;
}

bool GxEPD2Display::stopLvgl() {
    if (_lvglDisplay) {
        ESP_LOGI(TAG, "Stopping LVGL...");
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
    }
    if (_drawBuf1) {
        heap_caps_free(_drawBuf1);
        _drawBuf1 = nullptr;
    }
    if (_drawBuf2) {
        heap_caps_free(_drawBuf2);
        _drawBuf2 = nullptr;
    }
    if (_frameBuffer) {
        heap_caps_free(_frameBuffer);
        _frameBuffer = nullptr;
    }
    if (_framebufferMutex) {
        vSemaphoreDelete(_framebufferMutex);
        _framebufferMutex = nullptr;
    }
    destroyWorker();
    return true;
}

lv_display_t* GxEPD2Display::getLvglDisplay() const {
    return _lvglDisplay;
}

bool GxEPD2Display::supportsDisplayDriver() const {
    return false;
}

std::shared_ptr<tt::hal::display::DisplayDriver> GxEPD2Display::getDisplayDriver() {
    return nullptr;
}

uint16_t GxEPD2Display::getWidth() const {
    return _config.width;
}

uint16_t GxEPD2Display::getHeight() const {
    return _config.height;
}

void GxEPD2Display::writeRawImage(const uint8_t* bitmap, int16_t x, int16_t y, int16_t w, int16_t h,
                                   bool invert, bool mirror_y) {
    if (!_epd2_bw) return;
    bool took = false;
    if (_spiMutex) {
        if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(200)) == pdTRUE) took = true;
        else ESP_LOGW(TAG, "writeRawImage: failed to take spi mutex");
    }
    _epd2_bw->writeImage(bitmap, x, y, w, h, invert, mirror_y, false);
    if (took && _spiMutex) xSemaphoreGive(_spiMutex);
}

void GxEPD2Display::refreshDisplay(bool partial) {
    if (!_epd2_bw) return;
    bool took = false;
    if (_spiMutex) {
        if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(200)) == pdTRUE) took = true;
        else ESP_LOGW(TAG, "refreshDisplay: failed to take spi mutex");
    }
    _epd2_bw->refresh(partial);
    if (took && _spiMutex) xSemaphoreGive(_spiMutex);
}

static inline bool bayer4x4Dither(lv_color_t pixel, int x, int y) {
    uint8_t r = pixel.red;
    uint8_t g = pixel.green;
    uint8_t b = pixel.blue;
    uint8_t brightness = (r * 77 + g * 151 + b * 28) >> 8;

    static const uint8_t bayer4[4][4] = {
        { 0,  8,  2, 10},
        {12,  4, 14,  6},
        { 3, 11,  1,  9},
        {15,  7, 13,  5}
    };

    uint8_t thresh = (uint8_t)(bayer4[y & 3][x & 3] * 16 + 8);
    return brightness > thresh;
}

void GxEPD2Display::displayWorkerTask(void* arg) {
    GxEPD2Display* self = static_cast<GxEPD2Display*>(arg);
    if (!self) {
        vTaskDelete(NULL);
        return;
    }

    QueueItem item;
    QueueItem pending_item = {nullptr, 0, 0, 0, 0};
    bool has_pending = false;
    bool first_frame = true;

    while (true) {
        if (xQueueReceive(self->_queue, &item, pdMS_TO_TICKS(300))) {
            if (item.buf == nullptr) {
                ESP_LOGI(TAG, "Worker: termination received");
                break;
            }

            if (has_pending && pending_item.buf) {
                ESP_LOGI(TAG, "Worker: Discarding old frame (batching)");
                heap_caps_free(pending_item.buf);
            }

            pending_item = item;
            has_pending = true;

        } else if (has_pending && pending_item.buf) {
            if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);

            ESP_LOGI(TAG, "Worker: Drawing frame (first=%d)", first_frame);

            int32_t lvgl_w = pending_item.w;
            int32_t lvgl_h = pending_item.h;

            self->_epd2_bw->fillScreen(GxEPD_WHITE);
            
            lv_color_t* pixels = (lv_color_t*)pending_item.buf;
            for (int ly = 0; ly < lvgl_h; ly++) {
                for (int lx = 0; lx < lvgl_w; lx++) {
                    lv_color_t pixel = pixels[ly * lvgl_w + lx];
                    bool is_white = bayer4x4Dither(pixel, lx, ly);
                    self->_epd2_bw->drawPixel(lx, ly, is_white ? GxEPD_WHITE : GxEPD_BLACK);
                }
            }

            self->_epd2_bw->display(first_frame ? false : true);
            first_frame = false;

            if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);

            heap_caps_free(pending_item.buf);
            pending_item.buf = nullptr;
            has_pending = false;
        }
    }

    if (has_pending && pending_item.buf) {
        heap_caps_free(pending_item.buf);
    }

    self->_workerRunning = false;
    vTaskDelete(NULL);
}

void GxEPD2Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
    if (!self || !self->_epd2_bw) {
        lv_display_flush_ready(disp);
        return;
    }

    int32_t hor_res = lv_display_get_horizontal_resolution(disp);
    int32_t ver_res = lv_display_get_vertical_resolution(disp);

    ESP_LOGI(TAG, "Flush: %dx%d", hor_res, ver_res);

    const size_t bufSize = (size_t)hor_res * (size_t)ver_res * sizeof(lv_color_t);
    lv_color_t* buffer_copy = (lv_color_t*)heap_caps_malloc(bufSize, MALLOC_CAP_SPIRAM);
    if (!buffer_copy) {
        ESP_LOGE(TAG, "Failed to allocate buffer copy");
        lv_display_flush_ready(disp);
        return;
    }

    memcpy(buffer_copy, px_map, bufSize);

    if (self->_queue) {
        QueueItem qi;
        qi.buf = (uint8_t*)buffer_copy;
        qi.x = 0;
        qi.y = 0;
        qi.w = (uint16_t)hor_res;
        qi.h = (uint16_t)ver_res;

        if (xQueueSend(self->_queue, &qi, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Display queue full; frame dropped");
            heap_caps_free(buffer_copy);
        }
    } else {
        if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);

        self->_epd2_bw->fillScreen(GxEPD_WHITE);

        for (int ly = 0; ly < ver_res; ly++) {
            for (int lx = 0; lx < hor_res; lx++) {
                lv_color_t pixel = buffer_copy[ly * hor_res + lx];
                bool is_white = bayer4x4Dither(pixel, lx, ly);
                self->_epd2_bw->drawPixel(lx, ly, is_white ? GxEPD_WHITE : GxEPD_BLACK);
            }
        }

        self->_epd2_bw->display(true);

        if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);
        heap_caps_free(buffer_copy);
    }

    lv_display_flush_ready(disp);
}
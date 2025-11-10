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
    return "E-paper display GDEY029T71H with Adafruit GFX dithered grayscale";
}

bool GxEPD2Display::start() {
    ESP_LOGI(TAG, "Starting e-paper display...");

    _epd2_native = std::make_unique<GxEPD2_290_GDEY029T71H>(
        _config.csPin, _config.dcPin, _config.rstPin, _config.busyPin);

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 10000000,
        .input_delay_ns = 0,
        .spics_io_num = _config.csPin,
        .flags = 0,
        .queue_size = 20,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };
    _epd2_native->selectSPI(_config.spiHost, devcfg);
    _epd2_native->init(0);

    // Adafruit GFX and GxEPD2_BW bridge
    _epd2_bw = std::make_unique<GxEPD2_BW<GxEPD2_290_GDEY029T71H, 8>>(*_epd2_native);
    _epd2_bw->setRotation(_config.rotation);
    _epd2_bw->fillScreen(GxEPD_WHITE);
    _epd2_bw->display();

    if (!_spiMutex) {
        _spiMutex = xSemaphoreCreateMutex();
        if (!_spiMutex) {
            ESP_LOGW(TAG, "Failed to create SPI mutex");
        }
    }

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

    lv_display_rotation_t lv_rotation = LV_DISPLAY_ROTATION_0;
    switch (_config.rotation) {
        case 1: lv_rotation = LV_DISPLAY_ROTATION_90; break;
        case 2: lv_rotation = LV_DISPLAY_ROTATION_180; break;
        case 3: lv_rotation = LV_DISPLAY_ROTATION_270; break;
        default: lv_rotation = LV_DISPLAY_ROTATION_0; break;
    }

    ESP_LOGI(TAG, "Starting LVGL: requested physical=%ux%u rotation=%d (config.rotation=%d)",
             _config.width, _config.height, lv_rotation, _config.rotation);

    _lvglDisplay = lv_display_create(_config.width, _config.height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display (lv_display_create)");
        return false;
    }

    lv_display_set_rotation(_lvglDisplay, lv_rotation);

    int32_t hor_res = lv_display_get_horizontal_resolution(_lvglDisplay);
    int32_t ver_res = lv_display_get_vertical_resolution(_lvglDisplay);

    ESP_LOGI(TAG, "LVGL reports logical resolution: %dx%d (after rotation)", hor_res, ver_res);

    const size_t fb_size = ((size_t)_config.width * (size_t)_config.height + 7) / 8;
    _frameBuffer = (uint8_t*)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM);
    if (!_frameBuffer) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer (%zu bytes)", fb_size);
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }
    memset(_frameBuffer, 0xFF, fb_size);
    ESP_LOGI(TAG, "Allocated framebuffer: %zu bytes", fb_size);

    _framebufferMutex = xSemaphoreCreateMutex();
    if (!_framebufferMutex) {
        ESP_LOGE(TAG, "Failed to create framebuffer mutex");
        heap_caps_free(_frameBuffer);
        _frameBuffer = nullptr;
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    const size_t bufSize = (size_t)hor_res * (size_t)ver_res;
    _drawBuf1 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!_drawBuf1) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffer (%zu bytes)", bufSize * sizeof(lv_color_t));
        heap_caps_free(_frameBuffer);
        _frameBuffer = nullptr;
        vSemaphoreDelete(_framebufferMutex);
        _framebufferMutex = nullptr;
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "Allocated %zu bytes for full-screen buffer (%d×%d = %zu pixels)", 
             bufSize * sizeof(lv_color_t), (int)hor_res, (int)ver_res, bufSize);

    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(_lvglDisplay, _drawBuf1, nullptr,
                           bufSize * sizeof(lv_color_t),
                           LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    if (!createWorker()) {
        ESP_LOGW(TAG, "Failed to create display worker - LVGL flushes will still attempt direct writes (risky)");
    }

    ESP_LOGI(TAG, "LVGL started successfully (physical=%ux%u logical=%dx%d rotation=%d mode=FULL)",
             _config.width, _config.height, hor_res, ver_res, (int)lv_rotation);

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

bool GxEPD2Display::dither8x8ToMono(lv_color_t pixel, int x, int y) {
    uint8_t r = pixel.red;
    uint8_t g = pixel.green;
    uint8_t b = pixel.blue;
    uint8_t brightness = (r * 77 + g * 151 + b * 28) >> 8;
    uint8_t thresh = (dither8[y & 7][x & 7] << 2); // scale 0-63 to 0-252
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

    while (true) {
        if (xQueueReceive(self->_queue, &item, pdMS_TO_TICKS(100))) {
            if (item.buf == nullptr) {
                ESP_LOGI(TAG, "Worker: termination received");
                break;
            }

            if (has_pending && pending_item.buf) {
                // Discard stale frame
                heap_caps_free(pending_item.buf);
            }

            pending_item = item;
            has_pending = true;

        } else if (has_pending && pending_item.buf) {
            // Process the pending item
            if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);

            // Use Adafruit GFX to manage the frame drawing and refresh
            ESP_LOGI(TAG, "Worker: Drawing pending frame at x=%d, y=%d, w=%d, h=%d",
                     pending_item.x, pending_item.y, pending_item.w, pending_item.h);

            self->_epd2_bw->drawBitmap(
                pending_item.x, pending_item.y, 
                pending_item.buf, pending_item.w, pending_item.h, 
                GxEPD_BLACK);

            self->_epd2_bw->display(true); // Partial refresh

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

    // Flush the LVGL-rendered frame using Adafruit GFX
    const int width = area->x2 - area->x1 + 1;
    const int height = area->y2 - area->y1 + 1;

    // Use Adafruit GFX for rendering directly
    ESP_LOGI(TAG, "Flush: Drawing LVGL area x1=%d, y1=%d, w=%d, h=%d",
             area->x1, area->y1, width, height);

    // Convert the LVGL buffer and render
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int index = y * width + x;
            lv_color_t color = ((lv_color_t*)px_map)[index];

            // Map the pixel directly as black or white using dither
            bool is_white = self->dither8x8ToMono(color, area->x1 + x, area->y1 + y);
            self->_epd2_bw->drawPixel(area->x1 + x, area->y1 + y, is_white ? GxEPD_WHITE : GxEPD_BLACK);
        }
    }

    self->_epd2_bw->display(true); // Partial refresh
    lv_display_flush_ready(disp);
}

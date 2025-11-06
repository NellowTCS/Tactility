#include "GxEPD2Display.h"
#include "GxEPD2/GxEPD2_290_GDEY029T71H.h"
#include "DisplayTester.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include <vector>

static const char* TAG = "GxEPD2Display";

GxEPD2Display::GxEPD2Display(const Configuration& config)
    : _config(config)
    , _display(nullptr)
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
{
}

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
    return "E-paper display GDEY029T71H";
}

bool GxEPD2Display::start() {
    ESP_LOGI(TAG, "Starting e-paper display...");

    _display = std::make_unique<GxEPD2_290_GDEY029T71H>(
        _config.csPin,
        _config.dcPin,
        _config.rstPin,
        _config.busyPin
    );

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

    _display->selectSPI(_config.spiHost, devcfg);
    _display->init(0);
    _display->clearScreen(0xFF);
    _display->refresh(false);

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
    if (_display) {
        ESP_LOGI(TAG, "Stopping display...");
        destroyWorker();

        bool have = false;
        if (_spiMutex) {
            if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                have = true;
            } else {
                ESP_LOGW(TAG, "Failed to take SPI mutex while stopping; proceeding anyway");
            }
        }

        _display->hibernate();
        _display.reset();

        if (have && _spiMutex) {
            xSemaphoreGive(_spiMutex);
        }
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
    // Ensure termination message is enqueued so worker can exit normally.
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
    // Drain and free any remaining buffers that might be left in the queue
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
    if (!_display) {
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

    ESP_LOGI(TAG, "Driver native panel: WIDTH=%u HEIGHT=%u SOURCE_SHIFT=%u",
             (unsigned)GxEPD2_290_GDEY029T71H::WIDTH,
             (unsigned)GxEPD2_290_GDEY029T71H::HEIGHT,
             (unsigned)GxEPD2_290_GDEY029T71H::SOURCE_SHIFT);

    _lvglDisplay = lv_display_create(_config.width, _config.height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display (lv_display_create)");
        return false;
    }

    lv_display_set_rotation(_lvglDisplay, lv_rotation);

    int32_t hor_res = lv_display_get_horizontal_resolution(_lvglDisplay);
    int32_t ver_res = lv_display_get_vertical_resolution(_lvglDisplay);

    ESP_LOGI(TAG, "LVGL reports logical resolution: %dx%d (after rotation)", hor_res, ver_res);

    // Allocate persistent framebuffer for e-paper (stores complete display state)
    const size_t fb_size = ((size_t)_config.width * (size_t)_config.height + 7) / 8;
    _frameBuffer = (uint8_t*)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM);
    if (!_frameBuffer) {
        ESP_LOGE(TAG, "Failed to allocate framebuffer (%zu bytes)", fb_size);
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }
    memset(_frameBuffer, 0xFF, fb_size); // Initialize to white
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

    // Allocate draw buffers using LVGL's reported horizontal resolution
    const size_t bufSize = (size_t)hor_res * DRAW_BUF_LINES;
    _drawBuf1 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DMA);
    _drawBuf2 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!_drawBuf1 || !_drawBuf2) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffers (requested %zu bytes each)", bufSize * sizeof(lv_color_t));
        if (_drawBuf1) heap_caps_free(_drawBuf1);
        if (_drawBuf2) heap_caps_free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        heap_caps_free(_frameBuffer);
        _frameBuffer = nullptr;
        vSemaphoreDelete(_framebufferMutex);
        _framebufferMutex = nullptr;
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "Allocated %zu bytes per buffer (hor_res=%d, draw_lines=%zu)", bufSize * sizeof(lv_color_t), (int)hor_res, DRAW_BUF_LINES);

    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(_lvglDisplay, _drawBuf1, _drawBuf2,
                           bufSize * sizeof(lv_color_t),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    if (!createWorker()) {
        ESP_LOGW(TAG, "Failed to create display worker - LVGL flushes will still attempt direct writes (risky)");
    }

    ESP_LOGI(TAG, "LVGL started successfully (physical=%ux%u logical=%dx%d rotation=%d)",
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
    if (!_display) return;
    bool took = false;
    if (_spiMutex) {
        if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(200)) == pdTRUE) took = true;
        else ESP_LOGW(TAG, "writeRawImage: failed to take spi mutex");
    }
    _display->writeImage(bitmap, x, y, w, h, invert, mirror_y, false);
    if (took && _spiMutex) xSemaphoreGive(_spiMutex);
}

void GxEPD2Display::refreshDisplay(bool partial) {
    if (!_display) return;
    bool took = false;
    if (_spiMutex) {
        if (xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(200)) == pdTRUE) took = true;
        else ESP_LOGW(TAG, "refreshDisplay: failed to take spi mutex");
    }
    _display->refresh(partial);
    if (took && _spiMutex) xSemaphoreGive(_spiMutex);
}

static inline bool bayer4x4Dither(lv_color_t pixel, int x, int y) {
    // Compute brightness 0..255
    uint8_t r = pixel.red;
    uint8_t g = pixel.green;
    uint8_t b = pixel.blue;
    uint8_t brightness = (r * 77 + g * 151 + b * 28) >> 8;

    // Standard 4x4 Bayer matrix (values 0..15)
    // Layout:
    // 0  8  2 10
    //12  4 14  6
    //3 11  1  9
    //15  7 13  5
    static const uint8_t bayer4[4][4] = {
        { 0,  8,  2, 10},
        {12,  4, 14,  6},
        { 3, 11,  1,  9},
        {15,  7, 13,  5}
    };

    // Map matrix value (0..15) to threshold 0..255 (roughly multiply by 16)
    uint8_t thresh = (uint8_t)(bayer4[y & 3][x & 3] * 16 + 8); // center
    return brightness > thresh;
}

void GxEPD2Display::displayWorkerTask(void* arg) {
    GxEPD2Display* self = static_cast<GxEPD2Display*>(arg);
    if (!self) {
        vTaskDelete(NULL);
        return;
    }

    QueueItem item;
    bool processed_since_refresh = false;

    // Keep all written buffers for "writeAgain" after refresh (two-phase for fast partial update panels)
    struct WrittenRegion {
        uint8_t* buf;
        int16_t x;
        int16_t y;
        uint16_t w;
        uint16_t h;
    };
    std::vector<WrittenRegion> written_regions;

    while (true) {
        // Wait for at least one item
        if (xQueueReceive(self->_queue, &item, pdMS_TO_TICKS(150))) {
            if (item.buf == nullptr) {
                ESP_LOGI(TAG, "Worker: termination received");
                break;
            }

            // Collect this and any immediately available items to merge/union them
            std::vector<QueueItem> items;
            items.push_back(item);

            // Pull any other immediate items to merge into one union write (short non-blocking loop)
            while (xQueueReceive(self->_queue, &item, pdMS_TO_TICKS(5)) == pdTRUE) {
                if (item.buf == nullptr) {
                    // re-enqueue termination marker for outer logic and stop collection
                    QueueItem term = { nullptr, 0, 0, 0, 0 };
                    xQueueSend(self->_queue, &term, portMAX_DELAY);
                    break;
                }
                items.push_back(item);
            }

            // Compute union rectangle of dequeued items
            int union_x0 = INT32_MAX, union_y0 = INT32_MAX;
            int union_x1 = INT32_MIN, union_y1 = INT32_MIN;
            for (const auto &qi : items) {
                union_x0 = std::min<int>(union_x0, qi.x);
                union_y0 = std::min<int>(union_y0, qi.y);
                union_x1 = std::max<int>(union_x1, qi.x + qi.w - 1);
                union_y1 = std::max<int>(union_y1, qi.y + qi.h - 1);
            }

            if (union_x0 == INT32_MAX) {
                // nothing to do
                for (auto &q : items) if (q.buf) heap_caps_free(q.buf);
                continue;
            }

            // Align horizontally to byte boundaries required by EPDs
            int aligned_union_x0 = (union_x0 / 8) * 8;
            int aligned_union_x1 = ((union_x1 + 8) / 8) * 8 - 1;
            if (aligned_union_x0 < 0) aligned_union_x0 = 0;
            if (aligned_union_x1 >= (int)self->_config.width) aligned_union_x1 = (int)self->_config.width - 1;
            int union_w = aligned_union_x1 - aligned_union_x0 + 1;

            // Vertical bounds (no special alignment)
            int aligned_union_y0 = std::max(0, union_y0);
            int aligned_union_y1 = std::min<int>(union_y1, self->_config.height - 1);
            int union_h = aligned_union_y0 <= aligned_union_y1 ? (aligned_union_y1 - aligned_union_y0 + 1) : 0;
            if (union_h <= 0 || union_w <= 0) {
                // free items and continue
                for (auto &q : items) if (q.buf) heap_caps_free(q.buf);
                continue;
            }

            int union_row_bytes = (union_w + 7) / 8;
            size_t union_size = (size_t)union_row_bytes * (size_t)union_h;
            uint8_t* union_buf = (uint8_t*)heap_caps_malloc(union_size, MALLOC_CAP_DMA);
            if (!union_buf) {
                ESP_LOGE(TAG, "Worker: failed to alloc union buffer %zu bytes", union_size);
                // free item buffers
                for (auto &q : items) if (q.buf) heap_caps_free(q.buf);
                continue;
            }
            // Initialize union buffer to the "white" state (0xFF)
            memset(union_buf, 0xFF, union_size);

            // Copy each item's packed data into the union buffer in dequeued order (later items override earlier)
            for (const auto &qi : items) {
                int item_row_bytes = (qi.w + 7) / 8;
                for (int ry = 0; ry < qi.h; ++ry) {
                    int dst_row = (qi.y + ry) - aligned_union_y0;
                    if (dst_row < 0 || dst_row >= union_h) continue;
                    uint8_t* dst = union_buf + (size_t)dst_row * union_row_bytes;
                    int dst_byte_offset = (qi.x - aligned_union_x0) / 8;
                    uint8_t* dst_ptr = dst + dst_byte_offset;
                    uint8_t* src_ptr = qi.buf + (size_t)ry * item_row_bytes;
                    memcpy(dst_ptr, src_ptr, item_row_bytes);
                }
            }

            // Free original per-item buffers
            for (auto &q : items) {
                if (q.buf) {
                    heap_caps_free(q.buf);
                    q.buf = nullptr;
                }
            }

            // Perform SPI write once for the union rect
            if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);

            ESP_LOGI(TAG, "Worker: writeImage union x=%d y=%d w=%d h=%d (merged %d items)",
                     aligned_union_x0, aligned_union_y0, union_w, union_h, (int)items.size());

            self->_display->writeImage(union_buf, (int16_t)aligned_union_x0, (int16_t)aligned_union_y0,
                                       (int16_t)union_w, (int16_t)union_h, false, false, false);

            if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);

            // Save this written region so we can replay it with writeImageAgain after the refresh
            WrittenRegion wr;
            wr.buf = union_buf;
            wr.x = (int16_t)aligned_union_x0;
            wr.y = (int16_t)aligned_union_y0;
            wr.w = (uint16_t)union_w;
            wr.h = (uint16_t)union_h;
            written_regions.push_back(wr);

            processed_since_refresh = true;
            continue;
        } else {
            // No item received within timeout: if we did writes since last refresh, perform refresh and post-step.
            if (processed_since_refresh) {
                if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);
                ESP_LOGI(TAG, "Worker: refresh(true)");
                self->_display->refresh(true);

                // If the driver supports fast partial update, replay all written regions with writeImageAgain
                // to synchronize the controller's "previous" buffer with "current" buffer.
                if (self->_display && self->_display->hasFastPartialUpdate) {
                    for (const auto &wr : written_regions) {
                        ESP_LOGI(TAG, "Worker: writeImageAgain for region x=%d y=%d w=%d h=%d",
                                 wr.x, wr.y, wr.w, wr.h);
                        self->_display->writeImageAgain(wr.buf, wr.x, wr.y, wr.w, wr.h, false, false, false);
                    }
                }

                if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);

                // Free all written region buffers now that we've completed the second phase
                for (auto &wr : written_regions) {
                    if (wr.buf) {
                        heap_caps_free(wr.buf);
                        wr.buf = nullptr;
                    }
                }
                written_regions.clear();

                processed_since_refresh = false;
            }
        }
    }

    // Termination: drain remaining items and free memory, then final refresh if necessary
    while (xQueueReceive(self->_queue, &item, 0) == pdTRUE) {
        if (item.buf) {
            if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);
            ESP_LOGI(TAG, "Worker: draining and writeImage x=%u y=%u w=%u h=%u", item.x, item.y, item.w, item.h);
            self->_display->writeImage(item.buf, item.x, item.y, item.w, item.h, false, false, false);
            if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);
            heap_caps_free(item.buf);
        }
    }
    if (processed_since_refresh) {
        if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);
        ESP_LOGI(TAG, "Worker: final refresh(true)");
        self->_display->refresh(true);
        if (self->_display && self->_display->hasFastPartialUpdate) {
            for (const auto &wr : written_regions) {
                ESP_LOGI(TAG, "Worker: final writeImageAgain for region x=%d y=%d w=%d h=%d",
                         wr.x, wr.y, wr.w, wr.h);
                self->_display->writeImageAgain(wr.buf, wr.x, wr.y, wr.w, wr.h, false, false, false);
            }
        }
        if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);
    }

    // Free any remaining written region buffers
    for (auto &wr : written_regions) {
        if (wr.buf) {
            heap_caps_free(wr.buf);
            wr.buf = nullptr;
        }
    }

    self->_workerRunning = false;
    vTaskDelete(NULL);
}

void GxEPD2Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
    if (!self || !self->_display || !self->_frameBuffer) {
        lv_display_flush_ready(disp);
        return;
    }

    const int logical_w = area->x2 - area->x1 + 1;
    const int logical_h = area->y2 - area->y1 + 1;
    lv_color_format_t cf = lv_display_get_color_format(disp);
    uint32_t src_row_stride_bytes = (uint32_t)lv_draw_buf_width_to_stride(logical_w, cf);
    uint8_t* src_bytes = (uint8_t*)px_map;

    const int panel_width = self->_config.width;
    const int panel_height = self->_config.height;
    const int panel_bytes_per_row = (panel_width + 7) / 8;

    int32_t hor_res = lv_display_get_horizontal_resolution(disp);
    int32_t ver_res = lv_display_get_vertical_resolution(disp);

    int min_px = panel_width, min_py = panel_height, max_px = -1, max_py = -1;

    if (xSemaphoreTake(self->_framebufferMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Framebuffer mutex busy; dropping LVGL flush");
        lv_display_flush_ready(disp);
        return;
    }

    lv_display_rotation_t lv_rotation = lv_display_get_rotation(disp);
    int rotation = 0;
    switch (lv_rotation) {
        case LV_DISPLAY_ROTATION_90: rotation = 1; break;
        case LV_DISPLAY_ROTATION_180: rotation = 2; break;
        case LV_DISPLAY_ROTATION_270: rotation = 3; break;
        default: rotation = 0; break;
    }
    ESP_LOGI(TAG, "lvglFlushCallback: LVGL rotation=%d config.rotation=%d hor_res=%d ver_res=%d area=(%d,%d)-(%d,%d)",
             rotation, (int)self->_config.rotation, (int)hor_res, (int)ver_res, area->x1, area->y1, area->x2, area->y2);

    auto map_logical_to_physical = [&](int logical_x_abs, int logical_y_abs, int& physical_x_abs, int& physical_y_abs) {
        switch (rotation) {
            case 1:
                physical_x_abs = logical_y_abs;
                physical_y_abs = (hor_res - 1) - logical_x_abs;
                break;
            case 2:
                physical_x_abs = (hor_res - 1) - logical_x_abs;
                physical_y_abs = (ver_res - 1) - logical_y_abs;
                break;
            case 3:
                physical_x_abs = (ver_res - 1) - logical_y_abs;
                physical_y_abs = logical_x_abs;
                break;
            case 0:
            default:
                physical_x_abs = logical_x_abs;
                physical_y_abs = logical_y_abs;
                break;
        }
        physical_x_abs += self->_gapX;
        physical_y_abs += self->_gapY;
    };

    for (int ly = 0; ly < logical_h; ++ly) {
        lv_color_t* src_row = (lv_color_t*)(src_bytes + (size_t)ly * src_row_stride_bytes);

        for (int lx = 0; lx < logical_w; ++lx) {
            int logical_x_abs = area->x1 + lx;
            int logical_y_abs = area->y1 + ly;

            int physical_x_abs = 0, physical_y_abs = 0;
            map_logical_to_physical(logical_x_abs, logical_y_abs, physical_x_abs, physical_y_abs);

            if (physical_x_abs < 0 || physical_x_abs >= panel_width ||
                physical_y_abs < 0 || physical_y_abs >= panel_height) {
                continue;
            }

            lv_color_t pixel = src_row[lx];
            bool is_white = bayer4x4Dither(pixel, physical_x_abs, physical_y_abs);

            const int byte_idx = physical_y_abs * panel_bytes_per_row + (physical_x_abs / 8);
            const int bit_pos = 7 - (physical_x_abs & 7);

            if (is_white) {
                self->_frameBuffer[byte_idx] |= (1 << bit_pos);
            } else {
                self->_frameBuffer[byte_idx] &= ~(1 << bit_pos);
            }

            min_px = std::min(min_px, physical_x_abs);
            max_px = std::max(max_px, physical_x_abs);
            min_py = std::min(min_py, physical_y_abs);
            max_py = std::max(max_py, physical_y_abs);
        }
    }

    if (max_px < 0 || max_py < 0) {
        xSemaphoreGive(self->_framebufferMutex);
        lv_display_flush_ready(disp);
        return;
    }

    int aligned_x0 = (min_px / 8) * 8;
    int aligned_x1 = ((max_px + 8) / 8) * 8 - 1;
    if (aligned_x0 < 0) aligned_x0 = 0;
    if (aligned_x1 >= panel_width) aligned_x1 = panel_width - 1;
    int aligned_w = aligned_x1 - aligned_x0 + 1;
    int packed_row_bytes = (aligned_w + 7) / 8;

    int y0 = min_py;
    int y1 = max_py;
    if (y0 < 0) y0 = 0;
    if (y1 >= panel_height) y1 = panel_height - 1;
    int packed_h = y1 - y0 + 1;

    size_t packed_size = (size_t)packed_row_bytes * (size_t)packed_h;
    uint8_t* packed = (uint8_t*)heap_caps_malloc(packed_size, MALLOC_CAP_DMA);
    if (!packed) {
        xSemaphoreGive(self->_framebufferMutex);
        ESP_LOGE(TAG, "Failed to allocate packed buffer of %zu bytes", packed_size);
        lv_display_flush_ready(disp);
        return;
    }

    for (int ry = 0; ry < packed_h; ++ry) {
        int src_y = y0 + ry;
        uint8_t* dst_row = packed + (size_t)ry * packed_row_bytes;
        for (int bx = 0; bx < packed_row_bytes; ++bx) {
            uint8_t out = 0;
            int base_x = aligned_x0 + bx * 8;
            for (int bit = 0; bit < 8; ++bit) {
                int sx = base_x + bit;
                out <<= 1;
                if (sx >= panel_width) {
                } else {
                    int src_byte_idx = src_y * panel_bytes_per_row + (sx / 8);
                    int src_bit_pos = 7 - (sx & 7);
                    bool white = (self->_frameBuffer[src_byte_idx] & (1 << src_bit_pos)) != 0;
                    if (white) out |= 1;
                }
            }
            dst_row[bx] = out;
        }
    }

    xSemaphoreGive(self->_framebufferMutex);

    // Debug: dump first 16 bytes of packed buffer and corresponding framebuffer bytes
    ESP_LOGI(TAG, "Mapped physical bbox (pre-align) = x[%d..%d] y[%d..%d] aligned_x0=%d y0=%d w=%d h=%d packed_bytes=%zu",
             min_px, max_px, min_py, max_py, aligned_x0, y0, aligned_w, packed_h, packed_size);
    
    char hex_buf[128];
    int print_bytes = std::min(16, (int)packed_size);
    char* p = hex_buf;
    for (int i = 0; i < print_bytes; ++i) {
        p += snprintf(p, hex_buf + sizeof(hex_buf) - p, "%02X ", packed[i]);
    }
    ESP_LOGI(TAG, "Packed first %d bytes: %s", print_bytes, hex_buf);

    // Also dump corresponding bytes from framebuffer at first row
    int fb_row0_start = y0 * panel_bytes_per_row + (aligned_x0 / 8);
    p = hex_buf;
    for (int i = 0; i < std::min(print_bytes, packed_row_bytes); ++i) {
        p += snprintf(p, hex_buf + sizeof(hex_buf) - p, "%02X ", self->_frameBuffer[fb_row0_start + i]);
    }
    ESP_LOGI(TAG, "Framebuffer row y=%d bytes [%d..%d]: %s", y0, fb_row0_start, fb_row0_start + print_bytes - 1, hex_buf);

    if (self->_queue) {
        QueueItem qi;
        qi.buf = packed;
        qi.x = (uint16_t)aligned_x0;
        qi.y = (uint16_t)y0;
        qi.w = (uint16_t)aligned_w;
        qi.h = (uint16_t)packed_h;

        if (xQueueSend(self->_queue, &qi, pdMS_TO_TICKS(50)) != pdTRUE) {
            ESP_LOGW(TAG, "Display queue full; minimal frame dropped. x=%d y=%d w=%d h=%d", aligned_x0, y0, aligned_w, packed_h);
            heap_caps_free(packed);
        } else {
            ESP_LOGI(TAG, "Enqueued frame x=%d y=%d w=%d h=%d", aligned_x0, y0, aligned_w, packed_h);
        }
    } else {
        if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);
        ESP_LOGI(TAG, "Direct write (no queue) x=%d y=%d w=%d h=%d", aligned_x0, y0, aligned_w, packed_h);
        self->_display->writeImage(packed, aligned_x0, y0, aligned_w, packed_h, false, false, false);
        if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);
        heap_caps_free(packed);
    }

    lv_display_flush_ready(disp);
}

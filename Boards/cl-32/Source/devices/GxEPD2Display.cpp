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
    , _display(nullptr)
    , _lvglDisplay(nullptr)
    , _drawBuf1(nullptr)
    , _drawBuf2(nullptr)
    , _queue(nullptr)
    , _workerTaskHandle(nullptr)
    , _spiMutex(nullptr)
    , _workerRunning(false)
    , _gapX(0)
    , _gapY(0)
    , _fullFb(nullptr)
    , _fullFbSize(0)
{
}

GxEPD2Display::~GxEPD2Display() {
    stopLvgl();
    stop();
    // destroy worker and mutex if still present
    destroyWorker();
    if (_spiMutex) {
        vSemaphoreDelete(_spiMutex);
        _spiMutex = nullptr;
    }
    if (_fullFb) {
        heap_caps_free(_fullFb);
        _fullFb = nullptr;
        _fullFbSize = 0;
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

    // spi_device_interface_config_t setup for SPI initialization
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
        // increase queue size to handle bursty LVGL traffic
        .queue_size = 20,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };

    _display->selectSPI(_config.spiHost, devcfg);
    _display->init(0);
    _display->clearScreen(0xFF);
    _display->refresh(false);

    // Create SPI mutex so direct calls and worker are serialized
    if (!_spiMutex) {
        _spiMutex = xSemaphoreCreateMutex();
        if (!_spiMutex) {
            ESP_LOGW(TAG, "Failed to create SPI mutex");
        }
    }

    // Run hardware tests once (these use direct writes)
    // display_tester::runTests(this);

    ESP_LOGI(TAG, "E-paper display started successfully");
    return true;
}

bool GxEPD2Display::stop() {
    if (_display) {
        ESP_LOGI(TAG, "Stopping display...");
        // Ensure worker stopped first to avoid it accessing _display after reset
        destroyWorker();

        // Protect calls with mutex
        if (_spiMutex) xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(200));
        _display->hibernate();
        _display.reset();
        if (_spiMutex) xSemaphoreGive(_spiMutex);
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
        8192, // stack size (adjust if needed)
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
    // send sentinel to stop task
    QueueItem term;
    term.buf = nullptr;
    term.x = term.y = term.w = term.h = 0;
    // best-effort send; if queue full, try to empty it
    xQueueSend(_queue, &term, pdMS_TO_TICKS(50));
    // wait a bit for task to exit
    if (_workerTaskHandle) {
        // give worker time to exit
        vTaskDelay(pdMS_TO_TICKS(100));
        // if still alive, force delete
        if (eTaskGetState(_workerTaskHandle) != eDeleted) {
            vTaskDelete(_workerTaskHandle);
        }
        _workerTaskHandle = nullptr;
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

    // --- LVGL Configuration based on Rotation ---
    // Map config rotation to LVGL rotation
    lv_display_rotation_t lv_rotation = LV_DISPLAY_ROTATION_0;
    switch (_config.rotation) {
        case 1: lv_rotation = LV_DISPLAY_ROTATION_90; break;
        case 2: lv_rotation = LV_DISPLAY_ROTATION_180; break;
        case 3: lv_rotation = LV_DISPLAY_ROTATION_270; break;
        default: lv_rotation = LV_DISPLAY_ROTATION_0; break;
    }

    ESP_LOGI(TAG, "Starting LVGL: requested physical=%ux%u rotation=%d (config.rotation=%d)",
             _config.width, _config.height, lv_rotation, _config.rotation);

    // log driver panel constants for debugging
    ESP_LOGI(TAG, "Driver native panel: WIDTH=%u HEIGHT=%u SOURCE_SHIFT=%u",
             (unsigned)GxEPD2_290_GDEY029T71H::WIDTH,
             (unsigned)GxEPD2_290_GDEY029T71H::HEIGHT,
             (unsigned)GxEPD2_290_GDEY029T71H::SOURCE_SHIFT);

    // Create the LVGL display using the physical dimensions first.
    // We'll set rotation, then query LVGL for the actual logical resolution it uses,
    // and allocate buffers based on that reported resolution so the stride matches.
    _lvglDisplay = lv_display_create(_config.width, _config.height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display (lv_display_create)");
        return false;
    }

    // Set rotation - LVGL will handle dimension swapping internally
    lv_display_set_rotation(_lvglDisplay, lv_rotation);

    // Query the actual logical resolution LVGL reports AFTER rotation
    int32_t hor_res = lv_display_get_horizontal_resolution(_lvglDisplay);
    int32_t ver_res = lv_display_get_vertical_resolution(_lvglDisplay);

    ESP_LOGI(TAG, "LVGL reports logical resolution: %dx%d (after rotation)", hor_res, ver_res);

    // Allocate draw buffers using LVGL's reported horizontal resolution
    const size_t bufSize = (size_t)hor_res * DRAW_BUF_LINES;
    _drawBuf1 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DMA);
    _drawBuf2 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!_drawBuf1 || !_drawBuf2) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffers (requested %zu bytes each)", bufSize * sizeof(lv_color_t));
        if (_drawBuf1) heap_caps_free(_drawBuf1);
        if (_drawBuf2) heap_caps_free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "Allocated %zu bytes per buffer (hor_res=%d, draw_lines=%zu)", bufSize * sizeof(lv_color_t), (int)hor_res, DRAW_BUF_LINES);

    // Configure LVGL color format and buffers
    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(_lvglDisplay, _drawBuf1, _drawBuf2,
                           bufSize * sizeof(lv_color_t),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    // set flush callback and user data
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    // Allocate a full logical shadow framebuffer in PSRAM so we can build full-screen packed frames.
    // Logical resolution is hor_res x ver_res (LVGL reports after rotation).
    size_t full_fb_bytes = (size_t)hor_res * (size_t)ver_res * sizeof(lv_color_t);
    _fullFb = (lv_color_t*)heap_caps_malloc(full_fb_bytes, MALLOC_CAP_SPIRAM);
    if (!_fullFb) {
        ESP_LOGW(TAG, "Failed to allocate full shadow framebuffer (%zu bytes) in PSRAM; full-refresh fallback will be disabled", full_fb_bytes);
        _fullFbSize = 0;
    } else {
        _fullFbSize = full_fb_bytes;
        // initialize to white
        for (size_t i = 0; i < (size_t)hor_res * (size_t)ver_res; ++i) _fullFb[i] = LV_COLOR_WHITE;
        ESP_LOGI(TAG, "Allocated full shadow framebuffer: %zu bytes (%dx%d)", full_fb_bytes, hor_res, ver_res);
    }

    // Create the display worker (queue + task) used to serialize writes and batch refreshes
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

    // Destroy worker (will flush remaining items and stop)
    destroyWorker();

    if (_fullFb) {
        heap_caps_free(_fullFb);
        _fullFb = nullptr;
        _fullFbSize = 0;
    }

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
    // serialize direct writes with mutex so they don't race the worker
    if (_spiMutex) xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(200));
    _display->writeImage(bitmap, x, y, w, h, invert, mirror_y, false);
    if (_spiMutex) xSemaphoreGive(_spiMutex);
}

void GxEPD2Display::refreshDisplay(bool partial) {
    if (!_display) return;
    // serialize refresh with mutex so it doesn't race writes
    if (_spiMutex) xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(200));
    _display->refresh(partial);
    if (_spiMutex) xSemaphoreGive(_spiMutex);
}

inline bool GxEPD2Display::rgb565ToMono(lv_color_t pixel) {
    uint8_t r = pixel.red;
    uint8_t g = pixel.green;
    uint8_t b = pixel.blue;
    uint8_t brightness = (r * 77 + g * 151 + b * 28) >> 8;
    return brightness > 127;
}

void GxEPD2Display::displayWorkerTask(void* arg) {
    GxEPD2Display* self = static_cast<GxEPD2Display*>(arg);
    if (!self) {
        vTaskDelete(NULL);
        return;
    }

    QueueItem item;
    bool processed_since_refresh = false;

    while (true) {
        if (xQueueReceive(self->_queue, &item, pdMS_TO_TICKS(150))) {
            if (item.buf == nullptr) {
                break;
            }
            if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);

            int wb_driver = (item.w + 7) / 8;
            size_t item_size = (size_t)wb_driver * (size_t)item.h;
            ESP_LOGI(TAG, "Worker: full writeImage called with x=%d y=%d w=%d h=%d wb=%d buf=%p size=%d",
                     item.x, item.y, item.w, item.h, wb_driver, (void*)item.buf, (int)item_size);

            // We do a full-screen write here: worker receives full-screen packed frames (x==0,y==0)
            // After writing the image, perform a full refresh to ensure the panel displays the content.
            self->_display->writeImage(item.buf, item.x, item.y, item.w, item.h, false, false, false);
            // Force a full refresh so partial-update complexities are avoided for now.
            ESP_LOGI(TAG, "Worker: forcing full refresh()");
            self->_display->refresh(false);

            if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);
            heap_caps_free(item.buf);
            item.buf = nullptr;
            processed_since_refresh = false; // already refreshed
            continue;
        } else {
            // nothing received; nothing to do
        }
    }

    while (xQueueReceive(self->_queue, &item, 0) == pdTRUE) {
        if (item.buf) {
            if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);
            int wb_driver = (item.w + 7) / 8;
            size_t item_size = (size_t)wb_driver * (size_t)item.h;
            ESP_LOGI(TAG, "Worker (drain): full writeImage x=%d y=%d w=%d h=%d wb=%d buf=%p size=%d",
                     item.x, item.y, item.w, item.h, wb_driver, (void*)item.buf, (int)item_size);

            self->_display->writeImage(item.buf, item.x, item.y, item.w, item.h, false, false, false);
            ESP_LOGI(TAG, "Worker (drain): forcing full refresh()");
            self->_display->refresh(false);
            if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);
            heap_caps_free(item.buf);
        }
    }
    self->_workerRunning = false;
    vTaskDelete(NULL);
}

// Immediate full-screen pack & enqueue strategy:
// - Copy LVGL partial px_map into the logical shadow framebuffer (if allocated).
// - Build a full-screen 1bpp packed buffer from the shadow and enqueue it as a single full-screen write.
// This avoids per-flush partial packing complexity. It's heavy but reliable as a first step.
void GxEPD2Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
    if (!self || !self->_display) {
        lv_display_flush_ready(disp);
        return;
    }

    lv_display_rotation_t rotation = lv_display_get_rotation(disp);
    int32_t hor_res = lv_display_get_horizontal_resolution(disp);
    int32_t ver_res = lv_display_get_vertical_resolution(disp);

    const int logical_w = area->x2 - area->x1 + 1;
    const int logical_h = area->y2 - area->y1 + 1;
    lv_color_format_t cf = lv_display_get_color_format(disp);
    uint32_t src_row_stride_bytes = (uint32_t)lv_draw_buf_width_to_stride(logical_w, cf);
    uint8_t* src_bytes = (uint8_t*)px_map;

    // If we don't have a full shadow FB, fall back to doing nothing and ack the flush.
    if (!self->_fullFb) {
        // best effort: still ack the flush so LVGL can continue
        lv_display_flush_ready(disp);
        return;
    }

    // Copy the px_map into the shadow full framebuffer at logical coordinates
    for (int ly = 0; ly < logical_h; ++ly) {
        lv_color_t* src_row = (lv_color_t*)(src_bytes + (size_t)ly * src_row_stride_bytes);
        // destination is fullFb at (area->x1, area->y1 + ly)
        int dst_y = area->y1 + ly;
        if (dst_y < 0 || dst_y >= ver_res) continue;
        lv_color_t* dst_row = self->_fullFb + (size_t)dst_y * (size_t)hor_res;
        // copy logical_w pixels
        for (int lx = 0; lx < logical_w; ++lx) {
            int dst_x = area->x1 + lx;
            if (dst_x < 0 || dst_x >= hor_res) continue;
            dst_row[dst_x] = src_row[lx];
        }
    }

    // Now build a full-screen packed 1bpp buffer (physical packing & rotation applied)
    const int epd_w = (int)GxEPD2_290_GDEY029T71H::WIDTH;  // 168
    const int epd_h = (int)GxEPD2_290_GDEY029T71H::HEIGHT; // 384
    const int epd_row_bytes = (epd_w + 7) / 8; // 21
    const size_t packed_size = (size_t)epd_row_bytes * (size_t)epd_h;
    uint8_t* packed = (uint8_t*)heap_caps_malloc(packed_size, MALLOC_CAP_DMA);
    if (!packed) {
        ESP_LOGE(TAG, "Failed to allocate full packed buffer (%zu bytes)", packed_size);
        lv_display_flush_ready(disp);
        return;
    }
    memset(packed, 0xFF, packed_size);

    // Map logical (fullFb) to physical panel coordinates and pack
    // fullFb logical resolution is hor_res x ver_res (384 x 168)
    for (int ly = 0; ly < ver_res; ++ly) {
        for (int lx = 0; lx < hor_res; ++lx) {
            // read logical pixel
            lv_color_t pixel = self->_fullFb[ly * hor_res + lx];
            bool is_white = rgb565ToMono(pixel);

            // map logical -> physical based on rotation (we set rotation earlier in startLvgl)
            int physical_x = 0;
            int physical_y = 0;
            if (rotation == LV_DISPLAY_ROTATION_0) {
                physical_x = lx;
                physical_y = ly;
            } else if (rotation == LV_DISPLAY_ROTATION_90) {
                physical_x = ly;
                physical_y = hor_res - 1 - lx;
            } else if (rotation == LV_DISPLAY_ROTATION_180) {
                physical_x = hor_res - 1 - lx;
                physical_y = ver_res - 1 - ly;
            } else { // 270
                physical_x = ver_res - 1 - ly;
                physical_y = lx;
            }

            // physical_x/physical_y are within panel's logical coordinate space after rotation.
            // But our panel native orientation is WIDTH x HEIGHT (168 x 384). Map accordingly:
            // epd_x ranges 0..(epd_w-1), epd_y ranges 0..(epd_h-1)
            if (physical_x < 0 || physical_x >= epd_w || physical_y < 0 || physical_y >= epd_h) {
                continue;
            }
            int byte_idx = physical_y * epd_row_bytes + (physical_x / 8);
            int bit_pos = 7 - (physical_x & 7);
            if (is_white) {
                packed[byte_idx] |= (1 << bit_pos);
            } else {
                packed[byte_idx] &= ~(1 << bit_pos);
            }
        }
    }

    // Enqueue a full-screen write (x=0,y=0,w=epd_w,h=epd_h)
    if (self->_queue) {
        QueueItem qi;
        qi.buf = packed;
        qi.x = 0;
        qi.y = 0;
        qi.w = (uint16_t)epd_w;
        qi.h = (uint16_t)epd_h;
        ESP_LOGI(TAG, "Enqueue full-screen update: w=%d h=%d packed_size=%d", qi.w, qi.h, (int)packed_size);
        if (xQueueSend(self->_queue, &qi, pdMS_TO_TICKS(50)) != pdTRUE) {
            ESP_LOGW(TAG, "Display queue full; dropping full-screen frame");
            heap_caps_free(packed);
        }
    } else {
        heap_caps_free(packed);
    }

    // Acknowledge LVGL flush immediately — we've captured the pixels into our shadow. (MWHAHAHA)
    lv_display_flush_ready(disp);
}

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
    display_tester::runTests(this);

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
        8192, // stack size (adjust needed)
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

    // Compute logical dimensions LVGL will use (LVGL swaps width/height when sw_rotate is used)
    uint16_t lv_logical_w = (_config.rotation == 1 || _config.rotation == 3) ? _config.height : _config.width;
    uint16_t lv_logical_h = (_config.rotation == 1 || _config.rotation == 3) ? _config.width : _config.height;

    ESP_LOGI(TAG, "Starting LVGL: physical=%ux%u rotation=%d (config.rotation=%d, sw_rotate=implicit) -> logical=%ux%u", 
             _config.width, _config.height, lv_rotation, _config.rotation, lv_logical_w, lv_logical_h);

    // Allocate draw buffers (RGB565 format - 2 bytes per pixel) using logical width so DRAW_BUF_LINES is correct
    const size_t bufSize = (size_t)lv_logical_w * DRAW_BUF_LINES;
    _drawBuf1 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DMA);
    _drawBuf2 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!_drawBuf1 || !_drawBuf2) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffers (requested %zu bytes each)", bufSize * sizeof(lv_color_t));
        if (_drawBuf1) heap_caps_free(_drawBuf1);
        if (_drawBuf2) heap_caps_free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "Allocated %zu bytes per buffer (logical width=%u, draw lines=%zu)", bufSize * sizeof(lv_color_t), lv_logical_w, DRAW_BUF_LINES);

    // Create LVGL display with logical dimensions
    _lvglDisplay = lv_display_create(lv_logical_w, lv_logical_h);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        heap_caps_free(_drawBuf1);
        heap_caps_free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        return false;
    }

    // Set rotation - LVGL will handle dimension swapping and software rotation
    lv_display_set_rotation(_lvglDisplay, lv_rotation);

    // Use RGB565 format - standard LVGL rendering
    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(_lvglDisplay, _drawBuf1, _drawBuf2, 
                           bufSize * sizeof(lv_color_t), 
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    // Create the display worker (queue + task) used to serialize writes and batch refreshes
    if (!createWorker()) {
        ESP_LOGW(TAG, "Failed to create display worker - LVGL flushes will still attempt direct writes (risky)");
    }

    ESP_LOGI(TAG, "LVGL started successfully");

    // Run hardware tests once (these use direct writes)
    display_tester::runLvglTest(_lvglDisplay);

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

// Helper: Convert RGB565 to monochrome using brightness threshold
// Uses ITU-R BT.601 luma coefficients for RGB to grayscale conversion
inline bool GxEPD2Display::rgb565ToMono(lv_color_t pixel) {
    // Extract RGB components from lv_color_t
    // Note: LVGL9 uses 8-bit color channels internally
    uint8_t r = pixel.red;   
    uint8_t g = pixel.green; 
    uint8_t b = pixel.blue;  
    
    // Calculate brightness using standard luminance weights
    // Y = 0.299*R + 0.587*G + 0.114*B
    // Approximated as (77*R + 151*G + 28*B) / 256 for speed
    uint8_t brightness = (r * 77 + g * 151 + b * 28) >> 8;
    
    // Threshold at 50% gray: >127 = white (bit=1), <=127 = black (bit=0)
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
        // Wait for an item, but with timeout so we can perform a refresh after bursts
        if (xQueueReceive(self->_queue, &item, pdMS_TO_TICKS(150))) {
            // termination sentinel: buf==nullptr => exit
            if (item.buf == nullptr) {
                break;
            }

            // Protect SPI access
            if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);
            // Write image synchronously
            self->_display->writeImage(item.buf, item.x, item.y, item.w, item.h, false, false, false);
            if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);

            // free buffer after write
            heap_caps_free(item.buf);
            item.buf = nullptr;
            processed_since_refresh = true;

            // and loop to pick up more items without refreshing yet (batching)
            continue;
        } else {
            // timeout; if we processed things since last refresh, perform a partial refresh
            if (processed_since_refresh) {
                if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);
                self->_display->refresh(true);
                if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);
                processed_since_refresh = false;
            }
            // continue waiting for more items (or termination)
        }
    }

    // Before exiting, flush any remaining items in queue
    while (xQueueReceive(self->_queue, &item, 0) == pdTRUE) {
        if (item.buf) {
            if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);
            self->_display->writeImage(item.buf, item.x, item.y, item.w, item.h, false, false, false);
            if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);
            heap_caps_free(item.buf);
        }
    }
    // Final refresh if anything was written
    if (processed_since_refresh) {
        if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, portMAX_DELAY);
        self->_display->refresh(true);
        if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);
    }

    // Mark not running and exit
    self->_workerRunning = false;
    vTaskDelete(NULL);
}

void GxEPD2Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
    if (!self || !self->_display) { 
        lv_display_flush_ready(disp); 
        return; 
    }

    // LVGL provides coordinates and pixel data in logical orientation (after rotation)
    // For software rotation, we need to transform them back to physical orientation
    // to match the hardware's native coordinate system (168×384 portrait)
    
    lv_display_rotation_t rotation = lv_display_get_rotation(disp);
    lv_area_t physical_area = *area;

    // Retrieve LVGL logical full resolution
    int32_t hor_res = lv_display_get_horizontal_resolution(disp);  // logical width LVGL is using
    int32_t ver_res = lv_display_get_vertical_resolution(disp);    // logical height LVGL is using

    // Map logical rectangle -> physical rectangle for known rotations
    // We'll compute per-pixel mapping below, but compute physical_area bounds as transformed area.
    if (rotation == LV_DISPLAY_ROTATION_90) {
        // logical (hor_res x ver_res) => physical (width x height)
        physical_area.x1 = area->y1;
        physical_area.y1 = hor_res - 1 - area->x2;
        physical_area.x2 = area->y2;
        physical_area.y2 = hor_res - 1 - area->x1;
    } else if (rotation == LV_DISPLAY_ROTATION_270) {
        // 270 CW (90 CCW)
        physical_area.x1 = ver_res - 1 - area->y2;
        physical_area.y1 = area->x1;
        physical_area.x2 = ver_res - 1 - area->y1;
        physical_area.y2 = area->x2;
    } else if (rotation == LV_DISPLAY_ROTATION_180) {
        physical_area.x1 = hor_res - 1 - area->x2;
        physical_area.y1 = ver_res - 1 - area->y2;
        physical_area.x2 = hor_res - 1 - area->x1;
        physical_area.y2 = ver_res - 1 - area->y1;
    } else {
        // LV_DISPLAY_ROTATION_0: no change
        physical_area = *area;
    }

    const int epd_x = physical_area.x1;
    const int epd_y = physical_area.y1;
    const int epd_w = physical_area.x2 - physical_area.x1 + 1;
    const int epd_h = physical_area.y2 - physical_area.y1 + 1;

    if (epd_w <= 0 || epd_h <= 0) { 
        lv_display_flush_ready(disp); 
        return; 
    }

    ESP_LOGD(TAG, "Flush: EPD=[%d,%d] %dx%d", epd_x, epd_y, epd_w, epd_h);

    // Get logical area dimensions (what LVGL rendered)
    const int logical_w = area->x2 - area->x1 + 1;
    const int logical_h = area->y2 - area->y1 + 1;

    // Allocate 1-bit packed buffer for monochrome e-paper (physical dimensions)
    // Format: MSB first, horizontal byte packing (left-to-right)
    const int epd_row_bytes = (epd_w + 7) / 8;
    const size_t packed_size = (size_t)epd_row_bytes * (size_t)epd_h;
    uint8_t* packed = (uint8_t*)heap_caps_malloc(packed_size, MALLOC_CAP_DMA);
    if (!packed) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes for 1-bit buffer", packed_size);
        lv_display_flush_ready(disp);
        return;
    }
    
    // Initialize to 0xFF (all white) - GxEPD2 uses 0xFF=white, 0x00=black
    memset(packed, 0xFF, packed_size); 

    // Convert LVGL pixel buffer -> 1-bit monochrome for e-paper
    lv_color_format_t cf = lv_display_get_color_format(disp);
    uint32_t src_row_bytes = (uint32_t)lv_draw_buf_width_to_stride(logical_w, cf);
    uint8_t* src_bytes = (uint8_t*)px_map;

    // Debugging: print a few diagnostics for the first several flushes to detect stride/packing issues
    static int s_flush_debug_count = 0;
    if (s_flush_debug_count < 8) {
        ESP_LOGI(TAG, "lvglFlush: logical=%dx%d physical=%dx%d at (%d,%d) rot=%d cf=%d src_row_bytes=%u hor_res=%d ver_res=%d", 
                 logical_w, logical_h, epd_w, epd_h, epd_x, epd_y, (int)rotation, (int)cf, (unsigned)src_row_bytes, hor_res, ver_res);
    }

    // For each pixel in the logical area, compute its absolute logical coords (lx_abs, ly_abs),
    // map to absolute physical coords (px_abs,py_abs) depending on rotation, then pack into 'packed' buffer
    for (int ly = 0; ly < logical_h; ++ly) {
        lv_color_t* src_row = (lv_color_t*)(src_bytes + (size_t)ly * src_row_bytes);
        for (int lx = 0; lx < logical_w; ++lx) {
            lv_color_t pixel = src_row[lx];
            bool is_white = rgb565ToMono(pixel);

            // absolute logical coordinates (within LVGL logical screen)
            int lx_abs = area->x1 + lx;
            int ly_abs = area->y1 + ly;

            int px_abs = 0;
            int py_abs = 0;

            // Map logical -> physical depending on rotation.
            // Physical panel dims: width = _config.width, height = _config.height
            if (rotation == LV_DISPLAY_ROTATION_0) {
                px_abs = lx_abs;
                py_abs = ly_abs;
            } else if (rotation == LV_DISPLAY_ROTATION_90) {
                // LVGL logical: width = panel height, height = panel width
                // 90° CW: (lx_abs,ly_abs) -> (px,py) = (ly_abs, panel_width-1 - lx_abs)
                px_abs = ly_abs;
                py_abs = (_config.width - 1) - lx_abs;
            } else if (rotation == LV_DISPLAY_ROTATION_180) {
                px_abs = (_config.width - 1) - lx_abs;
                py_abs = (_config.height - 1) - ly_abs;
            } else if (rotation == LV_DISPLAY_ROTATION_270) {
                // 270° CW (or 90° CCW): (lx_abs,ly_abs) -> (px,py) = (panel_height-1 - ly_abs, lx_abs)
                px_abs = (_config.height - 1) - ly_abs;
                py_abs = lx_abs;
            } else {
                // default fallback
                px_abs = lx_abs;
                py_abs = ly_abs;
            }

            // Convert to buffer-relative coordinates using computed physical_area origin
            int px_rel = px_abs - epd_x;
            int py_rel = py_abs - epd_y;

            if (px_rel < 0 || px_rel >= epd_w || py_rel < 0 || py_rel >= epd_h) {
                continue; // outside this packed rectangle; skip
            }

            const int byte_idx = py_rel * epd_row_bytes + (px_rel / 8);
            const int bit_pos = 7 - (px_rel & 7);

            if (is_white) {
                packed[byte_idx] |= (1 << bit_pos);
            } else {
                packed[byte_idx] &= ~(1 << bit_pos);
            }
        }
    }

    if (s_flush_debug_count < 8) {
        const int dump_bytes = std::min<int>((int)packed_size, 16);
        char buf_hex[16 * 3 + 1];
        char *p = buf_hex;
        for (int i = 0; i < dump_bytes; ++i) {
            int n = sprintf(p, "%02X ", packed[i]);
            p += n;
        }
        *p = '\0';
        ESP_LOGI(TAG, "packed[%d]= %s", dump_bytes, buf_hex);
        ++s_flush_debug_count;
    }

    // Enqueue the packed buffer for the worker to consume (batch + refresh)
    if (self->_queue) {
        QueueItem qi;
        qi.buf = packed;
        qi.x = (uint16_t)epd_x;
        qi.y = (uint16_t)epd_y;
        qi.w = (uint16_t)epd_w;
        qi.h = (uint16_t)epd_h;
        if (xQueueSend(self->_queue, &qi, pdMS_TO_TICKS(50)) != pdTRUE) {
            // Queue full or send failed - fallback to direct write (safest)
            ESP_LOGW(TAG, "Display queue full; performing direct write");
            if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, pdMS_TO_TICKS(500));
            self->_display->writeImage(packed, epd_x, epd_y, epd_w, epd_h, false, false, false);
            self->_display->refresh(true);
            if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);
            heap_caps_free(packed);
        } else {
            // queued successfully; worker will free the packed buffer
        }
    } else {
        // No worker created: fallback to direct write + immediate refresh
        ESP_LOGW(TAG, "No display worker - performing direct write");
        if (self->_spiMutex) xSemaphoreTake(self->_spiMutex, pdMS_TO_TICKS(500));
        self->_display->writeImage(packed, epd_x, epd_y, epd_w, epd_h, false, false, false);
        self->_display->refresh(true);
        if (self->_spiMutex) xSemaphoreGive(self->_spiMutex);
        heap_caps_free(packed);
    }

    lv_display_flush_ready(disp);
}

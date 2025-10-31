#include "GxEPD2Display.h"
#include "GxEPD2/GxEPD2_290_GDEY029T71H.h"
#include "DisplayTester.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>

static const char* TAG = "GxEPD2Display";

GxEPD2Display::GxEPD2Display(const Configuration& config)
    : _config(config)
    , _display(nullptr)
    , _lvglDisplay(nullptr)
    , _drawBuf1(nullptr)
    , _drawBuf2(nullptr)
    , rotation_(config.rotation)
{
}

GxEPD2Display::~GxEPD2Display() {
    stop();
    stopLvgl();
}

std::string GxEPD2Display::getName() const { return "GxEPD2"; }
std::string GxEPD2Display::getDescription() const { return "E-paper display GDEY029T71H"; }

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
        .queue_size = 7,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };

    _display->selectSPI(_config.spiHost, devcfg);

    ESP_LOGI(TAG, "Initializing display hardware...");
    _display->init(0);

    ESP_LOGI(TAG, "Clearing screen...");
    _display->clearScreen(0xFF);
    _display->refresh(false);

    // Run tests once to verify hardware works
    display_tester::runTests(this);

    ESP_LOGI(TAG, "E-paper display started successfully");
    return true;
}

bool GxEPD2Display::stop() {
    if (_display) {
        ESP_LOGI(TAG, "Stopping display...");
        _display->hibernate();
        _display.reset();
    }
    return true;
}

std::shared_ptr<tt::hal::touch::TouchDevice> GxEPD2Display::getTouchDevice() { return nullptr; }
bool GxEPD2Display::supportsLvgl() const { return true; }

bool GxEPD2Display::startLvgl() {
    if (_lvglDisplay != nullptr) {
        ESP_LOGW(TAG, "LVGL already started");
        return false;
    }
    if (!_display) {
        ESP_LOGE(TAG, "Display not started, cannot start LVGL");
        return false;
    }
    ESP_LOGI(TAG, "Starting LVGL integration...");

    // Calculate LVGL dimensions based on rotation
    uint16_t lv_width, lv_height;
    if (rotation_ == 0) {
        // No rotation: use physical dimensions
        lv_width = _config.width;
        lv_height = _config.height;
    } else {
        // Rotated 90°: swap dimensions
        lv_width = _config.height;
        lv_height = _config.width;
    }

    ESP_LOGI(TAG, "Physical: %ux%u, LVGL: %ux%u, rotation=%u", 
             _config.width, _config.height, lv_width, lv_height, rotation_);

    const size_t bufSize = lv_width * DRAW_BUF_LINES;

    _drawBuf1 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DMA);
    _drawBuf2 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!_drawBuf1 || !_drawBuf2) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffers");
        if (_drawBuf1) heap_caps_free(_drawBuf1);
        if (_drawBuf2) heap_caps_free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "Allocated draw buffers: %zu bytes each", bufSize * sizeof(lv_color_t));

    // Create LVGL display with logical dimensions
    _lvglDisplay = lv_display_create(lv_width, lv_height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        heap_caps_free(_drawBuf1);
        heap_caps_free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        return false;
    }

    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_L8);
    lv_display_set_buffers(_lvglDisplay, _drawBuf1, _drawBuf2, bufSize * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    ESP_LOGI(TAG, "LVGL integration started successfully");
    return true;
}

bool GxEPD2Display::stopLvgl() {
    if (_lvglDisplay) {
        ESP_LOGI(TAG, "Stopping LVGL integration...");
        lv_display_delete(_lvglDisplay);
        _lvglDisplay = nullptr;
    }
    if (_drawBuf1) { heap_caps_free(_drawBuf1); _drawBuf1 = nullptr; }
    if (_drawBuf2) { heap_caps_free(_drawBuf2); _drawBuf2 = nullptr; }
    return true;
}

lv_display_t* GxEPD2Display::getLvglDisplay() const { return _lvglDisplay; }
bool GxEPD2Display::supportsDisplayDriver() const { return false; }
std::shared_ptr<tt::hal::display::DisplayDriver> GxEPD2Display::getDisplayDriver() { return nullptr; }

uint16_t GxEPD2Display::getWidth() const { return _config.width; }
uint16_t GxEPD2Display::getHeight() const { return _config.height; }

void GxEPD2Display::writeRawImage(const uint8_t* bitmap, int16_t x, int16_t y, int16_t w, int16_t h, bool invert, bool mirror_y) {
    if (!_display) return;
    _display->writeImage(bitmap, x, y, w, h, invert, mirror_y, false);
}

void GxEPD2Display::refreshDisplay(bool partial) {
    if (!_display) return;
    _display->refresh(partial);
}

// Bit manipulation helpers
static inline bool read_bit(const uint8_t* buf, int row_bytes, int x, int y) {
    int byte_idx = y * row_bytes + (x / 8);
    int bit = 7 - (x & 7);
    return ((buf[byte_idx] >> bit) & 1) != 0;
}

static inline void write_bit(uint8_t* buf, int row_bytes, int x, int y, bool v) {
    int byte_idx = y * row_bytes + (x / 8);
    int bit = 7 - (x & 7);
    if (v) buf[byte_idx] |= (1 << bit);
    else buf[byte_idx] &= ~(1 << bit);
}

void GxEPD2Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
    if (!self || !self->_display) { 
        lv_display_flush_ready(disp); 
        return; 
    }

    int src_x = area->x1;
    int src_y = area->y1;
    int src_w = area->x2 - area->x1 + 1;
    int src_h = area->y2 - area->y1 + 1;
    int src_row_bytes = (src_w + 7) / 8;

    if (src_w <= 0 || src_h <= 0) { 
        lv_display_flush_ready(disp); 
        return; 
    }

    // No rotation - direct write
    if (self->rotation_ == 0) {
        ESP_LOGD(TAG, "Direct write [%d,%d %dx%d]", src_x, src_y, src_w, src_h);
        self->_display->writeImage(px_map, src_x, src_y, src_w, src_h, false, false, false);
        self->_display->refresh(true);
        lv_display_flush_ready(disp);
        return;
    }

    // Rotation required
    int dst_w, dst_h, epd_x, epd_y;
    
    if (self->rotation_ == 2) {
        // 90° CW: LVGL (384x168) -> EPD (168x384)
        dst_w = src_h;
        dst_h = src_w;
        // Map LVGL coordinates to EPD coordinates
        epd_x = self->_config.height - (src_y + src_h);
        epd_y = src_x;
    } else {
        // 90° CCW
        dst_w = src_h;
        dst_h = src_w;
        epd_x = src_y;
        epd_y = self->_config.height - (src_x + src_w);
    }

    int dst_row_bytes = (dst_w + 7) / 8;
    uint8_t* rotated = (uint8_t*)heap_caps_malloc(dst_row_bytes * dst_h, MALLOC_CAP_DMA);
    if (!rotated) {
        ESP_LOGE(TAG, "Failed to allocate rotation buffer");
        lv_display_flush_ready(disp);
        return;
    }
    memset(rotated, 0, dst_row_bytes * dst_h);

    // Rotate the bitmap
    for (int y = 0; y < src_h; ++y) {
        for (int x = 0; x < src_w; ++x) {
            bool pixel = read_bit(px_map, src_row_bytes, x, y);
            int rx, ry;
            if (self->rotation_ == 2) {
                // 90° CW
                rx = dst_w - 1 - y;
                ry = x;
            } else {
                // 90° CCW
                rx = y;
                ry = dst_h - 1 - x;
            }
            write_bit(rotated, dst_row_bytes, rx, ry, pixel);
        }
    }

    // Bounds check
    if (epd_x < 0) epd_x = 0;
    if (epd_y < 0) epd_y = 0;
    if (epd_x + dst_w > (int)self->_config.width) dst_w = self->_config.width - epd_x;
    if (epd_y + dst_h > (int)self->_config.height) dst_h = self->_config.height - epd_y;

    ESP_LOGD(TAG, "Rotated write [%d,%d %dx%d]", epd_x, epd_y, dst_w, dst_h);

    self->_display->writeImage(rotated, epd_x, epd_y, dst_w, dst_h, false, false, false);
    
    // Only refresh on large updates to reduce flickering
    bool is_large = (src_w * src_h) > (384 * 168 / 4);
    if (is_large) {
        self->_display->refresh(true);
    }

    heap_caps_free(rotated);
    lv_display_flush_ready(disp);
}

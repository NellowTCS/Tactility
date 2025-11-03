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
{
}

GxEPD2Display::~GxEPD2Display() {
    stopLvgl();
    stop();
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
        .queue_size = 7,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };

    _display->selectSPI(_config.spiHost, devcfg);
    _display->init(0);
    _display->clearScreen(0xFF);
    _display->refresh(false);

    // Run hardware tests once
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

std::shared_ptr<tt::hal::touch::TouchDevice> GxEPD2Display::getTouchDevice() { 
    return nullptr; 
}

bool GxEPD2Display::supportsLvgl() const { 
    return true; 
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

    // Calculate LVGL dimensions based on rotation
    uint16_t lv_width = _config.width;
    uint16_t lv_height = _config.height;
    if (_config.rotation == 2) {
        // 90° CW rotation: swap dimensions
        lv_width = _config.height;
        lv_height = _config.width;
    }

    ESP_LOGI(TAG, "Starting LVGL: physical=%ux%u, logical=%ux%u, rotation=%u", 
             _config.width, _config.height, lv_width, lv_height, _config.rotation);

    // Allocate draw buffers (RGB565 format - 2 bytes per pixel)
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

    ESP_LOGI(TAG, "Allocated %zu bytes per buffer", bufSize * sizeof(lv_color_t));

    // Create LVGL display
    _lvglDisplay = lv_display_create(lv_width, lv_height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        heap_caps_free(_drawBuf1);
        heap_caps_free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        return false;
    }

    // Use RGB565 format - standard LVGL rendering
    // We convert to 1-bit in the flush callback (like SSD1306 approach)
    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(_lvglDisplay, _drawBuf1, _drawBuf2, 
                           bufSize * sizeof(lv_color_t), 
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    ESP_LOGI(TAG, "LVGL started successfully (RGB565 -> 1-bit monochrome)");
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
    _display->writeImage(bitmap, x, y, w, h, invert, mirror_y, false);
}

void GxEPD2Display::refreshDisplay(bool partial) {
    if (!_display) return;
    _display->refresh(partial);
}

// Helper: Convert RGB565 to monochrome using brightness threshold
inline bool GxEPD2Display::rgb565ToMono(lv_color_t pixel) {
    // Extract RGB components from RGB565
    // RGB565 layout: RRRRRGGG GGGBBBBB
    uint8_t r = pixel.red;   // Already 8-bit in LVGL9
    uint8_t g = pixel.green; // Already 8-bit in LVGL9
    uint8_t b = pixel.blue;  // Already 8-bit in LVGL9
    
    // Calculate brightness using standard weights
    uint8_t brightness = (r * 77 + g * 151 + b * 28) >> 8;
    
    // Threshold: >127 = white, <=127 = black
    return brightness > 127;
}

void GxEPD2Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
    if (!self || !self->_display) { 
        lv_display_flush_ready(disp); 
        return; 
    }

    const int src_w = area->x2 - area->x1 + 1;
    const int src_h = area->y2 - area->y1 + 1;

    if (src_w <= 0 || src_h <= 0) { 
        lv_display_flush_ready(disp); 
        return; 
    }

    ESP_LOGD(TAG, "Flush: [%d,%d] %dx%d", area->x1, area->y1, src_w, src_h);

    // Calculate destination based on rotation
    int dst_w, dst_h, epd_x, epd_y;
    
    if (self->_config.rotation == 0) {
        // No rotation
        dst_w = src_w;
        dst_h = src_h;
        epd_x = area->x1;
        epd_y = area->y1;
    } else if (self->_config.rotation == 2) {
        // 90° CW: LVGL (384x168) -> EPD (168x384)
        dst_w = src_h;
        dst_h = src_w;
        epd_x = self->_config.height - (area->y1 + src_h);
        epd_y = area->x1;
    } else {
        ESP_LOGE(TAG, "Unsupported rotation: %u", self->_config.rotation);
        lv_display_flush_ready(disp);
        return;
    }

    // Allocate 1-bit packed buffer
    const int dst_row_bytes = (dst_w + 7) / 8;
    const size_t packed_size = dst_row_bytes * dst_h;
    uint8_t* packed = (uint8_t*)heap_caps_malloc(packed_size, MALLOC_CAP_DMA);
    if (!packed) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes", packed_size);
        lv_display_flush_ready(disp);
        return;
    }
    memset(packed, 0xFF, packed_size); // White background

    // Convert RGB565 -> 1-bit with rotation
    lv_color_t* src_pixels = (lv_color_t*)px_map;

    for (int y = 0; y < src_h; ++y) {
        for (int x = 0; x < src_w; ++x) {
            lv_color_t pixel = src_pixels[y * src_w + x];
            bool is_white = rgb565ToMono(pixel);

            // Apply rotation
            int dx, dy;
            if (self->_config.rotation == 0) {
                dx = x;
                dy = y;
            } else { // rotation == 2
                dx = dst_w - 1 - y;
                dy = x;
            }

            // Pack bit (MSB first per row)
            const int byte_idx = dy * dst_row_bytes + (dx / 8);
            const int bit = 7 - (dx & 7);
            if (is_white) {
                packed[byte_idx] |= (1 << bit);
            } else {
                packed[byte_idx] &= ~(1 << bit);
            }
        }
    }

    // Bounds check
    if (epd_x < 0) epd_x = 0;
    if (epd_y < 0) epd_y = 0;
    if (epd_x + dst_w > (int)self->_config.width) {
        dst_w = self->_config.width - epd_x;
    }
    if (epd_y + dst_h > (int)self->_config.height) {
        dst_h = self->_config.height - epd_y;
    }

    // Write to display and refresh
    self->_display->writeImage(packed, epd_x, epd_y, dst_w, dst_h, false, false, false);
    self->_display->refresh(true); // Partial refresh (~400ms)

    heap_caps_free(packed);
    lv_display_flush_ready(disp);
}

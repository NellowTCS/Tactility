#include "GxEPD2Display.h"
#include "GxEPD2/GxEPD2_290_GDEY029T71H.h"
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
    stop();
    stopLvgl();
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

    ESP_LOGI(TAG, "Initializing display hardware...");
    _display->init(0);

    ESP_LOGI(TAG, "Clearing screen...");
    _display->clearScreen(0xFF);
    _display->refresh(false);

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

    ESP_LOGI(TAG, "Starting LVGL integration...");

    // Keep using LVGL with the physical EPD orientation by default (no internal swap)
    // The flush callback below will handle rotation if needed.
    const uint16_t lv_width = _config.width;
    const uint16_t lv_height = _config.height;

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

    _lvglDisplay = lv_display_create(lv_width, lv_height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        heap_caps_free(_drawBuf1);
        heap_caps_free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        return false;
    }

    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_I1);

    lv_display_set_buffers(
        _lvglDisplay,
        _drawBuf1,
        _drawBuf2,
        bufSize * sizeof(lv_color_t),
        LV_DISPLAY_RENDER_MODE_PARTIAL
    );

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

/*
Rotation handling modes:
 - ROTATE_NONE: write LVGL px_map directly
 - ROTATE_90_CW: rotate 90 degrees clockwise
 - ROTATE_90_CCW: rotate 90 degrees counter-clockwise
*/
#define ROTATE_NONE    0
#define ROTATE_90_CCW  1
#define ROTATE_90_CW   2

#ifndef EPD_ROTATION_MODE
#define EPD_ROTATION_MODE ROTATE_90_CW
#endif

// Read a single pixel from a packed 1bpp buffer (MSB-first per byte)
static inline bool read_bit_msbf(const uint8_t* buf, int buf_row_bytes, int x, int y) {
    int byte_index = y * buf_row_bytes + (x / 8);
    uint8_t b = buf[byte_index];
    int bit = 7 - (x & 7);
    return ((b >> bit) & 0x1) != 0;
}

// Write a single pixel to a packed 1bpp buffer (MSB-first per byte)
static inline void write_bit_msbf(uint8_t* buf, int buf_row_bytes, int x, int y, bool v) {
    int byte_index = y * buf_row_bytes + (x / 8);
    int bit = 7 - (x & 7);
    if (v) buf[byte_index] |= (1 << bit);
    else buf[byte_index] &= ~(1 << bit);
}

// Helper to align x to byte boundary for EPD constraints
static inline int align_x_to_byte(int x) {
    return x & ~7; // floor to multiple of 8
}

// Flush callback with better rotation & packing
void GxEPD2Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
    if (!self || !self->_display) {
        ESP_LOGE(TAG, "Invalid display in flush callback");
        lv_display_flush_ready(disp);
        return;
    }

    // LVGL area in pixels
    int src_x = area->x1;
    int src_y = area->y1;
    int src_w = area->x2 - area->x1 + 1;
    int src_h = area->y2 - area->y1 + 1;

    // Row bytes in source (LVGL provides packed 1bpp rows)
    int src_row_bytes = (src_w + 7) / 8; // note: LVGL might supply per-line packing; we compute per-area

    ESP_LOGD(TAG, "Flush request: LVGL area [%d,%d %dx%d] (src_row_bytes=%d)", src_x, src_y, src_w, src_h, src_row_bytes);

    // If no rotation is requested, write directly (but ensure x is byte aligned)
    if (EPD_ROTATION_MODE == ROTATE_NONE) {
        int epd_x = align_x_to_byte(src_x);
        int epd_y = src_y;
        int write_w = src_w + (src_x - epd_x); // expand width if we moved x left to align
        // pad write_w to byte boundary
        int write_w_padded = ((write_w + 7) / 8) * 8;

        if (epd_x < 0) epd_x = 0;
        if (epd_y < 0) epd_y = 0;
        if (epd_x + write_w_padded > (int)self->_config.width) write_w_padded = self->_config.width - epd_x;
        if (epd_y + src_h > (int)self->_config.height) src_h = self->_config.height - epd_y;

        // If LVGL px_map already contains the full row-by-row bytes for this region, we can write directly.
        // However px_map is tightly packed for src_w; we may need to expand each row to include byte alignment.
        int dst_row_bytes = (write_w_padded + 7) / 8;
        uint8_t* tmp = (uint8_t*)heap_caps_malloc(dst_row_bytes * src_h, MALLOC_CAP_DMA);
        if (!tmp) {
            ESP_LOGE(TAG, "Failed to allocate tmp buffer for direct write");
            lv_display_flush_ready(disp);
            return;
        }
        // Clear tmp
        memset(tmp, 0, dst_row_bytes * src_h);
        // Copy row by row, shifting bits into position
        for (int row = 0; row < src_h; ++row) {
            for (int x = 0; x < src_w; ++x) {
                bool px = read_bit_msbf(px_map, src_row_bytes, x, row);
                int dst_x = x + (src_x - epd_x); // account for left shift when aligning
                write_bit_msbf(tmp, dst_row_bytes, dst_x, row, px);
            }
        }

        self->_display->writeImage(tmp, epd_x, epd_y, write_w_padded, src_h, false, false, false);
        // Refresh decision: keep same heuristic
        bool is_large_update = (src_w * src_h) > (384 * 168 / 4);
        if (is_large_update) {
            ESP_LOGI(TAG, "Large update (no-rotate), refreshing");
            self->_display->refresh(true);
        }
        heap_caps_free(tmp);
        lv_display_flush_ready(disp);
        return;
    }

    // For rotation we will build a rotated bitmap with dst size = src_h x src_w
    int dst_w = src_h;
    int dst_h = src_w;
    int dst_row_bytes = (dst_w + 7) / 8;
    int dst_bytes = dst_row_bytes * dst_h;

    uint8_t* rotated = (uint8_t*)heap_caps_malloc(dst_bytes, MALLOC_CAP_DMA);
    if (!rotated) {
        ESP_LOGE(TAG, "Failed to allocate rotated bitmap");
        lv_display_flush_ready(disp);
        return;
    }
    memset(rotated, 0, dst_bytes);

    // For reading source px_map we need source row bytes relative to area width.
    // LVGL px_map is organized tightly for the area width; treat it as src_row_bytes per row
    int src_bytes_per_row = (src_w + 7) / 8;

    // Fill rotated buffer per chosen rotation mode
#if (EPD_ROTATION_MODE == ROTATE_90_CCW)
    // CCW: (x,y) -> (y, dst_h - 1 - x)
    for (int y = 0; y < src_h; ++y) {
        for (int x = 0; x < src_w; ++x) {
            bool px = read_bit_msbf(px_map, src_bytes_per_row, x, y);
            int dx = y;
            int dy = dst_h - 1 - x;
            write_bit_msbf(rotated, dst_row_bytes, dx, dy, px);
        }
    }
#elif (EPD_ROTATION_MODE == ROTATE_90_CW)
    // CW: (x,y) -> (dst_w - 1 - y, x)
    for (int y = 0; y < src_h; ++y) {
        for (int x = 0; x < src_w; ++x) {
            bool px = read_bit_msbf(px_map, src_bytes_per_row, x, y);
            int dx = dst_w - 1 - y;
            int dy = x;
            write_bit_msbf(rotated, dst_row_bytes, dx, dy, px);
        }
    }
#else
#error "Invalid EPD_ROTATION_MODE"
#endif

    // Decide EPD coordinates for writing rotated bitmap.
    // We compute epd_x/epd_y by mapping the LVGL area top-left to EPD top-left depending on rotation.
    int epd_x = 0;
    int epd_y = 0;

#if (EPD_ROTATION_MODE == ROTATE_90_CCW)
    // Original mapping used earlier: epd_x = area->y1; epd_y = height - (area->x1 + src_w)
    epd_x = src_y;
    epd_y = self->_config.height - (src_x + src_w);
#elif (EPD_ROTATION_MODE == ROTATE_90_CW)
    // For CW mapping: epd_x = height - (area->y1 + src_h); epd_y = area->x1
    epd_x = self->_config.height - (src_y + src_h);
    epd_y = src_x;
#endif

    // Align epd_x to byte boundary expected by EPD driver
    int aligned_x = align_x_to_byte(epd_x);
    int x_shift = epd_x - aligned_x; // how many pixels we need to shift right in the dst buffer
    int write_w = dst_w + x_shift;
    int write_w_padded = ((write_w + 7) / 8) * 8;

    // Clip to EPD bounds
    if (aligned_x < 0) aligned_x = 0;
    if (epd_y < 0) epd_y = 0;
    if (aligned_x + write_w_padded > (int)self->_config.width) {
        write_w_padded = self->_config.width - aligned_x;
    }
    if (epd_y + dst_h > (int)self->_config.height) {
        dst_h = self->_config.height - epd_y;
    }

    // If x_shift == 0 we can write rotated directly. Otherwise we need to shift rows into a tmp buffer.
    uint8_t* write_buf = rotated;
    uint8_t* tmp_buf = nullptr;

    if (x_shift != 0 || write_w_padded != dst_row_bytes * dst_h) {
        int dst_bytes_aligned = ((write_w_padded + 7) / 8) * dst_h;
        tmp_buf = (uint8_t*)heap_caps_malloc(dst_bytes_aligned, MALLOC_CAP_DMA);
        if (!tmp_buf) {
            ESP_LOGE(TAG, "Failed to allocate tmp shift buffer");
            heap_caps_free(rotated);
            lv_display_flush_ready(disp);
            return;
        }
        memset(tmp_buf, 0, dst_bytes_aligned);
        int tmp_row_bytes = (write_w_padded + 7) / 8;
        // shift each row by x_shift to the right (positive x_shift means rotated contents must move right)
        for (int row = 0; row < dst_h; ++row) {
            for (int x = 0; x < dst_w; ++x) {
                bool px = read_bit_msbf(rotated, dst_row_bytes, x, row);
                write_bit_msbf(tmp_buf, tmp_row_bytes, x + x_shift, row, px);
            }
        }
        write_buf = tmp_buf;
    }

    ESP_LOGD(TAG, "Writing rotated bitmap to EPD @%d,%d size %dx%d (aligned_x=%d, x_shift=%d)",
             aligned_x, epd_y, write_w_padded, dst_h, aligned_x, x_shift);

    self->_display->writeImage(write_buf, aligned_x, epd_y, write_w_padded, dst_h, false, false, false);

    // Refresh heuristic
    bool is_large_update = (src_w * src_h) > (384 * 168 / 4);
    if (is_large_update) {
        ESP_LOGI(TAG, "Large rotated update, refreshing display");
        self->_display->refresh(true);
    }

    if (tmp_buf) heap_caps_free(tmp_buf);
    heap_caps_free(rotated);
    lv_display_flush_ready(disp);
}

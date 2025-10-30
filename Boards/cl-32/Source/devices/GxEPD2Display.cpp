#include "GxEPD2Display.h"
#include "GxEPD2/GxEPD2_290_GDEY029T71H.h"
#include "DisplayTester.h"
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include <cstdio>

static const char* TAG = "GxEPD2Display";

GxEPD2Display::GxEPD2Display(const Configuration& config)
    : _config(config)
    , _display(nullptr)
    , _lvglDisplay(nullptr)
    , _drawBuf1(nullptr)
    , _drawBuf2(nullptr)
    , rotation_(0)
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

    // run tests once (blocking) to observe results; remove/disable when done
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

    const uint16_t lv_width = _config.width;
    const uint16_t lv_height = _config.height;

    ESP_LOGI(TAG, "EPD config width=%u height=%u, LVGL width=%u height=%u", _config.width, _config.height, lv_width, lv_height);

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

    _lvglDisplay = lv_display_create(lv_width, lv_height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        heap_caps_free(_drawBuf1);
        heap_caps_free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        return false;
    }

    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_I1);
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

// Helpers for bit access (MSB-first)
static inline bool read_bit_msbf(const uint8_t* buf, int row_bytes, int x, int y) {
    int byte_index = y * row_bytes + (x / 8);
    uint8_t b = buf[byte_index];
    int bit = 7 - (x & 7);
    return ((b >> bit) & 0x1) != 0;
}
static inline void write_bit_msbf(uint8_t* buf, int row_bytes, int x, int y, bool v) {
    int byte_index = y * row_bytes + (x / 8);
    int bit = 7 - (x & 7);
    if (v) buf[byte_index] |= (1 << bit);
    else buf[byte_index] &= ~(1 << bit);
}
static inline int align_x_to_byte(int x) { return x & ~7; }

void GxEPD2Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
    if (!self || !self->_display) { lv_display_flush_ready(disp); return; }

    int src_x = area->x1;
    int src_y = area->y1;
    int src_w = area->x2 - area->x1 + 1;
    int src_h = area->y2 - area->y1 + 1;
    int src_row_bytes = (src_w + 7) / 8;

    ESP_LOGD(TAG, "Flush: LVGL area [%d,%d %dx%d] rot=%u row_bytes=%d", src_x, src_y, src_w, src_h, self->rotation_, src_row_bytes);

    // quick sanity
    if (src_w <= 0 || src_h <= 0) { lv_display_flush_ready(disp); return; }

    // If rotation is none, perform the aligned direct copy path (fast, fewer allocations)
    if (self->rotation_ == 0) {
        int epd_x = align_x_to_byte(src_x);
        int epd_y = src_y;
        int left_shift = src_x - epd_x;
        int write_w = src_w + left_shift;
        int write_w_padded = ((write_w + 7) / 8) * 8;
        if (epd_x < 0) epd_x = 0;
        if (epd_y < 0) epd_y = 0;
        if (epd_x + write_w_padded > (int)self->_config.width) write_w_padded = self->_config.width - epd_x;
        if (epd_y + src_h > (int)self->_config.height) src_h = self->_config.height - epd_y;

        int dst_row_bytes = (write_w_padded + 7) / 8;
        uint8_t* tmp = (uint8_t*)heap_caps_malloc(dst_row_bytes * src_h, MALLOC_CAP_DMA);
        if (!tmp) { lv_display_flush_ready(disp); return; }
        memset(tmp, 0xFF, dst_row_bytes * src_h);

        for (int row = 0; row < src_h; ++row) {
            for (int x = 0; x < src_w; ++x) {
                int src_byte = row * src_row_bytes + (x / 8);
                int src_bit = 7 - (x & 7);
                bool px = ((px_map[src_byte] >> src_bit) & 1) != 0;
                int dst_x = left_shift + x;
                int dst_byte_index = row * dst_row_bytes + (dst_x / 8);
                int dst_bit = 7 - (dst_x & 7);
                if (!px) tmp[dst_byte_index] &= ~(1 << dst_bit);
                else tmp[dst_byte_index] |= (1 << dst_bit);
            }
        }

        ESP_LOGD(TAG, "Direct write @%d,%d %dx%d (padded_w=%d)", epd_x, epd_y, write_w_padded, src_h, write_w_padded);
        self->_display->writeImage(tmp, epd_x, epd_y, write_w_padded, src_h, false, false, false);
        self->_display->refresh(true);
        heap_caps_free(tmp);
        lv_display_flush_ready(disp);
        return;
    }

    // For rotations 90° CCW or CW: allocate rotated buffer dst = src_h x src_w
    int dst_w = src_h;
    int dst_h = src_w;
    int dst_row_bytes = (dst_w + 7) / 8;
    uint8_t* rotated = (uint8_t*)heap_caps_malloc(dst_row_bytes * dst_h, MALLOC_CAP_DMA);
    if (!rotated) { lv_display_flush_ready(disp); return; }
    memset(rotated, 0, dst_row_bytes * dst_h);

    // copy/rotate into rotated buffer
    int src_bytes_per_row = src_row_bytes;
    if (self->rotation_ == 1) {
        // CCW: (x,y) --> (y, dst_h - 1 - x)
        for (int y = 0; y < src_h; ++y)
            for (int x = 0; x < src_w; ++x)
                write_bit_msbf(rotated, dst_row_bytes, y, dst_h - 1 - x, read_bit_msbf(px_map, src_bytes_per_row, x, y));
    } else { // rotation_ == 2 -> CW
        // CW: (x,y) --> (dst_w - 1 - y, x)
        for (int y = 0; y < src_h; ++y)
            for (int x = 0; x < src_w; ++x)
                write_bit_msbf(rotated, dst_row_bytes, dst_w - 1 - y, x, read_bit_msbf(px_map, src_bytes_per_row, x, y));
    }

    // compute epd_x/epd_y from LVGL area and rotation
    int epd_x = 0, epd_y = 0;
    if (self->rotation_ == 1) { // CCW
        epd_x = src_y;
        epd_y = self->_config.height - (src_x + src_w);
    } else { // CW
        epd_x = self->_config.height - (src_y + src_h);
        epd_y = src_x;
    }

    int aligned_x = align_x_to_byte(epd_x);
    int x_shift = epd_x - aligned_x;
    int write_w = dst_w + x_shift;
    int write_w_padded = ((write_w + 7) / 8) * 8;
    if (aligned_x < 0) aligned_x = 0;
    if (epd_y < 0) epd_y = 0;
    if (aligned_x + write_w_padded > (int)self->_config.width) write_w_padded = self->_config.width - aligned_x;
    if (epd_y + dst_h > (int)self->_config.height) dst_h = self->_config.height - epd_y;

    uint8_t* write_buf = rotated;
    uint8_t* tmp_buf = nullptr;
    if (x_shift != 0 || write_w_padded != dst_row_bytes * dst_h) {
        int dst_bytes_aligned = ((write_w_padded + 7) / 8) * dst_h;
        tmp_buf = (uint8_t*)heap_caps_malloc(dst_bytes_aligned, MALLOC_CAP_DMA);
        if (!tmp_buf) { heap_caps_free(rotated); lv_display_flush_ready(disp); return; }
        memset(tmp_buf, 0, dst_bytes_aligned);
        int tmp_row_bytes = (write_w_padded + 7) / 8;
        for (int row = 0; row < dst_h; ++row)
            for (int x = 0; x < dst_w; ++x)
                if (read_bit_msbf(rotated, dst_row_bytes, x, row))
                    write_bit_msbf(tmp_buf, tmp_row_bytes, x + x_shift, row, true);
        write_buf = tmp_buf;
    }

    ESP_LOGD(TAG, "Writing rotated bitmap to EPD @%d,%d size %dx%d (aligned_x=%d, x_shift=%d)",
             aligned_x, epd_y, write_w_padded, dst_h, aligned_x, x_shift);

    self->_display->writeImage(write_buf, aligned_x, epd_y, write_w_padded, dst_h, false, false, false);

    bool is_large_update = (src_w * src_h) > (384 * 168 / 4);
    if (is_large_update) {
        ESP_LOGI(TAG, "Large rotated update, refreshing display");
        self->_display->refresh(true);
    }

    if (tmp_buf) heap_caps_free(tmp_buf);
    heap_caps_free(rotated);
    lv_display_flush_ready(disp);
}

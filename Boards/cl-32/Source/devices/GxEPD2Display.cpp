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
    
    // Create the GxEPD2 display instance
    _display = std::make_unique<GxEPD2_290_GDEY029T71H>(
        _config.csPin,
        _config.dcPin,
        _config.rstPin,
        _config.busyPin
    );

    // Configure SPI device settings for the display
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 10000000, // 10 MHz for e-paper
        .input_delay_ns = 0,
        .spics_io_num = _config.csPin,
        .flags = 0,
        .queue_size = 7,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };

    // Tell GxEPD2 to use our SPI host
    _display->selectSPI(_config.spiHost, devcfg);

    // Initialize the display (this adds it to the SPI bus)
    ESP_LOGI(TAG, "Initializing display hardware...");
    _display->init(0); // 0 = no serial diagnostics

    // Clear the screen to white
    ESP_LOGI(TAG, "Clearing screen...");
    _display->clearScreen(0xFF); // 0xFF = white
    _display->refresh(false); // Full refresh

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
    return nullptr; // No touch on e-paper
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

    // Calculate buffer size (10 lines worth of the physical width)
    const size_t bufSize = _config.width * DRAW_BUF_LINES;

    // Allocate draw buffers in DMA-capable memory
    _drawBuf1 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DMA);
    _drawBuf2 = (lv_color_t*)heap_caps_malloc(bufSize * sizeof(lv_color_t), MALLOC_CAP_DMA);

    if (!_drawBuf1 || !_drawBuf2) {
        ESP_LOGE(TAG, "Failed to allocate LVGL draw buffers");
        if (_drawBuf1) free(_drawBuf1);
        if (_drawBuf2) free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        return false;
    }

    ESP_LOGI(TAG, "Allocated draw buffers: %zu bytes each", bufSize * sizeof(lv_color_t));

    // Create LVGL display with physical dimensions (168x384)
    // The rotation happens in the flush callback
    _lvglDisplay = lv_display_create(_config.width, _config.height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        free(_drawBuf1);
        free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        return false;
    }

    // Set color format to monochrome (1-bit per pixel)
    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_I1);

    // Set draw buffers
    lv_display_set_buffers(
        _lvglDisplay,
        _drawBuf1,
        _drawBuf2,
        bufSize * sizeof(lv_color_t),
        LV_DISPLAY_RENDER_MODE_PARTIAL
    );

    // Set flush callback
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    // Rotate display 90° to landscape (LVGL will think it's 384x168)
    lv_display_set_rotation(_lvglDisplay, LV_DISPLAY_ROTATION_90);

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
        free(_drawBuf1);
        _drawBuf1 = nullptr;
    }

    if (_drawBuf2) {
        free(_drawBuf2);
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

// Rotate monochrome bitmap 90° CCW (counter-clockwise) for landscape display
// LVGL thinks display is 168x384 (portrait), but with rotation=90 it presents 384x168 (landscape)
// So we need to rotate LVGL's landscape output 90° CCW to match the physical portrait panel
static void rotate_bitmap_90ccw_1bpp(const uint8_t* src, uint8_t* dst, int src_w, int src_h) {
    // After 90° CCW rotation: dst_w = src_h, dst_h = src_w
    const int dst_w = src_h;
    const int dst_h = src_w;
    const int dst_bytes = (dst_w * dst_h + 7) / 8;
    memset(dst, 0, dst_bytes);

    for (int y = 0; y < src_h; ++y) {
        for (int x = 0; x < src_w; ++x) {
            // Read pixel from source
            int src_idx = y * src_w + x;
            int src_byte = src_idx / 8;
            int src_bit = 7 - (src_idx % 8);
            bool pixel = (src[src_byte] >> src_bit) & 0x01;

            // 90° CCW rotation: (x, y) -> (y, dst_h - 1 - x)
            int dst_x = y;
            int dst_y = dst_h - 1 - x;

            // Write pixel to destination
            int dst_idx = dst_y * dst_w + dst_x;
            int dst_byte = dst_idx / 8;
            int dst_bit = 7 - (dst_idx % 8);
            
            if (pixel) {
                dst[dst_byte] |= (1 << dst_bit);
            }
        }
    }
}

void GxEPD2Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
    
    if (!self || !self->_display) {
        ESP_LOGE(TAG, "Invalid display in flush callback");
        lv_display_flush_ready(disp);
        return;
    }

    // Source dimensions (LVGL area in landscape: 384x168)
    int src_w = area->x2 - area->x1 + 1;
    int src_h = area->y2 - area->y1 + 1;

    // Destination dimensions after 90° CCW rotation (portrait: 168x384)
    int dst_w = src_h;
    int dst_h = src_w;
    
    int dst_bytes = (dst_w * dst_h + 7) / 8;
    uint8_t* rotated_bitmap = (uint8_t*)heap_caps_malloc(dst_bytes, MALLOC_CAP_DMA);
    if (!rotated_bitmap) {
        ESP_LOGE(TAG, "Failed to allocate rotated bitmap");
        lv_display_flush_ready(disp);
        return;
    }

    // Rotate the bitmap 90° CCW
    rotate_bitmap_90ccw_1bpp(px_map, rotated_bitmap, src_w, src_h);

    // Calculate the position on the e-paper display
    // LVGL area (landscape 384x168): top-left = (area->x1, area->y1)
    // After 90° CCW rotation to portrait (168x384):
    // top-left (x, y) maps to (y, 384 - (x + w))
    int epd_x = area->y1;
    int epd_y = self->_config.height - (area->x1 + src_w);

    // Make sure coordinates are within bounds
    if (epd_x < 0) epd_x = 0;
    if (epd_y < 0) epd_y = 0;
    if (epd_x + dst_w > (int)self->_config.width) dst_w = self->_config.width - epd_x;
    if (epd_y + dst_h > (int)self->_config.height) dst_h = self->_config.height - epd_y;

    ESP_LOGD(TAG, "Flush: LVGL [%d,%d %dx%d] -> EPD [%d,%d %dx%d]",
             area->x1, area->y1, src_w, src_h, epd_x, epd_y, dst_w, dst_h);

    // Write to e-paper display buffer (don't refresh yet)
    self->_display->writeImage(rotated_bitmap, epd_x, epd_y, dst_w, dst_h, false, false, false);
    
    // Only refresh if this is a full screen update or large area
    // This reduces flickering for small incremental updates
    bool is_large_update = (src_w * src_h) > (384 * 168 / 4); // More than 25% of screen
    if (is_large_update) {
        ESP_LOGI(TAG, "Large update, refreshing display");
        self->_display->refresh(true); // Partial refresh
    }

    heap_caps_free(rotated_bitmap);
    lv_display_flush_ready(disp);
}

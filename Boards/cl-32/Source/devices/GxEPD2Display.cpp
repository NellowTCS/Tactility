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

    // setRotation(3) rotates the panel 90° clockwise so LVGL can render in landscape without manual rotate.
    ESP_LOGI(TAG, "Setting display rotation to 3...");
    _display->setRotation(3);

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

    // If we rotated the hardware (setRotation(3)), swap width/height for LVGL so LVGL draws in the
    // correct orientation and we can write px_map directly to the panel.
    const uint16_t lv_width = _config.height; // swap
    const uint16_t lv_height = _config.width; // swap

    // Calculate buffer size (10 lines worth of the LVGL width)
    const size_t bufSize = lv_width * DRAW_BUF_LINES;

    // Allocate draw buffers in DMA-capable memory
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

    // Create LVGL display using the swapped (rotated) dimensions so LVGL coordinates match panel rotation
    _lvglDisplay = lv_display_create(lv_width, lv_height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        heap_caps_free(_drawBuf1);
        heap_caps_free(_drawBuf2);
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

    // Set flush callback and user data
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    // Do not ask LVGL to rotate; we let the hardware driver handle rotation via setRotation(3)
    // (so we removed lv_display_set_rotation)

    ESP_LOGI(TAG, "LVGL integration started successfully (LV size %ux%u)", lv_width, lv_height);
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

// The rotated hardware means LVGL's px_map should now match the panel orientation.
// We will write the incoming px_map directly to the panel. If the px_map bit-packing
// doesn't match the panel's expectation, we will detect that in testing and fall back.

void GxEPD2Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
    
    if (!self || !self->_display) {
        ESP_LOGE(TAG, "Invalid display in flush callback");
        lv_display_flush_ready(disp);
        return;
    }

    // Source dimensions (LVGL area)
    int src_w = area->x2 - area->x1 + 1;
    int src_h = area->y2 - area->y1 + 1;

    // LVGL is using swapped dimensions to match rotated hardware.
    // So px_map should be in the correct orientation for the panel; write directly.
    // compute bytes for the bitmap (rows padded to byte boundary)
    int row_bytes = (src_w + 7) / 8;
    int src_bytes = row_bytes * src_h;

    ESP_LOGD(TAG, "Flush: LVGL [%d,%d %dx%d] bytes=%d", area->x1, area->y1, src_w, src_h, src_bytes);

    // Coordinates on the EPD: since hardware rotation already applied, area coords map directly
    int epd_x = area->x1;
    int epd_y = area->y1;

    // Bounds check / clip
    if (epd_x < 0) epd_x = 0;
    if (epd_y < 0) epd_y = 0;
    int w = src_w;
    int h = src_h;
    if (epd_x + w > (int)self->_config.width) w = self->_config.width - epd_x;
    if (epd_y + h > (int)self->_config.height) h = self->_config.height - epd_y;

    // Write to display; GxEPD2 expects packed 1bpp with rows padded to byte boundary.
    // If px_map bit-order differs (LSB vs MSB) this may require conversion.
    self->_display->writeImage(px_map, epd_x, epd_y, w, h, false, false, false);

    // Decide refresh strategy - keep existing heuristic
    bool is_large_update = (src_w * src_h) > (384 * 168 / 4); // >25%
    if (is_large_update) {
        ESP_LOGI(TAG, "Large update, refreshing display");
        self->_display->refresh(true); // Partial refresh
    }

    lv_display_flush_ready(disp);
}

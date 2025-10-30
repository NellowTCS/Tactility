#include "GxEPD2Display.h"
#include "GxEPD2/GxEPD2_290_GDEY029T71H.h"
#include <esp_log.h>
#include <esp_heap_caps.h>

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

    // Calculate buffer size (10 lines worth)
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

    // Create LVGL display
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

    // Rotate to landscape (display is 168x384, we want it as 384x168)
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

void GxEPD2Display::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
    auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
    
    if (!self || !self->_display) {
        ESP_LOGE(TAG, "Invalid display in flush callback");
        lv_display_flush_ready(disp);
        return;
    }

    int16_t x = area->x1;
    int16_t y = area->y1;
    int16_t w = area->x2 - area->x1 + 1;
    int16_t h = area->y2 - area->y1 + 1;

    // Write bitmap to display buffer
    self->_display->writeImage(px_map, x, y, w, h, false, false, false);

    // Perform partial refresh
    self->_display->refresh(true); // true = partial update

    lv_display_flush_ready(disp);
}

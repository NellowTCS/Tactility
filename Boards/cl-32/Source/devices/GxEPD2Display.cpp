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

    // --- LVGL Configuration based on Rotation ---
    // Physical display dimensions (portrait: 168x384)
    uint16_t lv_width = _config.width;
    uint16_t lv_height = _config.height;
    lv_display_rotation_t lv_rotation = LV_DISPLAY_ROTATION_0;

    // Map config rotation to LVGL rotation
    // 0 = 0° (portrait), 1 = 90° CW (landscape), 2 = 180° (portrait flipped), 3 = 270° CW (landscape flipped)
    switch (_config.rotation) {
        case 1:
            lv_rotation = LV_DISPLAY_ROTATION_90;  // Landscape
            break;
        case 2:
            lv_rotation = LV_DISPLAY_ROTATION_180; // Portrait upside down
            break;
        case 3:
            lv_rotation = LV_DISPLAY_ROTATION_270; // Landscape flipped
            break;
        default:
            lv_rotation = LV_DISPLAY_ROTATION_0;   // Portrait (default)
            break;
    }

    ESP_LOGI(TAG, "Starting LVGL: physical=%ux%u rotation=%d (config.rotation=%d, sw_rotate=implicit)", 
             lv_width, lv_height, lv_rotation, _config.rotation);

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

    // Create LVGL display with physical dimensions
    _lvglDisplay = lv_display_create(lv_width, lv_height);
    if (!_lvglDisplay) {
        ESP_LOGE(TAG, "Failed to create LVGL display");
        heap_caps_free(_drawBuf1);
        heap_caps_free(_drawBuf2);
        _drawBuf1 = _drawBuf2 = nullptr;
        return false;
    }

    // Set rotation - LVGL will handle dimension swapping and software rotation
    // This is equivalent to esp_lvgl_port with sw_rotate=true
    lv_display_set_rotation(_lvglDisplay, lv_rotation);

    // Use RGB565 format - standard LVGL rendering
    lv_display_set_color_format(_lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(_lvglDisplay, _drawBuf1, _drawBuf2, 
                           bufSize * sizeof(lv_color_t), 
                           LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(_lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(_lvglDisplay, this);

    ESP_LOGI(TAG, "LVGL started successfully");
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
    
    // Transform logical area to physical area based on rotation
    if (rotation == LV_DISPLAY_ROTATION_90) {
        // Logical: 384×168 (landscape) → Physical: 168×384 (portrait)
        // Transformation: (x,y) → (y, width-1-x)
        int32_t hor_res = lv_display_get_horizontal_resolution(disp);  // 384 (logical width)
        physical_area.x1 = area->y1;
        physical_area.y1 = hor_res - 1 - area->x2;
        physical_area.x2 = area->y2;
        physical_area.y2 = hor_res - 1 - area->x1;
    }
    // TODO: Add support for other rotations (180, 270)
    
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
    const size_t packed_size = epd_row_bytes * epd_h;
    uint8_t* packed = (uint8_t*)heap_caps_malloc(packed_size, MALLOC_CAP_DMA);
    if (!packed) {
        ESP_LOGE(TAG, "Failed to allocate %zu bytes for 1-bit buffer", packed_size);
        lv_display_flush_ready(disp);
        return;
    }
    
    // Initialize to 0xFF (all white) - GxEPD2 uses 0xFF=white, 0x00=black
    memset(packed, 0xFF, packed_size); 

    // Convert LVGL pixel buffer -> 1-bit monochrome for e-paper
    // The source buffer is in logical dimensions (after LVGL rendering)
    lv_color_format_t cf = lv_display_get_color_format(disp);
    uint32_t src_row_bytes = (uint32_t)lv_draw_buf_width_to_stride(logical_w, cf);
    uint8_t* src_bytes = (uint8_t*)px_map;

    // Debugging: print a few diagnostics for the first several flushes to detect stride/packing issues
    static int s_flush_debug_count = 0;
    if (s_flush_debug_count < 6) {
        ESP_LOGI(TAG, "lvglFlush: logical=%dx%d physical=%dx%d at (%d,%d) rot=%d cf=%d src_row_bytes=%u", 
                 logical_w, logical_h, epd_w, epd_h, epd_x, epd_y, (int)rotation, (int)cf, (unsigned)src_row_bytes);
    }

    // Convert and rotate pixels from logical to physical orientation
    if (rotation == LV_DISPLAY_ROTATION_90) {
        // 90° CW rotation: logical (384×h) → physical (h×384)
        // For each pixel at logical (lx, ly), it maps to physical (ly, 383-lx)
        for (int ly = 0; ly < logical_h; ++ly) {
            lv_color_t* src_row = (lv_color_t*)(src_bytes + (size_t)ly * src_row_bytes);
            for (int lx = 0; lx < logical_w; ++lx) {
                lv_color_t pixel = src_row[lx];
                bool is_white = self->rgb565ToMono(pixel);

                // Map logical (lx, ly) to physical (px, py)
                int px = ly;
                int py = (logical_w - 1) - lx;

                // Pack bit into physical buffer (MSB first, horizontal addressing)
                const int byte_idx = py * epd_row_bytes + (px / 8);
                const int bit_pos = 7 - (px & 7);  // MSB = leftmost pixel

                if (is_white) {
                    packed[byte_idx] |= (1 << bit_pos);   // Set bit (white)
                } else {
                    packed[byte_idx] &= ~(1 << bit_pos);  // Clear bit (black)
                }
            }
        }
    } else {
        // No rotation (or unhandled rotation) - direct copy
        for (int y = 0; y < epd_h; ++y) {
            lv_color_t* src_row = (lv_color_t*)(src_bytes + (size_t)y * src_row_bytes);
            for (int x = 0; x < epd_w; ++x) {
                lv_color_t pixel = src_row[x];
                bool is_white = self->rgb565ToMono(pixel);

                // Pack bit into byte (MSB first, horizontal addressing)
                const int byte_idx = y * epd_row_bytes + (x / 8);
                const int bit_pos = 7 - (x & 7);  // MSB = leftmost pixel

                if (is_white) {
                    packed[byte_idx] |= (1 << bit_pos);   // Set bit (white)
                } else {
                    packed[byte_idx] &= ~(1 << bit_pos);  // Clear bit (black)
                }
            }
        }
    }

    if (s_flush_debug_count < 6) {
        // Dump first few bytes of packed buffer
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

    // Write the 1-bit buffer to the e-paper display
    self->_display->writeImage(packed, epd_x, epd_y, epd_w, epd_h, false, false, false);
    self->_display->refresh(true); // Partial refresh (~400ms)

    heap_caps_free(packed);
    lv_display_flush_ready(disp);
}

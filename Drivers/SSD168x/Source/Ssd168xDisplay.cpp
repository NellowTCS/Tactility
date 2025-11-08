#include "Ssd168xDisplay.h"

#include <Tactility/Log.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>
#include <cstring>

#define TAG "ssd168x_display"

Ssd168xDisplay::Ssd168xDisplay(std::unique_ptr<Configuration> inConfiguration)
    : configuration(*inConfiguration),
      ssd1680_handle(nullptr),
      lvglDisplay(nullptr),
      frameBuffer(nullptr),
      framebufferMutex(nullptr)
{
    TT_LOG_I(TAG, "Display config: %ux%u", configuration.width, configuration.height);
}

Ssd168xDisplay::~Ssd168xDisplay()
{
    stopLvgl();
    stop();
    
    if (framebufferMutex) {
        vSemaphoreDelete(framebufferMutex);
        framebufferMutex = nullptr;
    }
}

std::string Ssd168xDisplay::getName() const
{
    return "SSD168x";
}

std::string Ssd168xDisplay::getDescription() const
{
    return "SSD168x e-paper display";
}

bool Ssd168xDisplay::start()
{
    TT_LOG_I(TAG, "Starting e-paper display...");

    // Allocate framebuffer
    size_t fb_size = configuration.width * configuration.height / 8;
    uint8_t* fb = (uint8_t*)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM);
    if (!fb) {
        TT_LOG_E(TAG, "Failed to allocate framebuffer (%zu bytes)", fb_size);
        return false;
    }
    memset(fb, 0xFF, fb_size); // White

    // Initialize ssd1680 driver
    ssd1680_config_t cfg = {
        .controller = configuration.controller,
        .rotation = SSD1680_ROT_000,  // We'll handle rotation in LVGL
        .cols = configuration.width,
        .rows = configuration.height,
        .framebuffer = fb,
        .host = configuration.spiHost,
        .cs_pin = configuration.csPin,
        .dc_pin = configuration.dcPin,
        .busy_pin = configuration.busyPin,
        .reset_pin = configuration.resetPin,
    };

    if (ssd1680_init(&cfg, &ssd1680_handle) != ESP_OK) {
        TT_LOG_E(TAG, "ssd1680_init failed");
        heap_caps_free(fb);
        return false;
    }

    frameBuffer = fb;

    // Do initial white fill
    TT_LOG_I(TAG, "Performing initial refresh");
    ssd1680_rect_t rect = {.x = 0, .y = 0, .w = configuration.width, .h = configuration.height};
    ssd1680_begin_frame(ssd1680_handle, SSD1680_REFRESH_FULL);
    ssd1680_flush(ssd1680_handle, rect);
    ssd1680_end_frame(ssd1680_handle);

    framebufferMutex = xSemaphoreCreateMutex();
    if (!framebufferMutex) {
        TT_LOG_W(TAG, "Failed to create framebuffer mutex");
    }

    TT_LOG_I(TAG, "E-paper display started successfully");
    return true;
}

bool Ssd168xDisplay::stop()
{
    if (ssd1680_handle) {
        TT_LOG_I(TAG, "Stopping display...");
        ssd1680_deinit(&ssd1680_handle);
        ssd1680_handle = nullptr;
    }
    if (frameBuffer) {
        heap_caps_free(frameBuffer);
        frameBuffer = nullptr;
    }
    return true;
}

std::shared_ptr<tt::hal::touch::TouchDevice> Ssd168xDisplay::getTouchDevice()
{
    return nullptr;
}

bool Ssd168xDisplay::supportsLvgl() const
{
    return true;
}

bool Ssd168xDisplay::startLvgl()
{
    if (lvglDisplay != nullptr) {
        TT_LOG_W(TAG, "LVGL already started");
        return false;
    }
    if (!ssd1680_handle) {
        TT_LOG_E(TAG, "Display not started, cannot start LVGL");
        return false;
    }

    // LVGL logical resolution (after rotation)
    uint32_t logical_width = configuration.width;
    uint32_t logical_height = configuration.height;
    
    if (configuration.rotation == 1 || configuration.rotation == 3) {
        // Swapped dimensions
        logical_width = configuration.height;
        logical_height = configuration.width;
    }

    uint32_t buffer_size = logical_width * logical_height;

    TT_LOG_I(TAG, "LVGL config: physical=%dx%d logical=%dx%d rotation=%d buffer_size=%lu",
             configuration.width, configuration.height, logical_width, logical_height,
             configuration.rotation, buffer_size);

    // Create LVGL display
    lvglDisplay = lv_display_create(logical_width, logical_height);
    if (!lvglDisplay) {
        TT_LOG_E(TAG, "Failed to create LVGL display");
        return false;
    }

    // Set rotation
    lv_display_rotation_t lv_rotation = LV_DISPLAY_ROTATION_0;
    switch (configuration.rotation) {
        case 1: lv_rotation = LV_DISPLAY_ROTATION_90; break;
        case 2: lv_rotation = LV_DISPLAY_ROTATION_180; break;
        case 3: lv_rotation = LV_DISPLAY_ROTATION_270; break;
    }
    lv_display_set_rotation(lvglDisplay, lv_rotation);

    // Allocate full-screen buffer
    lv_color_t* drawBuf = (lv_color_t*)heap_caps_malloc(buffer_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!drawBuf) {
        TT_LOG_E(TAG, "Failed to allocate LVGL draw buffer");
        lv_display_delete(lvglDisplay);
        lvglDisplay = nullptr;
        return false;
    }

    lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(lvglDisplay, drawBuf, nullptr, buffer_size * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_flush_cb(lvglDisplay, lvglFlushCallback);
    lv_display_set_user_data(lvglDisplay, this);

    TT_LOG_I(TAG, "LVGL started successfully");
    return true;
}

bool Ssd168xDisplay::stopLvgl()
{
    if (lvglDisplay) {
        TT_LOG_I(TAG, "Stopping LVGL...");
        lv_display_delete(lvglDisplay);
        lvglDisplay = nullptr;
    }
    return true;
}

lv_display_t* Ssd168xDisplay::getLvglDisplay() const
{
    return lvglDisplay;
}

bool Ssd168xDisplay::supportsDisplayDriver() const
{
    return false;
}

std::shared_ptr<tt::hal::display::DisplayDriver> Ssd168xDisplay::getDisplayDriver()
{
    return nullptr;
}

// Bayer 4x4 dithering for RGB565 to monochrome
static inline bool bayer4x4Dither(lv_color_t pixel, int x, int y)
{
    uint8_t r = pixel.red;
    uint8_t g = pixel.green;
    uint8_t b = pixel.blue;
    uint8_t brightness = (r * 77 + g * 151 + b * 28) >> 8;

    static const uint8_t bayer4[4][4] = {
        {0, 8, 2, 10},
        {12, 4, 14, 6},
        {3, 11, 1, 9},
        {15, 7, 13, 5}
    };

    uint8_t thresh = (uint8_t)(bayer4[y & 3][x & 3] * 16 + 8);
    return brightness > thresh;
}

void Ssd168xDisplay::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map)
{
    auto* self = static_cast<Ssd168xDisplay*>(lv_display_get_user_data(disp));
    if (!self || !self->ssd1680_handle || !self->frameBuffer) {
        lv_display_flush_ready(disp);
        return;
    }

    int32_t hor_res = lv_display_get_horizontal_resolution(disp);
    int32_t ver_res = lv_display_get_vertical_resolution(disp);
    
    lv_color_format_t cf = lv_display_get_color_format(disp);
    uint32_t src_row_stride_bytes = (uint32_t)lv_draw_buf_width_to_stride(hor_res, cf);
    uint8_t* src_bytes = (uint8_t*)px_map;

    const int panel_width = self->configuration.width;
    const int panel_height = self->configuration.height;
    const int panel_bytes_per_row = (panel_width + 7) / 8;

    if (xSemaphoreTake(self->framebufferMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        TT_LOG_W(TAG, "Framebuffer mutex busy; dropping LVGL flush");
        lv_display_flush_ready(disp);
        return;
    }

    // Dither and pack to 1-bit framebuffer
    for (int ly = 0; ly < ver_res; ++ly) {
        lv_color_t* src_row = (lv_color_t*)(src_bytes + (size_t)ly * src_row_stride_bytes);

        for (int lx = 0; lx < hor_res; ++lx) {
            // Map logical to physical based on rotation
            int physical_x = lx;
            int physical_y = ly;

            lv_display_rotation_t lv_rotation = lv_display_get_rotation(disp);
            switch (lv_rotation) {
                case LV_DISPLAY_ROTATION_90:
                    physical_x = ly;
                    physical_y = (panel_height - 1) - lx;
                    break;
                case LV_DISPLAY_ROTATION_180:
                    physical_x = (panel_width - 1) - lx;
                    physical_y = (panel_height - 1) - ly;
                    break;
                case LV_DISPLAY_ROTATION_270:
                    physical_x = (panel_width - 1) - ly;
                    physical_y = lx;
                    break;
                default:  // 0
                    physical_x = lx;
                    physical_y = ly;
            }

            if (physical_x < 0 || physical_x >= panel_width || physical_y < 0 || physical_y >= panel_height) {
                continue;
            }

            lv_color_t pixel = src_row[lx];
            bool is_white = bayer4x4Dither(pixel, physical_x, physical_y);

            const int byte_idx = physical_y * panel_bytes_per_row + (physical_x / 8);
            const int bit_pos = 7 - (physical_x & 7);

            if (is_white) {
                self->frameBuffer[byte_idx] |= (1 << bit_pos);
            } else {
                self->frameBuffer[byte_idx] &= ~(1 << bit_pos);
            }
        }
    }

    xSemaphoreGive(self->framebufferMutex);

    // Send to display
    ssd1680_rect_t rect = {
        .x = 0,
        .y = 0,
        .w = (uint16_t)panel_width,
        .h = (uint16_t)panel_height
    };

    ssd1680_begin_frame(self->ssd1680_handle, SSD1680_REFRESH_FULL);
    ssd1680_flush(self->ssd1680_handle, rect);
    ssd1680_end_frame(self->ssd1680_handle);

    lv_display_flush_ready(disp);
}

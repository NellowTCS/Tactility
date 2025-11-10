#include "Ssd168xDisplay.h"

#include <Tactility/Log.h>
#include <esp_heap_caps.h>
#include <lvgl.h>
#include <cstring>
#include <algorithm>
#include <mutex>
#include <freertos/semphr.h>

#define TAG "ssd168x_display"

Ssd168xDisplay::Ssd168xDisplay(std::unique_ptr<Configuration> inConfiguration)
    : configuration(*inConfiguration),
      ssd1680_handle(nullptr),
      lvglDisplay(nullptr),
      frameBuffer(nullptr)
{
    TT_LOG_I(TAG, "Display config: %ux%u", configuration.width, configuration.height);
}

Ssd168xDisplay::~Ssd168xDisplay()
{
    stopLvgl();
    stop();
}

std::string Ssd168xDisplay::getName() const
{
    return "SSD168x";
}

std::string Ssd168xDisplay::getDescription() const
{
    return "SSD168x e-paper display";
}

static SemaphoreHandle_t renderSemaphore = nullptr; // Semaphore to synchronize rendering

bool Ssd168xDisplay::start()
{
    TT_LOG_I(TAG, "Starting e-paper display...");

    // Create the render semaphore
    if (renderSemaphore == nullptr) {
        renderSemaphore = xSemaphoreCreateBinary();
        xSemaphoreGive(renderSemaphore); // Initialize the semaphore
    }

    // Validate dimensions against SSD1685 limits
    if (configuration.width > SSD1685_MAX_COLS || configuration.height > SSD1685_MAX_ROWS) {
        TT_LOG_E(TAG, "Invalid dimensions: %ux%u exceed SSD1685 limits (%ux%u)",
                 configuration.width, configuration.height, SSD1685_MAX_COLS, SSD1685_MAX_ROWS);
        return false;
    }

    // Allocate framebuffer
    size_t fb_size = configuration.width * configuration.height / 8;
    uint8_t* fb = (uint8_t*)heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM);
    if (!fb) {
        TT_LOG_E(TAG, "Failed to allocate framebuffer (%zu bytes)", fb_size);
        return false;
    }
    memset(fb, 0xFF, fb_size); // White

    // Initialize ssd1680 driver
    ssd1680_rotation_t hw_rotation = SSD1680_ROT_000;

    ssd1680_config_t cfg = {
        .controller = configuration.controller,
        .rotation = hw_rotation,
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

    // Validate buffer size
    const uint32_t buffer_size = configuration.width * configuration.height;
    if (buffer_size > SSD1685_MAX_COLS * SSD1685_MAX_ROWS) {
        TT_LOG_E(TAG, "Buffer size exceeds maximum allowed dimensions for SSD1685");
        return false;
    }

    TT_LOG_I(TAG, "LVGL config: width=%d height=%d buffer_size=%lu",
             configuration.width, configuration.height, buffer_size);

    // Create LVGL display
    lvglDisplay = lv_display_create(configuration.width, configuration.height);
    if (!lvglDisplay) {
        TT_LOG_E(TAG, "Failed to create LVGL display");
        return false;
    }

    // Allocate full-screen buffer
    lv_color_t* drawBuf = (lv_color_t*)heap_caps_malloc(buffer_size * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (!drawBuf) {
        TT_LOG_E(TAG, "Failed to allocate LVGL draw buffer");
        lv_display_delete(lvglDisplay);
        lvglDisplay = nullptr;
        return false;
    }

    lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(lvglDisplay, drawBuf, nullptr, buffer_size * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_DIRECT);
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

static inline bool isPixelWhite(lv_color_t pixel)
{
    // Simple brightness threshold; RGB565 max sum = 31 + 63 + 31 = 125
    const uint16_t brightness = pixel.red + pixel.green + pixel.blue;
    return brightness > 62;
}

std::mutex renderMutex; // Mutex to prevent concurrent renders

void Ssd168xDisplay::lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map)
{
    auto* self = static_cast<Ssd168xDisplay*>(lv_display_get_user_data(disp));
    if (!self || !self->ssd1680_handle || !self->frameBuffer) {
        lv_display_flush_ready(disp);
        return;
    }

    // Take the semaphore to ensure no concurrent renders
    if (xSemaphoreTake(renderSemaphore, portMAX_DELAY) != pdTRUE) {
        TT_LOG_E(TAG, "Failed to take render semaphore");
        lv_display_flush_ready(disp);
        return;
    }

    TT_LOG_I(TAG, "Flush callback started - area: x1=%d y1=%d x2=%d y2=%d", 
             area->x1, area->y1, area->x2, area->y2);

    const int panel_width = self->configuration.width;
    const int panel_height = self->configuration.height;
    const int panel_bytes_per_row = (panel_width + 7) / 8;

    const int area_width = std::min(area->x2 + 1, panel_width) - std::max(area->x1, 0);
    const int area_height = std::min(area->y2 + 1, panel_height) - std::max(area->y1, 0);
    const lv_color_t* src_pixels = reinterpret_cast<const lv_color_t*>(px_map);

    // Convert RGB565 pixels to 1-bit framebuffer
    for (int row = 0; row < area_height; ++row) {
        const int fy = area->y1 + row;
        if (fy < 0 || fy >= panel_height) {
            src_pixels += area_width;
            continue;
        }

        for (int col = 0; col < area_width; ++col) {
            const int fx = area->x1 + col;
            if (fx < 0 || fx >= panel_width) {
                ++src_pixels;
                continue;
            }

            const lv_color_t pixel = *src_pixels++;
            const bool white = isPixelWhite(pixel);

            const int byte_idx = fy * panel_bytes_per_row + (fx / 8);
            const int bit_pos = 7 - (fx & 7);

            if (white) {
                self->frameBuffer[byte_idx] |= (1 << bit_pos);
            } else {
                self->frameBuffer[byte_idx] &= ~(1 << bit_pos);
            }
        }
    }

    TT_LOG_I(TAG, "Framebuffer updated, starting e-paper refresh");

    // Partial refresh for the specified area
    ssd1680_rect_t rect = {
        .x = static_cast<uint16_t>(std::max(area->x1, 0)),
        .y = static_cast<uint16_t>(std::max(area->y1, 0)),
        .w = static_cast<uint16_t>(area_width),
        .h = static_cast<uint16_t>(area_height)
    };

    // Start the refresh
    ssd1680_begin_frame(self->ssd1680_handle, SSD1680_REFRESH_PARTIAL);
    ssd1680_flush(self->ssd1680_handle, rect);
    ssd1680_end_frame(self->ssd1680_handle);

    // Wait for e-paper to actually finish before telling LVGL we're ready
    TT_LOG_I(TAG, "Waiting for e-paper BUSY to clear...");
    esp_err_t wait_result = ssd1680_wait_until_idle(self->ssd1680_handle);
    if (wait_result != ESP_OK) {
        TT_LOG_E(TAG, "Error waiting for e-paper: %d", wait_result);
    }

    TT_LOG_I(TAG, "E-paper refresh complete, notifying LVGL");

    // Release the semaphore after rendering is complete
    xSemaphoreGive(renderSemaphore);

    // NOW tell LVGL we're done - this blocks the next render
    lv_display_flush_ready(disp);
}

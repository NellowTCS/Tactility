/*
 * Ssd1685Display.cpp
 *
 * Tactility HAL driver for SSD1685 / SSD168x e-paper panels.
 */

#include "Ssd1685Display.h"

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <tactility/check.h>
#include <tactility/log.h>
#include <lvgl.h>
#include <cstring>

static constexpr const char* TAG = "Ssd1685Display";

// Constructor / Destructor

Ssd1685Display::Ssd1685Display(std::unique_ptr<Configuration> cfg)
    : config(std::move(cfg))
{
    check(config != nullptr, "Ssd1685Display: config must not be null");
}

Ssd1685Display::~Ssd1685Display()
{
    if (lvglDisplay != nullptr) {
        stopLvgl();
    }
    if (started) {
        stop();
    }
}

// Geometry helpers

uint16_t Ssd1685Display::lvglWidth() const
{
    return (config->rotation == 1 || config->rotation == 3)
           ? config->height : config->width;
}

uint16_t Ssd1685Display::lvglHeight() const
{
    return (config->rotation == 1 || config->rotation == 3)
           ? config->width : config->height;
}

// Rotation

esp_err_t Ssd1685Display::applyRotation()
{
    bool swap_xy  = false;
    bool mirror_x = false;
    bool mirror_y = false;

    switch (config->rotation) {
    case 0: break;
    case 1: swap_xy = true;  mirror_x = true;  break;
    case 2: mirror_x = true; mirror_y = true;  break;
    case 3: swap_xy = true;  mirror_y = true;  break;
    default: LOG_W(TAG, "Unknown rotation %d, using 0", config->rotation); break;
    }

    if (swap_xy) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(panelHandle, true), TAG, "swap_xy");
    }
    if (mirror_x || mirror_y) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(panelHandle, mirror_x, mirror_y), TAG, "mirror");
    }
    return ESP_OK;
}

// Background refresh task
//
// Waits on refreshSem. When posted:
//   1. Runs the EPD refresh waveform sequence (slow, BUSY-blocking).
//   2. Goes back to waiting.
//
// Exits cleanly when stopRefreshTask is set and the semaphore is posted once
// more (as a kick to unblock the wait).

void Ssd1685Display::refreshTaskFunc(void* arg)
{
    auto* self = static_cast<Ssd1685Display*>(arg);

    LOG_I(TAG, "refresh task started");

    while (true) {
        // Block until flushCallback posts the semaphore
        xSemaphoreTake(self->refreshSem, portMAX_DELAY);

        if (self->stopRefreshTask.load()) {
            break;
        }

        if (self->panelHandle) {
            esp_err_t err = esp_lcd_ssd1685_refresh(
                self->panelHandle, self->config->refreshMode);
            if (err != ESP_OK) {
                LOG_E(TAG, "refresh failed: %s", esp_err_to_name(err));
            }
        }
    }

    LOG_I(TAG, "refresh task exiting");
    vTaskDelete(nullptr);
}

// start()

bool Ssd1685Display::start()
{
    if (started) {
        LOG_W(TAG, "Already started");
        return true;
    }

    // SPI panel IO
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num        = config->csPin,
        .dc_gpio_num        = config->dcPin,
        .spi_mode           = 0,
        .pclk_hz            = config->spiClockHz,
        .trans_queue_depth  = 4,
        .on_color_trans_done = nullptr,
        .user_ctx           = nullptr,
        .lcd_cmd_bits       = 8,
        .lcd_param_bits     = 8,
        .cs_ena_pretrans    = 0,
        .cs_ena_posttrans   = 0,
        .flags = {
            .dc_high_on_cmd  = 0,
            .dc_low_on_data  = 0,
            .dc_low_on_param = 0,
            .octal_mode      = 0,
            .quad_mode       = 0,
            .sio_mode        = 1,
            .lsb_first       = 0,
            .cs_high_active  = 0,
        }
    };

    if (esp_lcd_new_panel_io_spi(
            (esp_lcd_spi_bus_handle_t)config->spiHost,
            &io_cfg, &ioHandle) != ESP_OK) {
        LOG_E(TAG, "Failed to create SPI panel IO");
        return false;
    }

    esp_lcd_panel_ssd1685_config_t epd_cfg = {
        .busy_gpio_num        = config->busyPin,
        .busy_timeout_ms      = config->busyTimeoutMs,
        .panel_width          = config->width,
        .panel_height         = config->height,
        // non_copy_mode = true: draw_bitmap writes RAM only, does NOT trigger
        // the refresh waveform. The refresh task handles that separately.
        .non_copy_mode        = true,
        .default_refresh_mode = config->refreshMode,
        .custom_lut           = config->customLut,
        .custom_lut_size      = config->customLutSize,
        .on_reset             = nullptr,
        .on_reset_user_data   = nullptr,
    };

    esp_lcd_panel_dev_config_t panel_cfg = {};
    panel_cfg.reset_gpio_num = config->resetPin;
    panel_cfg.bits_per_pixel = 1;
    panel_cfg.vendor_config  = &epd_cfg;

    if (esp_lcd_new_panel_ssd1685(ioHandle, &panel_cfg, &panelHandle) != ESP_OK) {
        LOG_E(TAG, "Failed to create SSD1685 panel");
        esp_lcd_panel_io_del(ioHandle);
        ioHandle = nullptr;
        return false;
    }

    if (esp_lcd_panel_reset(panelHandle) != ESP_OK) {
        LOG_E(TAG, "Panel reset failed");
        goto fail;
    }

    if (esp_lcd_panel_init(panelHandle) != ESP_OK) {
        LOG_E(TAG, "Panel init failed");
        goto fail;
    }

    if (config->gapX != 0 || config->gapY != 0) {
        if (esp_lcd_panel_set_gap(panelHandle, config->gapX, config->gapY) != ESP_OK) {
            LOG_W(TAG, "set_gap failed (non-fatal)");
        }
    }

    if (applyRotation() != ESP_OK) {
        LOG_W(TAG, "applyRotation failed (non-fatal)");
    }

    LOG_I(TAG, "Initial clear...");
    if (esp_lcd_ssd1685_clear(panelHandle, 0xFF) != ESP_OK) {
        LOG_W(TAG, "Initial clear failed (non-fatal)");
    }

    started = true;
    LOG_I(TAG, "Started  %dx%d  rotation=%d  gap=(%d,%d)",
          config->width, config->height,
          config->rotation, config->gapX, config->gapY);
    return true;

fail:
    esp_lcd_panel_del(panelHandle);
    esp_lcd_panel_io_del(ioHandle);
    panelHandle = nullptr;
    ioHandle    = nullptr;
    return false;
}

// stop()

bool Ssd1685Display::stop()
{
    if (!started) return true;

    if (lvglDisplay) stopLvgl();

    esp_lcd_ssd1685_sleep(panelHandle, SSD1685_DEEP_SLEEP_MODE1);

    if (panelHandle) { esp_lcd_panel_del(panelHandle);  panelHandle = nullptr; }
    if (ioHandle)    { esp_lcd_panel_io_del(ioHandle);  ioHandle    = nullptr; }

    started = false;
    LOG_I(TAG, "Stopped");
    return true;
}

// startLvgl()

bool Ssd1685Display::startLvgl()
{
    if (lvglDisplay) {
        LOG_W(TAG, "LVGL already started");
        return true;
    }
    if (!started) {
        LOG_E(TAG, "Call start() before startLvgl()");
        return false;
    }

    const uint16_t w = lvglWidth();
    const uint16_t h = lvglHeight();
    const size_t row_bytes = ((size_t)w + 7U) / 8U;
    drawBufSize = row_bytes * h;

    // Prefer PSRAM, fall back to internal DMA RAM
    auto alloc = [](size_t sz) -> uint8_t* {
        uint8_t* p = static_cast<uint8_t*>(
            heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!p) p = static_cast<uint8_t*>(
            heap_caps_malloc(sz, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
        return p;
    };

    drawBuf1 = alloc(drawBufSize);
    drawBuf2 = alloc(drawBufSize);

    if (!drawBuf1 || !drawBuf2) {
        LOG_E(TAG, "Failed to allocate draw buffers (%zu bytes each)", drawBufSize);
        heap_caps_free(drawBuf1); drawBuf1 = nullptr;
        heap_caps_free(drawBuf2); drawBuf2 = nullptr;
        return false;
    }

    memset(drawBuf1, 0xFF, drawBufSize);
    memset(drawBuf2, 0xFF, drawBufSize);

    // Binary semaphore: starts empty; flushCallback gives, refresh task takes
    refreshSem = xSemaphoreCreateBinary();
    if (!refreshSem) {
        LOG_E(TAG, "Failed to create refresh semaphore");
        heap_caps_free(drawBuf1); drawBuf1 = nullptr;
        heap_caps_free(drawBuf2); drawBuf2 = nullptr;
        return false;
    }

    stopRefreshTask.store(false);

    // Stack size: 4 KB is plenty for the refresh task (no heavy computation)
    BaseType_t taskCreated = xTaskCreate(
        refreshTaskFunc,
        "epd_refresh",
        4096,
        this,
        config->refreshTaskPriority,
        &refreshTask);

    if (taskCreated != pdPASS) {
        LOG_E(TAG, "Failed to create refresh task");
        vSemaphoreDelete(refreshSem); refreshSem = nullptr;
        heap_caps_free(drawBuf1); drawBuf1 = nullptr;
        heap_caps_free(drawBuf2); drawBuf2 = nullptr;
        return false;
    }

    // Create LVGL display
    lvglDisplay = lv_display_create(w, h);
    if (!lvglDisplay) {
        LOG_E(TAG, "lv_display_create failed");
        goto fail_lvgl;
    }

    lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_I1);
    lv_display_set_buffers(
        lvglDisplay,
        drawBuf1, drawBuf2,
        drawBufSize,
        LV_DISPLAY_RENDER_MODE_FULL);

    lv_display_set_flush_cb(lvglDisplay, flushCallback);
    lv_display_set_user_data(lvglDisplay, this);

    if (config->touch && config->touch->supportsLvgl()) {
        if (!config->touch->startLvgl(lvglDisplay)) {
            LOG_W(TAG, "Touch startLvgl failed (non-fatal)");
        }
    }

    LOG_I(TAG, "LVGL started  %dx%d  I1  full-refresh (deferred)", w, h);
    return true;

fail_lvgl:
    stopRefreshTask.store(true);
    xSemaphoreGive(refreshSem);          // unblock task so it can exit
    vTaskDelay(pdMS_TO_TICKS(50));       // let it exit
    vSemaphoreDelete(refreshSem); refreshSem = nullptr;
    refreshTask = nullptr;
    heap_caps_free(drawBuf1); drawBuf1 = nullptr;
    heap_caps_free(drawBuf2); drawBuf2 = nullptr;
    return false;
}

// stopLvgl()

bool Ssd1685Display::stopLvgl()
{
    if (!lvglDisplay) return true;

    if (config->touch) config->touch->stopLvgl();

    lv_display_delete(lvglDisplay);
    lvglDisplay = nullptr;

    // Stop the refresh task gracefully
    if (refreshTask) {
        stopRefreshTask.store(true);
        xSemaphoreGive(refreshSem);      // kick the task out of its wait
        // Give it time to exit before we delete the semaphore it may be
        // using. 200 ms > one full refresh timeout slice.
        vTaskDelay(pdMS_TO_TICKS(200));
        refreshTask = nullptr;
    }

    if (refreshSem) {
        vSemaphoreDelete(refreshSem);
        refreshSem = nullptr;
    }

    heap_caps_free(drawBuf1); drawBuf1 = nullptr;
    heap_caps_free(drawBuf2); drawBuf2 = nullptr;
    drawBufSize = 0;

    LOG_I(TAG, "LVGL stopped");
    return true;
}

// flushCallback  (static)
//
// Called by LVGL (inside the LVGL task, holding the LVGL mutex).
// MUST return as fast as possible.
//
// Strategy:
//   1. Write pixel data to panel RAM via SPI DMA  → fast, async-ish
//   2. Call lv_display_flush_ready()              → release LVGL mutex
//   3. Post semaphore                             → wake refresh task
//
// The refresh task then does the slow BUSY-wait + waveform sequence
// completely outside the LVGL task.

void Ssd1685Display::flushCallback(lv_display_t* disp,
                                    const lv_area_t* area,
                                    uint8_t* pixelMap)
{
    auto* self = static_cast<Ssd1685Display*>(lv_display_get_user_data(disp));
    if (!self || !self->panelHandle) {
        lv_display_flush_ready(disp);
        return;
    }

    const int x0 = area->x1;
    const int y0 = area->y1;
    const int x1 = area->x2 + 1;
    const int y1 = area->y2 + 1;

    // Write to B/W RAM – SPI transfer (DMA), returns before transfer completes
    // but that's fine: the semaphore is posted AFTER this call, so the refresh
    // task won't start until the SPI transaction has had time to be queued.
    esp_err_t err = esp_lcd_panel_draw_bitmap(
        self->panelHandle, x0, y0, x1, y1, pixelMap);

    if (err != ESP_OK) {
        LOG_E(TAG, "draw_bitmap failed: %s", esp_err_to_name(err));
    }

    // Release LVGL immediately – do NOT block here
    lv_display_flush_ready(disp);

    // Wake the refresh task (only on the last partial flush of this cycle)
    if (lv_display_flush_is_last(disp) && self->refreshSem) {
        xSemaphoreGive(self->refreshSem);
    }
}

// Extended public ops

esp_err_t Ssd1685Display::refresh(ssd1685_refresh_mode_t mode)
{
    if (!panelHandle) return ESP_ERR_INVALID_STATE;
    return esp_lcd_ssd1685_refresh(panelHandle, mode);
}

esp_err_t Ssd1685Display::clearScreen(uint8_t colorByte)
{
    if (!panelHandle) return ESP_ERR_INVALID_STATE;
    return esp_lcd_ssd1685_clear(panelHandle, colorByte);
}

esp_err_t Ssd1685Display::sleep()
{
    if (!panelHandle) return ESP_ERR_INVALID_STATE;
    return esp_lcd_ssd1685_sleep(panelHandle, SSD1685_DEEP_SLEEP_MODE1);
}

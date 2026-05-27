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
    if (lvglDisplay != nullptr) stopLvgl();
    if (started)                stop();
}

// Geometry

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
    default:
        LOG_W(TAG, "Unknown rotation %d, using 0", config->rotation);
        break;
    }
    if (swap_xy) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(panelHandle, true), TAG, "swap_xy");
    }
    if (mirror_x || mirror_y) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(panelHandle, mirror_x, mirror_y), TAG, "mirror");
    }
    return ESP_OK;
}

// Refresh task
//
// Woken by flushCallback via semReady.
// Does the entire slow EPD sequence outside the LVGL task:
//   1. draw_bitmap  – writes pendingBuf to panel RAM via SPI
//   2. refresh      – triggers EPD waveform, blocks on BUSY (~3 s)
//   3. flush_ready  – tells LVGL the source buffer is free to reuse

void Ssd1685Display::refreshTaskFunc(void* arg)
{
    auto* self = static_cast<Ssd1685Display*>(arg);
    LOG_I(TAG, "refresh task started");

    while (true) {
        xSemaphoreTake(self->semReady, portMAX_DELAY);

        if (self->stopRefreshTask.load()) {
            break;
        }

        if (self->panelHandle && self->pendingBuf) {
            esp_err_t err = esp_lcd_panel_draw_bitmap(
                self->panelHandle,
                0, 0,
                (int)self->lvglWidth(),
                (int)self->lvglHeight(),
                self->pendingBuf);

            if (err != ESP_OK) {
                LOG_E(TAG, "draw_bitmap failed: %s", esp_err_to_name(err));
            } else {
                err = esp_lcd_ssd1685_refresh(
                    self->panelHandle, self->config->refreshMode);
                if (err != ESP_OK) {
                    LOG_E(TAG, "refresh failed: %s", esp_err_to_name(err));
                }
            }
        }

        // Signal LVGL that the buffer is free – called from outside LVGL task,
        // which is explicitly supported by LVGL docs.
        if (self->lvglDisplayForFlush) {
            lv_display_flush_ready(self->lvglDisplayForFlush);
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

    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num         = config->csPin,
        .dc_gpio_num         = config->dcPin,
        .spi_mode            = 0,
        .pclk_hz             = config->spiClockHz,
        .trans_queue_depth   = 4,
        .on_color_trans_done = nullptr,
        .user_ctx            = nullptr,
        .lcd_cmd_bits        = 8,
        .lcd_param_bits      = 8,
        .cs_ena_pretrans     = 0,
        .cs_ena_posttrans    = 0,
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

    if (esp_lcd_panel_reset(panelHandle) != ESP_OK ||
        esp_lcd_panel_init(panelHandle)  != ESP_OK) {
        LOG_E(TAG, "Panel reset/init failed");
        esp_lcd_panel_del(panelHandle);
        esp_lcd_panel_io_del(ioHandle);
        panelHandle = nullptr;
        ioHandle    = nullptr;
        return false;
    }

    if (config->gapX != 0 || config->gapY != 0) {
        esp_lcd_panel_set_gap(panelHandle, config->gapX, config->gapY);
    }
    applyRotation();

    LOG_I(TAG, "Initial clear...");
    esp_lcd_ssd1685_clear(panelHandle, 0xFF);

    started = true;
    LOG_I(TAG, "Started  %dx%d  rotation=%d  gap=(%d,%d)",
          config->width, config->height,
          config->rotation, config->gapX, config->gapY);
    return true;
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
    const size_t rowBytes = ((size_t)w + 7U) / 8U;
    drawBufSize = rowBytes * h;

    auto alloc = [](size_t sz) -> uint8_t* {
        uint8_t* p = static_cast<uint8_t*>(
            heap_caps_malloc(sz, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        if (!p) p = static_cast<uint8_t*>(
            heap_caps_malloc(sz, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
        return p;
    };

    drawBuf1   = alloc(drawBufSize);
    drawBuf2   = alloc(drawBufSize);
    pendingBuf = alloc(drawBufSize);

    if (!drawBuf1 || !drawBuf2 || !pendingBuf) {
        LOG_E(TAG, "Buffer alloc failed (%zu bytes each)", drawBufSize);
        heap_caps_free(drawBuf1);   drawBuf1   = nullptr;
        heap_caps_free(drawBuf2);   drawBuf2   = nullptr;
        heap_caps_free(pendingBuf); pendingBuf = nullptr;
        return false;
    }

    memset(drawBuf1,   0xFF, drawBufSize);
    memset(drawBuf2,   0xFF, drawBufSize);
    memset(pendingBuf, 0xFF, drawBufSize);

    // Binary semaphore: starts at 0 (refresh task waits for flush_cb to post)
    semReady = xSemaphoreCreateBinary();
    if (!semReady) {
        LOG_E(TAG, "Failed to create semaphore");
        goto fail_sem;
    }

    stopRefreshTask.store(false);

    if (xTaskCreate(refreshTaskFunc, "epd_refresh",
                    4096, this,
                    config->refreshTaskPriority,
                    &refreshTask) != pdPASS) {
        LOG_E(TAG, "Failed to create refresh task");
        goto fail_sem;
    }

    lvglDisplay = lv_display_create(w, h);
    if (!lvglDisplay) {
        LOG_E(TAG, "lv_display_create failed");
        goto fail_lvgl;
    }

    lvglDisplayForFlush = lvglDisplay;

    lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_I1);
    lv_display_set_buffers(
        lvglDisplay,
        drawBuf1, drawBuf2,
        drawBufSize,
        LV_DISPLAY_RENDER_MODE_FULL);

    lv_display_set_flush_cb(lvglDisplay, flushCallback);
    lv_display_set_user_data(lvglDisplay, this);
    // NO flush_wait_cb – flush_ready is called from the refresh task instead

    if (config->touch && config->touch->supportsLvgl()) {
        if (!config->touch->startLvgl(lvglDisplay)) {
            LOG_W(TAG, "Touch startLvgl failed (non-fatal)");
        }
    }

    LOG_I(TAG, "LVGL started  %dx%d  I1  async (flush_ready from task)", w, h);
    return true;

fail_lvgl:
    stopRefreshTask.store(true);
    xSemaphoreGive(semReady);
    vTaskDelay(pdMS_TO_TICKS(100));
    refreshTask = nullptr;
    lvglDisplayForFlush = nullptr;

fail_sem:
    if (semReady) { vSemaphoreDelete(semReady); semReady = nullptr; }
    heap_caps_free(drawBuf1);   drawBuf1   = nullptr;
    heap_caps_free(drawBuf2);   drawBuf2   = nullptr;
    heap_caps_free(pendingBuf); pendingBuf = nullptr;
    return false;
}

// stopLvgl()

bool Ssd1685Display::stopLvgl()
{
    if (!lvglDisplay) return true;

    lvglDisplayForFlush = nullptr;  // prevent refresh task calling flush_ready on deleted display

    if (config->touch) config->touch->stopLvgl();

    lv_display_delete(lvglDisplay);
    lvglDisplay = nullptr;

    if (refreshTask) {
        stopRefreshTask.store(true);
        xSemaphoreGive(semReady);
        vTaskDelay(pdMS_TO_TICKS(200));
        refreshTask = nullptr;
    }

    if (semReady) { vSemaphoreDelete(semReady); semReady = nullptr; }

    heap_caps_free(drawBuf1);   drawBuf1   = nullptr;
    heap_caps_free(drawBuf2);   drawBuf2   = nullptr;
    heap_caps_free(pendingBuf); pendingBuf = nullptr;
    drawBufSize = 0;

    LOG_I(TAG, "LVGL stopped");
    return true;
}

// flushCallback  (static)
//
// Called inside the LVGL task. MUST return immediately – zero blocking.
//
// If the refresh task is idle (semReady empty):
//   • Check flush_is_last (before any state changes)
//   • memcpy pixel data into pendingBuf
//   • give(semReady) – wake refresh task
//   • return WITHOUT calling flush_ready (refresh task does that)
//
// If the refresh task is still busy (semReady already has a token):
//   • Drop this frame: call flush_ready immediately so LVGL can proceed

void Ssd1685Display::flushCallback(lv_display_t* disp,
                                    const lv_area_t* /*area*/,
                                    uint8_t* pixelMap)
{
    auto* self = static_cast<Ssd1685Display*>(lv_display_get_user_data(disp));
    if (!self || !self->pendingBuf || !self->semReady) {
        lv_display_flush_ready(disp);
        return;
    }

    // Must check flush_is_last before calling flush_ready
    const bool isLast = lv_display_flush_is_last(disp);

    if (!isLast) {
        // With RENDER_MODE_FULL this shouldn't happen, but handle gracefully
        lv_display_flush_ready(disp);
        return;
    }

    // Try to post to the refresh task without blocking.
    // Timeout = 0: if already full (previous frame still pending), drop this one.
    if (xSemaphoreGive(self->semReady) == pdFALSE) {
        // Semaphore already given (task still busy) – drop frame
        LOG_D(TAG, "EPD busy, dropping frame");
        lv_display_flush_ready(disp);
        return;
    }

    // Copy pixel data BEFORE returning so LVGL can't overwrite pixelMap.
    // (With double buffers, LVGL writes into the OTHER buffer after flush_cb
    // returns, so pixelMap itself is stable until flush_ready is called –
    // but we copy anyway for clarity and to allow future single-buffer use.)
    memcpy(self->pendingBuf, pixelMap, self->drawBufSize);

    // Do NOT call flush_ready here.
    // The refresh task calls lv_display_flush_ready() after the EPD refresh
    // completes, which tells LVGL this buffer slot is free again.
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

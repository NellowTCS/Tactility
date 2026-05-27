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
        ESP_RETURN_ON_ERROR(
            esp_lcd_panel_swap_xy(panelHandle, true), TAG, "swap_xy");
    }
    if (mirror_x || mirror_y) {
        ESP_RETURN_ON_ERROR(
            esp_lcd_panel_mirror(panelHandle, mirror_x, mirror_y), TAG, "mirror");
    }
    return ESP_OK;
}

// Refresh task
//
// Sits waiting on semReady. On each wakeup:
//   1. Writes pendingBuf to panel RAM (SPI).
//   2. Triggers EPD waveform refresh (blocks on BUSY).
//   3. Posts semDone to unblock the wait_cb.

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
            // Write pixel data to panel RAM – SPI, returns quickly
            esp_err_t err = esp_lcd_panel_draw_bitmap(
                self->panelHandle,
                0, 0,
                (int)self->lvglWidth(),
                (int)self->lvglHeight(),
                self->pendingBuf);

            if (err != ESP_OK) {
                LOG_E(TAG, "draw_bitmap failed: %s", esp_err_to_name(err));
            } else {
                // Trigger EPD waveform – blocks until BUSY deasserts (~3 s)
                err = esp_lcd_ssd1685_refresh(
                    self->panelHandle, self->config->refreshMode);
                if (err != ESP_OK) {
                    LOG_E(TAG, "refresh failed: %s", esp_err_to_name(err));
                }
            }
        }

        // Unblock the wait_cb so LVGL can proceed with the next render
        xSemaphoreGive(self->semDone);
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
        .non_copy_mode        = true,   // draw_bitmap writes RAM only, no auto-refresh
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

    // semReady: starts empty – refresh task waits until flush_cb posts a frame
    // semDone:  starts empty – wait_cb blocks until refresh task completes
    semReady = xSemaphoreCreateBinary();
    semDone  = xSemaphoreCreateBinary();

    if (!semReady || !semDone) {
        LOG_E(TAG, "Semaphore creation failed");
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

    lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_I1);
    lv_display_set_buffers(
        lvglDisplay,
        drawBuf1, drawBuf2,
        drawBufSize,
        LV_DISPLAY_RENDER_MODE_FULL);

    lv_display_set_flush_cb(lvglDisplay, flushCallback);
    lv_display_set_flush_wait_cb(lvglDisplay, flushWaitCallback);
    lv_display_set_user_data(lvglDisplay, this);

    if (config->touch && config->touch->supportsLvgl()) {
        if (!config->touch->startLvgl(lvglDisplay)) {
            LOG_W(TAG, "Touch startLvgl failed (non-fatal)");
        }
    }

    LOG_I(TAG, "LVGL started  %dx%d  I1  async (flush_wait_cb)", w, h);
    return true;

fail_lvgl:
    stopRefreshTask.store(true);
    xSemaphoreGive(semReady);
    vTaskDelay(pdMS_TO_TICKS(100));
    refreshTask = nullptr;

fail_sem:
    if (semReady) { vSemaphoreDelete(semReady); semReady = nullptr; }
    if (semDone)  { vSemaphoreDelete(semDone);  semDone  = nullptr; }
    heap_caps_free(drawBuf1);   drawBuf1   = nullptr;
    heap_caps_free(drawBuf2);   drawBuf2   = nullptr;
    heap_caps_free(pendingBuf); pendingBuf = nullptr;
    return false;
}

// stopLvgl()

bool Ssd1685Display::stopLvgl()
{
    if (!lvglDisplay) return true;

    if (config->touch) config->touch->stopLvgl();

    lv_display_delete(lvglDisplay);
    lvglDisplay = nullptr;

    if (refreshTask) {
        stopRefreshTask.store(true);
        xSemaphoreGive(semReady);        // kick task out of wait
        xSemaphoreGive(semDone);         // unblock wait_cb if it's waiting
        vTaskDelay(pdMS_TO_TICKS(200));
        refreshTask = nullptr;
    }

    if (semReady) { vSemaphoreDelete(semReady); semReady = nullptr; }
    if (semDone)  { vSemaphoreDelete(semDone);  semDone  = nullptr; }

    heap_caps_free(drawBuf1);   drawBuf1   = nullptr;
    heap_caps_free(drawBuf2);   drawBuf2   = nullptr;
    heap_caps_free(pendingBuf); pendingBuf = nullptr;
    drawBufSize = 0;

    LOG_I(TAG, "LVGL stopped");
    return true;
}

// flushCallback  (static)
//
// Called by lv_timer_handler() inside the LVGL task.
// Contract: must not block. Must NOT call lv_display_flush_ready() –
// that is handled by LVGL after flushWaitCallback returns.
//
// 1. Check flush_is_last BEFORE any state changes (it reads internal flags).
// 2. memcpy pixel data into pendingBuf (owned by us until semDone is given).
// 3. Give semReady to wake the refresh task.

void Ssd1685Display::flushCallback(lv_display_t* disp,
                                    const lv_area_t* /*area*/,
                                    uint8_t* pixelMap)
{
    auto* self = static_cast<Ssd1685Display*>(lv_display_get_user_data(disp));
    if (!self || !self->pendingBuf) {
        // Must call flush_ready if we're bailing early, otherwise LVGL hangs
        lv_display_flush_ready(disp);
        return;
    }

    // Check BEFORE any state change – flush_ready would clear the flag
    const bool isLast = lv_display_flush_is_last(disp);

    if (isLast) {
        // Safe copy: LVGL won't reuse pixelMap until flush_ready is called,
        // which happens only after flushWaitCallback returns.
        memcpy(self->pendingBuf, pixelMap, self->drawBufSize);
        xSemaphoreGive(self->semReady);
        // Do NOT call flush_ready here – flushWaitCallback does it implicitly
        // by returning (LVGL calls flush_ready after wait_cb returns)
    } else {
        // Non-last chunk (shouldn't occur with RENDER_MODE_FULL, but be safe)
        lv_display_flush_ready(disp);
    }
}

// flushWaitCallback  (static)
//
// Called by LVGL after flushCallback returns, to wait for the flush to
// complete before allowing the next render cycle.
//
// LVGL calls flush_ready automatically when this returns, so we must NOT
// call it ourselves.
//
// This function runs inside lv_timer_handler() and therefore inside the
// LVGL task – but the WDT timeout (default 5 s) gives us enough headroom
// for a full EPD refresh (~3 s).  If needed, increase via:
//   CONFIG_ESP_TASK_WDT_TIMEOUT_S=15  in sdkconfig

void Ssd1685Display::flushWaitCallback(lv_display_t* disp)
{
    auto* self = static_cast<Ssd1685Display*>(lv_display_get_user_data(disp));
    if (!self || !self->semDone) {
        return;
    }

    // Block until the refresh task signals completion.
    // The refresh task gives semDone after wait_busy returns.
    xSemaphoreTake(self->semDone, portMAX_DELAY);
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
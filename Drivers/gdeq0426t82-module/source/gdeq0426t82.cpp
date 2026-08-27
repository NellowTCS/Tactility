// SPDX-License-Identifier: Apache-2.0
#include <drivers/gdeq0426t82.h>
#include <gdeq0426t82_module.h>

#include <tactility/check.h>
#include <tactility/delay.h>
#include <tactility/device.h>
#include <tactility/driver.h>
#include <tactility/drivers/display.h>
#include <tactility/drivers/esp32_spi.h>
#include <tactility/drivers/gpio_controller.h>
#include <tactility/drivers/spi_controller.h>
#include <tactility/error.h>
#include <tactility/log.h>
#include <tactility/time.h>

#include <driver/spi_master.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/timers.h>

#include <cstdlib>
#include <cstring>

constexpr auto* TAG = "GDEQ0426T82";
#define GET_CONFIG(device) (static_cast<const Gdeq0426t82Config*>((device)->config))

static constexpr int WIDTH = 800;
static constexpr int HEIGHT = 480;
static constexpr size_t FRAMEBUFFER_SIZE = (WIDTH * HEIGHT) / 8; // 1 bpp
static constexpr int BYTES_PER_ROW = WIDTH / 8;

/** Band height for tile-streaming (keeps the working set small). */
static constexpr int BAND_HEIGHT = 16;
static constexpr size_t BAND_SIZE = (WIDTH * BAND_HEIGHT) / 8;

/** How long to keep batching draw_bitmap() calls before pushing the pixels to the panel. */
static constexpr TickType_t COALESCE_PERIOD_MS = pdMS_TO_TICKS(80);

// SSD1677-family commands and values. The full-refresh sequence follows GxEPD2_4G's
// GxEPD2_426_GDEQ0426T82 implementation; the differential partial (RED vs BW compare) follows
// FreeInk's Ssd1677Driver for this exact panel, which is hardware-verified on the X4.
static constexpr uint8_t CMD_DRIVER_OUTPUT_CONTROL = 0x01;
static constexpr uint8_t CMD_BOOSTER_SOFT_START = 0x0C;
static constexpr uint8_t CMD_DEEP_SLEEP = 0x10;
static constexpr uint8_t CMD_RAM_ENTRY_MODE = 0x11;
static constexpr uint8_t CMD_SW_RESET = 0x12;
static constexpr uint8_t CMD_TEMP_SENSOR = 0x18;
static constexpr uint8_t CMD_MASTER_ACTIVATION = 0x20;
static constexpr uint8_t CMD_UPDATE_CTRL = 0x21;
static constexpr uint8_t CMD_UPDATE_CTRL2 = 0x22;
static constexpr uint8_t CMD_WRITE_RAM_BW = 0x24;
/** RED RAM holds the old frame for the differential partial; bypassed during full refresh. */
static constexpr uint8_t CMD_WRITE_RAM_RED = 0x26;
static constexpr uint8_t CMD_BORDER_SETTING = 0x3C;
static constexpr uint8_t CMD_SET_RAM_X_WINDOW = 0x44;
static constexpr uint8_t CMD_SET_RAM_Y_WINDOW = 0x45;
static constexpr uint8_t CMD_SET_RAM_X_ADDR = 0x4E;
static constexpr uint8_t CMD_SET_RAM_Y_ADDR = 0x4F;

static constexpr uint8_t DEEP_SLEEP_CODE = 0x01;
/** Explicit power-off sequence, used before deep sleep. */
static constexpr uint8_t POWER_OFF = 0x83;
/** B/W full refresh (baseline): bypasses the RED plane, drives every pixel (flashes). */
static constexpr uint8_t REFRESH_FULL = 0xF7;
/**
 * Fast differential partial refresh: the panel's LUT drives only pixels where RED RAM (0x26)
 * differs from BW RAM (0x24), so the host must keep RED = previously displayed frame. Stock X4
 * value from FreeInk's Ssd1677Driver (hardware-verified); it includes the clock/analog power-on
 * bits (0xC0), so the panel powers itself for each partial. GxEPD2's 0x1C fast value does NOT
 * select the partial waveform on this panel and is not used.
 */
static constexpr uint8_t REFRESH_PARTIAL = 0xFC;
/** Border value written before every refresh; required for the partial/DU waveform to engage. */
static constexpr uint8_t BORDER_REFRESH = 0xC0;

/**
 * Ordered Bayer 4x4 dither matrix. Each 4x4 cell maps 4 quantized grey levels (from
 * GRAYSCALE8 >> 6) onto a black/white checkerboard; the panel's default OTP LUT then
 * performs a single flashless B/W partial update of those pixels.
 */
static constexpr uint8_t BAYER_4X4[16] = {
    0, 8, 2, 10,
    12, 4, 14, 6,
    3, 11, 1, 9,
    15, 7, 13, 5
};

/** Fraction of pixels driven black per quantized level: L0=black, L1, L2, L3=white. */
static constexpr uint8_t DITHER_THRESHOLD[4] = {16, 5, 10, 0};

static inline bool dither_pixel_black(int32_t x, int32_t y, uint8_t level) {
    return BAYER_4X4[((y & 3) << 2) | (x & 3)] < DITHER_THRESHOLD[level];
}

extern "C" {

struct Gdeq0426t82Internal {
    spi_device_handle_t spi_device;
    struct GpioDescriptor* dc;
    struct GpioDescriptor* reset; // optional
    struct GpioDescriptor* busy;
    /** Full-frame 1bpp shadow of the panel's B/W RAM (bit 1 = white, 0 = black). */
    uint8_t* framebuffer;
    /** Full-frame 1bpp shadow of the frame currently displayed (the differential's old frame). */
    uint8_t* old_framebuffer;
    /** Scratch buffer for one band of the dirty window (streamed, never retained). */
    uint8_t* band_buffer;
    /** Serializes panel/SPI access between the refresh timer and power on/off calls. */
    SemaphoreHandle_t panel_mutex;
    /** One-shot timer that batches consecutive draw_bitmap() calls into a single refresh. */
    TimerHandle_t coalesce_timer;
    /** False once stop() has begun tearing down; the timer callback checks it without the mutex. */
    bool timer_alive;
    /** True while a refresh's BUSY wait is running outside the mutex (see refresh_in_flight). */
    bool refresh_in_flight;
    /** True while the controller is initialized; false after deep sleep or a hardware reset. */
    bool initialized;
    /** disp_on_off state; the panel is in deep sleep while false. */
    bool display_on;
    /** The next refresh must be a full baseline update (cold boot / reset). */
    bool first_refresh;
    /** Inclusive-exclusive bounding box of pixels pending refresh (framebuffer coords). */
    int32_t dirty_x0;
    int32_t dirty_y0;
    int32_t dirty_x1;
    int32_t dirty_y1;
    bool dirty_pending;
};

// region Panel protocol

static bool write_command(Gdeq0426t82Internal* internal, uint8_t command) {
    gpio_descriptor_set_level(internal->dc, false);
    spi_transaction_t transaction = {};
    transaction.length = 8;
    transaction.tx_buffer = &command;
    if (spi_device_polling_transmit(internal->spi_device, &transaction) != ESP_OK) {
        LOG_E(TAG, "SPI command transfer failed");
        return false;
    }
    return true;
}

static bool write_data(Gdeq0426t82Internal* internal, const uint8_t* data, size_t length) {
    gpio_descriptor_set_level(internal->dc, true);
    spi_transaction_t transaction = {};
    transaction.length = length * 8;
    transaction.tx_buffer = data;
    if (spi_device_polling_transmit(internal->spi_device, &transaction) != ESP_OK) {
        LOG_E(TAG, "SPI data transfer failed");
        return false;
    }
    return true;
}

static bool write_data_byte(Gdeq0426t82Internal* internal, uint8_t data) {
    return write_data(internal, &data, 1);
}

static bool wait_while_busy(Gdeq0426t82Internal* internal) {
    // The GDEQ0426T82 BUSY pin is active-high (busy when the pin reads high).
    const TickType_t timeout = pdMS_TO_TICKS(10000);
    const TickType_t start = get_ticks();
    bool busy = true;
    while (gpio_descriptor_get_level(internal->busy, &busy) == ERROR_NONE && busy) {
        if (get_ticks() - start > timeout) {
            LOG_E(TAG, "Timed out waiting for panel BUSY");
            return false;
        }
        delay_millis(2);
    }
    return !busy;
}

static void hardware_reset(Gdeq0426t82Internal* internal) {
    if (internal->reset != nullptr) {
        // X4 reset sequence: RST HIGH -> 20ms -> LOW -> 2ms -> HIGH -> 20ms.
        gpio_descriptor_set_level(internal->reset, true);
        delay_millis(20);
        gpio_descriptor_set_level(internal->reset, false);
        delay_millis(2);
        gpio_descriptor_set_level(internal->reset, true);
        delay_millis(20);
    }
}

/**
 * Set the RAM window; the panel's gates are wired bottom-to-top, so the Y window
 * is reversed (GxEPD2 _setPartialRamArea, 0x11 = X increase, Y decrease).
 */
static bool set_ram_area(Gdeq0426t82Internal* internal, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    y = HEIGHT - y - h;
    bool ok = write_command(internal, CMD_RAM_ENTRY_MODE);
    ok = ok && write_data_byte(internal, 0x01);
    ok = ok && write_command(internal, CMD_SET_RAM_X_WINDOW);
    ok = ok && write_data_byte(internal, x % 256);
    ok = ok && write_data_byte(internal, x / 256);
    ok = ok && write_data_byte(internal, (x + w - 1) % 256);
    ok = ok && write_data_byte(internal, (x + w - 1) / 256);
    ok = ok && write_command(internal, CMD_SET_RAM_Y_WINDOW);
    ok = ok && write_data_byte(internal, (y + h - 1) % 256);
    ok = ok && write_data_byte(internal, (y + h - 1) / 256);
    ok = ok && write_data_byte(internal, y % 256);
    ok = ok && write_data_byte(internal, y / 256);
    ok = ok && write_command(internal, CMD_SET_RAM_X_ADDR);
    ok = ok && write_data_byte(internal, x % 256);
    ok = ok && write_data_byte(internal, x / 256);
    ok = ok && write_command(internal, CMD_SET_RAM_Y_ADDR);
    ok = ok && write_data_byte(internal, (y + h - 1) % 256);
    ok = ok && write_data_byte(internal, (y + h - 1) / 256);
    if (!ok) {
        LOG_E(TAG, "Failed to set RAM window");
    }
    return ok;
}

// endregion

// region Init and refresh

/**
 * Initialize the controller for B/W operation (GxEPD2_4G _InitDisplay sequence). Uses the panel's
 * built-in default LUT, so no 4-grey LUT or plane voltages are written. B/W and RED RAM (and the
 * displayed image) survive deep sleep, so waking up keeps the previous frame and its differential
 * baseline; cold boot relies on the first full 0xF7 refresh, which also seeds the RED plane.
 */
static bool init_panel(Gdeq0426t82Internal* internal) {
    hardware_reset(internal);
    delay_millis(10); // spec settling time
    bool ok = write_command(internal, CMD_SW_RESET);
    delay_millis(10); // spec settling time
    ok = ok && write_command(internal, CMD_BOOSTER_SOFT_START);
    const uint8_t soft_start[5] = {0xAE, 0xC7, 0xC3, 0xC0, 0x80};
    ok = ok && write_data(internal, soft_start, sizeof(soft_start));
    ok = ok && write_command(internal, CMD_DRIVER_OUTPUT_CONTROL);
    ok = ok && write_data_byte(internal, (HEIGHT - 1) % 256);
    ok = ok && write_data_byte(internal, (HEIGHT - 1) / 256);
    ok = ok && write_data_byte(internal, 0x02); // SM (interlaced) off
    ok = ok && write_command(internal, CMD_BORDER_SETTING);
    ok = ok && write_data_byte(internal, 0x80); // border follows LUT; refresh overrides with 0xC0
    ok = ok && write_command(internal, CMD_TEMP_SENSOR);
    ok = ok && write_data_byte(internal, 0x80); // internal temperature sensor
    if (!ok) {
        LOG_E(TAG, "B/W init failed");
        return false;
    }
    internal->initialized = true;
    return true;
}

/** Align the dirty bbox to whole bytes (panel RAM is byte-oriented) and clamp it to the panel. */
static bool get_dirty_window(
    Gdeq0426t82Internal* internal,
    int32_t* out_x0,
    int32_t* out_y0,
    int32_t* out_x1,
    int32_t* out_y1
) {
    if (!internal->dirty_pending) {
        return false;
    }
    int32_t x0 = internal->dirty_x0 & ~7;
    int32_t x1 = (internal->dirty_x1 + 7) & ~7;
    if (x0 < 0) {
        x0 = 0;
    }
    if (x1 > WIDTH) {
        x1 = WIDTH;
    }
    int32_t y0 = internal->dirty_y0 < 0 ? 0 : internal->dirty_y0;
    int32_t y1 = internal->dirty_y1 > HEIGHT ? HEIGHT : internal->dirty_y1;
    if (x1 <= x0 || y1 <= y0) {
        internal->dirty_pending = false;
        return false;
    }
    *out_x0 = x0;
    *out_y0 = y0;
    *out_x1 = x1;
    *out_y1 = y1;
    return true;
}

/**
 * Stream one RAM plane (0x24 BW or 0x26 RED) window from a full-frame source. The window is the
 * dirty bbox aligned to whole bytes; boundary bytes were merged in draw_bitmap(), so this write is
 * always complete, never read-modify-write.
 */
static bool write_ram_window(
    Gdeq0426t82Internal* internal,
    uint8_t plane_command,
    const uint8_t* source,
    int32_t x0,
    int32_t y0,
    int32_t x1,
    int32_t y1
) {
    const int32_t w = x1 - x0;
    const int32_t h = y1 - y0;
    const size_t band_row_bytes = (size_t)(w / 8);
    if (!set_ram_area(internal, (uint16_t)x0, (uint16_t)y0, (uint16_t)w, (uint16_t)h)) {
        return false;
    }
    if (!write_command(internal, plane_command)) {
        return false;
    }
    for (int32_t band_y = y0; band_y < y1; band_y += BAND_HEIGHT) {
        const int32_t band_h = (band_y + BAND_HEIGHT <= y1) ? BAND_HEIGHT : y1 - band_y;
        for (int32_t row = 0; row < band_h; row++) {
            const uint8_t* src = source + (size_t)(band_y + row) * BYTES_PER_ROW + (size_t)(x0 / 8);
            std::memcpy(internal->band_buffer + (size_t)row * band_row_bytes, src, band_row_bytes);
        }
        if (!write_data(internal, internal->band_buffer, (size_t)band_h * band_row_bytes)) {
            LOG_E(TAG, "Failed to write RAM window band at y=%ld", (long)band_y);
            return false;
        }
    }
    return true;
}

/** Copy a window from the current frame shadow into the old-frame shadow (row-strided, in RAM). */
static void copy_window(uint8_t* dst, const uint8_t* src, int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    const size_t row_bytes = (size_t)((x1 - x0) / 8);
    const size_t col_offset = (size_t)(x0 / 8);
    for (int32_t y = y0; y < y1; y++) {
        std::memcpy(dst + (size_t)y * BYTES_PER_ROW + col_offset, src + (size_t)y * BYTES_PER_ROW + col_offset, row_bytes);
    }
}

/**
 * Trigger the update for the RAM planes just written. The first refresh after init/reset is a full
 * baseline (0xF7, RED bypassed, panel powers down at the end); every later refresh is a flashless
 * differential partial (0xFC) that compares RED vs BW. Both sequences are absolute: the border
 * (0x3C 0xC0) precedes the update, and 0xFC/0xF7 fold in the power transitions, so no separate
 * power-on step is needed.
 */
static bool trigger_refresh(Gdeq0426t82Internal* internal) {
    const bool full_refresh = internal->first_refresh;
    bool ok = write_command(internal, CMD_UPDATE_CTRL);
    ok = ok && write_data_byte(internal, full_refresh ? 0x40 : 0x00); // RED bypass (full) / compare (partial)
    ok = ok && write_data_byte(internal, 0x00); // single chip application
    ok = ok && write_command(internal, CMD_BORDER_SETTING);
    ok = ok && write_data_byte(internal, BORDER_REFRESH);
    ok = ok && write_command(internal, CMD_UPDATE_CTRL2);
    ok = ok && write_data_byte(internal, full_refresh ? REFRESH_FULL : REFRESH_PARTIAL);
    ok = ok && write_command(internal, CMD_MASTER_ACTIVATION);
    if (ok) {
        internal->first_refresh = false;
    }
    if (!ok) {
        LOG_E(TAG, "Failed to trigger refresh");
    }
    return ok;
}

/**
 * Write both RAM planes for the pending dirty window and trigger the refresh. A full baseline
 * carries the new frame on both planes (RED is bypassed anyway); a differential partial carries the
 * new frame on BW and the previously displayed frame on RED, so the LUT drives only changed pixels.
 * The old-frame shadow is refreshed before the trigger while the mutex is still held, so concurrent
 * draw_bitmap() calls cannot corrupt the baseline of the next differential.
 */
static bool refresh_panel(Gdeq0426t82Internal* internal) {
    int32_t x0, y0, x1, y1;
    if (!get_dirty_window(internal, &x0, &y0, &x1, &y1)) {
        return true;
    }
    const bool full_refresh = internal->first_refresh;
    const uint8_t* red_source = full_refresh ? internal->framebuffer : internal->old_framebuffer;
    bool ok = write_ram_window(internal, CMD_WRITE_RAM_BW, internal->framebuffer, x0, y0, x1, y1);
    ok = ok && write_ram_window(internal, CMD_WRITE_RAM_RED, red_source, x0, y0, x1, y1);
    if (ok) {
        copy_window(internal->old_framebuffer, internal->framebuffer, x0, y0, x1, y1);
        ok = trigger_refresh(internal);
    }
    internal->dirty_pending = false;
    if (!ok) {
        LOG_E(TAG, "Panel refresh failed");
    }
    return ok;
}

/** Explicit power-off sequence (0x22 0x83 + master activation). */
static bool panel_power_off(Gdeq0426t82Internal* internal) {
    bool ok = write_command(internal, CMD_UPDATE_CTRL2);
    ok = ok && write_data_byte(internal, POWER_OFF);
    ok = ok && write_command(internal, CMD_MASTER_ACTIVATION);
    if (!ok || !wait_while_busy(internal)) {
        LOG_E(TAG, "Panel did not confirm power-off");
        return false;
    }
    return true;
}

/** Power off and enter deep sleep; waking requires a hardware reset (init_panel). */
static bool enter_deep_sleep(Gdeq0426t82Internal* internal) {
    panel_power_off(internal);
    delay_millis(100);
    bool ok = write_command(internal, CMD_DEEP_SLEEP);
    ok = ok && write_data_byte(internal, DEEP_SLEEP_CODE);
    internal->initialized = false;
    return ok;
}

// endregion

// region Coalesced refresh

/**
 * Draws into the framebuffer shadow only (no SPI); the panel refresh happens later on the
 * coalesce timer, so LVGL's flush callback never blocks on the panel's long update time.
 */
static void coalesce_timer_cb(TimerHandle_t timer) {
    auto* internal = static_cast<Gdeq0426t82Internal*>(pvTimerGetTimerID(timer));
    if (!internal->timer_alive) {
        return;
    }

    xSemaphoreTake(internal->panel_mutex, portMAX_DELAY);
    if (!internal->display_on || !internal->dirty_pending || internal->refresh_in_flight) {
        xSemaphoreGive(internal->panel_mutex);
        return;
    }

    if (!internal->initialized) {
        // A hardware reset invalidated the controller while we were asleep; re-init (RAM and
        // the displayed image survive, so the panel keeps its old frame until this refresh).
        if (!init_panel(internal)) {
            xSemaphoreGive(internal->panel_mutex);
            return;
        }
    }

    internal->refresh_in_flight = true;
    if (internal->first_refresh) {
        // A full refresh drives every pixel from RAM, so it needs the whole plane written
        // first; the shadow always holds the complete frame (initialized white).
        internal->dirty_x0 = 0;
        internal->dirty_y0 = 0;
        internal->dirty_x1 = WIDTH;
        internal->dirty_y1 = HEIGHT;
    }
    const bool ok = refresh_panel(internal);
    xSemaphoreGive(internal->panel_mutex);

    // Wait outside the mutex so draw_bitmap() (and LVGL with it) never stalls on the panel's
    // ~500ms partial / ~1.8s full update. Draws during this window only touch the shadow and
    // re-arm the timer, so the next refresh picks up whatever changed.
    if (!ok || !wait_while_busy(internal)) {
        LOG_E(TAG, "Refresh did not complete");
    }

    xSemaphoreTake(internal->panel_mutex, portMAX_DELAY);
    internal->refresh_in_flight = false;
    xSemaphoreGive(internal->panel_mutex);
}

// endregion

// region DisplayApi

static error_t gdeq0426t82_reset(Device* device) {
    auto* internal = static_cast<Gdeq0426t82Internal*>(device_get_driver_data(device));
    xSemaphoreTake(internal->panel_mutex, portMAX_DELAY);
    while (internal->refresh_in_flight) {
        xSemaphoreGive(internal->panel_mutex);
        delay_millis(10);
        xSemaphoreTake(internal->panel_mutex, portMAX_DELAY);
    }
    // The reset pulse restores register defaults; the next refresh re-runs init_panel and a
    // full baseline update.
    hardware_reset(internal);
    internal->initialized = false;
    internal->first_refresh = true;
    internal->dirty_x0 = 0;
    internal->dirty_y0 = 0;
    internal->dirty_x1 = WIDTH;
    internal->dirty_y1 = HEIGHT;
    internal->dirty_pending = true;
    xSemaphoreGive(internal->panel_mutex);
    return ERROR_NONE;
}

static error_t gdeq0426t82_init(Device* device) {
    auto* internal = static_cast<Gdeq0426t82Internal*>(device_get_driver_data(device));
    xSemaphoreTake(internal->panel_mutex, portMAX_DELAY);
    while (internal->refresh_in_flight) {
        xSemaphoreGive(internal->panel_mutex);
        delay_millis(10);
        xSemaphoreTake(internal->panel_mutex, portMAX_DELAY);
    }
    const bool ok = init_panel(internal);
    if (ok) {
        internal->first_refresh = true;
        internal->dirty_x0 = 0;
        internal->dirty_y0 = 0;
        internal->dirty_x1 = WIDTH;
        internal->dirty_y1 = HEIGHT;
        internal->dirty_pending = true;
    }
    xSemaphoreGive(internal->panel_mutex);
    return ok ? ERROR_NONE : ERROR_RESOURCE;
}

/**
 * LVGL calls this with one independent tile per flush (PARTIAL mode, GRAYSCALE8): color_data is
 * the tile's top-left pixel, row-major with stride = tile width. Each pixel is hard-quantized to
 * 4 levels (GRAYSCALE8 >> 6) and spatially dithered via a Bayer 4x4 matrix onto the 1bpp shadow
 * (bit 1 = white, matching both the panel's B/W polarity and LVGL's own I1 convention).
 * No SPI happens here: only the shadow and the dirty bbox are updated, and the coalesce timer is
 * re-armed. Boundary bytes of partial windows are always complete because they were merged with
 * the shadow, never read back from the panel (it has no MISO).
 */
static error_t gdeq0426t82_draw_bitmap(Device* device, int32_t x_start, int32_t y_start, int32_t x_end, int32_t y_end, const void* color_data) {
    auto* internal = static_cast<Gdeq0426t82Internal*>(device_get_driver_data(device));

    const int32_t x0 = x_start < 0 ? 0 : x_start;
    const int32_t y0 = y_start < 0 ? 0 : y_start;
    const int32_t x1 = x_end > WIDTH ? WIDTH : x_end;
    const int32_t y1 = y_end > HEIGHT ? HEIGHT : y_end;
    if (x1 <= x0 || y1 <= y0) {
        return ERROR_NONE;
    }

    const auto* source = static_cast<const uint8_t*>(color_data);
    const int32_t stride = x_end - x_start;

    xSemaphoreTake(internal->panel_mutex, portMAX_DELAY);
    for (int32_t y = y0; y < y1; y++) {
        const uint8_t* src_row = source + (size_t)(y - y_start) * stride;
        for (int32_t x = x0; x < x1; x++) {
            uint8_t level = src_row[x - x_start] >> 6;
            if (level > 3) {
                level = 3;
            }
            const bool black = dither_pixel_black(x, y, level);
            uint8_t& byte = internal->framebuffer[(size_t)y * BYTES_PER_ROW + (x >> 3)];
            const uint8_t mask = (uint8_t)(1u << (7 - (x & 7)));
            if (black) {
                byte &= (uint8_t)~mask;
            } else {
                byte |= mask;
            }
        }
    }

    if (!internal->dirty_pending) {
        internal->dirty_x0 = x0;
        internal->dirty_y0 = y0;
        internal->dirty_x1 = x1;
        internal->dirty_y1 = y1;
        internal->dirty_pending = true;
    } else {
        if (x0 < internal->dirty_x0) {
            internal->dirty_x0 = x0;
        }
        if (y0 < internal->dirty_y0) {
            internal->dirty_y0 = y0;
        }
        if (x1 > internal->dirty_x1) {
            internal->dirty_x1 = x1;
        }
        if (y1 > internal->dirty_y1) {
            internal->dirty_y1 = y1;
        }
    }

    // Coalesce consecutive tiles (LVGL flushes one per refresh slice) and consecutive refresh
    // cycles into a single panel update, so a full frame redraw is one partial refresh.
    if (internal->coalesce_timer != nullptr) {
        xTimerReset(internal->coalesce_timer, 0);
    }
    xSemaphoreGive(internal->panel_mutex);
    return ERROR_NONE;
}

static error_t gdeq0426t82_disp_on_off(Device* device, bool on_off) {
    auto* internal = static_cast<Gdeq0426t82Internal*>(device_get_driver_data(device));

    xSemaphoreTake(internal->panel_mutex, portMAX_DELAY);
    while (internal->refresh_in_flight) {
        xSemaphoreGive(internal->panel_mutex);
        delay_millis(10);
        xSemaphoreTake(internal->panel_mutex, portMAX_DELAY);
    }
    if (on_off == internal->display_on) {
        xSemaphoreGive(internal->panel_mutex);
        return ERROR_NONE;
    }

    bool ok = true;
    if (on_off) {
        // Wake from deep sleep: reload registers. The hardware reset in init_panel discards the
        // panel RAM (and the displayed image), so force the next refresh to a full baseline rather
        // than diffing a partial against post-reset garbage.
        ok = init_panel(internal);
        internal->first_refresh = true;
        if (ok) {
            // Draws while the panel slept kept the shadow current; mark the whole panel dirty
            // so a single refresh catches up, and arm the timer so it runs even without a new draw.
            internal->dirty_x0 = 0;
            internal->dirty_y0 = 0;
            internal->dirty_x1 = WIDTH;
            internal->dirty_y1 = HEIGHT;
            internal->dirty_pending = true;
            if (internal->coalesce_timer != nullptr) {
                xTimerReset(internal->coalesce_timer, 0);
            }
        }
    } else {
        if (internal->coalesce_timer != nullptr) {
            xTimerStop(internal->coalesce_timer, 0);
        }
        ok = enter_deep_sleep(internal);
    }

    if (ok) {
        internal->display_on = on_off;
    }

    xSemaphoreGive(internal->panel_mutex);
    return ok ? ERROR_NONE : ERROR_RESOURCE;
}

static DisplayColorFormat gdeq0426t82_get_color_format(Device*) {
    return DISPLAY_COLOR_FORMAT_GRAYSCALE8;
}

static uint16_t gdeq0426t82_get_resolution_x(Device*) {
    return WIDTH;
}

static uint16_t gdeq0426t82_get_resolution_y(Device*) {
    return HEIGHT;
}

static void gdeq0426t82_get_frame_buffer(Device*, uint8_t, void** out_buffer) {
    *out_buffer = nullptr;
}

static uint8_t gdeq0426t82_get_frame_buffer_count(Device*) {
    return 0;
}

// endregion

static const DisplayApi gdeq0426t82_display_api = {
    .capabilities = DISPLAY_CAPABILITY_ON_OFF | DISPLAY_CAPABILITY_SLOW_REFRESH | DISPLAY_CAPABILITY_MINIMAL_BUFFER,
    .reset = gdeq0426t82_reset,
    .init = gdeq0426t82_init,
    .draw_bitmap = gdeq0426t82_draw_bitmap,
    .mirror = nullptr,
    .swap_xy = nullptr,
    .get_swap_xy = nullptr,
    .get_mirror_x = nullptr,
    .get_mirror_y = nullptr,
    .set_gap = nullptr,
    .get_gap_x = nullptr,
    .get_gap_y = nullptr,
    .invert_color = nullptr,
    .disp_on_off = gdeq0426t82_disp_on_off,
    .disp_sleep = nullptr,
    .get_color_format = gdeq0426t82_get_color_format,
    .get_resolution_x = gdeq0426t82_get_resolution_x,
    .get_resolution_y = gdeq0426t82_get_resolution_y,
    .get_frame_buffer = gdeq0426t82_get_frame_buffer,
    .get_frame_buffer_count = gdeq0426t82_get_frame_buffer_count,
    .get_backlight = nullptr,
    .has_capability = nullptr,
};

// region Driver lifecycle

static void free_internal(Gdeq0426t82Internal* internal) {
    if (internal->spi_device != nullptr) {
        spi_bus_remove_device(internal->spi_device);
    }
    if (internal->dc != nullptr) {
        gpio_descriptor_release(internal->dc);
    }
    if (internal->reset != nullptr) {
        gpio_descriptor_release(internal->reset);
    }
    if (internal->busy != nullptr) {
        gpio_descriptor_release(internal->busy);
    }
    if (internal->panel_mutex != nullptr) {
        vSemaphoreDelete(internal->panel_mutex);
    }
    free(internal->framebuffer);
    free(internal->old_framebuffer);
    free(internal->band_buffer);
    free(internal);
}

static error_t start(Device* device) {
    auto* parent = device_get_parent(device);
    check(device_get_type(parent) == &SPI_CONTROLLER_TYPE);

    const auto* spi_config = static_cast<const Esp32SpiConfig*>(parent->config);
    const auto* config = GET_CONFIG(device);

    struct GpioPinSpec cs_pin;
    if (esp32_spi_get_cs_pin(device, &cs_pin) != ERROR_NONE) {
        LOG_E(TAG, "Failed to resolve CS pin");
        return ERROR_RESOURCE;
    }

    auto* internal = static_cast<Gdeq0426t82Internal*>(calloc(1, sizeof(Gdeq0426t82Internal)));
    if (internal == nullptr) {
        return ERROR_OUT_OF_MEMORY;
    }

    internal->dc = gpio_descriptor_acquire(
        config->pin_dc.gpio_controller,
        config->pin_dc.pin,
        config->pin_dc.flags | GPIO_FLAG_DIRECTION_OUTPUT,
        GPIO_OWNER_GPIO
    );

    if (config->pin_reset.gpio_controller != nullptr) {
        internal->reset = gpio_descriptor_acquire(
            config->pin_reset.gpio_controller,
            config->pin_reset.pin,
            config->pin_reset.flags | GPIO_FLAG_DIRECTION_OUTPUT,
            GPIO_OWNER_GPIO
        );
    } else {
        internal->reset = nullptr;
    }

    internal->busy = gpio_descriptor_acquire(
        config->pin_busy.gpio_controller,
        config->pin_busy.pin,
        config->pin_busy.flags | GPIO_FLAG_DIRECTION_INPUT,
        GPIO_OWNER_GPIO
    );

    internal->panel_mutex = xSemaphoreCreateMutex();
    internal->framebuffer = static_cast<uint8_t*>(malloc(FRAMEBUFFER_SIZE));
    internal->old_framebuffer = static_cast<uint8_t*>(malloc(FRAMEBUFFER_SIZE));
    internal->band_buffer = static_cast<uint8_t*>(malloc(BAND_SIZE));
    internal->coalesce_timer = xTimerCreate("gdeq0426", COALESCE_PERIOD_MS, pdFALSE, internal, coalesce_timer_cb);
    if (
        internal->dc == nullptr ||
        internal->busy == nullptr ||
        internal->panel_mutex == nullptr ||
        internal->framebuffer == nullptr ||
        internal->old_framebuffer == nullptr ||
        internal->band_buffer == nullptr ||
        internal->coalesce_timer == nullptr
        // Note: reset pin is not checked as it is optional
    ) {
        LOG_E(TAG, "Failed to acquire GPIO pins or allocate buffers");
        free_internal(internal);
        return ERROR_OUT_OF_MEMORY;
    }
    internal->timer_alive = true;

    gpio_descriptor_set_flags(internal->dc, GPIO_FLAG_DIRECTION_OUTPUT);
    gpio_descriptor_set_flags(internal->busy, GPIO_FLAG_DIRECTION_INPUT);
    if (internal->reset != nullptr) {
        gpio_descriptor_set_flags(internal->reset, GPIO_FLAG_DIRECTION_OUTPUT);
    }

    spi_device_interface_config_t device_config = {};
    device_config.mode = 0;
    device_config.clock_speed_hz = config->clock_speed_hz;
    device_config.spics_io_num = cs_pin.gpio_controller == nullptr ? -1 : static_cast<int>(cs_pin.pin);
    device_config.queue_size = 1;

    if (spi_bus_add_device(spi_config->host, &device_config, &internal->spi_device) != ESP_OK) {
        LOG_E(TAG, "Failed to add SPI device");
        free_internal(internal);
        return ERROR_RESOURCE;
    }

    // Both shadows start blank (white); the first full refresh drives the whole panel from the
    // current one and copies it into the old one, seeding the differential baseline.
    std::memset(internal->framebuffer, 0xFF, FRAMEBUFFER_SIZE);
    std::memset(internal->old_framebuffer, 0xFF, FRAMEBUFFER_SIZE);

    if (!init_panel(internal)) {
        free_internal(internal);
        return ERROR_RESOURCE;
    }

    internal->display_on = true;
    internal->first_refresh = true;

    device_set_driver_data(device, internal);
    return ERROR_NONE;
}

static error_t stop(Device* device) {
    auto* internal = static_cast<Gdeq0426t82Internal*>(device_get_driver_data(device));

    // Stop the timer first: clear the alive flag so no callback starts, then block until the
    // timer task has finished any callback that was already running. The callback may be parked
    // in a ~500ms BUSY wait, but it only touches `internal` (still valid here) until it returns.
    internal->timer_alive = false;
    if (internal->coalesce_timer != nullptr) {
        xTimerDelete(internal->coalesce_timer, pdMS_TO_TICKS(100));
        internal->coalesce_timer = nullptr;
    }

    xSemaphoreTake(internal->panel_mutex, portMAX_DELAY);
    while (internal->refresh_in_flight) {
        xSemaphoreGive(internal->panel_mutex);
        delay_millis(10);
        xSemaphoreTake(internal->panel_mutex, portMAX_DELAY);
    }
    if (internal->display_on) {
        enter_deep_sleep(internal);
        internal->display_on = false;
    }
    xSemaphoreGive(internal->panel_mutex);

    free_internal(internal);
    device_set_driver_data(device, nullptr);
    return ERROR_NONE;
}

// endregion

Driver gdeq0426t82_driver = {
    .name = "gdeq0426t82",
    .compatible = (const char*[]) {"gooddisplay,gdeq0426t82", nullptr},
    .start_device = start,
    .stop_device = stop,
    .api = &gdeq0426t82_display_api,
    .device_type = &DISPLAY_TYPE,
    .owner = &gdeq0426t82_module,
    .internal = nullptr
};
}

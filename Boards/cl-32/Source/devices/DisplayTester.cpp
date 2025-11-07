 url=https://github.com/NellowTCS/Tactility/blob/395ff83aef3679c3adb51ce6f3bd598adbd6f8f3/Boards/cl-32/Source/devices/DisplayTester.cpp
#include "DisplayTester.h"
#include "GxEPD2Display.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <lvgl.h>

static const char* TAG = "DisplayTester";

namespace display_tester {

static void test_fullscreen_fill(GxEPD2Display* disp, uint8_t fill)
{
    if (!disp) return;
    const int w = disp->getWidth();
    const int h = disp->getHeight();
    const int row_bytes = (w + 7) / 8;
    size_t total = (size_t)row_bytes * (size_t)h;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(total, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "alloc failed for fullscreen fill");
        return;
    }
    memset(buf, fill, total);
    ESP_LOGI(TAG, "Full-screen write: fill=0x%02X w=%d h=%d row_bytes=%d", fill, w, h, row_bytes);
    disp->writeRawImage(buf, 0, 0, w, h, false, false);
    disp->refreshDisplay(false); // full refresh to make it visible
    heap_caps_free(buf);
}

static void test_three_stripes(GxEPD2Display* disp)
{
    if (!disp) return;
    const int w = disp->getWidth();
    const int h = disp->getHeight();
    int stripe_w = w / 3;
    const int row_bytes = (stripe_w + 7) / 8;
    ESP_LOGI(TAG, "Three stripes test: width=%d height=%d stripe_w=%d", w, h, stripe_w);

    uint8_t* left = (uint8_t*)heap_caps_malloc(row_bytes * h, MALLOC_CAP_DMA);
    uint8_t* mid  = (uint8_t*)heap_caps_malloc(row_bytes * h, MALLOC_CAP_DMA);
    uint8_t* right = (uint8_t*)heap_caps_malloc(row_bytes * h, MALLOC_CAP_DMA);
    if (!left || !mid || !right) {
        ESP_LOGE(TAG, "stripe alloc fail");
        if (left) heap_caps_free(left);
        if (mid) heap_caps_free(mid);
        if (right) heap_caps_free(right);
        return;
    }

    memset(left,  0x00, row_bytes * h); // black
    memset(mid,   0xFF, row_bytes * h); // white
    memset(right, 0x00, row_bytes * h); // black

    disp->writeRawImage(left, 0, 0, stripe_w, h, false, false);
    disp->writeRawImage(mid, stripe_w, 0, stripe_w, h, false, false);
    int right_w = w - 2*stripe_w;
    disp->writeRawImage(right, 2*stripe_w, 0, right_w, h, false, false);

    disp->refreshDisplay(false);

    heap_caps_free(left);
    heap_caps_free(mid);
    heap_caps_free(right);
}

// Write each scanline with a unique byte value so repeated/shifted regions
// are easy to spot. For row r we fill every byte in that row with the value (uint8_t)r.
// This is a low-cost test (one buffer) that highlights stride/rotation/source-shift bugs.
static void test_scanline_pattern(GxEPD2Display* disp)
{
    if (!disp) return;
    const int w = disp->getWidth();
    const int h = disp->getHeight();
    const int row_bytes = (w + 7) / 8;
    size_t total = (size_t)row_bytes * (size_t)h;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(total, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "alloc failed for scanline pattern");
        return;
    }

    ESP_LOGI(TAG, "Scanline pattern test: w=%d h=%d row_bytes=%d total=%u", w, h, row_bytes, (unsigned)total);

    // Fill each row with a unique value. This will create horizontal bands of different
    // byte patterns; if the driver writes using incorrect stride/orientation the bands
    // will appear duplicated, wrapped, or shifted.
    for (int r = 0; r < h; ++r) {
        uint8_t value = (uint8_t)(r & 0xFF);
        uint8_t* row_ptr = buf + (size_t)r * row_bytes;
        memset(row_ptr, value, row_bytes);
    }

    // Log a small sample of the first few bytes for debugging visibility
    if (h > 0 && row_bytes > 0) {
        ESP_LOGI(TAG, "First row sample bytes: 0x%02X 0x%02X 0x%02X", buf[0], buf[1 % row_bytes], buf[2 % row_bytes]);
    }

    // Write the full buffer to the panel and perform a full refresh
    disp->writeRawImage(buf, 0, 0, w, h, false, false);
    disp->refreshDisplay(false); // full refresh so it's easy to observe
    heap_caps_free(buf);
}

// LVGL test: create a simple LVGL screen with three horizontal bands and a centered label.
// This test requires LVGL to be running and the display to have an LVGL display object.
static void test_lvgl(GxEPD2Display* disp)
{
    if (!disp) return;
    lv_display_t* lvdisp = disp->getLvglDisplay();
    if (!lvdisp) {
        ESP_LOGI(TAG, "LVGL display not available - skipping LVGL test");
        return;
    }

    ESP_LOGI(TAG, "Starting LVGL test: creating simple screen");

    // Create a new screen object
    lv_obj_t* scr = lv_obj_create(NULL);

    // Logical dimensions according to LVGL will be the display's LVGL resolution.
    // Use the physical panel dimensions for sizing the bands so they map to the panel.
    const int phys_w = disp->getWidth();
    const int phys_h = disp->getHeight();
    const int band_h = phys_h / 3;

    for (int i = 0; i < 3; ++i) {
        lv_obj_t* band = lv_obj_create(scr);
        lv_obj_set_size(band, phys_w, band_h);
        lv_obj_set_pos(band, 0, i * band_h);

        // Set band background color: alternating black/white/black for high contrast on e-paper
        if (i == 0 || i == 2) {
            lv_obj_set_style_bg_color(band, lv_color_black(), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(band, LV_OPA_COVER, LV_PART_MAIN);
        } else {
            lv_obj_set_style_bg_color(band, lv_color_white(), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(band, LV_OPA_COVER, LV_PART_MAIN);
        }

        // Remove borders/margins for a clean band look
        lv_obj_set_style_border_width(band, 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(band, 0, LV_PART_MAIN);
    }

    // Add a centered label
    lv_obj_t* label = lv_label_create(scr);
    lv_label_set_text(label, "LVGL Test");
    lv_obj_set_style_text_font(label, lv_font_get_default(), LV_PART_MAIN);
    lv_obj_center(label);

    // Load the screen (LVGL will render next refresh)
    lv_scr_load(scr);

    ESP_LOGI(TAG, "LVGL test screen created and loaded");

    vTaskDelay(pdMS_TO_TICKS(2000));
}

// Public helper so callers can explicitly run the LVGL test when LVGL is up
void runLvglTest(GxEPD2Display* display)
{
    test_lvgl(display);
}

// Public helper to run the scanline diagnostic
void runScanlineTest(GxEPD2Display* display)
{
    test_scanline_pattern(display);
}

void runTests(GxEPD2Display* display)
{
    ESP_LOGI(TAG, "=== perform_display_tests START ===");
    // white then black full-screen to validate basic behaviour
    test_fullscreen_fill(display, 0xFF);
    vTaskDelay(pdMS_TO_TICKS(300));
    test_fullscreen_fill(display, 0x00);
    vTaskDelay(pdMS_TO_TICKS(700));
    // 3 stripes (left/mid/right) so you can see mapping in one shot
    test_three_stripes(display);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Scanline diagnostic to reveal stride/rotation/shift issues
    test_scanline_pattern(display);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 4. Clear screen to White (0xFF)
    test_fullscreen_fill(display, 0xFF);
    vTaskDelay(pdMS_TO_TICKS(300));

    // Attempt LVGL test if LVGL is available; this is safe to call even if LVGL isn't running,
    // the test function will detect and skip in that case.
    test_lvgl(display);
    vTaskDelay(pdMS_TO_TICKS(300));

    // 5. Clear screen to White (0xFF)
    test_fullscreen_fill(display, 0xFF);
    vTaskDelay(pdMS_TO_TICKS(300));

    ESP_LOGI(TAG, "=== perform_display_tests END ===");
}

} // namespace display_tester

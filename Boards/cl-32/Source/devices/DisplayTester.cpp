#include "DisplayTester.h"
#include "GxEPD2Display.h"

#include <esp_log.h>
#include <esp_heap_caps.h>
#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "DisplayTester";

namespace display_tester {

static void test_fullscreen_fill(GxEPD2Display* self, uint8_t fill)
{
    if (!self || !self->_display) return;
    const int w = self->_config.width;
    const int h = self->_config.height;
    const int row_bytes = (w + 7) / 8;
    size_t total = (size_t)row_bytes * (size_t)h;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(total, MALLOC_CAP_DMA);
    if (!buf) {
        ESP_LOGE(TAG, "alloc failed for fullscreen fill");
        return;
    }
    memset(buf, fill, total);
    ESP_LOGI(TAG, "Full-screen write: fill=0x%02X w=%d h=%d row_bytes=%d", fill, w, h, row_bytes);
    self->_display->writeImage(buf, 0, 0, w, h, false, false, false);
    self->_display->refresh(false); // full refresh to make it visible
    heap_caps_free(buf);
}

static void test_three_stripes(GxEPD2Display* self)
{
    if (!self || !self->_display) return;
    const int w = self->_config.width;
    const int h = self->_config.height;
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

    // GxEPD2 typical convention: 0x00 = black, 0xFF = white
    memset(left,  0x00, row_bytes * h); // black
    memset(mid,   0xFF, row_bytes * h); // white
    memset(right, 0x00, row_bytes * h); // black

    // left
    self->_display->writeImage(left, 0, 0, stripe_w, h, false, false, false);
    // mid
    self->_display->writeImage(mid, stripe_w, 0, stripe_w, h, false, false, false);
    // right (may be slightly wider if w not divisible by 3)
    int right_w = w - 2*stripe_w;
    self->_display->writeImage(right, 2*stripe_w, 0, right_w, h, false, false, false);

    self->_display->refresh(false);

    heap_caps_free(left);
    heap_caps_free(mid);
    heap_caps_free(right);
}

static void test_narrow_scan(GxEPD2Display* self)
{
    if (!self || !self->_display) return;
    const int w = self->_config.width;
    const int h = self->_config.height;
    const int stripe_w = 8;
    const int row_bytes = (stripe_w + 7) / 8;
    uint8_t* buf = (uint8_t*)heap_caps_malloc(row_bytes * h, MALLOC_CAP_DMA);
    if (!buf) { ESP_LOGE(TAG,"alloc fail"); return; }
    memset(buf, 0x00, row_bytes * h); // black stripe
    ESP_LOGI(TAG,"Narrow scan start w=%d h=%d", w, h);
    for (int x = 0; x + stripe_w <= w; x += stripe_w) {
        self->_display->writeImage(buf, x, 0, stripe_w, h, false, false, false);
        self->_display->refresh(false);
        vTaskDelay(pdMS_TO_TICKS(900)); // pause so you can observe
        // clear after each step
        uint8_t* white = (uint8_t*)heap_caps_malloc(row_bytes * h, MALLOC_CAP_DMA);
        memset(white, 0xFF, row_bytes * h);
        self->_display->writeImage(white, x, 0, stripe_w, h, false, false, false);
        self->_display->refresh(false);
        heap_caps_free(white);
    }
    heap_caps_free(buf);
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
    // run narrow scan (slow) for very fine mapping
    // test_narrow_scan(display);
    ESP_LOGI(TAG, "=== perform_display_tests END ===");
}

} // namespace display_tester
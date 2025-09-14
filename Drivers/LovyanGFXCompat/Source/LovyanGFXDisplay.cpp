#include "LovyanGFXDisplay.h"
#include <Tactility/Check.h>
#include <Tactility/Log.h>
#include <lgfx/v1/misc/pixelcopy.hpp>

static const char* TAG = "LovyanGFXDisplay";


LovyanGFXDisplay::~LovyanGFXDisplay() {
    if (panel != nullptr && panel.use_count() > 1) {
        tt_crash("Panel_LCD is still in use. This will cause memory access violations.");
    }
}

bool LovyanGFXDisplay::start() {
    if (!createPanel(panel)) {
        TT_LOG_E(TAG, "Failed to create LovyanGFX panel");
        return false;
    }
    if (!panel->init(true)) {
        TT_LOG_E(TAG, "Failed to init panel");
        return false;
    }
    return true;
}

bool LovyanGFXDisplay::stop() {
    if (lvglDisplay != nullptr) {
        stopLvgl();
        lvglDisplay = nullptr;
    }

    panel = nullptr;
    return true;
}

bool LovyanGFXDisplay::startLvgl() {
    assert(lvglDisplay == nullptr);

    // Create LVGL draw buffer - use a much smaller buffer size for ESP32 memory constraints
    // Start with just 1 row of with just 1 row of display as buffer size as buffer size (minimum viable size)
    size_t buf_size = panel->width() * 10;  // Just 10 rows worth of pixels
    
    // Log buffer allocation details for debugging
    TT_LOG_I(TAG, "Allocating display buffers: %d bytes each (screen: %dx%d, rows: 10)", 
             buf_size * sizeof(lv_color_t), panel->width(), panel->height());
    
    // Use MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL to ensure allocation in internal memory
    // We'll attempt only one buffer first to ensure we can allocate it successfully
    void* buf1 = heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    void* buf2 = nullptr;
    
    // Only try to allocate second buffer if first one succeeded
    if (buf1) {
        buf2 = heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    }
    
    if (!buf1 || !buf2) {
        // Free any buffer that was successfully allocated
        if (buf1) heap_caps_free(buf1);
        if (buf2) heap_caps_free(buf2);
        
        // Log the error with more details
        TT_LOG_E(TAG, "Failed to allocate display buffers (requested %d bytes each)",
                buf_size * sizeof(lv_color_t));
        return false;
    }
    
    // Create two draw buffers
    lv_draw_buf_t* draw_buf1 = lv_draw_buf_create(
        panel->width(),
        panel->height(),
        LV_COLOR_FORMAT_RGB565,
        panel->width() * sizeof(uint16_t)
    );
    lv_draw_buf_t* draw_buf2 = lv_draw_buf_create(
        panel->width(),
        panel->height(),
        LV_COLOR_FORMAT_RGB565,
        panel->width() * sizeof(uint16_t)
    );
    
    if (!draw_buf1 || !draw_buf2) {
        TT_LOG_E(TAG, "Failed to create draw buffers");
        if (draw_buf1) lv_draw_buf_destroy(draw_buf1);
        if (draw_buf2) lv_draw_buf_destroy(draw_buf2);
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        return false;
    }
    
    // Initialize both draw buffers
    lv_draw_buf_init(draw_buf1, panel->width(), panel->height(), LV_COLOR_FORMAT_RGB565, 
                     panel->width() * sizeof(uint16_t), buf1, buf_size * sizeof(lv_color_t));
                     
    lv_draw_buf_init(draw_buf2, panel->width(), panel->height(), LV_COLOR_FORMAT_RGB565, 
                     panel->width() * sizeof(uint16_t), buf2, buf_size * sizeof(lv_color_t));
    
    // Store the first buffer for cleanup later
    lv_draw_buf = draw_buf1;
    
    // Create display driver
    lvglDisplay = lv_display_create(panel->width(), panel->height());
    if (!lvglDisplay) {
        TT_LOG_E(TAG, "Failed to create display");
        lv_draw_buf_destroy(draw_buf1);
        lv_draw_buf_destroy(draw_buf2);
        heap_caps_free(buf1);
        heap_caps_free(buf2);
        lv_draw_buf = nullptr;
        return false;
    }
    
    lv_display_set_user_data(lvglDisplay, this);
    lv_display_set_draw_buffers(lvglDisplay, draw_buf1, draw_buf2);
    lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_RGB565);
    
    lv_display_set_flush_cb(lvglDisplay, [](lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
        auto display = static_cast<LovyanGFXDisplay*>(lv_display_get_user_data(disp));
        
        // Setup the pixel copy parameters
        lgfx::pixelcopy_t pixelcopy;
        pixelcopy.src_data = px_map;
        pixelcopy.src_width = area->x2 - area->x1 + 1;
        pixelcopy.src_height = area->y2 - area->y1 + 1;
        pixelcopy.src_bits = 16; // RGB565
        pixelcopy.src_x = 0;
        pixelcopy.src_y = 0;
        
        // Write the image to the panel
        display->panel->writeImage(
            area->x1, area->y1,
            area->x2 - area->x1 + 1,
            area->y2 - area->y1 + 1,
            &pixelcopy,
            true // Use DMA for faster transfers
        );
        
        lv_display_flush_ready(disp);
    });
    
    return true;
}

bool LovyanGFXDisplay::stopLvgl() {
    if (lvglDisplay == nullptr) return false;

    // Free the draw buffer memory
    if (lv_draw_buf) {
        // In our initialization, we stored these buffers
        void* data1 = buf1;  // First buffer we allocated directly
        void* data2 = buf2;  // Second buffer we allocated directly
        
        // Free the memory we allocated
        if (data1) heap_caps_free(data1);
        if (data2) heap_caps_free(data2);
        
        // Destroy the draw buffers - they're managed by the display but we have the pointer to the first one
        lv_draw_buf_destroy(lv_draw_buf);
        lv_draw_buf = nullptr;
    }

    // Destroy the display
    if (lvglDisplay) {
        lv_display_delete(lvglDisplay);
        lvglDisplay = nullptr;
    }

    return true;
}

#include "LovyanGFXDisplayDriver.h"

std::shared_ptr<tt::hal::display::DisplayDriver> LovyanGFXDisplay::getDisplayDriver() {
    assert(lvglDisplay == nullptr); // Must stop LVGL first
    return std::make_shared<LovyanGFXDisplayDriver>(panel);
}

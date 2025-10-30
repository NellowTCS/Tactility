#include "Display.h"
#include "GxEPD2/GxEPD2_290_GDEY029T71H.h"
#include <Tactility/hal/Configuration.h>

#define DISP_BUF_SIZE (168 * 384 / 8)  // Monochrome: 1 bit per pixel

class GxEPD2Display : public tt::hal::display::DisplayDevice {
private:
    GxEPD2_290_GDEY029T71H display;
    lv_display_t* lvglDisplay = nullptr;
    lv_color_t* draw_buf1 = nullptr;
    lv_color_t* draw_buf2 = nullptr;
    
    static constexpr size_t DRAW_BUF_SIZE = 168 * 10; // 10 lines buffer

    static void lvglFlushCallback(lv_display_t* disp, const lv_area_t* area, uint8_t* px_map) {
        auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(disp));
        
        int16_t x = area->x1;
        int16_t y = area->y1;
        int16_t w = area->x2 - area->x1 + 1;
        int16_t h = area->y2 - area->y1 + 1;
        
        // Write the monochrome bitmap to e-paper
        self->display.writeImage(px_map, x, y, w, h, false, false, false);
        
        // For e-paper, we typically only refresh at the end, not on every flush
        // But for testing, let's do a partial refresh
        self->display.refresh(true); // partial update
        
        lv_display_flush_ready(disp);
    }

public:
    GxEPD2Display() : display(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY) {}

    std::string getName() const override { return "GxEPD2"; }
    std::string getDescription() const override { return "E-paper display (GDEY029T71H)"; }

    bool start() override {
        // Get SPI configuration from Tactility HAL
        auto& halConfig = tt::hal::getConfiguration();
        
        // Configure the display to use Tactility's SPI
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
            .spics_io_num = EPD_CS,
            .flags = 0,
            .queue_size = 7,
            .pre_cb = nullptr,
            .post_cb = nullptr
        };
        
        display.selectSPI(SPI2_HOST, devcfg);
        
        // Initialize display (this will handle SPI device registration)
        display.init(0); // 0 = no serial diagnostics
        
        // Clear the display initially
        display.clearScreen(0xFF); // White
        display.refresh(false); // Full refresh
        
        return true;
    }

    bool stop() override {
        display.hibernate();
        return true;
    }

    std::shared_ptr<tt::hal::touch::TouchDevice> _Nullable getTouchDevice() override { 
        return nullptr; 
    }

    bool supportsLvgl() const override { return true; }
    
    bool startLvgl() override {
        if (lvglDisplay != nullptr) return false;
        
        // Allocate draw buffers
        draw_buf1 = (lv_color_t*)heap_caps_malloc(DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
        draw_buf2 = (lv_color_t*)heap_caps_malloc(DRAW_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
        
        if (!draw_buf1 || !draw_buf2) {
            if (draw_buf1) free(draw_buf1);
            if (draw_buf2) free(draw_buf2);
            return false;
        }
        
        // Create LVGL display
        lvglDisplay = lv_display_create(display.WIDTH, display.HEIGHT);
        if (!lvglDisplay) {
            free(draw_buf1);
            free(draw_buf2);
            return false;
        }
        
        // Set color format to monochrome
        lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_I1);
        
        // Set draw buffers
        lv_display_set_buffers(lvglDisplay, draw_buf1, draw_buf2, DRAW_BUF_SIZE * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
        
        // Set flush callback
        lv_display_set_flush_cb(lvglDisplay, lvglFlushCallback);
        lv_display_set_user_data(lvglDisplay, this);
        
        return true;
    }
    
    bool stopLvgl() override {
        if (lvglDisplay != nullptr) {
            lv_display_delete(lvglDisplay);
            lvglDisplay = nullptr;
        }
        if (draw_buf1) {
            free(draw_buf1);
            draw_buf1 = nullptr;
        }
        if (draw_buf2) {
            free(draw_buf2);
            draw_buf2 = nullptr;
        }
        return true;
    }
    
    lv_display_t* _Nullable getLvglDisplay() const override { 
        return lvglDisplay; 
    }

    bool supportsDisplayDriver() const override { return false; }
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable getDisplayDriver() override { 
        return nullptr; 
    }
};

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    return std::make_shared<GxEPD2Display>();
}

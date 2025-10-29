#include "Display.h"
#include "GxEPD2/GxEPD2_290_GDEY029T71H.h"

class GxEPD2Display : public tt::hal::display::DisplayDevice {
private:
    GxEPD2_290_GDEY029T71H display; // The ESP-IDF ported GxEPD2 instance
    lv_display_t* lvglDisplay = nullptr; // LVGL display handle

    static void lvglFlushCallback(lv_display_t* display, const lv_area_t* area, uint8_t* px_map) {
        auto* self = static_cast<GxEPD2Display*>(lv_display_get_user_data(display));
        int16_t w = area->x2 - area->x1 + 1;
        int16_t h = area->y2 - area->y1 + 1;
        // Write the monochrome bitmap to e-paper
        self->display.writeImage(px_map, area->x1, area->y1, w, h);
        // Refresh the display (use partial if possible for better performance)
        if (w == self->display.WIDTH && h == self->display.HEIGHT) {
            self->display.refresh(false); // Full refresh for full screen
        } else {
            self->display.refresh(true); // Partial refresh
        }
        lv_display_flush_ready(display);
    }

public:
    GxEPD2Display() : display(EPD_CS, EPD_DC, EPD_RST, EPD_BUSY) {}

    std::string getName() const override { return "GxEPD2"; }
    std::string getDescription() const override { return "E-paper display (GDEY029T71H)"; }

    bool start() override {
        display.init(0); // Serial diag bitrate 0 (disabled, uses ESP-IDF logging)
        return true;
    }

    bool stop() override {
        display.hibernate(); // Power down the display
        return true;
    }

    std::shared_ptr<tt::hal::touch::TouchDevice> _Nullable getTouchDevice() override { return nullptr; } // No touch for e-paper

    // Enable LVGL support for monochrome
    bool supportsLvgl() const override { return true; }
    bool startLvgl() override {
        if (lvglDisplay != nullptr) return false;
        lvglDisplay = lv_display_create(display.WIDTH, display.HEIGHT);
        lv_display_set_color_format(lvglDisplay, LV_COLOR_FORMAT_I1); // Monochrome 1-bit
        lv_display_set_flush_cb(lvglDisplay, lvglFlushCallback);
        lv_display_set_user_data(lvglDisplay, this);
        return true;
    }
    bool stopLvgl() override {
        if (lvglDisplay != nullptr) {
            lv_display_delete(lvglDisplay);
            lvglDisplay = nullptr;
        }
        return true;
    }
    lv_display_t* _Nullable getLvglDisplay() const override { return lvglDisplay; }

    // DisplayDriver not implemented
    bool supportsDisplayDriver() const override { return false; }
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable getDisplayDriver() override { return nullptr; }
};

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    return std::make_shared<GxEPD2Display>();
}

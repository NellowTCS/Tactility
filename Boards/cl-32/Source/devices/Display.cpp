#include "Display.h"
#include "GxEPD2/GxEPD2_290_GDEY029T71H.h"

class GxEPD2Display : public tt::hal::display::DisplayDevice {
private:
    GxEPD2_290_GDEY029T71H display; // The ESP-IDF ported GxEPD2 instance

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

    // E-paper is slow and monochrome
    bool supportsLvgl() const override { return false; }
    bool startLvgl() override { return false; }
    bool stopLvgl() override { return true; }
    lv_display_t* _Nullable getLvglDisplay() const override { return nullptr; }

    // DisplayDriver not implemented (could add for custom drawing if needed)
    bool supportsDisplayDriver() const override { return false; }
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable getDisplayDriver() override { return nullptr; }
};

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay() {
    return std::make_shared<GxEPD2Display>();
}

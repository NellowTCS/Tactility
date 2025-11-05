#pragma once

#include "Tactility/hal/display/DisplayDevice.h"
#include "GxEPD2Display.h"

#include <memory>

class TdeckDisplay final : public tt::hal::display::DisplayDevice {

    std::unique_ptr<GxEPD2Display> gxDisplay;

public:

    explicit TdeckDisplay() {}

    std::string getName() const { return "Epaper"; }
    std::string getDescription() const { return "Epaper display"; }

    bool start();

    bool stop();

    std::shared_ptr<tt::hal::touch::TouchDevice> _Nullable getTouchDevice() override { return nullptr; }

    bool supportsLvgl() const override { return gxDisplay ? gxDisplay->supportsLvgl() : false; }
    bool startLvgl() override { return gxDisplay ? gxDisplay->startLvgl() : false; }
    bool stopLvgl() override { return gxDisplay ? gxDisplay->stopLvgl() : false; }

    bool supportsDisplayDriver() const override { return gxDisplay ? gxDisplay->supportsDisplayDriver() : false; }
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable getDisplayDriver() override { return gxDisplay ? gxDisplay->getDisplayDriver() : nullptr; }

    lv_display_t* _Nullable getLvglDisplay() const { return gxDisplay ? gxDisplay->getLvglDisplay() : nullptr; }
};

std::shared_ptr<tt::hal::display::DisplayDevice> createDisplay();

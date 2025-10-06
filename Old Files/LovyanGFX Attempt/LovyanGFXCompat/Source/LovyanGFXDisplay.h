#pragma once

#include <memory>
#include <Tactility/hal/display/DisplayDevice.h>
#include <Tactility/Lock.h>
#include <lgfx/v1/panel/Panel_LCD.hpp>

class LovyanGFXDisplay : public tt::hal::display::DisplayDevice {
protected:
    std::shared_ptr<tt::Lock> lock;

    std::shared_ptr<lgfx::Panel_LCD> panel;  // The LovyanGFX panel

    // LVGL objects
    lv_display_t* _Nullable lvglDisplay = nullptr;
    lv_draw_buf_t* _Nullable lv_draw_buf = nullptr;
    
    // Memory buffers
    void* _Nullable buf1 = nullptr;
    void* _Nullable buf2 = nullptr;

    // Create / initialize panel
    virtual bool createPanel(std::shared_ptr<lgfx::Panel_LCD>& outPanel) = 0;

public:
    explicit LovyanGFXDisplay(std::shared_ptr<tt::Lock> lock) : lock(lock) {}

    ~LovyanGFXDisplay() override;

    std::shared_ptr<tt::Lock> getLock() const { return lock; }

    // Start / stop
    bool start() final;
    bool stop() final;

    // LVGL integration
    bool supportsLvgl() const final { return true; }
    bool startLvgl() final;
    bool stopLvgl() final;
    lv_display_t* _Nullable getLvglDisplay() const final { return lvglDisplay; }

    // DisplayDriver integration
    bool supportsDisplayDriver() const override { return true; }
    std::shared_ptr<tt::hal::display::DisplayDriver> _Nullable getDisplayDriver() final;
};

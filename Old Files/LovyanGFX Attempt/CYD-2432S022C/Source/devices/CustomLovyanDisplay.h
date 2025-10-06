#pragma once

#include <LovyanGFXDisplay.h>
#include <memory>
#include <Tactility/Lock.h>

class CustomLovyanGFXDisplay : public LovyanGFXDisplay {
private:
    std::shared_ptr<tt::hal::touch::TouchDevice> touchDevice;

public:
    explicit CustomLovyanGFXDisplay(std::shared_ptr<tt::Lock> lock) : LovyanGFXDisplay(lock) {}

    // Device interface
    std::string getName() const override { return "CYD-2432S022C Display"; }
    std::string getDescription() const override { return "320x240 ST7789 LCD"; }
    
    // DisplayDevice interface
    std::shared_ptr<tt::hal::touch::TouchDevice> getTouchDevice() override { return touchDevice; }
    void setTouchDevice(std::shared_ptr<tt::hal::touch::TouchDevice> device) { touchDevice = device; }

protected:
    bool createPanel(std::shared_ptr<lgfx::Panel_LCD>& outPanel) override;
};
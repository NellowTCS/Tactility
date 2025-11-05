#pragma once

#include <Tactility/hal/keyboard/KeyboardDevice.h>
#include <Tactility/TactilityCore.h>
#include <esp_lcd_panel_io_interface.h>
#include <esp_lcd_touch.h>

class TdeckKeyboard : public tt::hal::keyboard::KeyboardDevice {

private:

    lv_indev_t* _Nullable deviceHandle = nullptr;

public:

    std::string getName() const final { return "T-Deck Keyboard"; }
    std::string getDescription() const final { return "I2C keyboard"; }

    bool startLvgl(lv_display_t* display) override;
    bool stopLvgl() override;
    bool isAttached() const override;
    lv_indev_t* _Nullable getLvglIndev() override { return deviceHandle; }
};

std::shared_ptr<tt::hal::keyboard::KeyboardDevice> createKeyboard();

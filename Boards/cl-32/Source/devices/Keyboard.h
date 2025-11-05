#pragma once

#include <Tactility/hal/keyboard/KeyboardDevice.h>
#include <Tactility/Timer.h>
#include <Tca8418.h>
#include <driver/gpio.h>
#include <freertos/queue.h>
#include <memory>
#include <lvgl.h>

class CL32Keyboard final : public tt::hal::keyboard::KeyboardDevice {
public:
    explicit CL32Keyboard(const std::shared_ptr<Tca8418>& tca);
    ~CL32Keyboard() override;

    std::string getName() const override { return "CL-32 Keyboard (TCA8418)"; }
    std::string getDescription() const override { return "CL-32 keyboard controller via TCA8418"; }

    bool startLvgl(lv_display_t* display) override;
    bool stopLvgl() override;
    bool isAttached() const override;
    lv_indev_t* _Nullable getLvglIndev() override { return kbHandle; }

private:
    // Event queue consumed by LVGL read callback (char or LV_KEY_* values)
    QueueHandle_t queue = nullptr;

    // TCA8418 wrapper instance owned by the board configuration
    std::shared_ptr<Tca8418> keypad;

    // LVGL input device handle
    lv_indev_t* _Nullable kbHandle = nullptr;

    // Periodic timer used to poll/process keypad events
    std::unique_ptr<tt::Timer> inputTimer;

    // Process events coming from the Tca wrapper, enqueue chars/LV keys
    void processKeyboard();

    // LVGL read callback (static trampoline)
    static void readCallback(lv_indev_t* indev, lv_indev_data_t* data);
};

#pragma once

#include <Tactility/hal/keyboard/KeyboardDevice.h>
#include <Tactility/Timer.h>
#include <Tca8418.h>
#include <driver/gpio.h>
#include <freertos/queue.h>
#include <memory>

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
    struct RawEvent {
        uint8_t code;
        uint8_t pressed; // 1 pressed, 0 released
    };

    // LVGL input device handle
    lv_indev_t* _Nullable kbHandle = nullptr;

    // queue for events
    QueueHandle_t queue = nullptr;

    // TCA wrapper (owned by board)
    std::shared_ptr<Tca8418> keypad;

    // polling timer (mirrors other drivers)
    std::unique_ptr<tt::Timer> pollTimer;

    // helper: poll chip and push RawEvent into queue
    void pollChip();

    // map TCA event code to LVGL key (ASCII or LV_KEY_*)
    uint32_t mapEventToLvKey(uint8_t event_code);

    // LVGL read callback
    static void readCallback(lv_indev_t* indev, lv_indev_data_t* data);
};

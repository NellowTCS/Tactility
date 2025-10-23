#pragma once

#include <Tactility/hal/encoder/EncoderDevice.h>
#include <Tactility/hal/gpio/Gpio.h>
#include <Tactility/TactilityCore.h>
#include <queue>
#include <memory>

class ButtonControl final : public tt::hal::encoder::EncoderDevice {

public:

    enum class Event {
        ShortPress,
        LongPress
    };

    enum class Action {
        UiSelectNext,
        UiSelectPrevious,
        UiPressSelected,
        AppClose,
    };

    struct PinConfiguration {
        tt::hal::gpio::Pin pin;
        Event event;
        Action action;
    };

private:

    // Internal event structure for queueing GPIO events
    struct GpioEvent {
        size_t pinIndex;
        bool pressed;
        uint32_t timestamp;
    };

    // Track state of each physical pin
    struct PinState {
        uint32_t pressStartTime = 0;
        bool isPressed = false;
        bool debouncing = false;
        uint32_t lastTransitionTime = 0;
    };

    // Track pending LVGL actions
    struct PendingAction {
        Action action;
        bool needsRelease;  // For UiPressSelected which needs press+release cycle
    };

    static constexpr uint32_t DEBOUNCE_MS = 50;
    static constexpr uint32_t LONG_PRESS_MS = 500;

    lv_indev_t* _Nullable deviceHandle = nullptr;
    std::vector<PinConfiguration> pinConfigurations;
    std::vector<PinState> pinStates;
    
    // ISR-safe event queue for ISR -> main thread communication
    // Using a simple ring buffer instead of std::queue + mutex
    static constexpr size_t EVENT_QUEUE_SIZE = 16;
    GpioEvent eventQueue[EVENT_QUEUE_SIZE];
    volatile size_t queueHead = 0;
    volatile size_t queueTail = 0;
    
    // Pending actions to deliver to LVGL
    std::queue<PendingAction> actionQueue;
    bool deliveredPress = false;  // Track if we need to send release next

    void handleGpioEvent(const GpioEvent& event);
    void processEventQueue();
    void deliverActionsToLvgl(lv_indev_data_t* data);
    
    // ISR-safe queue operations
    bool queuePush(const GpioEvent& event);
    bool queuePop(GpioEvent& event);
    
    static void readCallback(lv_indev_t* indev, lv_indev_data_t* data);

public:

    explicit ButtonControl(const std::vector<PinConfiguration>& pinConfigurations);
    ~ButtonControl() override;

    std::string getName() const override { return "ButtonControl"; }
    std::string getDescription() const override { return "Interrupt-driven button input"; }

    bool startLvgl(lv_display_t* display) override;
    bool stopLvgl() override;

    lv_indev_t* _Nullable getLvglIndev() override { return deviceHandle; }

    // Factory methods
    static std::shared_ptr<ButtonControl> createOneButtonControl(tt::hal::gpio::Pin pin) {
        return std::make_shared<ButtonControl>(std::vector {
            PinConfiguration {
                .pin = pin,
                .event = Event::ShortPress,
                .action = Action::UiSelectNext
            },
            PinConfiguration {
                .pin = pin,
                .event = Event::LongPress,
                .action = Action::UiPressSelected
            }
        });
    }

    static std::shared_ptr<ButtonControl> createTwoButtonControl(
        tt::hal::gpio::Pin primaryPin, 
        tt::hal::gpio::Pin secondaryPin
    ) {
        return std::make_shared<ButtonControl>(std::vector {
            PinConfiguration {
                .pin = primaryPin,
                .event = Event::ShortPress,
                .action = Action::UiPressSelected
            },
            PinConfiguration {
                .pin = primaryPin,
                .event = Event::LongPress,
                .action = Action::AppClose
            },
            PinConfiguration {
                .pin = secondaryPin,
                .event = Event::ShortPress,
                .action = Action::UiSelectNext
            },
            PinConfiguration {
                .pin = secondaryPin,
                .event = Event::LongPress,
                .action = Action::UiSelectPrevious
            }
        });
    }
};

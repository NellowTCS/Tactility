#include "ButtonControl.h"
#include <Tactility/Log.h>
#include <esp_lvgl_port.h>

constexpr auto* TAG = "ButtonControl";

ButtonControl::ButtonControl(const std::vector<PinConfiguration>& pinConfigurations) 
    : pinConfigurations(pinConfigurations) {
    
    pinStates.resize(pinConfigurations.size());
    isrCallbacks.resize(pinConfigurations.size());
    
    // Configure each pin and set up interrupt
    for (size_t i = 0; i < pinConfigurations.size(); i++) {
        const auto& config = pinConfigurations[i];
        
        // Configure as input with pull-down (assuming active-high buttons)
        tt::hal::gpio::configure(config.pin, tt::hal::gpio::Mode::Input, false, true);
        
        // Create ISR callback that captures the pin index
        isrCallbacks[i] = [this, i]() {
            GpioEvent event;
            event.pinIndex = i;
            event.pressed = tt::hal::gpio::getLevel(pinConfigurations[i].pin);
            event.timestamp = tt::kernel::getMillis();
            
            // Queue event from ISR context (keep ISR fast!)
            if (queueMutex.lock(0)) {  // Non-blocking in ISR
                eventQueue.push(event);
                queueMutex.unlock();
            }
        };
        
        // Attach interrupt for both edges (press and release)
        tt::hal::gpio::attachInterrupt(
            config.pin,
            tt::hal::gpio::InterruptMode::AnyEdge,
            isrCallbacks[i]
        );
    }
    
    TT_LOG_I(TAG, "Initialized with %d button(s)", pinConfigurations.size());
}

ButtonControl::~ButtonControl() {
    // Detach all interrupts
    for (size_t i = 0; i < pinConfigurations.size(); i++) {
        tt::hal::gpio::detachInterrupt(pinConfigurations[i].pin);
    }
}

void ButtonControl::handleGpioEvent(const GpioEvent& event) {
    PinState& state = pinStates[event.pinIndex];
    const PinConfiguration& config = pinConfigurations[event.pinIndex];
    
    // Debounce: ignore transitions too close together
    if (event.timestamp - state.lastTransitionTime < DEBOUNCE_MS) {
        return;
    }
    state.lastTransitionTime = event.timestamp;
    
    if (event.pressed) {
        // Button pressed
        if (!state.isPressed) {
            state.isPressed = true;
            state.pressStartTime = event.timestamp;
            TT_LOG_D(TAG, "Pin %d pressed", event.pinIndex);
        }
    } else {
        // Button released
        if (state.isPressed) {
            state.isPressed = false;
            uint32_t pressDuration = event.timestamp - state.pressStartTime;
            
            TT_LOG_D(TAG, "Pin %d released after %dms", event.pinIndex, pressDuration);
            
            // Determine if short or long press
            bool isLongPress = pressDuration >= LONG_PRESS_MS;
            Event triggeredEvent = isLongPress ? Event::LongPress : Event::ShortPress;
            
            // Check if this pin configuration matches the triggered event
            if (config.event == triggeredEvent) {
                TT_LOG_D(TAG, "Queueing action: %d", static_cast<int>(config.action));
                
                PendingAction action;
                action.action = config.action;
                action.needsRelease = (config.action == Action::UiPressSelected);
                actionQueue.push(action);
            }
        }
    }
}

void ButtonControl::processEventQueue() {
    // Process all queued GPIO events
    queueMutex.lock();
    while (!eventQueue.empty()) {
        GpioEvent event = eventQueue.front();
        eventQueue.pop();
        queueMutex.unlock();
        
        handleGpioEvent(event);
        
        queueMutex.lock();
    }
    queueMutex.unlock();
}

void ButtonControl::deliverActionsToLvgl(lv_indev_data_t* data) {
    // Default state
    data->enc_diff = 0;
    data->state = LV_INDEV_STATE_RELEASED;
    
    // If we previously delivered a press, deliver the release now
    if (deliveredPress) {
        data->state = LV_INDEV_STATE_RELEASED;
        deliveredPress = false;
        return;  // Only deliver release this cycle
    }
    
    // Process pending actions
    if (actionQueue.empty()) {
        return;
    }
    
    // Take one action per read cycle for predictable behavior
    PendingAction action = actionQueue.front();
    actionQueue.pop();
    
    switch (action.action) {
        case Action::UiSelectNext:
            data->enc_diff = 1;
            TT_LOG_D(TAG, "LVGL: Select next");
            break;
            
        case Action::UiSelectPrevious:
            data->enc_diff = -1;
            TT_LOG_D(TAG, "LVGL: Select previous");
            break;
            
        case Action::UiPressSelected:
            // Deliver press now, release on next read
            data->state = LV_INDEV_STATE_PRESSED;
            deliveredPress = true;
            TT_LOG_D(TAG, "LVGL: Press selected");
            break;
            
        case Action::AppClose: {
            lv_group_t* group = lv_indev_get_group(deviceHandle);
            if (group != nullptr && lv_group_get_editing(group)) {
                // Exit editing mode (e.g., keyboard)
                lv_group_set_editing(group, false);
                TT_LOG_D(TAG, "LVGL: Exit editing mode");
            } else {
                // TODO: Implement app close logic
                TT_LOG_D(TAG, "LVGL: App close requested");
            }
            break;
        }
    }
}

void ButtonControl::readCallback(lv_indev_t* indev, lv_indev_data_t* data) {
    auto* self = static_cast<ButtonControl*>(lv_indev_get_driver_data(indev));
    
    // Process any queued GPIO events first
    self->processEventQueue();
    
    // Then deliver actions to LVGL
    self->deliverActionsToLvgl(data);
}

bool ButtonControl::startLvgl(lv_display_t* display) {
    if (deviceHandle != nullptr) {
        TT_LOG_W(TAG, "Already started");
        return false;
    }
    
    TT_LOG_I(TAG, "Starting LVGL input device");
    
    deviceHandle = lv_indev_create();
    lv_indev_set_type(deviceHandle, LV_INDEV_TYPE_ENCODER);
    lv_indev_set_driver_data(deviceHandle, this);
    lv_indev_set_read_cb(deviceHandle, readCallback);
    
    // Clear any startup glitches
    queueMutex.lock();
    while (!eventQueue.empty()) {
        eventQueue.pop();
    }
    queueMutex.unlock();
    
    return true;
}

bool ButtonControl::stopLvgl() {
    if (deviceHandle == nullptr) {
        TT_LOG_W(TAG, "Not started");
        return false;
    }
    
    TT_LOG_I(TAG, "Stopping LVGL input device");
    
    lv_indev_delete(deviceHandle);
    deviceHandle = nullptr;
    
    // Clear queues
    queueMutex.lock();
    while (!eventQueue.empty()) {
        eventQueue.pop();
    }
    while (!actionQueue.empty()) {
        actionQueue.pop();
    }
    queueMutex.unlock();
    
    deliveredPress = false;
    
    return true;
}

#include "Keyboard.h"
#include <Tactility/hal/i2c/I2c.h>
#include <lvgl.h>
#include <Tactility/Log.h>

constexpr auto* TAG = "CL32Keyboard";

// Matrix size used by CL-32 (TCA8418 supports up to 8x10 -> 80 keys)
constexpr int KB_ROWS = 8;
constexpr int KB_COLS = 10;

// Flattened maps from the original CL-32 firmware have been reshaped into row-major [KB_ROWS][KB_COLS].
// These come from the stock CL-32 lower/upper arrays (split into 8 rows of 10).
static constexpr char keymap_lc[KB_ROWS][KB_COLS] = {
    {'%','1','2','3','4','5','6','7','8', 0},
    {'9','0', 0,'[',']','+','"','\'', 0, 0},
    { 0,'q','w','e','r','t','y','u','i', 0},
    {'o','p', 0,'(',')','-','; ',':', 0, 0},
    { 0,'a','s','d','f','g','h','j','k', 0},
    {'l', 0,'#','{','}','*',',','.',' ', 0},
    {'z','x','c','v','b',' ',' ','n','m', 0},
    { 0, 0, 0, 0,'<','>','/','\\','=',' ', 0}
};

static constexpr char keymap_uc[KB_ROWS][KB_COLS] = {
    {'%','1','2','3','4','5','6','7','8', 0},
    {'9','0', 0,'[',']','+','"','\'', 0, 0},
    { 0,'Q','W','E','R','T','Y','U','I', 0},
    {'O','P', 0,'(',')','-','; ',':', 0, 0},
    { 0,'A','S','D','F','G','H','J','K', 0},
    {'L', 0,'#','{','}','*',',','.',' ', 0},
    {'Z','X','C','V','B',' ',' ','N','M', 0},
    { 0, 0, 0, 0,'<','>','/','\\','=',' ', 0}
};

// Special codes array from original firmware laid out in the same order (1..80 -> index 0..79).
static const uint8_t key_map_codes_flat[80] = {
  0,0,0,0,0,0,0,0,0,0, 0,0,42,0,0,0,0,0,58,0,
  43,0,0,0,0,0,0,0,0,0,0,0,40,0,0,0,0,0,59,0,
  225,0,0,0,0,0,0,0,0,0,0,82,0,0,0,0,0,0,60,0,
  0,0,0,0,0,0,0,0,0,0,80,81,79,0,0,0,0,0,61,0
};

// helper to get flat index from row/col (row-major)
static inline int rc_to_index(int row, int col) {
    if (row < 0 || row >= KB_ROWS || col < 0 || col >= KB_COLS) return -1;
    return row * KB_COLS + col;
}

void Keyboard::readCallback(lv_indev_t* indev, lv_indev_data_t* data) {
    auto keyboard = static_cast<Keyboard*>(lv_indev_get_user_data(indev));
    char keypress = 0;

    // non-blocking receive - if we have a queued key we present it as pressed
    if (xQueueReceive(keyboard->queue, &keypress, 0) == pdPASS) {
        data->key = keypress;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->key = 0;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void Keyboard::processKeyboard() {
    static bool shift_pressed = false;
    static bool sym_pressed = false;
    static bool cap_toggle = false;
    static bool cap_toggle_armed = true;
    bool anykey_pressed = false;

    // Use the Tca8418 wrapper's update() like TpagerKeyboard does.
    if (keypad->update()) {
        anykey_pressed = (keypad->pressed_key_count > 0);

        // First pass: detect modifiers held (shift/sym) to handle cap toggle semantics.
        for (int i = 0; i < keypad->pressed_key_count; ++i) {
            auto row = keypad->pressed_list[i].row;
            auto col = keypad->pressed_list[i].col;
            // The original firmware treats one button as sym and one as shift (specific row/col).
            // CL-32 stock used specific key indices - we don't assume magic here; match the original codes:
            int idx = rc_to_index(row, col);
            uint8_t special = (idx >= 0 && idx < 80) ? key_map_codes_flat[idx] : 0;
            if (special == 225) shift_pressed = true;     // shift-like
            if (special == 60) sym_pressed = true;        // menu/symbol key in original mapping
        }

        // Toggle caps mode when both sym+shift pressed (mirrors CL-32 behaviour)
        if ((sym_pressed && shift_pressed) && cap_toggle_armed) {
            cap_toggle = !cap_toggle;
            cap_toggle_armed = false;
        }

        // Second pass: enqueue characters / special keys
        for (int i = 0; i < keypad->pressed_key_count; ++i) {
            auto row = keypad->pressed_list[i].row;
            auto col = keypad->pressed_list[i].col;
            int idx = rc_to_index(row, col);
            if (idx < 0 || idx >= 80) continue;

            // Special map code (like enter/backspace/arrows) from original firmware
            uint8_t special = key_map_codes_flat[idx];

            // If special contains a printable ASCII? handle below; else check for special codes
            if (special == 42) { // Backspace
                char ch = (char)LV_KEY_BACKSPACE;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            } else if (special == 40) { // Enter
                char ch = (char)LV_KEY_ENTER;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            } else if (special == 79) { // Right
                char ch = (char)LV_KEY_RIGHT;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            } else if (special == 80) { // Left
                char ch = (char)LV_KEY_LEFT;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            } else if (special == 81) { // Down
                char ch = (char)LV_KEY_DOWN;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            } else if (special == 82) { // Up
                char ch = (char)LV_KEY_UP;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            } else if (special == 225) {
                // Shift key — handled as modifier state, do not queue a key event
                continue;
            } else if (special != 0) {
                // other special codes: map to ESC for menu or ignore
                if (special == 60) {
                    char ch = (char)LV_KEY_ESC;
                    xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                    continue;
                }
            }

            // Printable char: choose upper/lower based on shift/caps/sym
            char chr = 0;
            if (sym_pressed) {
                // No explicit symbol map available in CL-32 stock arrays;
                // fall back to upper for symbols if shift pressed, else lower.
                chr = keymap_lc[row][col];
            } else if (shift_pressed || cap_toggle) {
                chr = keymap_uc[row][col];
            } else {
                chr = keymap_lc[row][col];
            }

            if (chr != 0) {
                xQueueSend(queue, &chr, 50 / portTICK_PERIOD_MS);
            }
        }

        // After processing pressed keys, handle released keys to update modifier states
        for (int i = 0; i < keypad->released_key_count; ++i) {
            auto row = keypad->released_list[i].row;
            auto col = keypad->released_list[i].col;
            int idx = rc_to_index(row, col);
            uint8_t special = (idx >= 0 && idx < 80) ? key_map_codes_flat[idx] : 0;
            if (special == 225) shift_pressed = false;
            if (special == 60) sym_pressed = false;
        }

        if ((!sym_pressed && !shift_pressed) && !cap_toggle_armed) {
            cap_toggle_armed = true;
        }

        if (anykey_pressed) {
            makeBacklightImpulse();
        }
    }
}

bool Keyboard::startLvgl(lv_display_t* display) {
    backlightOkay = initBacklight(BACKLIGHT, 30000, LEDC_TIMER_0, LEDC_CHANNEL_1);
    // keypad wrapper initialization mirrors other drivers
    keypad->init(KB_ROWS, KB_COLS);

    assert(inputTimer == nullptr);
    inputTimer = std::make_unique<tt::Timer>(tt::Timer::Type::Periodic, [this] {
        processKeyboard();
    });

    assert(backlightImpulseTimer == nullptr);
    backlightImpulseTimer = std::make_unique<tt::Timer>(tt::Timer::Type::Periodic, [this] {
        processBacklightImpulse();
    });

    kbHandle = lv_indev_create();
    lv_indev_set_type(kbHandle, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(kbHandle, &readCallback);
    lv_indev_set_display(kbHandle, display);
    lv_indev_set_user_data(kbHandle, this);

    inputTimer->start(20 / portTICK_PERIOD_MS);
    backlightImpulseTimer->start(50 / portTICK_PERIOD_MS);

    return true;
}

bool Keyboard::stopLvgl() {
    assert(inputTimer);
    inputTimer->stop();
    inputTimer = nullptr;

    assert(backlightImpulseTimer);
    backlightImpulseTimer->stop();
    backlightImpulseTimer = nullptr;

    lv_indev_delete(kbHandle);
    kbHandle = nullptr;
    return true;
}

bool Keyboard::isAttached() const {
    return tt::hal::i2c::masterHasDeviceAtAddress(keypad->getPort(), keypad->getAddress(), 100);
}

bool Keyboard::initBacklight(gpio_num_t pin, uint32_t frequencyHz, ledc_timer_t timer, ledc_channel_t channel) {
    backlightPin = pin;
    backlightTimer = timer;
    backlightChannel = channel;

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = backlightTimer,
        .freq_hz = frequencyHz,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false
    };

    if (ledc_timer_config(&ledc_timer) != ESP_OK) {
        TT_LOG_E(TAG, "Backlight timer config failed");
        return false;
    }

    ledc_channel_config_t ledc_channel = {
        .gpio_num = backlightPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = backlightChannel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = backlightTimer,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        .flags = {
            .output_invert = 0
        }
    };

    if (ledc_channel_config(&ledc_channel) != ESP_OK) {
        TT_LOG_E(TAG, "Backlight channel config failed");
    }

    return true;
}

bool Keyboard::setBacklightDuty(uint8_t duty) {
    if (!backlightOkay) {
        TT_LOG_E(TAG, "Backlight not ready");
        return false;
    }
    return (ledc_set_duty(LEDC_LOW_SPEED_MODE, backlightChannel, duty) == ESP_OK) &&
        (ledc_update_duty(LEDC_LOW_SPEED_MODE, backlightChannel) == ESP_OK);
}

void Keyboard::makeBacklightImpulse() {
    backlightImpulseDuty = 255;
    setBacklightDuty(backlightImpulseDuty);
}

void Keyboard::processBacklightImpulse() {
    if (backlightImpulseDuty > 64) {
        backlightImpulseDuty--;
        setBacklightDuty(backlightImpulseDuty);
    }
}

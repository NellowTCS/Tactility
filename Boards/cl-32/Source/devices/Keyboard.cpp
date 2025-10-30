#include "CL32Keyboard.h"
#include <Tactility/Log.h>
#include <Tactility/hal/i2c/I2c.h>
#include <lvgl.h>

constexpr auto* TAG = "CL32Keyboard";

// Keymaps copied/adapted from CL-32 stock firmware
static constexpr char lower_map[80] = {
  '%' ,'1' ,'2' ,'3' ,'4' ,'5' ,'6' ,'7' ,'8' ,0   ,'9' ,'0' ,0   ,'[' ,']' ,'+' ,'"' ,'\'',0   ,0   ,
  0   ,'q' ,'w' ,'e' ,'r' ,'t' ,'y' ,'u' ,'i' ,0   ,'o' ,'p' ,0   ,'(' ,')' ,'-' ,';' ,':' ,0   ,0   ,
  0   ,'a' ,'s' ,'d' ,'f' ,'g' ,'h' ,'j' ,'k' ,0   ,'l' ,0   ,'#' ,'{' ,'}' ,'*' ,',' ,'.' ,0   ,0   ,
  'z' ,'x' ,'c' ,'v' ,'b' ,' ' ,' ' ,'n' ,'m' ,0   ,0   ,0   ,0   ,'<' ,'>' ,'/' ,'\\','=' ,0   ,0
};

static constexpr char upper_map[80] = {
  '%' ,'1' ,'2' ,'3' ,'4' ,'5' ,'6' ,'7' ,'8' ,0   ,'9' ,'0' ,0   ,'[' ,']' ,'+' ,'"' ,'\'',0   ,0   ,
  0   ,'Q' ,'W' ,'E' ,'R' ,'T' ,'Y' ,'U' ,'I' ,0   ,'O' ,'P' ,0   ,'(' ,')' ,'-' ,';' ,':' ,0   ,0   ,
  0   ,'A' ,'S' ,'D' ,'F' ,'G' ,'H' ,'J' ,'K' ,0   ,'L' ,0   ,'#' ,'{' ,'}' ,'*' ,',' ,'.' ,0   ,0   ,
  'Z' ,'X' ,'C' ,'V' ,'B' ,' ' ,' ' ,'N' ,'M' ,0   ,0   ,0   ,0   ,'<' ,'>' ,'/' ,'\\','=' ,0   ,0
};

// keyMap from CL-32 original (special codes)
static const uint8_t key_map_codes[80] = {
  0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,42  ,0   ,0   ,0   ,0   ,0   ,58  ,0   ,
  43  ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,40  ,0   ,0   ,0   ,0   ,0   ,59  ,0   ,
  225 ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,82  ,0   ,0   ,0   ,0   ,0   ,0   ,60  ,0   ,
  0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,80  ,81  ,79  ,0   ,0   ,0   ,0   ,0   ,61  ,0
};

CL32Keyboard::CL32Keyboard(const std::shared_ptr<Tca8418>& tca)
    : keypad(tca)
{
    queue = xQueueCreate(32, sizeof(RawEvent));
}

CL32Keyboard::~CL32Keyboard()
{
    if (pollTimer) {
        pollTimer->stop();
        pollTimer = nullptr;
    }
    if (kbHandle) {
        lv_indev_delete(kbHandle);
        kbHandle = nullptr;
    }
    if (queue) {
        vQueueDelete(queue);
        queue = nullptr;
    }
}

bool CL32Keyboard::isAttached() const {
    if (!keypad) return false;
    return tt::hal::i2c::masterHasDeviceAtAddress(keypad->getPort(), keypad->getAddress(), 100);
}

uint32_t CL32Keyboard::mapEventToLvKey(uint8_t event_code) {
    // event_code is 1..80 (per Arduino code), map to arrays 0-based
    if (event_code == 0 || event_code > 80) return 0;
    const int idx = static_cast<int>(event_code - 1);

    uint8_t special = key_map_codes[idx];
    // Map special numeric codes to LVGL keys
    switch (special) {
        case 42: return LV_KEY_BACKSPACE;
        case 40: return LV_KEY_ENTER;
        case 79: return LV_KEY_RIGHT;
        case 80: return LV_KEY_LEFT;
        case 81: return LV_KEY_DOWN;
        case 82: return LV_KEY_UP;
        case 60: return LV_KEY_ESC;    // mapped menu -> ESC (adjust if you want different)
        case 225: return LV_KEY_SHIFT; // shift-like; you may want to track modifier state instead
        default: break;
    }

    // Printable char fallback - prefer lower_map (no modifier state tracked here)
    char c = lower_map[idx];
    if (c != 0) return static_cast<uint32_t>(c);

    return 0;
}

void CL32Keyboard::pollChip() {
    // Prefer Tca8418 wrapper if provided
    if (keypad) {
        // Tca8418::update() and pressed/released lists are used by TpagerKeyboard,
        // so reuse that pattern if the wrapper provides the same interface.
        if (!keypad->update()) return;

        for (int i = 0; i < keypad->pressed_key_count; ++i) {
            auto r = keypad->pressed_list[i].row;
            auto c = keypad->pressed_list[i].col;
            uint8_t code = keypad->pressed_list[i].code; // if wrapper provides code; else compute
            // If wrapper does not expose code, compute as index: code = row*??? + col...
            // Here we prefer the wrapper's code field; fallback not implemented.
            RawEvent ev{ code, 1 };
            xQueueSend(queue, &ev, 0);
        }
        for (int i = 0; i < keypad->released_key_count; ++i) {
            auto r = keypad->released_list[i].row;
            auto c = keypad->released_list[i].col;
            uint8_t code = keypad->released_list[i].code;
            RawEvent ev{ code, 0 };
            xQueueSend(queue, &ev, 0);
        }
        return;
    }

    // If no wrapper, fall back to register pop (TCA behavior) using i2c helper:
    // Read event count (reg 0x03) then read event(s) from 0x04
    uint8_t count = 0;
    if (!tt::hal::i2c::masterRead(/*bus*/I2C_NUM_0, /*addr*/0x34, &count, 1, 50 / portTICK_PERIOD_MS)) {
        TT_LOG_D(TAG, "i2c read count failed");
        return;
    }
    if (count == 0) return;

    for (uint8_t i = 0; i < count; ++i) {
        uint8_t ev_byte = 0;
        if (!tt::hal::i2c::masterReadRegister(I2C_NUM_0, 0x34, 0x04, &ev_byte, 1, 100 / portTICK_PERIOD_MS)) {
            // fallback to plain read if needed
            if (!tt::hal::i2c::masterRead(I2C_NUM_0, 0x34, &ev_byte, 1, 100 / portTICK_PERIOD_MS)) {
                continue;
            }
        }
        uint8_t ev_code = ev_byte & 0x7F;
        uint8_t ev_pressed = (ev_byte & 0x80) ? 1 : 0;
        RawEvent ev{ ev_code, ev_pressed };
        xQueueSend(queue, &ev, 0);
    }
}

void CL32Keyboard::readCallback(lv_indev_t* indev, lv_indev_data_t* data) {
    auto kb = static_cast<CL32Keyboard*>(lv_indev_get_user_data(indev));
    RawEvent ev;
    data->key = 0;
    data->state = LV_INDEV_STATE_RELEASED;

    if (kb && kb->queue) {
        if (xQueueReceive(kb->queue, &ev, 0) == pdPASS) {
            uint32_t mapped = kb->mapEventToLvKey(ev.code);
            if (mapped == 0) {
                data->key = 0;
                data->state = LV_INDEV_STATE_RELEASED;
            } else {
                data->key = static_cast<int>(mapped);
                data->state = ev.pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
            }
            return;
        }
    }
    // nothing -> released/no key
    data->key = 0;
    data->state = LV_INDEV_STATE_RELEASED;
}

bool CL32Keyboard::startLvgl(lv_display_t* display) {
    // Create LVGL input device
    kbHandle = lv_indev_create();
    lv_indev_set_type(kbHandle, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(kbHandle, &CL32Keyboard::readCallback);
    lv_indev_set_display(kbHandle, display);
    lv_indev_set_user_data(kbHandle, this);

    // Start polling timer (20 ms)
    pollTimer = std::make_unique<tt::Timer>(tt::Timer::Type::Periodic, [this] {
        this->pollChip();
    });
    pollTimer->start(20 / portTICK_PERIOD_MS);

    return true;
}

bool CL32Keyboard::stopLvgl() {
    if (pollTimer) {
        pollTimer->stop();
        pollTimer = nullptr;
    }
    if (kbHandle) {
        lv_indev_delete(kbHandle);
        kbHandle = nullptr;
    }
    return true;
}

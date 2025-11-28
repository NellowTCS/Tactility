#include "Keyboard.h"
#include <Tactility/hal/i2c/I2c.h>
#include <lvgl.h>
#include <Tactility/Log.h>

constexpr auto* TAG = "CL32Keyboard";

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

static const uint8_t key_map_codes[80] = {
  0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,42  ,0   ,0   ,0   ,0   ,0   ,58  ,0   ,
  43  ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,40  ,0   ,0   ,0   ,0   ,0   ,59  ,0   ,
  225 ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,82  ,0   ,0   ,0   ,0   ,0   ,0   ,60  ,0   ,
  0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,0   ,80  ,81  ,79  ,0   ,0   ,0   ,0   ,0   ,61  ,0
};

static inline int rc_to_index(int row, int col) {
    if (row < 0 || col < 0) return -1;
    if (row >= 8 || col >= 10) return -1;
    int idx = (row * 10) + col;
    if (idx < 0 || idx >= 80) return -1;
    return idx;
}

CL32Keyboard::CL32Keyboard(const std::shared_ptr<Tca8418>& tca)
    : queue(nullptr)
    , keypad(tca)
    , kbHandle(nullptr)
    , inputTimer(nullptr)
{
    TT_LOG_I(TAG, "CL32Keyboard constructor called");
    queue = xQueueCreate(20, sizeof(char));
    if (!queue) {
        TT_LOG_E(TAG, "Failed to create keyboard queue!");
    } else {
        TT_LOG_I(TAG, "Keyboard queue created successfully");
    }
}

CL32Keyboard::~CL32Keyboard() {
    TT_LOG_I(TAG, "CL32Keyboard destructor called");
    stopLvgl();
    if (queue) {
        vQueueDelete(queue);
        queue = nullptr;
        TT_LOG_I(TAG, "Keyboard queue deleted");
    }
}

void CL32Keyboard::readCallback(lv_indev_t* indev, lv_indev_data_t* data) {
    auto keyboard = static_cast<CL32Keyboard*>(lv_indev_get_user_data(indev));
    char keypress = 0;

    if (keyboard == nullptr || keyboard->queue == nullptr) {
        data->key = 0;
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    if (xQueueReceive(keyboard->queue, &keypress, 0) == pdPASS) {
        data->key = keypress;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->key = 0;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void CL32Keyboard::processKeyboard() {
    static bool shift_pressed = false;
    static bool sym_pressed = false;
    static bool cap_toggle = false;
    static bool cap_toggle_armed = true;

    if (!keypad) {
        TT_LOG_E(TAG, "processKeyboard: keypad is null!");
        return;
    }

    if (!keypad->update()) {
        return;
    }

    TT_LOG_I(TAG, "Keypad updated: %d pressed, %d released", 
             keypad->pressed_key_count, keypad->released_key_count);

    shift_pressed = false;
    sym_pressed = false;

    for (int i = 0; i < keypad->pressed_key_count; ++i) {
        int row = keypad->pressed_list[i].row;
        int col = keypad->pressed_list[i].col;
        int idx = rc_to_index(row, col);
        if (idx < 0) continue;
        
        uint8_t special = key_map_codes[idx];
        if (special == 225) shift_pressed = true;
        if (special == 60) sym_pressed = true;
    }

    if ((sym_pressed && shift_pressed) && cap_toggle_armed) {
        cap_toggle = !cap_toggle;
        cap_toggle_armed = false;
    }

    for (int i = 0; i < keypad->pressed_key_count; ++i) {
        int row = keypad->pressed_list[i].row;
        int col = keypad->pressed_list[i].col;
        int idx = rc_to_index(row, col);
        if (idx < 0) continue;

        uint8_t special = key_map_codes[idx];

        if (special == 42) {
            char ch = (char)LV_KEY_BACKSPACE;
            xQueueSend(queue, &ch, 0);
            continue;
        }
        if (special == 40) {
            char ch = (char)LV_KEY_ENTER;
            xQueueSend(queue, &ch, 0);
            continue;
        }
        if (special == 79) {
            char ch = (char)LV_KEY_RIGHT;
            xQueueSend(queue, &ch, 0);
            continue;
        }
        if (special == 80) {
            char ch = (char)LV_KEY_LEFT;
            xQueueSend(queue, &ch, 0);
            continue;
        }
        if (special == 81) {
            char ch = (char)LV_KEY_DOWN;
            xQueueSend(queue, &ch, 0);
            continue;
        }
        if (special == 82) {
            char ch = (char)LV_KEY_UP;
            xQueueSend(queue, &ch, 0);
            continue;
        }
        if (special == 225) {
            continue;
        }
        if (special == 60) {
            char ch = (char)LV_KEY_ESC;
            xQueueSend(queue, &ch, 0);
            continue;
        }

        char chr = 0;
        if (sym_pressed) {
            chr = lower_map[idx];
        } else if (shift_pressed || cap_toggle) {
            chr = upper_map[idx];
        } else {
            chr = lower_map[idx];
        }

        if (chr != 0) {
            TT_LOG_I(TAG, "Key: row=%d col=%d idx=%d char='%c' (0x%02X)", 
                     row, col, idx, chr, (uint8_t)chr);
            xQueueSend(queue, &chr, 0);
        }
    }

    for (int i = 0; i < keypad->released_key_count; ++i) {
        int row = keypad->released_list[i].row;
        int col = keypad->released_list[i].col;
        int idx = rc_to_index(row, col);
        if (idx < 0) continue;
        
        uint8_t special = key_map_codes[idx];
        if (special == 225) shift_pressed = false;
        if (special == 60) sym_pressed = false;
    }

    if ((!sym_pressed && !shift_pressed) && !cap_toggle_armed) {
        cap_toggle_armed = true;
    }
}

bool CL32Keyboard::startLvgl(lv_display_t* display) {
    TT_LOG_I(TAG, "startLvgl called");
    
    if (!keypad) {
        TT_LOG_E(TAG, "startLvgl: keypad is null!");
        return false;
    }

    if (!queue) {
        TT_LOG_E(TAG, "startLvgl: queue is null!");
        return false;
    }

    TT_LOG_I(TAG, "Initializing keypad with 8 rows x 10 cols");
    keypad->init(8, 10);

    TT_LOG_I(TAG, "Creating input timer");
    inputTimer = std::make_unique<tt::Timer>(tt::Timer::Type::Periodic, [this] {
        this->processKeyboard();
    });

    TT_LOG_I(TAG, "Creating LVGL input device");
    kbHandle = lv_indev_create();
    lv_indev_set_type(kbHandle, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(kbHandle, &CL32Keyboard::readCallback);
    lv_indev_set_display(kbHandle, display);
    lv_indev_set_user_data(kbHandle, this);

    TT_LOG_I(TAG, "Starting input timer (20ms interval)");
    inputTimer->start(20 / portTICK_PERIOD_MS);

    TT_LOG_I(TAG, "CL32Keyboard startLvgl completed successfully");
    return true;
}

bool CL32Keyboard::stopLvgl() {
    TT_LOG_I(TAG, "stopLvgl called");
    
    if (inputTimer) {
        TT_LOG_I(TAG, "Stopping input timer");
        inputTimer->stop();
        inputTimer = nullptr;
    }
    if (kbHandle) {
        TT_LOG_I(TAG, "Deleting LVGL input device");
        lv_indev_delete(kbHandle);
        kbHandle = nullptr;
    }
    
    TT_LOG_I(TAG, "CL32Keyboard stopLvgl completed");
    return true;
}

bool CL32Keyboard::isAttached() const {
    if (!keypad) {
        TT_LOG_W(TAG, "isAttached: keypad is null");
        return false;
    }
    
    bool attached = tt::hal::i2c::masterHasDeviceAtAddress(keypad->getPort(), keypad->getAddress(), 100);
    
    if (!attached) {
        TT_LOG_W(TAG, "isAttached: TCA8418 not detected at 0x%02X", keypad->getAddress());
    } else {
        TT_LOG_I(TAG, "isAttached: TCA8418 detected successfully at 0x%02X", keypad->getAddress());
    }
    
    return attached;
}

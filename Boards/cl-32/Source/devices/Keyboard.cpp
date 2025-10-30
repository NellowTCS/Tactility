#include "Keyboard.h"
#include <Tactility/hal/i2c/I2c.h>
#include <lvgl.h>
#include <Tactility/Log.h>

constexpr auto* TAG = "CL32Keyboard";

// The CL-32 stock firmware uses flat 80-entry arrays.
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
    // The TCA8418 wrapper used elsewhere in the project exposes rows/cols
    // in the same coordinate system the original firmware expects.
    // Ensure values are within bounds and convert to flat 1..80 -> 0..79 indexing.
    if (row < 0 || col < 0) return -1;
    // Some wrappers present rows 0..3 and cols 0..9 (40 keys), others report 8x10 matrix.
    // The CL-32 hardware uses up to 80 keys. We assume row*10+col mapping here.
    int idx = (row * 10) + col;
    if (idx < 0 || idx >= 80) return -1;
    return idx;
}

CL32Keyboard::CL32Keyboard(const std::shared_ptr<Tca8418>& tca)
    : keypad(tca)
    , kbHandle(nullptr)
    , inputTimer(nullptr)
{
    ESP_LOGI(TAG, "CL32Keyboard constructor called");
    // Create FreeRTOS queue for key events (16 chars deep)
    queue = xQueueCreate(16, sizeof(char));
    if (!queue) {
        ESP_LOGE(TAG, "Failed to create keyboard queue!");
    } else {
        ESP_LOGI(TAG, "Keyboard queue created successfully");
    }
}

CL32Keyboard::~CL32Keyboard() {
    ESP_LOGI(TAG, "CL32Keyboard destructor called");
    stopLvgl();
    if (queue) {
        vQueueDelete(queue);
        queue = nullptr;
        ESP_LOGI(TAG, "Keyboard queue deleted");
    }
}

void CL32Keyboard::readCallback(lv_indev_t* indev, lv_indev_data_t* data) {
    auto keyboard = static_cast<CL32Keyboard*>(lv_indev_get_user_data(indev));
    char keypress = 0;

    if (keyboard == nullptr) {
        data->key = 0;
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    // Non-blocking: provide a pressed event when a key is available in queue
    if (xQueueReceive(keyboard->queue, &keypress, 0) == pdPASS) {
        data->key = keypress;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->key = 0;
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void CL32Keyboard::processKeyboard() {
    // Mirror the shift/sym/caps toggle behavior from the CL-32 stock firmware.
    static bool shift_pressed = false;
    static bool sym_pressed = false;
    static bool cap_toggle = false;
    static bool cap_toggle_armed = true;
    static bool first_run = true;

    if (first_run) {
        ESP_LOGI(TAG, "processKeyboard: first run");
        first_run = false;
    }

    if (!keypad) {
        ESP_LOGE(TAG, "processKeyboard: keypad is null!");
        return;
    }

    if (keypad->update()) {
        ESP_LOGI(TAG, "Keypad updated: %d pressed, %d released", 
                 keypad->pressed_key_count, keypad->released_key_count);
        
        bool anykey_pressed = (keypad->pressed_key_count > 0);

        // detect modifiers first
        for (int i = 0; i < keypad->pressed_key_count; ++i) {
            int row = keypad->pressed_list[i].row;
            int col = keypad->pressed_list[i].col;
            int idx = rc_to_index(row, col);
            uint8_t special = (idx >= 0) ? key_map_codes[idx] : 0;
            if (special == 225) shift_pressed = true;
            if (special == 60) sym_pressed = true;
        }

        if ((sym_pressed && shift_pressed) && cap_toggle_armed) {
            cap_toggle = !cap_toggle;
            cap_toggle_armed = false;
        }

        // enqueue pressed keys
        for (int i = 0; i < keypad->pressed_key_count; ++i) {
            int row = keypad->pressed_list[i].row;
            int col = keypad->pressed_list[i].col;
            int idx = rc_to_index(row, col);
            if (idx < 0) continue;

            uint8_t special = key_map_codes[idx];

            // Handle special-coded keys first
            if (special == 42) { // backspace
                char ch = (char)LV_KEY_BACKSPACE;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            }
            if (special == 40) { // enter
                char ch = (char)LV_KEY_ENTER;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            }
            if (special == 79) { // right
                char ch = (char)LV_KEY_RIGHT;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            }
            if (special == 80) { // left
                char ch = (char)LV_KEY_LEFT;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            }
            if (special == 81) { // down
                char ch = (char)LV_KEY_DOWN;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            }
            if (special == 82) { // up
                char ch = (char)LV_KEY_UP;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            }
            if (special == 225) {
                // shift key pressed — handled as modifier, do not enqueue
                continue;
            }
            if (special == 60) {
                // menu/symbol key: map to ESC (original toggled mode)
                char ch = (char)LV_KEY_ESC;
                xQueueSend(queue, &ch, 50 / portTICK_PERIOD_MS);
                continue;
            }

            // Printable character: choose map based on modifiers
            char chr = 0;
            if (sym_pressed) {
                // CL-32 original used a symbol mode; stock firmware used a different symbol array.
                // Fallback: use lower_map for symbols (preserve behaviour for basic keys).
                chr = lower_map[idx];
            } else if (shift_pressed || cap_toggle) {
                chr = upper_map[idx];
            } else {
                chr = lower_map[idx];
            }

            if (chr != 0) {
                ESP_LOGI(TAG, "Key: row=%d col=%d idx=%d char='%c' (0x%02X)", 
                         row, col, idx, chr, (uint8_t)chr);
                xQueueSend(queue, &chr, 50 / portTICK_PERIOD_MS);
            }
        }

        // update modifier states on released keys
        for (int i = 0; i < keypad->released_key_count; ++i) {
            int row = keypad->released_list[i].row;
            int col = keypad->released_list[i].col;
            int idx = rc_to_index(row, col);
            uint8_t special = (idx >= 0) ? key_map_codes[idx] : 0;
            if (special == 225) shift_pressed = false;
            if (special == 60) sym_pressed = false;
        }

        if ((!sym_pressed && !shift_pressed) && !cap_toggle_armed) {
            cap_toggle_armed = true;
        }

        // no keyboard backlight on CL-32 hardware — skip any backlight impulses
        (void)anykey_pressed;
    }
}

bool CL32Keyboard::startLvgl(lv_display_t* display) {
    ESP_LOGI(TAG, "startLvgl called");
    
    if (!keypad) {
        ESP_LOGE(TAG, "startLvgl: keypad is null!");
        return false;
    }

    // Initialize TCA wrapper keypad to expected rows/cols (mirror Tpager)
    ESP_LOGI(TAG, "Initializing keypad with 8 rows x 10 cols");
    keypad->init(8, 10); // 8 rows x 10 cols (CL-32 mapping)

    // create the periodic input polling timer (similar interval to other drivers)
    assert(inputTimer == nullptr);
    ESP_LOGI(TAG, "Creating input timer");
    inputTimer = std::make_unique<tt::Timer>(tt::Timer::Type::Periodic, [this] {
        this->processKeyboard();
    });

    // Create LVGL input device and hook read callback
    ESP_LOGI(TAG, "Creating LVGL input device");
    kbHandle = lv_indev_create();
    lv_indev_set_type(kbHandle, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(kbHandle, &CL32Keyboard::readCallback);
    lv_indev_set_display(kbHandle, display);
    lv_indev_set_user_data(kbHandle, this);

    ESP_LOGI(TAG, "Starting input timer (20ms interval)");
    inputTimer->start(20 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "CL32Keyboard startLvgl completed successfully");
    return true;
}

bool CL32Keyboard::stopLvgl() {
    ESP_LOGI(TAG, "stopLvgl called");
    
    if (inputTimer) {
        ESP_LOGI(TAG, "Stopping input timer");
        inputTimer->stop();
        inputTimer = nullptr;
    }
    if (kbHandle) {
        ESP_LOGI(TAG, "Deleting LVGL input device");
        lv_indev_delete(kbHandle);
        kbHandle = nullptr;
    }
    
    ESP_LOGI(TAG, "CL32Keyboard stopLvgl completed");
    return true;
}

bool CL32Keyboard::isAttached() const {
    if (!keypad) {
        ESP_LOGW(TAG, "isAttached: keypad is null");
        return false;
    }
    
    bool attached = tt::hal::i2c::masterHasDeviceAtAddress(keypad->getPort(), keypad->getAddress(), 100);
    ESP_LOGI(TAG, "isAttached: %s (port=%d, addr=0x%02X)", 
             attached ? "YES" : "NO", keypad->getPort(), keypad->getAddress());
    return attached;
}

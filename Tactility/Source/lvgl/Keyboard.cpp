#include "Tactility/lvgl/Keyboard.h"
#include "Tactility/service/gui/GuiService.h"

#include <Tactility/service/espnow/EspNowService.h>

namespace tt::lvgl {

static lv_indev_t* keyboard_device = nullptr;

static void keyboard_hide_focus_timer_cb(lv_timer_t* t) {
    lv_group_t* g = static_cast<lv_group_t*>(t->user_data);
    if (g != nullptr) {
        // Only advance if we're no longer in editing mode
        if (!lv_group_get_editing(g)) {
            lv_group_focus_next(g);
        }
    }
    lv_timer_del(t); // single-shot
}

void software_keyboard_show(lv_obj_t* textarea) {
    auto gui_service = service::gui::findService();
    if (gui_service != nullptr) {
        gui_service->softwareKeyboardShow(textarea);
    }
}

void software_keyboard_hide() {
    auto gui_service = service::gui::findService();
    if (gui_service != nullptr) {
        gui_service->softwareKeyboardHide();
    }

    // Ensure LVGL leaves "group editing" mode when the software keyboard is hidden.
    // Schedule a short single-shot timer to advance focus once, avoiding races and double-advances.
    lv_group_t* g = lv_group_get_default();
    if (g != nullptr) {
        if (lv_group_get_editing(g)) {
            lv_group_set_editing(g, false);
            // schedule a single-shot timer (100 ms) to do the focus change
            lv_timer_t* t = lv_timer_create(keyboard_hide_focus_timer_cb, 100, g);
            (void)t;
        }
    }
}

bool software_keyboard_is_enabled() {
    auto gui_service = service::gui::findService();
    if (gui_service != nullptr) {
        return gui_service->softwareKeyboardIsEnabled();
    } else {
        return false;
    }
}

void software_keyboard_activate(lv_group_t* group) {
    if (keyboard_device != nullptr) {
        lv_indev_set_group(keyboard_device, group);
    }
}

void software_keyboard_deactivate() {
    if (keyboard_device != nullptr) {
        lv_indev_set_group(keyboard_device, nullptr);
    }
}

bool hardware_keyboard_is_available() {
    return keyboard_device != nullptr;
}

void hardware_keyboard_set_indev(lv_indev_t* device) {
    keyboard_device = device;
}

}

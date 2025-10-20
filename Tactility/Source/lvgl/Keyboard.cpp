#include "Tactility/lvgl/Keyboard.h"
#include "Tactility/service/gui/GuiService.h"

#include <Tactility/service/espnow/EspNowService.h>

namespace tt::lvgl {

static lv_indev_t* keyboard_device = nullptr;

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
    // On LVGL v9 the encoder/group navigation is ignored while the group is in editing mode,
    // so explicitly turn editing off and advance focus so the user can continue tabbing.
    lv_group_t* g = lv_group_get_default();
    if (g != nullptr) {
        if (lv_group_get_editing(g)) {
            lv_group_set_editing(g, false);
            lv_group_focus_next(g);
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

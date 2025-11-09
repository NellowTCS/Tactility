#include "Tactility/lvgl/Style.h"

namespace tt::lvgl {

void obj_set_style_bg_blacken(lv_obj_t* obj) {
    lv_obj_set_style_bg_color(obj, lv_color_black(), 0);
    lv_obj_set_style_border_color(obj, lv_color_black(), 0);
}

void obj_set_style_bg_invisible(lv_obj_t* obj) {
    lv_obj_set_style_bg_opa(obj, 0, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
}

void obj_set_style_no_padding(lv_obj_t* obj) {
    // Legacy helper: explicitly sets zero padding
    // Consider using UiStyle::setContainerPadding() or UiStyle::setZeroPadding() 
    // for new code to ensure proper scaling behavior
    lv_obj_set_style_pad_all(obj, 0, 0);
    lv_obj_set_style_pad_gap(obj, 0, 0);
}

} // namespace

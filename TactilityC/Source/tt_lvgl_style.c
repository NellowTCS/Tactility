#include "tt_lvgl_style.h"
#include "tt_hal.h"
#include <lvgl.h>

// Get display width from LVGL
static int32_t get_display_width(void) {
    lv_obj_t* screen = lv_scr_act();
    if (screen) {
        return lv_obj_get_width(screen);
    }
    return 320; // Default fallback
}

void tt_lvgl_set_container_padding(lv_obj_t* obj, tt_container_type_t type, int32_t custom_value) {
    if (!obj) return;
    
    int32_t display_width = get_display_width();
    int32_t general_padding = tt_hal_configuration_get_general_vertical_padding();
    int32_t button_padding = tt_hal_configuration_get_button_padding();
    int32_t padding_value;
    
    switch (type) {
        case TT_CONTAINER_FULLSCREEN:
            // Full-screen containers: always 0 padding
            padding_value = 0;
            break;
            
        case TT_CONTAINER_LAYOUT:
            // Layout containers: minimal padding for structure
            // Small displays: use explicit small value
            // Large displays: let LVGL theme control
            if (display_width < 320) {
                padding_value = 2;
            } else if (display_width < 800) {
                padding_value = 4;
            } else {
                padding_value = -1; // Let LVGL theme control
            }
            break;
            
        case TT_CONTAINER_CONTENT:
            // Content containers: comfortable reading/interaction padding
            if (display_width < 170) {
                padding_value = general_padding;
            } else if (display_width < 320) {
                padding_value = general_padding * 2;
            } else if (display_width < 800) {
                padding_value = general_padding * 3;
            } else {
                padding_value = -1; // Let LVGL theme control
            }
            break;
            
        case TT_CONTAINER_INTERACTIVE:
            // Interactive elements: button-like padding
            if (button_padding >= 0) {
                padding_value = button_padding;
            } else if (display_width < 170) {
                padding_value = 4;
            } else if (display_width < 320) {
                padding_value = 6;
            } else if (display_width < 800) {
                padding_value = 8;
            } else {
                padding_value = -1; // Let LVGL theme control
            }
            break;
            
        case TT_CONTAINER_CUSTOM:
            // Custom: use provided value
            padding_value = custom_value;
            break;
            
        default:
            // Unknown type: don't set anything
            return;
    }
    
    lv_obj_set_style_pad_all(obj, padding_value, LV_STATE_DEFAULT);
}

void tt_lvgl_set_layout_gap(lv_obj_t* obj, tt_container_type_t gap_type) {
    if (!obj) return;
    
    int32_t display_width = get_display_width();
    int32_t general_padding = tt_hal_configuration_get_general_vertical_padding();
    int32_t object_gap = tt_hal_configuration_get_object_gap();
    int32_t gap_value;
    
    switch (gap_type) {
        case TT_CONTAINER_FULLSCREEN:
            gap_value = 0;
            break;
            
        case TT_CONTAINER_LAYOUT:
            if (object_gap >= 0) {
                gap_value = object_gap;
            } else if (display_width < 170) {
                gap_value = 2;
            } else if (display_width < 320) {
                gap_value = 4;
            } else {
                gap_value = 6;
            }
            break;
            
        case TT_CONTAINER_CONTENT:
            if (display_width < 170) {
                gap_value = general_padding;
            } else if (display_width < 320) {
                gap_value = general_padding * 2;
            } else {
                gap_value = general_padding * 3;
            }
            break;
            
        case TT_CONTAINER_INTERACTIVE:
            if (display_width < 170) {
                gap_value = 4;
            } else if (display_width < 320) {
                gap_value = 6;
            } else {
                gap_value = 8;
            }
            break;
            
        case TT_CONTAINER_CUSTOM:
            // For custom, caller should use lv_obj_set_style_pad_gap directly
            return;
            
        default:
            return;
    }
    
    lv_obj_set_style_pad_gap(obj, gap_value, LV_STATE_DEFAULT);
    lv_obj_set_style_pad_row(obj, gap_value, LV_STATE_DEFAULT);
    lv_obj_set_style_pad_column(obj, gap_value, LV_STATE_DEFAULT);
}

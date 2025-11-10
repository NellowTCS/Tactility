#pragma once

#include <lvgl.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file tt_lvgl_style.h
 * @brief Semantic styling API for responsive UI design
 * 
 * Provides container type-based padding that automatically scales with display size.
 * This replaces hardcoded padding values with semantic container types that maintain
 * proper layouts across different screen sizes.
 */

/**
 * @brief Semantic container types for automatic padding
 * 
 * Each container type represents a UI purpose and applies appropriate padding
 * that scales with display size through UiMetrics.
 */
typedef enum {
    /** Full-screen containers with no padding (e.g., app root, image viewers) */
    TT_CONTAINER_FULLSCREEN = 0,
    
    /** Layout containers with minimal padding (e.g., lists, grids, panels) */
    TT_CONTAINER_LAYOUT = 1,
    
    /** Standard content containers with comfortable padding (e.g., forms, cards) */
    TT_CONTAINER_CONTENT = 2,
    
    /** Interactive elements with button-like padding (e.g., buttons, list items) */
    TT_CONTAINER_INTERACTIVE = 3,
    
    /** Custom padding - requires explicit value */
    TT_CONTAINER_CUSTOM = 4
} tt_container_type_t;

/**
 * @brief Set container padding based on semantic type
 * 
 * Automatically applies appropriate padding based on container type and display size.
 * Uses UiMetrics to ensure consistent, scalable layouts.
 * 
 * @param obj The LVGL object to style
 * @param type The semantic container type
 * @param custom_value Custom padding value (only used if type is TT_CONTAINER_CUSTOM)
 * 
 * Example:
 * @code
 * lv_obj_t* root = lv_obj_create(parent);
 * tt_lvgl_set_container_padding(root, TT_CONTAINER_FULLSCREEN, 0);
 * 
 * lv_obj_t* form = lv_obj_create(root);
 * tt_lvgl_set_container_padding(form, TT_CONTAINER_CONTENT, 0);
 * 
 * lv_obj_t* button_area = lv_obj_create(form);
 * tt_lvgl_set_container_padding(button_area, TT_CONTAINER_INTERACTIVE, 0);
 * @endcode
 */
void tt_lvgl_set_container_padding(lv_obj_t* obj, tt_container_type_t type, int32_t custom_value);

/**
 * @brief Set layout gap (spacing between child elements)
 * 
 * Uses UiMetrics to provide consistent gaps across display sizes.
 * 
 * @param obj The LVGL object to style
 * @param gap_type The gap type (same as container type for consistency)
 */
void tt_lvgl_set_layout_gap(lv_obj_t* obj, tt_container_type_t gap_type);

#ifdef __cplusplus
}
#endif

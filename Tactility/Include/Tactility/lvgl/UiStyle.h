#pragma once

#include <lvgl.h>

namespace tt::lvgl {

/**
 * UI styling utilities that work with UiMetrics for consistent, scalable layouts.
 * 
 * These functions provide semantic styling options that automatically scale
 * with the display size while preserving intentional layout decisions.
 */

/**
 * Layout container types that define padding behavior
 */
enum class ContainerType {
    // Root containers that should have zero padding for full-screen layouts
    FullScreen,     // App containers, screen roots, image viewers
    
    // Layout containers that organize content but shouldn't add spacing
    Layout,         // Flex containers, grid wrappers, section containers
    
    // Content containers that should have padding for visual separation
    Content,        // Cards, panels, grouped content areas
    
    // Interactive containers that need specific padding for touch targets
    Interactive,    // Button containers, form fields, input areas
    
    // Special containers that need custom padding (manual override)
    Custom          // Use explicit padding values
};

/**
 * Apply appropriate padding based on container purpose and current UI metrics.
 * This replaces direct lv_obj_set_style_pad_all() calls in most cases.
 * 
 * @param obj The LVGL object to style
 * @param type The semantic type of container
 * @param customPadding For ContainerType::Custom, the explicit padding value
 */
void setContainerPadding(lv_obj_t* obj, ContainerType type, int customPadding = 0);

/**
 * Apply gap spacing for flex containers based on UI metrics.
 * Use this instead of hardcoded gap values.
 * 
 * @param obj The flex container object
 * @param scale Scale factor (1.0 = normal, 0.5 = half spacing, 2.0 = double spacing)
 */
void setFlexGap(lv_obj_t* obj, float scale = 1.0f);

/**
 * Set padding for specific layout purposes with semantic meaning
 */
void setPaddingForFormLayout(lv_obj_t* obj);      // Form fields and labels
void setPaddingForButtonLayout(lv_obj_t* obj);    // Button containers  
void setPaddingForListLayout(lv_obj_t* obj);      // List items and containers
void setPaddingForCardLayout(lv_obj_t* obj);      // Card/panel content areas

/**
 * Legacy compatibility: explicitly set zero padding (use sparingly)
 * Only use when you specifically need zero padding for layout purposes.
 */
void setZeroPadding(lv_obj_t* obj);

} // namespace
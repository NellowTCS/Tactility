#include "Tactility/Tactility.h"
#include "Tactility/lvgl/UiStyle.h"
#include "Tactility/hal/UiMetrics.h"
#include "Tactility/hal/Configuration.h"

namespace tt::lvgl {

void setContainerPadding(lv_obj_t* obj, ContainerType type, int customPadding) {
    const auto& metrics = tt::hal::getConfiguration()->uiMetrics;
    
    switch (type) {
        case ContainerType::FullScreen:
            // Full-screen containers should have zero padding
            lv_obj_set_style_pad_all(obj, 0, LV_STATE_DEFAULT);
            break;
            
        case ContainerType::Layout:
            // Layout containers use minimal padding or theme default
            if (metrics.objectPadding >= 0) {
                // For small displays, use minimal padding
                int layoutPadding = metrics.objectPadding > 0 ? 1 : 0;
                lv_obj_set_style_pad_all(obj, layoutPadding, LV_STATE_DEFAULT);
            }
            // For large displays (metrics.objectPadding == -1), let theme decide
            break;
            
        case ContainerType::Content:
            // Content containers use standard object padding
            if (metrics.objectPadding >= 0) {
                lv_obj_set_style_pad_all(obj, metrics.objectPadding, LV_STATE_DEFAULT);
            }
            // For large displays, let wrapper/theme handle it
            break;
            
        case ContainerType::Interactive:
            // Interactive containers use button padding for touch targets
            if (metrics.buttonPadding >= 0) {
                lv_obj_set_style_pad_all(obj, metrics.buttonPadding, LV_STATE_DEFAULT);
            }
            break;
            
        case ContainerType::Custom:
            // Use explicit custom padding value
            lv_obj_set_style_pad_all(obj, customPadding, LV_STATE_DEFAULT);
            break;
    }
}

void setFlexGap(lv_obj_t* obj, float scale) {
    const auto& metrics = tt::hal::getConfiguration()->uiMetrics;
    
    if (metrics.objectGap >= 0) {
        int gap = static_cast<int>(metrics.objectGap * scale);
        lv_obj_set_style_pad_gap(obj, gap, LV_STATE_DEFAULT);
    }
    // For large displays with theme defaults, let wrapper handle it
}

void setPaddingForFormLayout(lv_obj_t* obj) {
    setContainerPadding(obj, ContainerType::Layout);
}

void setPaddingForButtonLayout(lv_obj_t* obj) {
    setContainerPadding(obj, ContainerType::Interactive);
}

void setPaddingForListLayout(lv_obj_t* obj) {
    const auto& metrics = tt::hal::getConfiguration()->uiMetrics;
    if (metrics.listPadding >= 0) {
        lv_obj_set_style_pad_all(obj, metrics.listPadding, LV_STATE_DEFAULT);
    }
}

void setPaddingForCardLayout(lv_obj_t* obj) {
    setContainerPadding(obj, ContainerType::Content);
}

void setZeroPadding(lv_obj_t* obj) {
    lv_obj_set_style_pad_all(obj, 0, LV_STATE_DEFAULT);
    lv_obj_set_style_pad_gap(obj, 0, LV_STATE_DEFAULT);
}

} // namespace

// C API accessor for TactilityC
extern "C" const UiMetrics* tt_get_ui_metrics() {
    return &tt::hal::getConfiguration()->uiMetrics;
}
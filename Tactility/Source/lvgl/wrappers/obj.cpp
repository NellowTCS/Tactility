#ifdef ESP_PLATFORM

#include <Tactility/Tactility.h>

#include <lvgl.h>

extern "C" {

extern void __real_lv_obj_set_flex_flow(lv_obj_t* obj, lv_flex_flow_t flow);
extern lv_obj_t* __real_lv_obj_create(lv_obj_t* parent);

void __wrap_lv_obj_set_flex_flow(lv_obj_t* obj, lv_flex_flow_t flow) {
    __real_lv_obj_set_flex_flow(obj, flow);

    const auto& metrics = tt::hal::getConfiguration()->uiMetrics;
    lv_obj_set_style_pad_gap(obj, metrics.objectGap, LV_STATE_DEFAULT);
}

lv_obj_t* __wrap_lv_obj_create(lv_obj_t* parent) {
    auto obj = __real_lv_obj_create(parent);
    const auto& metrics = tt::hal::getConfiguration()->uiMetrics;
    lv_obj_set_style_pad_all(obj, metrics.objectPadding, LV_STATE_DEFAULT);
    lv_obj_set_style_pad_gap(obj, metrics.objectGap, LV_STATE_DEFAULT);
    lv_obj_set_style_radius(obj, metrics.objectRadius, LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(obj, metrics.objectBorderWidth, LV_STATE_DEFAULT);
    return obj;
}

}

#endif // ESP_PLATFORM
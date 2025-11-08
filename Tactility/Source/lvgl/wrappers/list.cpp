#ifdef ESP_PLATFORM

#include <Tactility/Tactility.h>

#include <lvgl.h>

extern "C" {

extern lv_obj_t* __real_lv_list_create(lv_obj_t* parent);
extern lv_obj_t* __real_lv_list_add_button(lv_obj_t* list, const void* icon, const char* txt);

lv_obj_t* __wrap_lv_list_create(lv_obj_t* parent) {
    auto* list = __real_lv_list_create(parent);

    const auto& metrics = tt::hal::getConfiguration()->uiMetrics;
    if (metrics.listPadding >= 0) {
        lv_obj_set_style_pad_row(list, metrics.listPadding, LV_STATE_DEFAULT);
        lv_obj_set_style_pad_column(list, metrics.listPadding, LV_STATE_DEFAULT);
        lv_obj_set_style_pad_all(list, metrics.listPadding, LV_STATE_DEFAULT);
    }

    return list;
}

lv_obj_t* __wrap_lv_list_add_button(lv_obj_t* list, const void* icon, const char* txt) {
    auto* button = __real_lv_list_add_button(list, icon, txt);

    const auto& metrics = tt::hal::getConfiguration()->uiMetrics;
    if (metrics.listButtonVertPadding >= 0) {
        lv_obj_set_style_pad_ver(button, metrics.listButtonVertPadding, LV_STATE_DEFAULT);
    }

    return button;
}

}

#endif // ESP_PLATFORM
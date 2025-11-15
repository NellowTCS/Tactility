#ifdef ESP_PLATFORM

#include <Tactility/Tactility.h>

#include <lvgl.h>

extern "C" {

extern lv_obj_t* __real_lv_dropdown_create(lv_obj_t* parent);

lv_obj_t* __wrap_lv_dropdown_create(lv_obj_t* parent) {
    auto dropdown = __real_lv_dropdown_create(parent);

    const auto& metrics = tt::hal::getConfiguration()->uiMetrics;
    if (metrics.dropdownHeight >= 0) {
        lv_dropdown_set_options_static(dropdown, "");
        lv_obj_set_height(lv_dropdown_get_list(dropdown), metrics.dropdownHeight);
    }

    return dropdown;
}

}

#endif // ESP_PLATFORM
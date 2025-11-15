#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Affects LVGL widget style (deprecated - use UiMetrics getters instead) */
typedef enum {
    /** Ideal for very small non-touch screen devices (e.g. Waveshare S3 LCD 1.3") */
    UiScaleSmallest,
    /** Nothing was changed in the LVGL UI/UX */
    UiScaleDefault
} UiScale;

/** @return the UI scaling setting for this device (deprecated - use tt_hal_configuration_get_ui_metrics_* instead). */
UiScale tt_hal_configuration_get_ui_scale();

// UiMetrics getters - use these instead of UiScale
int tt_hal_configuration_get_toolbar_height();
int tt_hal_configuration_get_toolbar_button_inset();
int tt_hal_configuration_get_toolbar_title_padding();
int tt_hal_configuration_get_button_padding();
int tt_hal_configuration_get_button_radius();
int tt_hal_configuration_get_object_padding();
int tt_hal_configuration_get_object_gap();
int tt_hal_configuration_get_object_radius();
int tt_hal_configuration_get_object_border_width();
int tt_hal_configuration_get_list_padding();
int tt_hal_configuration_get_list_button_vert_padding();
int tt_hal_configuration_get_dropdown_height();
int tt_hal_configuration_get_switch_width();
int tt_hal_configuration_get_switch_height();
int tt_hal_configuration_get_textarea_padding();
int tt_hal_configuration_get_launcher_button_size();
int tt_hal_configuration_get_general_vertical_padding();
int tt_hal_configuration_get_locale_settings_offset();
int tt_hal_configuration_get_system_info_padding();
int tt_hal_configuration_get_wifi_manage_scrollbar_width();

#ifdef __cplusplus
}
#endif

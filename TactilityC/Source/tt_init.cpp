#ifdef ESP_PLATFORM

#include "tt_app.h"
#include "tt_app_alertdialog.h"
#include "tt_app_manifest.h"
#include "tt_app_selectiondialog.h"
#include "tt_bundle.h"
#include "tt_gps.h"
#include "tt_hal_i2c.h"
#include "tt_lvgl_keyboard.h"
#include "tt_lvgl_spinner.h"
#include "tt_lvgl_toolbar.h"
#include "tt_message_queue.h"
#include "tt_mutex.h"
#include "tt_preferences.h"
#include "tt_semaphore.h"
#include "tt_thread.h"
#include "tt_time.h"
#include "tt_timer.h"
#include "tt_wifi.h"

#include <cstring>
#include <ctype.h>
#include <private/elf_symbol.h>
#include <esp_log.h>
#include <cassert>

#include <lvgl.h>

extern "C" {

// Hidden functions work-around
extern void* _Znwj(uint32_t size);
extern void _ZdlPvj(void* p, uint64_t size);
extern double __adddf3(double a, double b);
extern double __subdf3(double a, double b);
extern double __muldf3 (double a, double b);
extern double __divdf3 (double a, double b);
extern int __nedf2 (double a, double b);

const struct esp_elfsym elf_symbols[] {
    // Hidden functions work-around
    ESP_ELFSYM_EXPORT(_ZdlPvj), // new?
    ESP_ELFSYM_EXPORT(_Znwj), // delete?
    ESP_ELFSYM_EXPORT(__adddf3), // Routines for floating point emulation:
    ESP_ELFSYM_EXPORT(__subdf3), // See https://gcc.gnu.org/onlinedocs/gccint/Soft-float-library-routines.html
    ESP_ELFSYM_EXPORT(__muldf3),
    ESP_ELFSYM_EXPORT(__nedf2),
    ESP_ELFSYM_EXPORT(__divdf3),
    // <cassert>
    ESP_ELFSYM_EXPORT(__assert_func),
    // <cstdio>
    ESP_ELFSYM_EXPORT(fclose),
    ESP_ELFSYM_EXPORT(feof),
    ESP_ELFSYM_EXPORT(ferror),
    ESP_ELFSYM_EXPORT(fflush),
    ESP_ELFSYM_EXPORT(fgetc),
    ESP_ELFSYM_EXPORT(fgetpos),
    ESP_ELFSYM_EXPORT(fgets),
    ESP_ELFSYM_EXPORT(fopen),
    ESP_ELFSYM_EXPORT(fputc),
    ESP_ELFSYM_EXPORT(fputs),
    ESP_ELFSYM_EXPORT(fprintf),
    ESP_ELFSYM_EXPORT(fread),
    ESP_ELFSYM_EXPORT(fseek),
    ESP_ELFSYM_EXPORT(fsetpos),
    ESP_ELFSYM_EXPORT(fscanf),
    ESP_ELFSYM_EXPORT(ftell),
    ESP_ELFSYM_EXPORT(fwrite),
    ESP_ELFSYM_EXPORT(getc),
    ESP_ELFSYM_EXPORT(putc),
    ESP_ELFSYM_EXPORT(puts),
    ESP_ELFSYM_EXPORT(printf),
    ESP_ELFSYM_EXPORT(sscanf),
    ESP_ELFSYM_EXPORT(snprintf),
    ESP_ELFSYM_EXPORT(sprintf),
    ESP_ELFSYM_EXPORT(vsprintf),
    // cstring
    ESP_ELFSYM_EXPORT(strlen),
    ESP_ELFSYM_EXPORT(strcmp),
    ESP_ELFSYM_EXPORT(strncpy),
    ESP_ELFSYM_EXPORT(strcpy),
    ESP_ELFSYM_EXPORT(strcat),
    ESP_ELFSYM_EXPORT(strchr),
    ESP_ELFSYM_EXPORT(strstr),
    ESP_ELFSYM_EXPORT(memset),
    ESP_ELFSYM_EXPORT(memcpy),
    // ctype
    ESP_ELFSYM_EXPORT(isalnum),
    ESP_ELFSYM_EXPORT(isalpha),
    ESP_ELFSYM_EXPORT(iscntrl),
    ESP_ELFSYM_EXPORT(isdigit),
    ESP_ELFSYM_EXPORT(isgraph),
    ESP_ELFSYM_EXPORT(islower),
    ESP_ELFSYM_EXPORT(isprint),
    ESP_ELFSYM_EXPORT(ispunct),
    ESP_ELFSYM_EXPORT(isspace),
    ESP_ELFSYM_EXPORT(isupper),
    ESP_ELFSYM_EXPORT(isxdigit),
    ESP_ELFSYM_EXPORT(tolower),
    ESP_ELFSYM_EXPORT(toupper),
    // ESP-IDF
    ESP_ELFSYM_EXPORT(esp_log_write),
    ESP_ELFSYM_EXPORT(esp_log_timestamp),
    // Tactility
    ESP_ELFSYM_EXPORT(tt_app_register),
    ESP_ELFSYM_EXPORT(tt_app_get_parameters),
    ESP_ELFSYM_EXPORT(tt_app_set_result),
    ESP_ELFSYM_EXPORT(tt_app_has_result),
    ESP_ELFSYM_EXPORT(tt_app_selectiondialog_start),
    ESP_ELFSYM_EXPORT(tt_app_selectiondialog_get_result_index),
    ESP_ELFSYM_EXPORT(tt_app_alertdialog_start),
    ESP_ELFSYM_EXPORT(tt_app_alertdialog_get_result_index),
    ESP_ELFSYM_EXPORT(tt_bundle_alloc),
    ESP_ELFSYM_EXPORT(tt_bundle_free),
    ESP_ELFSYM_EXPORT(tt_bundle_opt_bool),
    ESP_ELFSYM_EXPORT(tt_bundle_opt_int32),
    ESP_ELFSYM_EXPORT(tt_bundle_opt_string),
    ESP_ELFSYM_EXPORT(tt_bundle_put_bool),
    ESP_ELFSYM_EXPORT(tt_bundle_put_int32),
    ESP_ELFSYM_EXPORT(tt_bundle_put_string),
    ESP_ELFSYM_EXPORT(tt_gps_has_coordinates),
    ESP_ELFSYM_EXPORT(tt_gps_get_coordinates),
    ESP_ELFSYM_EXPORT(tt_hal_i2c_start),
    ESP_ELFSYM_EXPORT(tt_hal_i2c_stop),
    ESP_ELFSYM_EXPORT(tt_hal_i2c_is_started),
    ESP_ELFSYM_EXPORT(tt_hal_i2c_master_read),
    ESP_ELFSYM_EXPORT(tt_hal_i2c_master_read_register),
    ESP_ELFSYM_EXPORT(tt_hal_i2c_master_write),
    ESP_ELFSYM_EXPORT(tt_hal_i2c_master_write_register),
    ESP_ELFSYM_EXPORT(tt_hal_i2c_master_write_read),
    ESP_ELFSYM_EXPORT(tt_hal_i2c_master_has_device_at_address),
    ESP_ELFSYM_EXPORT(tt_hal_i2c_lock),
    ESP_ELFSYM_EXPORT(tt_hal_i2c_unlock),
    ESP_ELFSYM_EXPORT(tt_lvgl_software_keyboard_show),
    ESP_ELFSYM_EXPORT(tt_lvgl_software_keyboard_hide),
    ESP_ELFSYM_EXPORT(tt_lvgl_software_keyboard_is_enabled),
    ESP_ELFSYM_EXPORT(tt_lvgl_software_keyboard_activate),
    ESP_ELFSYM_EXPORT(tt_lvgl_software_keyboard_deactivate),
    ESP_ELFSYM_EXPORT(tt_lvgl_hardware_keyboard_is_available),
    ESP_ELFSYM_EXPORT(tt_lvgl_hardware_keyboard_set_indev),
    ESP_ELFSYM_EXPORT(tt_lvgl_keyboard_add_textarea),
    ESP_ELFSYM_EXPORT(tt_lvgl_toolbar_create),
    ESP_ELFSYM_EXPORT(tt_lvgl_toolbar_create_for_app),
    ESP_ELFSYM_EXPORT(tt_message_queue_alloc),
    ESP_ELFSYM_EXPORT(tt_message_queue_free),
    ESP_ELFSYM_EXPORT(tt_message_queue_put),
    ESP_ELFSYM_EXPORT(tt_message_queue_get),
    ESP_ELFSYM_EXPORT(tt_message_queue_get_capacity),
    ESP_ELFSYM_EXPORT(tt_message_queue_get_message_size),
    ESP_ELFSYM_EXPORT(tt_message_queue_get_count),
    ESP_ELFSYM_EXPORT(tt_message_queue_reset),
    ESP_ELFSYM_EXPORT(tt_mutex_alloc),
    ESP_ELFSYM_EXPORT(tt_mutex_free),
    ESP_ELFSYM_EXPORT(tt_mutex_lock),
    ESP_ELFSYM_EXPORT(tt_mutex_unlock),
    ESP_ELFSYM_EXPORT(tt_preferences_alloc),
    ESP_ELFSYM_EXPORT(tt_preferences_free),
    ESP_ELFSYM_EXPORT(tt_preferences_opt_bool),
    ESP_ELFSYM_EXPORT(tt_preferences_opt_int32),
    ESP_ELFSYM_EXPORT(tt_preferences_opt_string),
    ESP_ELFSYM_EXPORT(tt_preferences_put_bool),
    ESP_ELFSYM_EXPORT(tt_preferences_put_int32),
    ESP_ELFSYM_EXPORT(tt_preferences_put_string),
    ESP_ELFSYM_EXPORT(tt_semaphore_alloc),
    ESP_ELFSYM_EXPORT(tt_semaphore_free),
    ESP_ELFSYM_EXPORT(tt_semaphore_acquire),
    ESP_ELFSYM_EXPORT(tt_semaphore_release),
    ESP_ELFSYM_EXPORT(tt_semaphore_get_count),
    ESP_ELFSYM_EXPORT(tt_thread_alloc),
    ESP_ELFSYM_EXPORT(tt_thread_alloc_ext),
    ESP_ELFSYM_EXPORT(tt_thread_free),
    ESP_ELFSYM_EXPORT(tt_thread_set_name),
    ESP_ELFSYM_EXPORT(tt_thread_set_stack_size),
    ESP_ELFSYM_EXPORT(tt_thread_set_callback),
    ESP_ELFSYM_EXPORT(tt_thread_set_priority),
    ESP_ELFSYM_EXPORT(tt_thread_set_state_callback),
    ESP_ELFSYM_EXPORT(tt_thread_get_state),
    ESP_ELFSYM_EXPORT(tt_thread_start),
    ESP_ELFSYM_EXPORT(tt_thread_join),
    ESP_ELFSYM_EXPORT(tt_thread_get_id),
    ESP_ELFSYM_EXPORT(tt_thread_get_return_code),
    ESP_ELFSYM_EXPORT(tt_timer_alloc),
    ESP_ELFSYM_EXPORT(tt_timer_free),
    ESP_ELFSYM_EXPORT(tt_timer_start),
    ESP_ELFSYM_EXPORT(tt_timer_restart),
    ESP_ELFSYM_EXPORT(tt_timer_stop),
    ESP_ELFSYM_EXPORT(tt_timer_is_running),
    ESP_ELFSYM_EXPORT(tt_timer_get_expire_time),
    ESP_ELFSYM_EXPORT(tt_timer_set_pending_callback),
    ESP_ELFSYM_EXPORT(tt_timer_set_thread_priority),
    ESP_ELFSYM_EXPORT(tt_timezone_set),
    ESP_ELFSYM_EXPORT(tt_timezone_get_name),
    ESP_ELFSYM_EXPORT(tt_timezone_get_code),
    ESP_ELFSYM_EXPORT(tt_timezone_is_format_24_hour),
    ESP_ELFSYM_EXPORT(tt_timezone_set_format_24_hour),
    ESP_ELFSYM_EXPORT(tt_wifi_get_radio_state),
    ESP_ELFSYM_EXPORT(tt_wifi_radio_state_to_string),
    ESP_ELFSYM_EXPORT(tt_wifi_scan),
    ESP_ELFSYM_EXPORT(tt_wifi_is_scanning),
    ESP_ELFSYM_EXPORT(tt_wifi_get_connection_target),
    ESP_ELFSYM_EXPORT(tt_wifi_set_enabled),
    ESP_ELFSYM_EXPORT(tt_wifi_connect),
    ESP_ELFSYM_EXPORT(tt_wifi_disconnect),
    ESP_ELFSYM_EXPORT(tt_wifi_is_connnection_secure),
    ESP_ELFSYM_EXPORT(tt_wifi_get_rssi),
    // tt::lvgl
    ESP_ELFSYM_EXPORT(tt_lvgl_spinner_create),
    // lv_event
    ESP_ELFSYM_EXPORT(lv_event_get_code),
    ESP_ELFSYM_EXPORT(lv_event_get_indev),
    ESP_ELFSYM_EXPORT(lv_event_get_key),
    ESP_ELFSYM_EXPORT(lv_event_get_param),
    ESP_ELFSYM_EXPORT(lv_event_get_scroll_anim),
    ESP_ELFSYM_EXPORT(lv_event_get_user_data),
    ESP_ELFSYM_EXPORT(lv_event_get_target_obj),
    ESP_ELFSYM_EXPORT(lv_event_get_target),
    ESP_ELFSYM_EXPORT(lv_event_get_current_target_obj),
    // lv_obj
    ESP_ELFSYM_EXPORT(lv_obj_create),
    ESP_ELFSYM_EXPORT(lv_obj_delete),
    ESP_ELFSYM_EXPORT(lv_obj_add_event_cb),
    ESP_ELFSYM_EXPORT(lv_obj_align),
    ESP_ELFSYM_EXPORT(lv_obj_align_to),
    ESP_ELFSYM_EXPORT(lv_obj_get_parent),
    ESP_ELFSYM_EXPORT(lv_obj_get_height),
    ESP_ELFSYM_EXPORT(lv_obj_get_width),
    ESP_ELFSYM_EXPORT(lv_obj_get_coords),
    ESP_ELFSYM_EXPORT(lv_obj_get_x),
    ESP_ELFSYM_EXPORT(lv_obj_get_display),
    ESP_ELFSYM_EXPORT(lv_obj_get_y),
    ESP_ELFSYM_EXPORT(lv_obj_get_content_width),
    ESP_ELFSYM_EXPORT(lv_obj_get_content_height),
    ESP_ELFSYM_EXPORT(lv_obj_center),
    ESP_ELFSYM_EXPORT(lv_color_make),
    ESP_ELFSYM_EXPORT(lv_obj_remove_event_cb),
    ESP_ELFSYM_EXPORT(lv_obj_get_user_data),
    ESP_ELFSYM_EXPORT(lv_obj_set_user_data),
    ESP_ELFSYM_EXPORT(lv_obj_remove_flag),
    ESP_ELFSYM_EXPORT(lv_obj_add_flag),
    ESP_ELFSYM_EXPORT(lv_obj_set_pos),
    ESP_ELFSYM_EXPORT(lv_obj_set_flex_align),
    ESP_ELFSYM_EXPORT(lv_obj_set_flex_flow),
    ESP_ELFSYM_EXPORT(lv_obj_set_flex_grow),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_bg_color),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_margin_hor),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_margin_ver),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_margin_top),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_margin_bottom),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_margin_left),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_margin_right),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_margin_all),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_pad_all),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_pad_hor),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_pad_ver),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_pad_top),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_pad_bottom),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_pad_left),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_pad_right),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_pad_column),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_pad_row),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_border_width),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_border_opa),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_border_post),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_border_side),
    ESP_ELFSYM_EXPORT(lv_obj_set_style_border_color),
    ESP_ELFSYM_EXPORT(lv_obj_set_align),
    ESP_ELFSYM_EXPORT(lv_obj_set_x),
    ESP_ELFSYM_EXPORT(lv_obj_set_y),
    ESP_ELFSYM_EXPORT(lv_obj_set_size),
    ESP_ELFSYM_EXPORT(lv_obj_set_width),
    ESP_ELFSYM_EXPORT(lv_obj_set_height),
    ESP_ELFSYM_EXPORT(lv_theme_get_color_primary),
    ESP_ELFSYM_EXPORT(lv_theme_get_color_secondary),
    // lv_button
    ESP_ELFSYM_EXPORT(lv_button_create),
    // lv_buttonmatrix
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_create),
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_get_button_text),
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_get_map),
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_get_one_checked),
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_get_selected_button),
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_set_button_ctrl),
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_set_button_ctrl_all),
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_set_ctrl_map),
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_set_map),
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_set_one_checked),
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_set_button_width),
    ESP_ELFSYM_EXPORT(lv_buttonmatrix_set_selected_button),
    // lv_label
    ESP_ELFSYM_EXPORT(lv_label_create),
    ESP_ELFSYM_EXPORT(lv_label_cut_text),
    ESP_ELFSYM_EXPORT(lv_label_get_long_mode),
    ESP_ELFSYM_EXPORT(lv_label_set_long_mode),
    ESP_ELFSYM_EXPORT(lv_label_get_text),
    ESP_ELFSYM_EXPORT(lv_label_set_text),
    ESP_ELFSYM_EXPORT(lv_label_set_text_fmt),
    // lv_switch
    ESP_ELFSYM_EXPORT(lv_switch_create),
    // lv_checkbox
    ESP_ELFSYM_EXPORT(lv_checkbox_create),
    ESP_ELFSYM_EXPORT(lv_checkbox_set_text),
    ESP_ELFSYM_EXPORT(lv_checkbox_get_text),
    ESP_ELFSYM_EXPORT(lv_checkbox_set_text_static),
    // lv_bar
    ESP_ELFSYM_EXPORT(lv_bar_create),
    ESP_ELFSYM_EXPORT(lv_bar_get_max_value),
    ESP_ELFSYM_EXPORT(lv_bar_get_min_value),
    ESP_ELFSYM_EXPORT(lv_bar_get_mode),
    ESP_ELFSYM_EXPORT(lv_bar_get_start_value),
    ESP_ELFSYM_EXPORT(lv_bar_get_value),
    ESP_ELFSYM_EXPORT(lv_bar_set_mode),
    ESP_ELFSYM_EXPORT(lv_bar_set_range),
    ESP_ELFSYM_EXPORT(lv_bar_set_start_value),
    ESP_ELFSYM_EXPORT(lv_bar_set_value),
    ESP_ELFSYM_EXPORT(lv_bar_is_symmetrical),
    // lv_dropdown
    ESP_ELFSYM_EXPORT(lv_dropdown_create),
    ESP_ELFSYM_EXPORT(lv_dropdown_add_option),
    ESP_ELFSYM_EXPORT(lv_dropdown_clear_options),
    ESP_ELFSYM_EXPORT(lv_dropdown_close),
    ESP_ELFSYM_EXPORT(lv_dropdown_get_dir),
    ESP_ELFSYM_EXPORT(lv_dropdown_get_list),
    ESP_ELFSYM_EXPORT(lv_dropdown_get_option_count),
    ESP_ELFSYM_EXPORT(lv_dropdown_get_option_index),
    ESP_ELFSYM_EXPORT(lv_dropdown_get_options),
    ESP_ELFSYM_EXPORT(lv_dropdown_set_dir),
    ESP_ELFSYM_EXPORT(lv_dropdown_set_options),
    ESP_ELFSYM_EXPORT(lv_dropdown_set_options_static),
    ESP_ELFSYM_EXPORT(lv_dropdown_set_selected),
    ESP_ELFSYM_EXPORT(lv_dropdown_set_selected_highlight),
    ESP_ELFSYM_EXPORT(lv_dropdown_set_symbol),
    ESP_ELFSYM_EXPORT(lv_dropdown_set_text),
    ESP_ELFSYM_EXPORT(lv_dropdown_open),
    // lv_list
    ESP_ELFSYM_EXPORT(lv_list_create),
    ESP_ELFSYM_EXPORT(lv_list_add_text),
    ESP_ELFSYM_EXPORT(lv_list_add_button),
    ESP_ELFSYM_EXPORT(lv_list_get_button_text),
    ESP_ELFSYM_EXPORT(lv_list_set_button_text),
    // lv_textarea
    ESP_ELFSYM_EXPORT(lv_textarea_create),
    ESP_ELFSYM_EXPORT(lv_textarea_get_accepted_chars),
    ESP_ELFSYM_EXPORT(lv_textarea_get_label),
    ESP_ELFSYM_EXPORT(lv_textarea_get_max_length),
    ESP_ELFSYM_EXPORT(lv_textarea_get_one_line),
    ESP_ELFSYM_EXPORT(lv_textarea_set_one_line),
    ESP_ELFSYM_EXPORT(lv_textarea_set_accepted_chars),
    ESP_ELFSYM_EXPORT(lv_textarea_set_align),
    ESP_ELFSYM_EXPORT(lv_textarea_set_password_bullet),
    ESP_ELFSYM_EXPORT(lv_textarea_set_password_mode),
    ESP_ELFSYM_EXPORT(lv_textarea_set_password_show_time),
    ESP_ELFSYM_EXPORT(lv_textarea_set_placeholder_text),
    ESP_ELFSYM_EXPORT(lv_textarea_set_text),
    ESP_ELFSYM_EXPORT(lv_textarea_set_text_selection),
    // lv_palette
    ESP_ELFSYM_EXPORT(lv_palette_main),
    ESP_ELFSYM_EXPORT(lv_palette_darken),
    ESP_ELFSYM_EXPORT(lv_palette_lighten),
    // lv_display
    ESP_ELFSYM_EXPORT(lv_display_get_horizontal_resolution),
    ESP_ELFSYM_EXPORT(lv_display_get_vertical_resolution),
    ESP_ELFSYM_EXPORT(lv_display_get_physical_horizontal_resolution),
    ESP_ELFSYM_EXPORT(lv_display_get_physical_vertical_resolution),
    // lv_pct
    ESP_ELFSYM_EXPORT(lv_pct),
    ESP_ELFSYM_EXPORT(lv_pct_to_px),
    // delimiter
    ESP_ELFSYM_END
};

void tt_init_tactility_c() {
    elf_set_custom_symbols(elf_symbols);
}

}

#else // Simulator

extern "C" {

void tt_init_tactility_c() {
}

}

#endif // ESP_PLATFORM

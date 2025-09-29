#define LV_USE_PRIVATE_API 1

#include <Tactility/app/AppManifest.h>
#include <Tactility/app/AppContext.h>
#include <Tactility/lvgl/Toolbar.h>
#include <Tactility/settings/Time.h>
#include <Tactility/Preferences.h>
#include <lvgl.h>
#include <ctime>
#include <cmath>
#include <chrono>

#ifdef ESP_PLATFORM
#include "Tactility/Timer.h"
#include "Tactility/lvgl/LvglSync.h"
#include "esp_sntp.h"
#endif

namespace tt::app::clock {

class ClockApp : public App {
private:
    struct AppWrapper {
        ClockApp* app;
        AppWrapper(ClockApp* app) : app(app) {}
    };

    lv_obj_t* toolbar;
    lv_obj_t* clock_container;
    lv_obj_t* time_label; // Digital
    lv_obj_t* clock_face; // Analog
    lv_obj_t* hour_hand;
    lv_obj_t* minute_hand;
    lv_obj_t* second_hand;
    lv_obj_t* wifi_label;
    lv_obj_t* wifi_button;
    lv_obj_t* toggle_btn;
#ifdef ESP_PLATFORM
    std::unique_ptr<Timer> timer; // Only declare timer for ESP
    bool last_sync_status; // Track previous sync status to detect changes
#endif
    bool is_analog;
    AppContext* context;

#ifdef ESP_PLATFORM
    static void timer_callback(std::shared_ptr<void> appWrapper) {
        auto* app = std::static_pointer_cast<AppWrapper>(appWrapper)->app;
        app->update_time_and_check_sync();
    }
#endif

    static void toggle_mode_cb(lv_event_t* e) {
        ClockApp* app = static_cast<ClockApp*>(lv_event_get_user_data(e));
        app->toggle_mode();
    }

    static void wifi_connect_cb(lv_event_t* e) {
        tt::app::start("WifiManage");
    }

    void load_mode() {
        tt::Preferences prefs("clock_settings");
        is_analog = false;
        prefs.optBool("is_analog", is_analog);
    }

    void save_mode() {
        tt::Preferences prefs("clock_settings");
        prefs.putBool("is_analog", is_analog);
    }

    void toggle_mode() {
        is_analog = !is_analog;
        save_mode();
        redraw_clock();
    }

#ifdef ESP_PLATFORM
    bool is_time_synced() {
        return sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED;
    }

    void update_time_and_check_sync() {
        // Use FreeRTOS ticks instead of chrono (50ms = 50 ticks at 1000Hz)
        auto lock = lvgl::getSyncLock()->asScopedLock();
        if (!lock.lock(pdMS_TO_TICKS(50))) {
            TT_LOG_W("Clock", "LVGL lock timeout in update_time_and_check_sync - skipping update");
            return; // Skip this update cycle rather than hang
        }

        bool current_sync_status = is_time_synced();
        
        // If sync status changed, redraw the entire clock
        if (current_sync_status != last_sync_status) {
            last_sync_status = current_sync_status;
            redraw_clock();
            return;
        }

        // If not synced, just update the wifi label
        if (!current_sync_status) {
            if (wifi_label && lv_obj_is_valid(wifi_label)) {
                lv_label_set_text(wifi_label, "No Wi-Fi - Time not synced");
            }
            return;
        }

        // Update the actual time display
        update_time_display();
    }

    void update_time_display() {
        time_t now;
        struct tm timeinfo;
        ::time(&now);
        localtime_r(&now, &timeinfo);

        if (is_analog && clock_face && lv_obj_is_valid(clock_face)) {
            lv_coord_t clock_size = lv_obj_get_width(clock_face);
            lv_coord_t center_x = clock_size / 2;
            lv_coord_t center_y = clock_size / 2;
            
            // Scale hand lengths based on clock size
            lv_coord_t hour_length = clock_size * 0.25;
            lv_coord_t minute_length = clock_size * 0.35;
            lv_coord_t second_length = clock_size * 0.4;

            float hour_angle = (timeinfo.tm_hour % 12 + timeinfo.tm_min / 60.0f) * 30.0f - 90;
            float minute_angle = timeinfo.tm_min * 6.0f - 90;
            float second_angle = timeinfo.tm_sec * 6.0f - 90;

            if (hour_hand && lv_obj_is_valid(hour_hand)) {
                static lv_point_precise_t hour_points[2];
                hour_points[0] = {center_x, center_y};
                hour_points[1] = {center_x + (lv_coord_t)(hour_length * cos(hour_angle * M_PI / 180)), 
                                 center_y + (lv_coord_t)(hour_length * sin(hour_angle * M_PI / 180))};
                lv_line_set_points(hour_hand, hour_points, 2);
            }
            if (minute_hand && lv_obj_is_valid(minute_hand)) {
                static lv_point_precise_t minute_points[2];
                minute_points[0] = {center_x, center_y};
                minute_points[1] = {center_x + (lv_coord_t)(minute_length * cos(minute_angle * M_PI / 180)), 
                                   center_y + (lv_coord_t)(minute_length * sin(minute_angle * M_PI / 180))};
                lv_line_set_points(minute_hand, minute_points, 2);
            }
            if (second_hand && lv_obj_is_valid(second_hand)) {
                static lv_point_precise_t second_points[2];
                second_points[0] = {center_x, center_y};
                second_points[1] = {center_x + (lv_coord_t)(second_length * cos(second_angle * M_PI / 180)), 
                                   center_y + (lv_coord_t)(second_length * sin(second_angle * M_PI / 180))};
                lv_line_set_points(second_hand, second_points, 2);
            }
        } else if (!is_analog && time_label && lv_obj_is_valid(time_label)) {
            char time_str[16];
            if (settings::isTimeFormat24Hour()) {
                strftime(time_str, sizeof(time_str), "%H:%M:%S", &timeinfo);
            } else {
                strftime(time_str, sizeof(time_str), "%I:%M:%S %p", &timeinfo);
            }
            lv_label_set_text(time_label, time_str);
        }
    }
#else
    bool is_time_synced() {
        return true; // Simulator assumes synced
    }

    void update_time_and_check_sync() {
        // No-op for simulator; static message handled in redraw_clock
    }

    void update_time_display() {
        // No-op for simulator; static message handled in redraw_clock
    }
#endif

    void update_toggle_button_visibility() {
#ifdef ESP_PLATFORM
        bool should_show = is_time_synced();
#else
        bool should_show = true; // Always show in simulator
#endif
        
        if (toggle_btn) {
            if (should_show) {
                lv_obj_clear_flag(toggle_btn, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(toggle_btn, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    void get_display_metrics(lv_coord_t* width, lv_coord_t* height, bool* is_small) {
        *width = lv_obj_get_width(clock_container);
        *height = lv_obj_get_height(clock_container);
        *is_small = (*width < 240 || *height < 180); // CardKB is around 240x135
    }

    void create_wifi_prompt() {
        lv_coord_t width, height;
        bool is_small;
        get_display_metrics(&width, &height, &is_small);

        // Create a card-style container for the WiFi prompt
        lv_obj_t* card = lv_obj_create(clock_container);
        lv_obj_set_size(card, LV_PCT(90), LV_SIZE_CONTENT);
        lv_obj_center(card);
        lv_obj_set_style_radius(card, is_small ? 8 : 16, 0);
        lv_obj_set_style_bg_color(card, lv_color_hex(0x333333), 0);
        lv_obj_set_style_bg_opa(card, LV_OPA_10, 0);
        lv_obj_set_style_border_width(card, 1, 0);
        lv_obj_set_style_border_color(card, lv_color_hex(0x666666), 0);
        lv_obj_set_style_border_opa(card, LV_OPA_30, 0);
        lv_obj_set_style_pad_all(card, is_small ? 12 : 20, 0);

        // WiFi icon (using symbols)
        lv_obj_t* icon = lv_label_create(card);
        lv_label_set_text(icon, LV_SYMBOL_WIFI);
        lv_obj_align(icon, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_set_style_text_font(icon, &lv_font_montserrat_18, 0); // Use 18 instead of 20/32
        lv_obj_set_style_text_color(icon, lv_color_hex(0xFF9500), 0); // Orange color

        // Title
        wifi_label = lv_label_create(card);
        lv_label_set_text(wifi_label, "Time Not Synced");
        lv_obj_align_to(wifi_label, icon, LV_ALIGN_OUT_BOTTOM_MID, 0, is_small ? 8 : 12);
        lv_obj_set_style_text_font(wifi_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(wifi_label, LV_TEXT_ALIGN_CENTER, 0);

        // Subtitle
        lv_obj_t* subtitle = lv_label_create(card);
        lv_label_set_text(subtitle, "Connect to Wi-Fi to sync time");
        lv_obj_align_to(subtitle, wifi_label, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
        lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(subtitle, lv_color_hex(0x888888), 0);
        lv_obj_set_style_text_align(subtitle, LV_TEXT_ALIGN_CENTER, 0);

        // Connect button
        wifi_button = lv_btn_create(card);
        lv_obj_set_size(wifi_button, LV_PCT(80), is_small ? 28 : 36);
        lv_obj_align_to(wifi_button, subtitle, LV_ALIGN_OUT_BOTTOM_MID, 0, is_small ? 12 : 16);
        lv_obj_set_style_radius(wifi_button, is_small ? 6 : 8, 0);
        lv_obj_set_style_bg_color(wifi_button, lv_color_hex(0x007BFF), 0); // Blue color

        lv_obj_t* btn_label = lv_label_create(wifi_button);
        lv_label_set_text(btn_label, "Connect to Wi-Fi");
        lv_obj_center(btn_label);
        lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);

        lv_obj_add_event_cb(wifi_button, wifi_connect_cb, LV_EVENT_CLICKED, context);
    }

    void create_analog_clock() {
        lv_coord_t width, height;
        bool is_small;
        get_display_metrics(&width, &height, &is_small);

        // Calculate optimal clock size (leave margins)
        lv_coord_t max_size = LV_MIN(width * 0.8, height * 0.7);
        lv_coord_t clock_size = LV_MAX(max_size, is_small ? 80 : 160);

        // Create clock face background
        clock_face = lv_obj_create(clock_container);
        lv_obj_set_size(clock_face, clock_size, clock_size);
        lv_obj_center(clock_face);
        lv_obj_set_style_radius(clock_face, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(clock_face, lv_color_white(), 0);
        lv_obj_set_style_bg_opa(clock_face, LV_OPA_10, 0);
        lv_obj_set_style_border_width(clock_face, is_small ? 2 : 3, 0);
        lv_obj_set_style_border_color(clock_face, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_obj_set_style_border_opa(clock_face, LV_OPA_50, 0);

        // Add hour markers
        for (int i = 0; i < 12; i++) {
            float angle = i * 30.0f * M_PI / 180.0f;
            lv_coord_t marker_size = (i % 3 == 0) ? (is_small ? 3 : 4) : (is_small ? 1 : 2);
            lv_coord_t marker_length = (i % 3 == 0) ? (clock_size / 8) : (clock_size / 12);
            
            lv_obj_t* marker = lv_obj_create(clock_face);
            lv_obj_set_size(marker, marker_size, marker_length);
            lv_obj_set_style_bg_color(marker, lv_color_hex(0x666666), 0);
            lv_obj_set_style_border_width(marker, 0, 0);
            lv_obj_set_style_radius(marker, 0, 0);
            
            lv_coord_t x = clock_size / 2 + (clock_size / 2 - marker_length / 2 - 5) * cos(angle - M_PI / 2) - marker_size / 2;
            lv_coord_t y = clock_size / 2 + (clock_size / 2 - marker_length / 2 - 5) * sin(angle - M_PI / 2) - marker_length / 2;
            lv_obj_set_pos(marker, x, y);
        }

        // Create clock hands
        hour_hand = lv_line_create(clock_face);
        minute_hand = lv_line_create(clock_face);
        second_hand = lv_line_create(clock_face);

        lv_obj_set_style_line_width(hour_hand, is_small ? 3 : 5, 0);
        lv_obj_set_style_line_width(minute_hand, is_small ? 2 : 3, 0);
        lv_obj_set_style_line_width(second_hand, 1, 0);
        lv_obj_set_style_line_color(hour_hand, lv_color_black(), 0);
        lv_obj_set_style_line_color(minute_hand, lv_color_black(), 0);
        lv_obj_set_style_line_color(second_hand, lv_color_hex(0xFF0000), 0); // Red color
        lv_obj_set_style_line_rounded(hour_hand, true, 0);
        lv_obj_set_style_line_rounded(minute_hand, true, 0);

        // Center dot
        lv_obj_t* center = lv_obj_create(clock_face);
        lv_obj_set_size(center, is_small ? 6 : 8, is_small ? 6 : 8);
        lv_obj_center(center);
        lv_obj_set_style_radius(center, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(center, lv_color_black(), 0);
        lv_obj_set_style_border_width(center, 0, 0);

        update_time_display();
    }

    void create_digital_clock() {
        lv_coord_t width, height;
        bool is_small;
        get_display_metrics(&width, &height, &is_small);

        // Create main time display
        time_label = lv_label_create(clock_container);
        lv_obj_align(time_label, LV_ALIGN_CENTER, 0, is_small ? -10 : -20);
        lv_obj_set_style_text_align(time_label, LV_TEXT_ALIGN_CENTER, 0);

        // Choose appropriate font size based on screen size
        const lv_font_t* time_font;
        if (is_small) {
            time_font = &lv_font_montserrat_14;
        } else {
            time_font = &lv_font_montserrat_18;
        }
        lv_obj_set_style_text_font(time_label, time_font, 0);

        // Add subtle background for better readability
        lv_obj_set_style_bg_color(time_label, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(time_label, LV_OPA_10, 0);
        lv_obj_set_style_radius(time_label, is_small ? 4 : 8, 0);
        lv_obj_set_style_pad_all(time_label, is_small ? 6 : 12, 0);

        // Create date display
        lv_obj_t* date_label = lv_label_create(clock_container);
        lv_obj_align_to(date_label, time_label, LV_ALIGN_OUT_BOTTOM_MID, 0, is_small ? 8 : 12);
        lv_obj_set_style_text_align(date_label, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_font(date_label, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(date_label, lv_color_hex(0x888888), 0); // Use hex color instead of palette

        // Update date
        time_t now;
        struct tm timeinfo;
        ::time(&now);
        localtime_r(&now, &timeinfo);
        
        char date_str[32];
        if (is_small) {
            strftime(date_str, sizeof(date_str), "%m/%d/%Y", &timeinfo);
        } else {
            strftime(date_str, sizeof(date_str), "%A, %B %d, %Y", &timeinfo);
        }
        lv_label_set_text(date_label, date_str);

        update_time_display();
    }

    void redraw_clock() {
#ifdef ESP_PLATFORM
        auto lock = lvgl::getSyncLock()->asScopedLock();
        if (!lock.lock(lvgl::defaultLockTime)) {
            TT_LOG_E("Clock", "LVGL lock failed in redraw_clock");
            return;
        }
#endif

        // Clear the clock container
        lv_obj_clean(clock_container);
        time_label = nullptr;
        clock_face = hour_hand = minute_hand = second_hand = nullptr;
        wifi_label = nullptr;
        wifi_button = nullptr;

        // Update toggle button visibility
        update_toggle_button_visibility();

#ifdef ESP_PLATFORM
        if (!is_time_synced()) {
            create_wifi_prompt();
        } else if (is_analog) {
            create_analog_clock();
        } else {
            create_digital_clock();
        }
#else
        // Simulator: show static message
        time_label = lv_label_create(clock_container);
        lv_label_set_text(time_label, "Clock not supported in simulator");
        lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_text_align(time_label, LV_TEXT_ALIGN_CENTER, 0);
#endif
    }

public:
    void onShow(AppContext& app_context, lv_obj_t* parent) override {
        context = &app_context;
        
        // Create toolbar
        toolbar = tt::lvgl::toolbar_create(parent, app_context);
        lv_obj_align(toolbar, LV_ALIGN_TOP_MID, 0, 0);

        // Create toggle button with better styling and responsive sizing
        toggle_btn = lv_btn_create(toolbar);
        lv_obj_set_height(toggle_btn, LV_PCT(80)); // Use percentage for better scaling
        lv_obj_set_style_radius(toggle_btn, 6, 0);
        lv_obj_set_style_bg_color(toggle_btn, lv_color_hex(0x007BFF), 0);
        lv_obj_set_style_bg_opa(toggle_btn, LV_OPA_80, 0);
        
        lv_obj_t* toggle_label = lv_label_create(toggle_btn);
        lv_label_set_text(toggle_label, LV_SYMBOL_REFRESH " Mode");
        lv_obj_center(toggle_label);
        
        // Position with better responsive alignment
        lv_obj_align(toggle_btn, LV_ALIGN_RIGHT_MID, -8, 0);
        lv_obj_add_event_cb(toggle_btn, toggle_mode_cb, LV_EVENT_CLICKED, this);

        // Create clock container
        clock_container = lv_obj_create(parent);
        lv_obj_set_size(clock_container, LV_PCT(100), LV_PCT(80));
        lv_obj_align_to(clock_container, toolbar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
        lv_obj_set_style_border_width(clock_container, 0, 0);
        lv_obj_set_style_pad_all(clock_container, 10, 0);

        // Load settings and initialize
        load_mode();
        
#ifdef ESP_PLATFORM
        last_sync_status = is_time_synced();
#endif

        redraw_clock();

        // Start timer - this will handle both sync checking and time updates
#ifdef ESP_PLATFORM
        auto wrapper = std::make_shared<AppWrapper>(this);
        timer = std::make_unique<Timer>(Timer::Type::Periodic, [wrapper]() { timer_callback(wrapper); });
        timer->start(1000);
        TT_LOG_I("Clock", "Timer started in onShow");
#endif
    }

    void onHide(AppContext& app_context) override {
#ifdef ESP_PLATFORM
        // Stop timer first to prevent any callbacks during cleanup
        if (timer) {
            timer->stop();
            TT_LOG_I("Clock", "Timer stopped in onHide");
        }
        
        // Clear object pointers to prevent stale access
        time_label = nullptr;
        clock_face = hour_hand = minute_hand = second_hand = nullptr;
        wifi_label = nullptr;
        wifi_button = nullptr;
        toggle_btn = nullptr;
        clock_container = nullptr;
        toolbar = nullptr;
#endif
    }
};

extern const AppManifest clock_app = {
    .appId = "Clock",
    .appName = "Clock",
    .createApp = create<ClockApp>
};

} // namespace tt::app::clock

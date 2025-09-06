#include "Application.h"
#include "drivers/DisplayDriver.h"
#include "drivers/TouchDriver.h"

#include <esp_log.h>

#include <tt_app.h>
#include <tt_app_alertdialog.h>
#include <tt_lvgl.h>

constexpr auto TAG = "Main";

/**
 * @brief Locate a display device that supports the native DisplayDriver interface.
 *
 * Searches for up to one display device and verifies it supports driver mode.
 *
 * @param[out] deviceId Receives the found device's identifier when the function returns true.
 * @return true if a display was found and supports the DisplayDriver interface.
 * @return false if no display was found or the found display does not support driver mode.
 */
static bool findUsableDisplay(DeviceId& deviceId) {
    uint16_t display_count = 0;
    if (!tt_hal_device_find(DEVICE_TYPE_DISPLAY, &deviceId, &display_count, 1)) {
        ESP_LOGE(TAG, "No display device found");
        return false;
    }

    if (!tt_hal_display_driver_supported(deviceId)) {
        ESP_LOGE(TAG, "Display doesn't support driver mode");
        return false;
    }

    return true;
}

/**
 * @brief Locate a touch device that supports the TouchDriver interface.
 *
 * Attempts to find a touch device and verifies it supports driver mode.
 *
 * @param deviceId[out] Set to the found device's ID on success.
 * @return true if a touch device was found and supports the touch driver interface.
 * @return false if no touch device was found or the device does not support driver mode.
 */
static bool findUsableTouch(DeviceId& deviceId) {
    uint16_t touch_count = 0;
    if (!tt_hal_device_find(DEVICE_TYPE_TOUCH, &deviceId, &touch_count, 1)) {
        ESP_LOGE(TAG, "No touch device found");
        return false;
    }

    if (!tt_hal_touch_driver_supported(deviceId)) {
        ESP_LOGE(TAG, "Touch doesn't support driver mode");
        return false;
    }

    return true;
}

/**
 * @brief Application entry: initialize hardware drivers and run the app.
 *
 * Initializes and validates a display and touch device, stops LVGL so the
 * drivers can be acquired, constructs DisplayDriver and TouchDriver, runs
 * the main application logic via runApplication(), then cleans up and stops
 * the application.
 *
 * If no usable display or touch device is found the function stops the
 * app and presents a user-facing alert dialog before returning.
 *
 * Note: Parameters are provided by the application framework and are not
 * used by this implementation.
 *
 * @param appHandle Framework handle for the launching application (unused).
 * @param data Opaque framework data pointer (unused).
 */
static void onCreate(AppHandle appHandle, void* data) {
    DeviceId display_id;
    if (!findUsableDisplay(display_id)) {
        tt_app_stop();
        tt_app_alertdialog_start("Error", "The display doesn't support the required features.", nullptr, 0);
        return;
    }

    DeviceId touch_id;
    if (!findUsableTouch(touch_id)) {
        tt_app_stop();
        tt_app_alertdialog_start("Error", "The touch driver doesn't support the required features.", nullptr, 0);
        return;
    }

    // Stop LVGL first (because it's currently using the drivers we want to use)
    tt_lvgl_stop();

    ESP_LOGI(TAG, "Creating display driver");
    auto display = new DisplayDriver(display_id);

    ESP_LOGI(TAG, "Creating touch driver");
    auto touch = new TouchDriver(touch_id);

    // Run the main logic
    ESP_LOGI(TAG, "Running application");
    runApplication(display, touch);

    ESP_LOGI(TAG, "Cleanup display driver");
    delete display;

    ESP_LOGI(TAG, "Cleanup touch driver");
    delete touch;

    ESP_LOGI(TAG, "Stopping application");
    tt_app_stop();
}

static void onDestroy(AppHandle appHandle, void* data) {
    // Restart LVGL to resume rendering of regular apps
    if (!tt_lvgl_is_started()) {
        ESP_LOGI(TAG, "Restarting LVGL");
        tt_lvgl_start();
    }
}

ExternalAppManifest manifest = {
    .name = "Hello World",
    .onCreate = onCreate,
    .onDestroy = onDestroy
};

extern "C" {

int main(int argc, char* argv[]) {
    tt_app_register(&manifest);
    return 0;
}

}

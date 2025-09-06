#include "tt_app.h"
#include <Tactility/app/App.h>
#include <Tactility/app/AppContext.h>

extern "C" {

constexpr auto* TAG = "tt_app";

#define HANDLE_AS_APP_CONTEXT(handle) ((tt::app::AppContext*)(handle))

BundleHandle _Nullable tt_app_get_parameters(AppHandle handle) {
    return (BundleHandle)HANDLE_AS_APP_CONTEXT(handle)->getParameters().get();
}

void tt_app_set_result(AppHandle handle, AppResult result, BundleHandle _Nullable bundle) {
    auto shared_bundle = std::unique_ptr<tt::Bundle>(static_cast<tt::Bundle*>(bundle));
    HANDLE_AS_APP_CONTEXT(handle)->getApp()->setResult(static_cast<tt::app::Result>(result), std::move(shared_bundle));
}

bool tt_app_has_result(AppHandle handle) {
    return HANDLE_AS_APP_CONTEXT(handle)->getApp()->hasResult();
}

void tt_app_start(const char* appId) {
    tt::app::start(appId);
}

void tt_app_start_with_bundle(const char* appId, BundleHandle parameters) {
    tt::app::start(appId, std::shared_ptr<tt::Bundle>(static_cast<tt::Bundle*>(parameters)));
}

/**
 * @brief Stops the currently running app.
 *
 * Stops the active application managed by the framework. If no app is running, this call has no effect.
 */
void tt_app_stop() {
    tt::app::stop();
}

/**
 * @brief Writes the app data-directory path into a caller-provided buffer.
 *
 * Copies the null-terminated data-directory path into `buffer` and updates `*size`:
 * - On entry, `*size` must contain the capacity of `buffer` (must be > 0).
 * - On success, `*size` is set to the length of the written path (number of characters, excluding the null terminator).
 * - If the provided buffer is too small, `*size` is set to 0 and `buffer` is set to an empty string.
 *
 * The function asserts that `buffer` and `size` are non-null and that `*size > 0`. The written string is always null-terminated.
 *
 * @param buffer Destination buffer to receive the path.
 * @param size Pointer to an in/out size value: input = buffer capacity, output = written string length or 0 on failure.
 */
void tt_app_get_data_directory(AppPathsHandle handle, char* buffer, size_t* size) {
    assert(buffer != nullptr);
    assert(size != nullptr);
    assert(*size > 0);
    auto paths = HANDLE_AS_APP_CONTEXT(handle)->getPaths();
    auto data_path = paths->getDataDirectory();
    auto expected_length = data_path.length() + 1;
    if (*size < expected_length) {
        TT_LOG_E(TAG, "Path buffer not large enough (%d < %d)", size, expected_length);
        *size = 0;
        buffer[0] = 0;
        return;
    }

    strcpy(buffer, data_path.c_str());
    *size = data_path.length();
}

/**
 * @brief Write the LVGL-specific app data directory path into a caller-provided buffer.
 *
 * Copies the path returned by the app context's getDataDirectoryLvgl() into `buffer`.
 * `size` is an in/out parameter: on entry it must point to the available buffer length (must be > 0);
 * on success `*size` is set to the number of characters written (excluding the null terminator).
 * On insufficient buffer space, the function sets `*size` to 0 and writes an empty string to `buffer`.
 *
 * Preconditions (asserted):
 * - `buffer` is not null.
 * - `size` is not null.
 * - `*size > 0`.
 *
 * Notes:
 * - The function null-terminates `buffer`.
 * - The caller must ensure the provided buffer remains valid for the call.
 *
 * @param buffer Destination buffer for the path.
 * @param size Pointer to the buffer size on entry; updated to the written length on success or 0 on failure.
 */
void tt_app_get_data_directory_lvgl(AppPathsHandle handle, char* buffer, size_t* size) {
    assert(buffer != nullptr);
    assert(size != nullptr);
    assert(*size > 0);
    auto paths = HANDLE_AS_APP_CONTEXT(handle)->getPaths();
    auto data_path = paths->getDataDirectoryLvgl();
    auto expected_length = data_path.length() + 1;
    if (*size < expected_length) {
        TT_LOG_E(TAG, "Path buffer not large enough (%d < %d)", size, expected_length);
        *size = 0;
        buffer[0] = 0;
        return;
    }

    strcpy(buffer, data_path.c_str());
    *size = data_path.length();
}

}
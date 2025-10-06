#include "Main.h"
#include <Tactility/TactilityCore.h>
#include <Tactility/Thread.h>

#include "FreeRTOS.h"
#include "task.h"

#ifdef __APPLE__
#include <unistd.h>
#endif

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include "LvglTask.h"
#include <lvgl.h>
#include <Tactility/Tactility.h>
#include <Tactility/Log.h>

// Forward declarations for SDL functions and display handle (from LvglTask.cpp)
extern "C" {
    lv_disp_t* lv_sdl_window_create(int hor_res, int ver_res);
    void lv_sdl_window_set_title(lv_disp_t* disp, const char* title);
}
extern lv_disp_t* displayHandle;

// Namespace must match the extern declaration in Thread.cpp
namespace tt {
// Helper to run thread functions on the main dispatcher for WASM builds
// Called from tt::Thread::start() when __EMSCRIPTEN__ is defined  
void scheduleWasmThreadFunction(Thread::MainFunction function) {
    TT_LOG_I("wasm_thread", "scheduleWasmThreadFunction called, dispatching...");
    getMainDispatcher().dispatch([function]() {
        TT_LOG_I("wasm_thread", "Executing thread function");
        int32_t result = function();
        TT_LOG_I("wasm_thread", "Thread function completed with result %d", result);
    });
    TT_LOG_I("wasm_thread", "scheduleWasmThreadFunction dispatch queued");
}
} // namespace tt
#endif

#define TAG "freertos"

namespace simulator {

MainFunction mainFunction = nullptr;

void setMain(MainFunction newMainFunction) {
    mainFunction = newMainFunction;
}

static void freertosMainTask(TT_UNUSED void* parameter) {
    TT_LOG_I(TAG, "starting app_main()");
    assert(simulator::mainFunction);
    mainFunction();
    TT_LOG_I(TAG, "returned from app_main()");
    vTaskDelete(nullptr);
}

void freertosMain() {
#ifdef __EMSCRIPTEN__
    // For WASM, skip FreeRTOS scheduler and use Emscripten's main loop
    TT_LOG_I(TAG, "WASM mode: running without FreeRTOS scheduler");
    assert(simulator::mainFunction);
    
    // Initialize LVGL before creating display
    TT_LOG_I(TAG, "Initializing LVGL");
    lv_init();
    
    // Create the LVGL display now (after Module.canvas is set up)
    TT_LOG_I(TAG, "Creating LVGL display before main initialization");
    ::displayHandle = lv_sdl_window_create(320, 240);
    if (::displayHandle) {
        lv_sdl_window_set_title(::displayHandle, "Tactility");
        TT_LOG_I(TAG, "LVGL display created successfully");
    } else {
        TT_LOG_E(TAG, "Failed to create LVGL display");
    }
    
    // Call main function to initialize everything
    mainFunction();
    
    // Set up the main loop to handle LVGL updates
    TT_LOG_I(TAG, "Setting up Emscripten main loop");
    emscripten_set_main_loop(lvgl_task_tick, 0, 1);
#elif defined(__APPLE__)
    // For macOS, skip FreeRTOS scheduler to avoid main thread issues with SDL
    TT_LOG_I(TAG, "macOS mode: running without FreeRTOS scheduler");
    assert(simulator::mainFunction);
    
    // Initialize LVGL before creating display
    TT_LOG_I(TAG, "Initializing LVGL");
    lv_init();
    
    // Create the LVGL display
    TT_LOG_I(TAG, "Creating LVGL display");
    ::displayHandle = lv_sdl_window_create(320, 240);
    if (::displayHandle) {
        lv_sdl_window_set_title(::displayHandle, "Tactility");
        TT_LOG_I(TAG, "LVGL display created successfully");
    } else {
        TT_LOG_E(TAG, "Failed to create LVGL display");
    }
    
    // Call main function to initialize everything
    mainFunction();
    
    // Set up the main loop to handle LVGL updates (simple loop for macOS)
    TT_LOG_I(TAG, "Starting macOS main loop");
    while (lvgl_task_is_running()) {
        lvgl_task_tick();
        // Sleep a bit to avoid busy loop
        usleep(10000); // 10ms
    }
#else
    BaseType_t task_result = xTaskCreate(
        freertosMainTask,
        "main",
        8192,
        nullptr,
        static_cast<UBaseType_t>(tt::Thread::Priority::Normal),
        nullptr
    );

    assert(task_result == pdTRUE);

    // Blocks forever
    vTaskStartScheduler();
#endif
}

} // namespace

/**
 * Assert implementation as defined in the FreeRTOSConfig.h
 * It allows you to set breakpoints and debug asserts.
 */
void vAssertCalled(unsigned long line, const char* const file) {
    static portBASE_TYPE xPrinted = pdFALSE;
    volatile uint32_t set_to_nonzero_in_debugger_to_continue = 0;

    TT_LOG_E(TAG, "assert triggered at %s:%d", file, line);
    taskENTER_CRITICAL();
    {
        // Step out by attaching a debugger and setting set_to_nonzero_in_debugger_to_continue
        while (set_to_nonzero_in_debugger_to_continue == 0) {
            // NO-OP
        }
    }
    taskEXIT_CRITICAL();
}


#include "Main.h"
#include <Tactility/TactilityCore.h>
#include <Tactility/Tactility.h>
#include <Tactility/Thread.h>

#include "FreeRTOS.h"
#include "task.h"

#include <cstdlib>
#include <cstdio>

#define TAG "freertos"

namespace websimulator {

MainFunction mainFunction = nullptr;

void setMain(MainFunction newMainFunction) {
    mainFunction = newMainFunction;
}

static void freertosMainTask(TT_UNUSED void* parameter) {
    TT_LOG_I(TAG, "starting app_main()");
    assert(websimulator::mainFunction);
    mainFunction();
    TT_LOG_I(TAG, "returned from app_main()");
    vTaskDelete(nullptr);
}

void freertosMain() {
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
}

// Emscripten idle hook: yield to browser when idle
extern "C" void vApplicationIdleHook(void) {
    // Yield control (prevents blocking the main thread)
}

} // namespace websimulator

/**
 * Emscripten main function - called when WASM module loads
 */
extern "C" int main(int argc, char* argv[]) {
    // Each board project declares this variable
    extern const tt::hal::Configuration hardwareConfiguration;

    static const tt::Configuration config = {
        .hardware = &hardwareConfiguration
    };

    // Set the main function for FreeRTOS
    websimulator::setMain([]() {
        tt::run(config);
    });

    // Start FreeRTOS (this will call our main function)
    websimulator::freertosMain();

    return 0;
}

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

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // This function is called when a stack overflow is detected.
    // For the simulator, we can simply log the error and stop execution.
    fprintf(stderr, "Stack overflow in task: %s\n", pcTaskName);
    abort(); // Stop execution, as stack overflow is a serious error.
}
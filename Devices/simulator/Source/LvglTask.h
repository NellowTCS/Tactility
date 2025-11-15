#pragma once

void lvgl_task_start();
bool lvgl_task_is_running();
void lvgl_task_interrupt();

#if defined(__EMSCRIPTEN__) || defined(__APPLE__)
// For WASM and macOS builds, expose the task function to be called in main loop
void lvgl_task_tick();
#endif

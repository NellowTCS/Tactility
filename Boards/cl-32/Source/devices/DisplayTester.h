#pragma once

#include <cstdint>

class GxEPD2Display;

namespace display_tester {
    // Run a set of one-shot display tests on the provided GxEPD2Display instance.
    // This function performs its own allocations and does blocking refreshes; use only for debugging.
    void runTests(GxEPD2Display* display);

    // Run a simple LVGL-based test (creates a temporary LVGL screen with three horizontal bands
    // and a centered label). The function will check whether LVGL is started for the provided
    // display and do nothing if not.
    void runLvglTest(GxEPD2Display* display);
}
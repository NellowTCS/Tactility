#include "Boards.h"
#include <Tactility/Tactility.h>

#ifdef ESP_PLATFORM
#include "tt_init.h"
#endif

extern const tt::app::AppManifest hello_world_app;

extern "C" {

/**
 * @brief Application entry point that configures and starts the Tactility runtime.
 *
 * Initializes a static tt::Configuration (auto-selecting the board via TT_BOARD_HARDWARE)
 * and starts the runtime by calling tt::run(config). When built for ESP_PLATFORM, it
 * also performs platform-specific ELF binding initialization required for side-loading.
 *
 * The configuration created here uses an empty apps list by default; individual apps
 * can be added to the config's apps array before running if desired.
 */
void app_main() {
    static const tt::Configuration config = {
        /**
         * Auto-select a board based on the ./sdkconfig.board.* file
         * that you copied to ./sdkconfig before you opened this project.
         */
        .hardware = TT_BOARD_HARDWARE,
        .apps = {
            // &hello_world_app,
        }
    };

#ifdef ESP_PLATFORM
    tt_init_tactility_c(); // ELF bindings for side-loading on ESP32
#endif

    tt::run(config);
}

} // extern
#include "board_config.h"
#include "tactility-esp.h"
#include "services/loader/loader.h"

// Apps
#include "hello_world/hello_world.h"

extern void wifi_main(void*);

extern const ServiceManifest wifi_service;
extern const AppManifest wifi_connect_app;
extern const AppManifest wifi_manage_app;

TT_UNUSED void app_main(void) {
    static const Config config = {
        /**
         * Auto-select a board based on the ./sdkconfig.board.* file
         * that you copied to ./sdkconfig before you opened this project.
         */
        .hardware = TT_BOARD_HARDWARE,
        .apps = {
            &hello_world_app,
            &wifi_connect_app,
            &wifi_manage_app
        },
        .services = {
            &wifi_service
        },
        .auto_start_app_id = NULL
    };

    tt_esp_init(&config);
    tt_init(&config.apps, CONFIG_APPS_LIMIT, &config.services, CONFIG_SERVICES_LIMIT);

    loader_start_app("desktop", true, NULL);

    wifi_main(NULL);
}
